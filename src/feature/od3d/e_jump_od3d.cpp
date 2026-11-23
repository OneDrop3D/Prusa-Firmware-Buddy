/// @file
#include "e_jump_od3d.hpp"
#include "filament_memory_od3d.hpp"
#include "live_od3d.hpp"
#include "flow_limit_od3d.hpp"
#include "measure_session_od3d.hpp"
#include "pa_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>

#include <common/marlin_server.hpp>
#include <config_store/store_instance.hpp>
#include <feature/pressure_advance/pressure_advance_config.hpp>
#include <feature/prusa/e-stall_detector.h>
#include <filament.hpp>
#include <i18n.h>
#include <loadcell.hpp>
#include <mapi/motion.hpp>
#include <mapi/parking.hpp>
#include <Marlin.h>
#include <module/motion.h>
#include <module/planner.h>
#include <raii/scope_guard.hpp>
#include <tool_index.hpp>

namespace od3d::e_jump {

static_assert(prusa_filament_accel == DEFAULT_MAX_FILAMENT_ACCELERATION, "120 de Prusa : sans mesure, on garde le sien");

// ---- Mémoire ----

Record record(uint8_t slot) {
    if (slot >= VirtualToolIndex::count) {
        return {};
    }
    return config_store().od3d_e_jump.get(slot);
}

void set_record(uint8_t slot, const Record &r) {
    if (slot < VirtualToolIndex::count && config_store().od3d_e_jump.get(slot) != r) {
        config_store().od3d_e_jump.set(slot, r);
    }
}

void clear(uint8_t slot) {
    set_record(slot, Record {});
}

float filament_accel_for(uint8_t slot) {
    const Record r = record(slot);
    const float k = pa::k_for_slot(slot).value_or(pressure_advance::get_axis_e_config().pressure_advance);
    const float f = r.has_jump() ? filament_accel(r.jump(), k) : NAN;
    return std::isfinite(f) ? f : prusa_filament_accel;
}

// ---- Application ----

namespace {
    float ours = NAN; ///< dernière F posée par refresh() ; NAN : la valeur en vigueur n'est pas la nôtre
} // namespace

void refresh() {
    if (flow_limit::suspended) {
        return; // mesure par la cellule : F de Prusa, la session remet la nôtre à la fin
    }
    const float current = planner.user_settings.max_filament_acceleration_mm_per_s2;
    float want = NAN;
    if (const auto tool = VirtualToolIndex::currently_selected_opt()) {
        if (const Record r = record(tool->to_raw()); r.has_jump()) {
            want = filament_accel(r.jump(), pressure_advance::get_axis_e_config().pressure_advance);
        }
    }
    float target = want;
    if (!std::isfinite(want)) {
        if (current != ours) {
            ours = NAN;
            return; // pas la nôtre (M204 F, réglages remis par Prusa) : gardée
        }
        target = prusa_filament_accel;
    }
    if (target != current) {
        // Comme M204 : sans planner.synchronize(), la tête ne s'arrête pas ; les blocs déjà en file gardent l'ancienne F
        auto s = planner.user_settings;
        s.max_filament_acceleration_mm_per_s2 = target;
        planner.apply_settings(s);
    }
    ours = std::isfinite(want) ? target : NAN;
}

// ---- Mesure ----

namespace {

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<uint16_t> runs = 0;
    std::atomic<uint16_t> revisions = 0;

    std::atomic<Phase> st_phase = Phase::idle;
    std::atomic<Error> st_error = Error::none;
    std::atomic<uint8_t> st_slot = 0;
    std::atomic<uint8_t> st_step = 0;
    std::atomic<uint8_t> st_steps = 0;
    std::atomic<uint16_t> st_jump = 0;
    std::atomic<int16_t> st_temperature = 0;

    LastRun last;

    constexpr uint32_t rest_ms = 300; ///< entre deux impulsions : la pression retombe
    constexpr float prime_mm = 3.f; ///< avant les paliers : la zone de fusion pleine
    constexpr int16_t min_temperature_margin = 10; ///< jamais sous EXTRUDE_MINTEMP + 10
    /// Comme le chargement du MMU de Prusa (FeedWithEStallDetection) : la cellule échantillonne à peine plus vite que les
    /// à-coups d'un moteur qui saute des pas, d'où un seuil plus bas que celui de l'impression (700 000)
    constexpr float stall_threshold = 500'000.f;

    void fail(Error e) {
        st_error = e;
        st_phase = e == Error::none ? Phase::done : Phase::failed;
    }

    Error error_of(session::Result r) {
        switch (r) {
        case session::Result::ok:
            return Error::none;
        case session::Result::homing:
            return Error::homing;
        case session::Result::heating:
            return Error::heating;
        default:
            return Error::stopped;
        }
    }

    /// Une impulsion ; stalled : le détecteur a vu un calage. false s'il faut s'arrêter
    bool run_pulse(const Pulse &p, bool &stalled) {
        EMotorStallDetector &emsd = EMotorStallDetector::Instance();
        emsd.ClearDetected();
        stalled = false;
        // Les trois en file d'un coup : le planificateur enchaîne les vitesses sans s'arrêter entre elles
        if (!mapi::extruder_move(p.before_mm, p.base_speed) || !mapi::extruder_move(p.jump_mm, p.peak_speed) || !mapi::extruder_move(p.after_mm, p.base_speed)) {
            return false;
        }
        // Lu pendant le mouvement : Marlin efface la détection dès que E s'arrête (EMotorStallDetector::Evaluate)
        while (planner.processing()) {
            stalled = stalled || emsd.DetectedRaw();
            if (session::must_stop()) {
                return false;
            }
            idle(true);
        }
        stalled = stalled || emsd.DetectedRaw();
        return session::wait_ms(rest_ms);
    }

    /// Préparation et paliers ; Error::none : last.verdict est à enregistrer
    Error measure(session::Session &session, const FilamentTypeParameters &material, int16_t temperature) {
        // PA coupé : le saut est fait par nos mouvements, pas par lui (file vide : la session a synchronisé)
        pressure_advance::set_axis_e_config({ .pressure_advance = 0.f, .smooth_time = session.smooth_time() });
        st_phase = Phase::homing;
        // Origine seulement si besoin (session.home attendrait sinon la buse descendre à sa température de palpage)
        if (axes_need_homing()) {
            if (const session::Result r = session.home(std::min<int16_t>(material.nozzle_preheat_temperature, temperature)); r != session::Result::ok) {
                return error_of(r);
            }
        }
        mapi::park(mapi::get_parking_position(mapi::ParkPosition::purge));
        if (session::must_stop()) {
            return Error::stopped;
        }
        st_phase = Phase::heating;
        if (const session::Result r = session.heat(temperature); r != session::Result::ok) {
            return error_of(r);
        }

        Loadcell::HighPrecisionEnabler high_precision(loadcell); // comme le chargement du MMU de Prusa
        EStallDetectionStateLatch latch; // seuil et état du détecteur remis à la fin
        EMotorStallDetector::Instance().SetDetectionThreshold(stall_threshold);
        mapi::extruder_move(prime_mm, base_speed);
        if (!session::wait_moves()) {
            return Error::stopped;
        }

        st_phase = Phase::measuring;
        std::array<float, max_steps> jumps;
        const size_t n = jump_steps(jumps);
        last.planned = uint8_t(n);
        for (size_t i = 0; i < n; ++i) {
            st_step = uint8_t(i + 1);
            st_jump = jump_to_e1(jumps[i]);
            live::set(N_("Extruder speed jump %d/%d: %d mm/s"), int(i + 1), int(n), int(std::lround(jumps[i])));
            const Pulse p = pulse(jumps[i], session.smooth_time());
            auto s = planner.user_settings; // remis par la session
            s.retract_acceleration = p.accel; // accélération des mouvements de E seul
            s.max_feedrate_mm_s[E_AXIS] = std::max(s.max_feedrate_mm_s[E_AXIS], p.peak_speed);
            planner.apply_settings(s);
            Step step { jumps[i], 0 };
            for (uint8_t k = 0; k < pulses; ++k) {
                bool stalled = false;
                if (!run_pulse(p, stalled)) {
                    return Error::stopped;
                }
                step.stalls = uint8_t(step.stalls + stalled);
            }
            last.step[i] = step;
            last.points = uint8_t(i + 1);
            ++revisions;
            if (must_stop(std::span<const Step>(last.step.data(), i + 1))) {
                break;
            }
        }
        last.verdict = classify(std::span<const Step>(last.step.data(), last.points), last.points == n);
        switch (last.verdict.fit) {
        case Fit::found:
        case Fit::not_reached:
            return Error::none;
        case Fit::too_low:
            return Error::too_low;
        case Fit::none:
            break;
        }
        return Error::stopped;
    }

} // namespace

Status status() {
    return {
        .phase = st_phase.load(),
        .error = st_error.load(),
        .slot = st_slot.load(),
        .step = st_step.load(),
        .steps = st_steps.load(),
        .jump_e1 = st_jump.load(),
        .temperature = st_temperature.load(),
    };
}

bool is_running() {
    return running.load();
}

uint16_t run_count() {
    return runs.load();
}

uint16_t revision() {
    return revisions.load();
}

void request_stop() {
    stop_requested = true;
}

const LastRun &last_run() {
    return last;
}

void calibrate(uint8_t slot) {
    stop_requested = false;
    st_slot = slot;
    st_step = 0;
    st_steps = 0;
    st_jump = 0;
    st_temperature = 0;
    st_error = Error::none;
    st_phase = Phase::preparing;
    ScopeGuard count_run = [] { ++runs; }; // après l'état : l'interface lit l'état une fois runs changé

    if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
        fail(Error::printing);
        return;
    }
    if (slot >= VirtualToolIndex::count) {
        fail(Error::bad_request);
        return;
    }
    const FilamentType type = FilamentType::for_tool(VirtualToolIndex::from_raw(slot));
    if (!type || filament_memory::loaded_slot() != slot) {
        fail(Error::not_loaded);
        return;
    }
    const FilamentTypeParameters material = type.parameters();
    const int16_t temperature = std::clamp<int16_t>(material.nozzle_temperature, EXTRUDE_MINTEMP + min_temperature_margin, session::max_temperature);
    st_temperature = temperature;
    st_steps = uint8_t(max_steps);
    last = LastRun {};
    last.slot = slot;
    ++revisions;

    running = true;
    ScopeGuard not_running = [] { running = false; };
    Error e;
    {
        session::Session session(stop_requested); // PA, réglages, consignes et détecteur remis à la fin
        e = measure(session, material, temperature);
    }
    if (e == Error::none) {
        Record r;
        r.jump_e1 = jump_to_e1(last.verdict.kept_jump());
        r.flags = last.verdict.fit == Fit::not_reached ? Record::lower_bound : 0;
        set_record(slot, r);
        st_jump = r.jump_e1;
    }
    refresh(); // après la session, qui a remis les réglages d'avant
    ++revisions;
    fail(e);
}

} // namespace od3d::e_jump
