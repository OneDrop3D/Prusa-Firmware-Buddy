/// @file
#include "spool_calib_od3d.hpp"
#include "flow_limit_od3d.hpp"
#include "force_probe_od3d.hpp"
#include "measure_session_od3d.hpp"
#include "pa_math_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <span>
#include <tuple>

#include <common/marlin_server.hpp>
#include <config_store/store_instance.hpp>
#include <filament.hpp>
#include <loadcell.hpp>
#include <Marlin.h>
#include <feature/pressure_advance/pressure_advance_config.hpp>
#include <metric.h>
#include <module/motion.h>
#include <module/planner.h>
#include <raii/scope_guard.hpp>
#include <tool_index.hpp>

METRIC_DEF(metric_od3d_calib, "od3d_calib", METRIC_VALUE_CUSTOM, 0, METRIC_DISABLED);

namespace od3d::calib {

// ---- Mémoire ----

namespace {
    std::atomic<float> applied_flow = 0.f;
} // namespace

Record record(uint8_t slot) {
    if (slot >= VirtualToolIndex::count) {
        return {};
    }
    return config_store().od3d_spool_calib.get(slot);
}

void set_record(uint8_t slot, const Record &r) {
    if (slot < VirtualToolIndex::count && config_store().od3d_spool_calib.get(slot) != r) {
        config_store().od3d_spool_calib.set(slot, r);
    }
    refresh_flow_limit();
}

void clear(uint8_t slot) {
    set_record(slot, Record {});
}

bool flow_limit_enabled() {
    return config_store().od3d_flow_limit.get();
}

void set_flow_limit_enabled(bool set) {
    config_store().od3d_flow_limit.set(set);
    refresh_flow_limit();
}

void refresh_flow_limit() {
    float flow = 0.f;
    if (flow_limit_enabled()) {
        if (const auto tool = VirtualToolIndex::currently_selected_opt()) {
            if (const Record r = record(tool->to_raw()); r.has_flow()) {
                flow = std::clamp(flow_from_e1(r.max_flow_e1), flow_limit_floor, flow_ceiling);
            }
        }
    }
    applied_flow = flow;
    flow_limit::e_speed_mm_s = e_speed_limit(flow, flow_limit_floor, flow_ceiling, pa::filament_area_mm2);
}

float applied_flow_limit() {
    return applied_flow.load();
}

// ---- Mesures ----

namespace {

    using session::must_stop;
    using session::wait_moves;
    using session::wait_ms;

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<uint16_t> runs = 0;

    std::atomic<Kind> st_kind = Kind::flow;
    std::atomic<Phase> st_phase = Phase::idle;
    std::atomic<Error> st_error = Error::none;
    std::atomic<uint8_t> st_slot = 0;
    std::atomic<uint8_t> st_step = 0;
    std::atomic<uint8_t> st_steps = 0;
    std::atomic<int16_t> st_value = 0;
    std::atomic<int16_t> st_temperature = 0;
    std::atomic<bool> st_csv = false;

    LastRun last;

    constexpr float step_speed = 5.f; ///< la tête avance lentement en X pendant un palier : le filament s'étale [mm/s]
    constexpr float row_pitch = 5.f; ///< paliers sur 4 lignes espacées de 5 mm en Y, en allers-retours
    constexpr uint8_t rows = 4;
    constexpr uint32_t rest_ms = 800; ///< tête arrêtée avant un palier : la pression retombe
    constexpr uint32_t zero_ms = 200; ///< zéro de la force, tête arrêtée
    constexpr uint32_t settle_ms = 1000; ///< début du palier écarté (montée de la pression)
    constexpr uint32_t tail_ms = 400; ///< fin du palier écartée (le mouvement peut partir un peu après la demande)
    constexpr uint32_t min_window_points = 40; ///< à 160 Hz : 0,25 s
    constexpr float max_force_g = 4000.f; ///< au-delà, la rampe s'arrête (extrudeur à la peine)
    constexpr float prime_mm = 8.f; ///< purge avant la mesure [mm de filament]
    constexpr float prime_speed = 2.f; ///< [mm/s de filament]
    constexpr int16_t min_temperature_margin = 10; ///< jamais sous EXTRUDE_MINTEMP + 10 (extrusion à froid refusée)
    constexpr int16_t default_drop = 60; ///< F6 : plancher par défaut, 60 °C sous la température nominale
    constexpr uint32_t settle_timeout_ms = 4 * 60 * 1000; ///< F6 : descente d'un palier de température

    void fail(Error e) {
        st_error = e;
        st_phase = e == Error::none ? Phase::done : Phase::failed;
    }

    Error error_of(session::Result r) {
        switch (r) {
        case session::Result::ok:
            return Error::none;
        case session::Result::stopped:
            return Error::stopped;
        case session::Result::homing:
            return Error::homing;
        case session::Result::no_room:
            return Error::no_room;
        case session::Result::heating:
            return Error::heating;
        }
        return Error::stopped;
    }

    /// Début d'une mesure : état remis à zéro
    void begin(Kind kind, uint8_t slot) {
        stop_requested = false;
        st_kind = kind;
        st_slot = slot;
        st_step = 0;
        st_steps = 0;
        st_value = 0;
        st_temperature = 0;
        st_csv = false;
        st_error = Error::none;
        st_phase = Phase::preparing;
    }

    /// Lignes des paliers : la tête va de x_lo à x_hi (ou l'inverse) à step_speed, sur la ligne row % rows
    struct Lines {
        float x_lo = 0;
        float x_hi = 0;
        float y0 = 0;
        bool forward = true;
        uint8_t row = 0;
    };

    enum class StepResult : uint8_t {
        ok,
        stopped,
        no_signal, ///< la cellule n'a pas donné assez d'échantillons
    };

    /// Un palier d'extrusion à débit constant (flow mm³/s pendant step_s) avec la tête qui avance en X ; force moyenne
    /// et écart type sur le milieu du palier, relatifs à la tête au repos juste avant
    StepResult run_step(Lines &lines, float flow, float step_s, float label, StepPoint &out, FILE *csv) {
        planner.synchronize();
        if (must_stop()) {
            return StepResult::stopped;
        }
        const float y = lines.y0 + row_pitch * float(lines.row % rows);
        do_blocking_move_to_xy(lines.forward ? lines.x_lo : lines.x_hi, y, session::travel_speed_xy);

        // Cellule en haute précision (320 Hz) le temps du palier seulement
        Loadcell::HighPrecisionEnabler high_precision(loadcell);
        if (!wait_ms(rest_ms)) {
            return StepResult::stopped;
        }
        force_probe::start();
        ScopeGuard stop_probe = [] { force_probe::stop(); };
        if (!wait_ms(zero_ms)) {
            return StepResult::stopped;
        }
        current_position.x = lines.forward ? lines.x_hi : lines.x_lo;
        current_position.e += flow / pa::filament_area_mm2 * step_s;
        line_to_current_position(step_speed);
        const uint32_t step_ms = uint32_t(step_s * 1000.f);
        if (!wait_ms(settle_ms)) {
            return StepResult::stopped;
        }
        force_probe::window_begin();
        if (!wait_ms(step_ms - settle_ms - tail_ms)) {
            return StepResult::stopped;
        }
        force_probe::window_end();
        if (!wait_moves() || !wait_ms(150)) {
            return StepResult::stopped;
        }
        force_probe::stop();
        lines.forward = !lines.forward;
        ++lines.row;

        const force_probe::WindowStats w = force_probe::window();
        out = { label, w.mean_g, w.sd_g };
        if (csv) {
            std::ignore = force_probe::write_csv(csv, label);
        }
        return w.count >= min_window_points ? StepResult::ok : StepResult::no_signal;
    }

    void record_step(FILE *csv, Kind kind, const StepPoint &p) {
        metric_record_custom(&metric_od3d_calib, " kind=%ui,x=%.2f,force=%.2f,noise=%.3f", unsigned(kind), double(p.x), double(p.force_g), double(p.noise_g));
        if (csv) {
            fprintf(csv, "R,%.2f,%.3f,%.3f\n", double(p.x), double(p.force_g), double(p.noise_g));
        }
    }

    /// Vérifications communes : pas d'impression, emplacement valide et chargé ; rend la matière, vide si refusé
    FilamentType check_slot(uint8_t slot) {
        if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
            fail(Error::printing);
            return FilamentType::none;
        }
        if (slot >= VirtualToolIndex::count) {
            fail(Error::bad_request);
            return FilamentType::none;
        }
        const auto loaded = VirtualToolIndex::currently_selected_opt();
        const FilamentType type = FilamentType::for_tool(VirtualToolIndex::from_raw(slot));
        if (!loaded || loaded->to_raw() != slot || !type) {
            fail(Error::not_loaded);
            return FilamentType::none;
        }
        return type;
    }

    /// Préparation commune : PA à 0 (pas d'effet du PA sur les paliers), position, chauffe, purge ; false si échec
    bool prepare(session::Session &session, Lines &lines, float x, float &y, float z, float step_s, int16_t temperature) {
        pressure_advance::set_axis_e_config({ .pressure_advance = 0.f, .smooth_time = session.smooth_time() });
        const float length = step_speed * step_s;
        const float x_center = std::isfinite(x) ? x : X_BED_SIZE / 2.f;
        lines.x_lo = x_center - length / 2.f;
        lines.x_hi = x_center + length / 2.f;
        if (const session::Result r = session.go_to(lines.x_lo, lines.x_hi, lines.x_lo, y, row_pitch * (rows - 1), z); r != session::Result::ok) {
            fail(error_of(r));
            return false;
        }
        lines.y0 = y;
        st_phase = Phase::heating;
        if (const session::Result r = session.heat(temperature); r != session::Result::ok) {
            fail(error_of(r));
            return false;
        }
        st_phase = Phase::priming;
        if (!session.prime(prime_mm, prime_speed)) {
            fail(Error::stopped);
            return false;
        }
        return true;
    }

    FILE *open_csv(bool wanted, const char *name, uint8_t slot) {
        if (!wanted || !marlin_server::get_media_inserted()) {
            return nullptr;
        }
        char path[24];
        snprintf(path, sizeof(path), "/usb/OD3D_%s%u.CSV", name, unsigned(slot + 1));
        return fopen(path, "w");
    }

} // namespace

Status status() {
    return {
        .kind = st_kind.load(),
        .phase = st_phase.load(),
        .error = st_error.load(),
        .slot = st_slot.load(),
        .step = st_step.load(),
        .steps = st_steps.load(),
        .value = st_value.load(),
        .temperature = st_temperature.load(),
        .csv = st_csv.load(),
    };
}

bool is_running() {
    return running.load();
}

uint16_t run_count() {
    return runs.load();
}

void request_stop() {
    stop_requested = true;
}

const LastRun &last_run() {
    return last;
}

// ---- F5 ----

void measure_flow(const FlowRequest &rq) {
    begin(Kind::flow, rq.slot);
    ScopeGuard count_run = [] { ++runs; }; // après l'état : l'interface lit l'état une fois runs changé

    const FilamentType type = check_slot(rq.slot);
    if (!type) {
        return;
    }
    std::array<float, LastRun::max_points> flows;
    const float flow_max = std::min(rq.flow_max, flow_ceiling); // jamais au-delà du plafond sûr
    const size_t expected = rq.flow_step > 0 && flow_max >= rq.flow_min ? size_t((flow_max - rq.flow_min) / rq.flow_step + 1e-3f) + 1 : 0;
    const size_t n = pa::sweep_values(rq.flow_min, flow_max, rq.flow_step, flows);
    KneeThresholds th;
    th.excess = rq.excess;
    if (!(rq.flow_min >= 0.5f) || !(rq.flow_step >= 0.5f) || n != expected || n < size_t(th.ref_points) + 2 || !(rq.step_s >= 2.f && rq.step_s <= 6.f)
        || !(rq.excess >= 0.05f && rq.excess <= 2.f)) {
        fail(Error::bad_request);
        return;
    }
    const int16_t temperature = std::clamp<int16_t>(rq.temperature > 0 ? rq.temperature : type.parameters().nozzle_temperature, EXTRUDE_MINTEMP + min_temperature_margin, session::max_temperature);
    st_temperature = temperature;
    st_steps = uint8_t(n);

    running = true;
    ScopeGuard not_running = [] { running = false; };
    session::Session session(stop_requested); // réglages remis à la fin (PA, mouvements, consigne de la buse)
    Lines lines;
    float y = rq.y;
    if (!prepare(session, lines, rq.x, y, rq.z, rq.step_s, temperature)) {
        return;
    }

    FILE *csv = open_csv(rq.csv, "FL", rq.slot);
    ScopeGuard close_csv = [&] {
        if (csv) {
            fclose(csv);
        }
    };
    st_csv = csv != nullptr;
    if (csv) {
        fprintf(csv, "# OneDrop3D max flow (F5) slot %u, %s, %d C, %.1f to %.1f mm3/s by %.1f, %.1f s per step, excess %.2f\n", unsigned(rq.slot + 1),
            type.parameters().name.data(), int(temperature), double(rq.flow_min), double(flow_max), double(rq.flow_step), double(rq.step_s), double(rq.excess));
        fprintf(csv, "# D,flow,t_s,x_mm,force_g / R,flow,force_g,noise_g / C,fit,flow,a,n,index,skipping\n");
    }

    st_phase = Phase::measuring;
    last = {};
    last.kind = Kind::flow;
    last.slot = rq.slot;
    std::array<StepPoint, LastRun::max_points> pts;
    size_t m = 0;
    bool overload = false;
    for (size_t i = 0; i < n; ++i) {
        st_step = uint8_t(i + 1);
        st_value = int16_t(std::lround(flows[i] * 10.f));
        StepPoint p;
        const StepResult r = run_step(lines, flows[i], rq.step_s, flows[i], p, csv);
        if (r != StepResult::ok) {
            fail(r == StepResult::stopped ? Error::stopped : Error::weak_signal);
            return;
        }
        pts[m++] = p;
        last.point[m - 1] = p;
        last.points = uint8_t(m);
        record_step(csv, Kind::flow, p);
        if (std::fabs(p.force_g) > max_force_g) {
            overload = true;
            break;
        }
        if (find_knee(std::span(pts.data(), m), false, th).fit == KneeFit::found) {
            break; // coude confirmé : pas plus loin
        }
    }
    Knee knee = find_knee(std::span(pts.data(), m), true, th);
    if (overload && knee.fit == KneeFit::not_reached && m >= 2) {
        // Force hors norme avant le coude : le palier précédent est le dernier sûr
        knee.fit = KneeFit::found;
        knee.skipping = true;
        knee.flow = pts[m - 2].x;
        knee.index = uint8_t(m - 1);
    }
    last.knee = knee;
    if (csv) {
        fprintf(csv, "C,%u,%.2f,%.4f,%.4f,%u,%u\n", unsigned(knee.fit), double(knee.flow), double(knee.a), double(knee.n), unsigned(knee.index), unsigned(knee.skipping));
    }
    if (knee.fit == KneeFit::weak_signal) {
        fail(Error::weak_signal);
        return;
    }
    if (knee.fit == KneeFit::too_few_points) {
        fail(Error::too_few_points);
        return;
    }
    Record rec = record(rq.slot);
    rec.max_flow_e1 = flow_to_e1(knee.flow);
    rec.flow_temperature = temperature;
    if (knee.fit == KneeFit::not_reached) {
        rec.flags = uint8_t(rec.flags | Record::flow_lower_bound);
    } else {
        rec.flags = uint8_t(rec.flags & ~Record::flow_lower_bound);
    }
    set_record(rq.slot, rec); // et la limite du planificateur
    st_value = int16_t(rec.max_flow_e1);
    fail(Error::none); // done
}

// ---- F6 ----

void measure_temperature(const TempRequest &rq) {
    begin(Kind::temperature, rq.slot);
    ScopeGuard count_run = [] { ++runs; };

    const FilamentType type = check_slot(rq.slot);
    if (!type) {
        return;
    }
    // Jamais au-dessus de la température de la matière, jamais sous la température minimale d'extrusion (+10 °C)
    const int16_t material = type.parameters().nozzle_temperature;
    const int16_t start = std::min<int16_t>(rq.start > 0 ? std::min(rq.start, material) : material, session::max_temperature);
    const int16_t lowest = EXTRUDE_MINTEMP + min_temperature_margin;
    const int16_t floor = std::max<int16_t>(rq.floor > 0 ? rq.floor : int16_t(start - default_drop), lowest);
    std::array<int16_t, LastRun::max_points> temps;
    const size_t n = temperature_steps(start, floor, rq.step, temps);
    MinTempThresholds th;
    th.max_ratio = rq.max_ratio;
    if (n < 2 || rq.step < 2 || rq.step > 20 || !(rq.flow >= 1.f && rq.flow <= 15.f) || !(rq.max_ratio >= 1.05f && rq.max_ratio <= 5.f)
        || !(rq.step_s >= 2.f && rq.step_s <= 6.f) || !(rq.hold_s >= 0.f && rq.hold_s <= 30.f)) {
        fail(Error::bad_request);
        return;
    }
    st_temperature = start;
    st_steps = uint8_t(n);

    running = true;
    ScopeGuard not_running = [] { running = false; };
    session::Session session(stop_requested);
    Lines lines;
    float y = rq.y;
    if (!prepare(session, lines, rq.x, y, rq.z, rq.step_s, start)) {
        return;
    }

    FILE *csv = open_csv(rq.csv, "TM", rq.slot);
    ScopeGuard close_csv = [&] {
        if (csv) {
            fclose(csv);
        }
    };
    st_csv = csv != nullptr;
    if (csv) {
        fprintf(csv, "# OneDrop3D min temperature (F6) slot %u, %s, %d to %d C by %d, %.1f mm3/s, %.1f s per step, ratio %.2f\n", unsigned(rq.slot + 1),
            type.parameters().name.data(), int(start), int(floor), int(rq.step), double(rq.flow), double(rq.step_s), double(rq.max_ratio));
        fprintf(csv, "# D,temp,t_s,x_mm,force_g / R,temp,force_g,noise_g / C,fit,temp,reference_g,index\n");
    }

    last = {};
    last.kind = Kind::temperature;
    last.slot = rq.slot;
    last.max_ratio = rq.max_ratio;
    std::array<StepPoint, LastRun::max_points> pts;
    size_t m = 0;
    bool overload = false;
    const uint32_t hold_ms = uint32_t(rq.hold_s * 1000.f);
    for (size_t i = 0; i < n; ++i) {
        st_step = uint8_t(i + 1);
        st_value = temps[i];
        if (i > 0) {
            // Palier suivant : la buse descend et se stabilise à ±1 °C
            st_phase = Phase::settling;
            st_temperature = temps[i];
            if (const session::Result r = session.heat(temps[i], 1.f, hold_ms, settle_timeout_ms); r != session::Result::ok) {
                fail(error_of(r));
                return;
            }
        }
        st_phase = Phase::measuring;
        StepPoint p;
        const StepResult r = run_step(lines, rq.flow, rq.step_s, float(temps[i]), p, csv);
        if (r != StepResult::ok) {
            fail(r == StepResult::stopped ? Error::stopped : Error::weak_signal);
            return;
        }
        pts[m++] = p;
        last.point[m - 1] = p;
        last.points = uint8_t(m);
        record_step(csv, Kind::temperature, p);
        if (std::fabs(p.force_g) > max_force_g) {
            overload = true;
            break;
        }
        if (find_min_temperature(std::span(pts.data(), m), float(floor), th).fit == MinTempFit::found) {
            break; // force au-delà du seuil : pas plus bas
        }
    }
    MinTemp mt = find_min_temperature(std::span(pts.data(), m), float(floor), th);
    if (overload && mt.fit != MinTempFit::found && m >= 2) {
        mt.fit = MinTempFit::found;
        mt.temperature = pts[m - 2].x;
        mt.index = uint8_t(m - 1);
    }
    last.min_temp = mt;
    if (csv) {
        fprintf(csv, "C,%u,%.1f,%.3f,%u\n", unsigned(mt.fit), double(mt.temperature), double(mt.reference_g), unsigned(mt.index));
    }
    if (mt.fit == MinTempFit::weak_signal) {
        fail(Error::weak_signal);
        return;
    }
    if (mt.fit == MinTempFit::too_few_points) {
        fail(Error::too_few_points);
        return;
    }
    Record rec = record(rq.slot);
    rec.min_temperature = std::max<int16_t>(int16_t(std::lround(mt.temperature)), lowest);
    if (mt.fit == MinTempFit::floor_reached) {
        rec.flags = uint8_t(rec.flags | Record::temperature_floor);
    } else {
        rec.flags = uint8_t(rec.flags & ~Record::temperature_floor);
    }
    set_record(rq.slot, rec);
    st_value = rec.min_temperature;
    fail(Error::none); // done
}

} // namespace od3d::calib
