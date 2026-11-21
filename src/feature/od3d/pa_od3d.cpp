/// @file
/// Méthode : PrusaPATuner (CNC Kitchen, AGPL-3.0), réimplémentée d'après sa description (voir pa_math_od3d.hpp).
#include "pa_od3d.hpp"
#include "force_probe_od3d.hpp"
#include "measure_session_od3d.hpp"
#include "spool_calib_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <tuple>
#include <span>

#include <common/marlin_server.hpp>
#include <config_store/store_instance.hpp>
#include <filament.hpp>
#include <loadcell.hpp>
#include <Marlin.h>
#include <feature/pressure_advance/pressure_advance_config.hpp>
#include <metric.h>
#include <module/motion.h>
#include <module/planner.h>
#include <module/temperature.h>
#include <raii/scope_guard.hpp>
#include <tool_index.hpp>

METRIC_DEF(metric_od3d_pa, "od3d_pa", METRIC_VALUE_CUSTOM, 0, METRIC_DISABLED);

namespace od3d::pa {

// ---- Mémoire ----

Record record(uint8_t slot) {
    if (slot >= VirtualToolIndex::count) {
        return {};
    }
    return config_store().od3d_pa.get(slot);
}

std::optional<float> k_for_slot(uint8_t slot) {
    const Record r = record(slot);
    if (!r.has_k()) {
        return std::nullopt;
    }
    return k_from_e4(r.k_e4);
}

void set_record(uint8_t slot, const Record &r) {
    if (slot < VirtualToolIndex::count && config_store().od3d_pa.get(slot) != r) {
        config_store().od3d_pa.set(slot, r);
    }
}

void clear(uint8_t slot) {
    set_record(slot, Record {});
}

bool printer_wins() {
    return config_store().od3d_pa_printer_wins.get();
}

void set_printer_wins(bool set) {
    config_store().od3d_pa_printer_wins.set(set);
}

bool needs_measure(uint8_t slot) {
    return printer_wins() && !record(slot).has_k();
}

// ---- Application ----

namespace {

    /// Dernière valeur demandée par le trancheur pendant l'impression (M572 / M900), appliquée ou non
    float slicer_requested = NAN;

    std::optional<uint8_t> loaded_slot() {
        if (const auto tool = VirtualToolIndex::currently_selected_opt()) {
            return tool->to_raw();
        }
        return std::nullopt;
    }

    /// Comme M572_internal, sans passer par lui (il ignorerait la valeur) : files vidées avant le changement
    void apply(float k) {
        if (!std::isfinite(k) || k < 0.f || k > 1.f) {
            return;
        }
        const pressure_advance::Config &current = pressure_advance::get_axis_e_config();
        const pressure_advance::Config wanted { .pressure_advance = k, .smooth_time = current.smooth_time };
        if (current == wanted) {
            return;
        }
        planner.synchronize();
        if (!planner.draining()) {
            pressure_advance::set_axis_e_config(wanted);
        }
    }

} // namespace

void on_tool_change() {
    calib::refresh_flow_limit(); // F5 : limite de débit de l'emplacement chargé
    if (!printer_wins()) {
        return;
    }
    const auto slot = loaded_slot();
    if (!slot) {
        return;
    }
    if (const auto k = k_for_slot(*slot)) {
        apply(*k);
    } else if (marlin_server::is_printing() && std::isfinite(slicer_requested)) {
        apply(slicer_requested); // emplacement sans K : la valeur du trancheur, pas celle de l'emplacement précédent
    }
}

void on_print_start() {
    // Pas de changement du PA ici (boucle du serveur : on ne vide pas les files) ; le T du début du G-code ou le
    // premier M572 posent le K de l'emplacement
    slicer_requested = NAN;
    calib::refresh_flow_limit(); // F5
}

std::optional<float> override_slicer_pa(float requested) {
    if (!marlin_server::is_printing()) {
        return std::nullopt; // hors impression (console, M572 à la main) : toujours appliqué
    }
    slicer_requested = requested;
    if (!printer_wins()) {
        return std::nullopt;
    }
    const auto slot = loaded_slot();
    return slot ? k_for_slot(*slot) : std::nullopt;
}

// ---- Mesure ----

namespace {

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<uint16_t> runs = 0;

    std::atomic<Phase> st_phase = Phase::idle;
    std::atomic<Error> st_error = Error::none;
    std::atomic<uint8_t> st_slot = 0;
    std::atomic<uint8_t> st_step = 0;
    std::atomic<uint8_t> st_steps = 0;
    std::atomic<uint16_t> st_k = k_none;
    std::atomic<int16_t> st_temperature = 0;
    std::atomic<bool> st_csv = false;

    LastRun last;

    using session::must_stop;
    using session::wait_moves;
    using session::wait_ms;
    using session::wait_queue;

    constexpr float accel_xy = 5000.f; ///< comme PrusaPATuner ; la limite d'accélération du filament de Prusa reste
    constexpr float prime_mm = 8.f; ///< purge avant la mesure [mm de filament]
    constexpr float prime_speed = 2.f; ///< [mm/s de filament]
    constexpr uint8_t min_confidence = 30; ///< en dessous, rien n'est enregistré
    constexpr float travel_speed_xy = session::travel_speed_xy;

    /// Un mouvement en X avec extrusion proportionnelle (le PA de Prusa n'agit qu'avec du XY) ; false s'il faut s'arrêter
    bool extrude_x(float dx, float speed, float e_per_mm) {
        if (!wait_queue()) {
            return false;
        }
        current_position.x += dx;
        current_position.e += std::fabs(dx) * e_per_mm;
        line_to_current_position(speed);
        return true;
    }

    void fail(Error e) {
        st_error = e;
        st_phase = e == Error::none ? Phase::done : Phase::failed;
    }

    /// Un bloc de mesure : PA = k, créneaux, trace analysée. Position de départ : centre p0 (après l'arrêt)
    bool measure_block(float k, float smooth_time, const Params &params, float p0, Waveform *wave, BlockResult &result, FILE *csv) {
        const Profile prof = profile(params);
        const float ls = prof.v_lo * params.slow_s, lf = prof.v_hi * params.fast_s;

        planner.synchronize();
        if (must_stop()) {
            return false;
        }
        pressure_advance::set_axis_e_config({ .pressure_advance = k, .smooth_time = smooth_time });
        do_blocking_move_to_x(p0, travel_speed_xy); // chaque bloc part du même point

        // Cellule en haute précision (320 Hz, sans le canal du capteur de filament) le temps du bloc seulement
        Loadcell::HighPrecisionEnabler high_precision(loadcell);
        if (!wait_ms(250)) {
            return false;
        }
        force_probe::start();
        ScopeGuard stop_probe = [] { force_probe::stop(); };
        if (!wait_ms(150)) { // tête arrêtée : zéro de la force
            return false;
        }

        // Lancement lent (aller-retour), puis créneaux : rapide, demi-lent, demi-tour, demi-lent
        if (!extrude_x(-ls / 2, prof.v_lo, params.e_per_mm) || !extrude_x(ls / 2, prof.v_lo, params.e_per_mm)) {
            return false;
        }
        float d = 1.f;
        for (uint8_t c = 0; c < params.cycles; ++c) {
            if (!extrude_x(d * lf, prof.v_hi, params.e_per_mm) || !extrude_x(d * ls / 2, prof.v_lo, params.e_per_mm)
                || !extrude_x(-d * ls / 2, prof.v_lo, params.e_per_mm)) {
                return false;
            }
            d = -d;
        }
        if (!wait_moves() || !wait_ms(150)) {
            return false;
        }
        force_probe::stop();

        const float dt = force_probe::dt();
        result = analyse_block(force_probe::trace(), dt, prof, wave);
        if (csv) {
            std::ignore = force_probe::write_csv(csv, k);
        }
        return true;
    }

    void record_result(FILE *csv, float k, const BlockResult &r) {
        metric_record_custom(&metric_od3d_pa, " k=%.4f,lag=%.5f,cost=%.5f,over=%.3f,under=%.3f,amp=%.2f,noise=%.3f,cyc=%ui",
            double(k), double(r.lag_s), double(r.cost), double(r.overshoot), double(r.undershoot), double(r.amplitude_g), double(r.noise), unsigned(r.cycles));
        if (csv) {
            fprintf(csv, "R,%.4f,%.5f,%.5f,%.3f,%.3f,%.2f,%.3f,%u\n", double(k), double(r.lag_s), double(r.cost), double(r.overshoot),
                double(r.undershoot), double(r.amplitude_g), double(r.noise), unsigned(r.cycles));
        }
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

    Error error_of(Fit fit) {
        switch (fit) {
        case Fit::ok:
            return Error::none;
        case Fit::too_few_points:
            return Error::too_few_points;
        case Fit::weak_signal:
            return Error::weak_signal;
        case Fit::no_crossing:
            return Error::no_crossing;
        case Fit::out_of_range:
            return Error::out_of_range;
        }
        return Error::too_few_points;
    }

} // namespace

Status status() {
    return {
        .phase = st_phase.load(),
        .error = st_error.load(),
        .slot = st_slot.load(),
        .step = st_step.load(),
        .steps = st_steps.load(),
        .k_e4 = st_k.load(),
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

void calibrate(const Request &rq) {
    stop_requested = false;
    ScopeGuard count_run = [] { ++runs; }; // après l'état : l'interface lit l'état une fois runs changé
    st_slot = rq.slot;
    st_step = 0;
    st_steps = 0;
    st_k = k_none;
    st_csv = false;
    st_error = Error::none;
    st_phase = Phase::preparing;

    if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
        fail(Error::printing);
        return;
    }
    std::array<float, LastRun::max_points> ks;
    const size_t coarse = sweep_values(rq.k_min, rq.k_max, rq.k_step, std::span(ks.data(), ks.size() - 8));
    if (rq.slot >= VirtualToolIndex::count || !is_valid(rq.params) || coarse < 3 || rq.k_min < 0.f || rq.k_max > 1.f || rq.refine > 8) {
        fail(Error::bad_request);
        return;
    }
    const auto loaded = VirtualToolIndex::currently_selected_opt();
    const FilamentType type = FilamentType::for_tool(VirtualToolIndex::from_raw(rq.slot));
    if (!loaded || loaded->to_raw() != rq.slot || !type) {
        fail(Error::not_loaded);
        return;
    }
    // La consigne est encore bornée par Prusa (température maximale de la buse) : on attend la consigne réelle
    const int16_t temperature = std::clamp<int16_t>(rq.temperature > 0 ? rq.temperature : type.parameters().nozzle_temperature, EXTRUDE_MINTEMP + 10, session::max_temperature);
    st_temperature = temperature;

    running = true;
    ScopeGuard not_running = [] { running = false; };
    st_steps = uint8_t(coarse + rq.refine);

    // Réglages remis à la fin, même sur un arrêt de Marlin : PA, mouvements, consigne de la buse (session commune)
    session::Session session(stop_requested);
    const float smooth_time = session.smooth_time();
    planner.set_stealth_mode(false); // ses limites fausseraient les créneaux
    {
        auto s = planner.user_settings;
        for (const AxisEnum axis : { X_AXIS, Y_AXIS }) {
            s.max_acceleration_mm_per_s2[axis] = std::max<uint32_t>(s.max_acceleration_mm_per_s2[axis], uint32_t(accel_xy));
            s.max_feedrate_mm_s[axis] = std::max(s.max_feedrate_mm_s[axis], profile(rq.params).v_hi * 1.2f);
        }
        s.acceleration = accel_xy;
        s.travel_acceleration = accel_xy;
        planner.apply_settings(s); // les limites matérielles de Prusa s'appliquent encore
    }

    // Origine si nécessaire, hauteur sûre, avant du plateau ; la tête va de p0 − ls/2 à p0 + lf + ls/2 : centrée sur x
    const Profile prof = profile(rq.params);
    const float ls = prof.v_lo * rq.params.slow_s, lf = prof.v_hi * rq.params.fast_s;
    const float x_center = std::isfinite(rq.x) ? rq.x : X_BED_SIZE / 2.f;
    const float p0 = x_center - lf / 2.f;
    float y = rq.y;
    if (const session::Result r = session.go_to(p0 - ls / 2, p0 + lf + ls / 2, p0, y, 0.f, rq.z); r != session::Result::ok) {
        fail(error_of(r));
        return;
    }

    // Buse à la température de la matière
    st_phase = Phase::heating;
    if (const session::Result r = session.heat(temperature); r != session::Result::ok) {
        fail(error_of(r));
        return;
    }

    // Purge sur place (hors de la zone des pièces), le filament tombe sur l'avant du plateau
    st_phase = Phase::priming;
    if (!session.prime(prime_mm, prime_speed)) {
        fail(Error::stopped);
        return;
    }

    // CSV facultatif sur la clé : un fichier par emplacement, réécrit à chaque mesure
    FILE *csv = nullptr;
    if (rq.csv && marlin_server::get_media_inserted()) {
        char path[24];
        snprintf(path, sizeof(path), "/usb/OD3D_PA%u.CSV", unsigned(rq.slot + 1));
        csv = fopen(path, "w");
        if (csv) {
            const auto name = type.parameters().name;
            fprintf(csv, "# OneDrop3D PA (F4) slot %u, %s, %d C, slow %.2f mm3/s %.2f s, fast %.2f mm3/s %.2f s, e/mm %.3f, cycles %u\n",
                unsigned(rq.slot + 1), name.data(), int(temperature), double(rq.params.slow_flow), double(rq.params.slow_s),
                double(rq.params.fast_flow), double(rq.params.fast_s), double(rq.params.e_per_mm), unsigned(rq.params.cycles));
            fprintf(csv, "# D,k,t_s,x_mm,force_g / R,k,lag_s,cost,overshoot,undershoot,amplitude_g,noise,cycles / C,k,confidence,fit,slope,k_lag,k_cost\n");
        }
    }
    ScopeGuard close_csv = [&] {
        if (csv) {
            fclose(csv);
        }
    };
    st_csv = csv != nullptr;

    // Balayage grossier, puis fin autour du premier choix
    st_phase = Phase::measuring;
    last = {};
    last.slot = rq.slot;
    std::array<SweepPoint, LastRun::max_points> sweep;
    size_t n = 0;
    float best_cost = INFINITY;
    Waveform wave;
    const auto run_block = [&](float k, bool first, bool keep_high) -> bool {
        st_step = uint8_t(n + 1);
        st_k = k_to_e4(k);
        BlockResult r;
        if (!measure_block(k, smooth_time, rq.params, p0, &wave, r, csv)) {
            return false;
        }
        sweep[n] = { k, r };
        last.k[n] = k;
        last.lag[n] = r.lag_s;
        last.cost[n] = r.cost;
        ++n;
        last.points = uint8_t(n);
        record_result(csv, k, r);
        if (r.valid()) {
            if (first) {
                last.low = wave;
                last.k_low = k;
            }
            if (keep_high) {
                last.high = wave;
                last.k_high = k;
            }
            if (r.cost < best_cost) {
                best_cost = r.cost;
                last.best = wave;
                last.k_best = k;
            }
        }
        return true;
    };
    for (size_t i = 0; i < coarse; ++i) {
        if (!run_block(ks[i], i == 0, i + 1 == coarse)) {
            fail(Error::stopped);
            return;
        }
    }
    Choice choice = choose_k(std::span(sweep.data(), n), rq.k_min, rq.k_max, rq.k_step);
    if (choice.fit == Fit::ok && rq.refine > 0) {
        std::array<float, 8> fine;
        const size_t m = refine_values(choice.k, rq.k_step / 4.f, rq.refine, 0.f, 1.f, fine);
        st_steps = uint8_t(coarse + m);
        for (size_t i = 0; i < m && n < sweep.size(); ++i) {
            if (!run_block(fine[i], false, false)) {
                fail(Error::stopped);
                return;
            }
        }
        choice = choose_k(std::span(sweep.data(), n), rq.k_min, rq.k_max, rq.k_step);
    }
    last.choice = choice;
    if (csv) {
        fprintf(csv, "C,%.4f,%u,%u,%.3f,%.4f,%.4f\n", double(choice.k), unsigned(choice.confidence), unsigned(choice.fit), double(choice.slope),
            double(choice.k_lag), double(choice.k_cost));
    }

    if (choice.fit != Fit::ok) {
        st_k = k_to_e4(choice.k);
        fail(error_of(choice.fit));
        return;
    }
    if (choice.confidence < min_confidence) {
        st_k = k_to_e4(choice.k);
        fail(Error::low_confidence);
        return;
    }
    set_record(rq.slot, { .k_e4 = k_to_e4(choice.k), .confidence = choice.confidence });
    st_k = k_to_e4(choice.k);
    fail(Error::none); // done
}

} // namespace od3d::pa
