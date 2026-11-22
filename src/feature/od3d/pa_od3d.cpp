/// @file
/// Méthode : inspirée de PrusaPATuner (CNC Kitchen, AGPL-3.0), réimplémentée d'après sa description, sur un motif
/// imprimé (voir pa_math_od3d.hpp).
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

    constexpr float accel_xy = 5000.f; ///< comme PrusaPATuner ; la limite d'accélération du filament de Prusa reste
    constexpr uint8_t min_confidence = 30; ///< en dessous, rien n'est enregistré (le choix à l'œil reste possible)
    constexpr float tick_speed = 10.f; ///< traits des repères [mm/s]
    constexpr float tick_hop = 0.4f; ///< levée entre deux traits [mm]

    void fail(Error e) {
        st_error = e;
        st_phase = e == Error::none ? Phase::done : Phase::failed;
    }

    /// Une ligne du motif : PA = k, lancement lent puis cycles × (rapide, lent) le long de +X, trace analysée, puis les
    /// traits du numéro de la ligne à sa droite. false s'il faut s'arrêter.
    bool measure_line(session::Session &session, float k, float smooth_time, const Params &params, const pattern::Lines &lines, uint8_t index, Waveform *wave,
        BlockResult &result, FILE *csv) {
        const Profile prof = profile(params);
        const float ls = prof.v_lo * params.slow_s, lf = prof.v_hi * params.fast_s;
        const float e_mm = pattern::e_per_mm(params.bead);
        const float z = params.bead.height;
        const float y = lines.y(index);

        // Buse au départ de la ligne, posée à hauteur de couche (déplacement levé, rétraction)
        if (!session.travel_to(lines.x0, y, z)) {
            return false;
        }
        planner.synchronize();
        if (must_stop()) {
            return false;
        }
        pressure_advance::set_axis_e_config({ .pressure_advance = k, .smooth_time = smooth_time });

        // Cellule en haute précision (320 Hz, sans le canal du capteur de filament) le temps de la ligne seulement
        {
            Loadcell::HighPrecisionEnabler high_precision(loadcell);
            if (!wait_ms(250)) {
                return false;
            }
            force_probe::start();
            ScopeGuard stop_probe = [] { force_probe::stop(); };
            if (!wait_ms(150)) { // tête arrêtée : zéro de la force
                return false;
            }
            float x = lines.x0;
            const auto segment = [&](float length, float speed) {
                x += length;
                return session.print_to(x, y, e_mm, speed);
            };
            if (!segment(ls, prof.v_lo)) {
                return false;
            }
            for (uint8_t c = 0; c < params.cycles; ++c) {
                if (!segment(lf, prof.v_hi) || !segment(ls, prof.v_lo)) {
                    return false;
                }
            }
            if (!wait_moves() || !wait_ms(150)) {
                return false;
            }
            force_probe::stop();
            result = analyse_block(force_probe::trace(), force_probe::dt(), prof, wave);
            if (csv) {
                std::ignore = force_probe::write_csv(csv, k);
            }
        }

        // Repères : numéro de la ligne en traits vers l'arrière, un sur cinq plus long
        std::array<pattern::Tick, max_lines> ticks;
        const size_t n = pattern::tally(uint8_t(index + 1), ticks);
        const float x_ticks = lines.x0 + lines.length + pattern::tick_gap;
        for (size_t i = 0; i < n; ++i) {
            const float tx = x_ticks + ticks[i].dx;
            if (i == 0) {
                if (!session.travel_to(tx, y, z)) {
                    return false;
                }
            } else {
                // Petit saut sans rétraction d'un trait au suivant
                if (!session.move_z(z + tick_hop)) {
                    return false;
                }
                do_blocking_move_to_xy(tx, y, session::travel_speed_xy);
                if (!session.move_z(z)) {
                    return false;
                }
            }
            if (!session.print_to(tx, y + ticks[i].length, e_mm, tick_speed) || !wait_moves()) {
                return false;
            }
        }
        session.retract();
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
        case session::Result::leveling:
            return Error::leveling;
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

bool choose_line(uint8_t slot, uint8_t line) {
    if (running || last.slot != slot || line >= last.points || slot >= VirtualToolIndex::count) {
        return false;
    }
    const uint16_t e4 = k_to_e4(last.k[line]);
    if (e4 == k_none) {
        return false;
    }
    set_record(slot, { .k_e4 = e4, .confidence = 100, .source = Record::chosen });
    return true;
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
    if (rq.slot >= VirtualToolIndex::count || !is_valid(rq.params) || coarse < 3 || coarse + rq.refine > max_lines || rq.k_min < 0.f || rq.k_max > 1.f
        || rq.refine > 8 || !(rq.pitch >= 3.f && rq.pitch <= 20.f) || !std::isfinite(rq.x) || !std::isfinite(rq.y)) {
        fail(Error::bad_request);
        return;
    }
    const auto loaded = VirtualToolIndex::currently_selected_opt();
    const FilamentType type = FilamentType::for_tool(VirtualToolIndex::from_raw(rq.slot));
    if (!loaded || loaded->to_raw() != rq.slot || !type) {
        fail(Error::not_loaded);
        return;
    }
    const FilamentTypeParameters material = type.parameters();
    // Les consignes sont encore bornées par Prusa (températures maximales) : on attend les consignes réelles
    const int16_t temperature = std::clamp<int16_t>(rq.temperature > 0 ? rq.temperature : material.nozzle_temperature, EXTRUDE_MINTEMP + 10, session::max_temperature);
    const int16_t bed = std::clamp<int16_t>(rq.bed_temperature > 0 ? rq.bed_temperature : material.heatbed_temperature, 0, session::max_bed_temperature);
    const int16_t probe = std::min<int16_t>(material.nozzle_preheat_temperature, temperature);
    st_temperature = temperature;

    // Motif : une ligne par K (grossier + fin), de l'avant vers l'arrière, repères à droite ; il doit tenir dans la
    // zone imprimable (bords du plateau et course)
    const uint8_t line_count = uint8_t(coarse + rq.refine);
    pattern::Lines lines;
    lines.x0 = rq.x;
    lines.y0 = rq.y;
    lines.pitch = rq.pitch;
    lines.length = line_length(rq.params);
    lines.marks = pattern::tally_width(line_count);
    lines.count = line_count;
    lines.bead = rq.params.bead;
    if (!session::printable_zone().contains(lines.area())) {
        fail(Error::no_room);
        return;
    }

    running = true;
    ScopeGuard not_running = [] { running = false; };
    st_steps = line_count;
    last = {};
    last.slot = rq.slot;

    // Réglages remis à la fin, même sur un arrêt de Marlin : PA, mouvements, consignes, maillage (session commune)
    session::Session session(stop_requested);
    const float smooth_time = session.smooth_time();
    planner.set_stealth_mode(false); // ses limites fausseraient les changements de vitesse
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

    // Plateau, puis origine et nivellement local de la zone du motif (buse à sa température de palpage)
    st_phase = Phase::bed;
    if (const session::Result r = session.heat_bed(bed, probe); r != session::Result::ok) {
        fail(error_of(r));
        return;
    }
    st_phase = Phase::leveling;
    if (const session::Result r = session.level(lines.area(), probe); r != session::Result::ok) {
        fail(error_of(r));
        return;
    }

    // Buse à la température de la matière au-dessus de la purge, puis ligne de purge au bord avant
    st_phase = Phase::heating;
    if (const session::Result r = session.go_to_purge(); r != session::Result::ok) {
        fail(error_of(r));
        return;
    }
    if (const session::Result r = session.heat(temperature); r != session::Result::ok) {
        fail(error_of(r));
        return;
    }
    st_phase = Phase::priming;
    if (!session.purge()) {
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
            fprintf(csv, "# OneDrop3D PA (F4) slot %u, %s, nozzle %d C, bed %d C, printed lines %.2f x %.2f mm from X%.1f Y%.1f pitch %.1f, slow %.2f mm3/s %.2f s, fast %.2f mm3/s %.2f s, cycles %u\n",
                unsigned(rq.slot + 1), material.name.data(), int(temperature), int(bed), double(rq.params.bead.width), double(rq.params.bead.height),
                double(rq.x), double(rq.y), double(rq.pitch), double(rq.params.slow_flow), double(rq.params.slow_s), double(rq.params.fast_flow),
                double(rq.params.fast_s), unsigned(rq.params.cycles));
            fprintf(csv, "# force up positive. D,k,t_s,x_mm,force_g / R,k,lag_s,cost,overshoot,undershoot,amplitude_g,noise,cycles / C,k,confidence,fit,slope,k_lag,k_cost\n");
        }
    }
    ScopeGuard close_csv = [&] {
        if (csv) {
            fclose(csv);
        }
    };
    st_csv = csv != nullptr;

    // Balayage grossier, puis fin autour du premier choix ; une ligne par K, dans l'ordre
    st_phase = Phase::measuring;
    std::array<SweepPoint, LastRun::max_points> sweep;
    size_t n = 0;
    float best_cost = INFINITY;
    Waveform wave;
    const auto run_line = [&](float k, bool first, bool keep_high) -> bool {
        st_step = uint8_t(n + 1);
        st_k = k_to_e4(k);
        BlockResult r;
        if (!measure_line(session, k, smooth_time, rq.params, lines, uint8_t(n), &wave, r, csv)) {
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
        if (!run_line(ks[i], i == 0, i + 1 == coarse)) {
            fail(Error::stopped);
            return;
        }
    }
    Choice choice = choose_k(std::span(sweep.data(), n), rq.k_min, rq.k_max, rq.k_step);
    if (choice.fit == Fit::ok && rq.refine > 0) {
        std::array<float, 8> fine;
        const size_t m = refine_values(choice.k, rq.k_step / 4.f, rq.refine, 0.f, 1.f, fine);
        st_steps = uint8_t(coarse + m);
        for (size_t i = 0; i < m && n < line_count; ++i) {
            if (!run_line(fine[i], false, false)) {
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
    set_record(rq.slot, { .k_e4 = k_to_e4(choice.k), .confidence = choice.confidence, .source = Record::measured });
    st_k = k_to_e4(choice.k);
    fail(Error::none); // done
}

} // namespace od3d::pa
