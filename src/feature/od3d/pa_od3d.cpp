/// @file
/// Méthode : celle de PrusaPATuner (CNC Kitchen, AGPL-3.0), étudiée puis réécrite sans reprendre son code ; mesure
/// dans le vide (voir pa_math_od3d.hpp).
#include "pa_od3d.hpp"
#include "force_probe_od3d.hpp"
#include "measure_session_od3d.hpp"
#include "spool_calib_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <tuple>
#include <span>

#include <common/gcode/gcode_info.hpp>
#include <common/marlin_server.hpp>
#include <config_store/store_instance.hpp>
#include <feature/prusa/MMU2/mmu2_mk4.h>
#include <marlin_stubs/skippable_gcode.hpp>
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

bool auto_measure() {
    return config_store().od3d_pa_auto.get();
}

void set_auto_measure(bool set) {
    config_store().od3d_pa_auto.set(set);
}

bool needs_measure(uint8_t slot) {
    return (printer_wins() || auto_measure()) && !record(slot).has_k();
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
    std::atomic<uint16_t> auto_id = 0; ///< mesure automatique en cours (0 : aucune)
    uint16_t auto_count = 0;
    std::atomic<bool> skip_print = false; ///< « Passer » : plus de mesure automatique jusqu'à la prochaine impression

    std::atomic<Phase> st_phase = Phase::idle;
    std::atomic<Error> st_error = Error::none;
    std::atomic<uint8_t> st_slot = 0;
    std::atomic<uint8_t> st_step = 0;
    std::atomic<uint8_t> st_steps = 0;
    std::atomic<uint16_t> st_k = k_none;
    std::atomic<int16_t> st_temperature = 0;
    std::atomic<bool> st_csv = false;
    std::atomic<bool> st_automatic = false;
    std::atomic<bool> st_pattern = false;

    LastRun last;

    using session::must_stop;
    using session::wait_moves;
    using session::wait_ms;

    constexpr uint8_t min_confidence = 30; ///< en dessous, rien n'est enregistré
    constexpr uint8_t warmup_legs = 5; ///< purge avant la mesure : 5 paliers lents (10 s, comme PrusaPATuner)
    constexpr float pattern_accel = 5000.f; ///< motif imprimé : changements de vitesse francs
    constexpr float tick_speed = 10.f; ///< traits des repères [mm/s]
    constexpr float tick_hop = 0.4f; ///< levée entre deux traits [mm]

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
        case Fit::saturated:
            return Error::saturated;
        }
        return Error::too_few_points;
    }

    /// Arrêt d'une mesure : au début d'une impression qui continue, c'est « Passer » (de notre écran ou de l'écran
    /// d'impression de Prusa) : plus de mesure automatique pour cette impression ; sinon « annulé »
    Error stop_error(bool automatic) {
        if (automatic && !marlin_server::aborting_or_aborted() && !planner.draining()) {
            skip_print = true;
            return Error::skipped;
        }
        return Error::stopped;
    }

    /// Texte sans format (csv_printf est borné à 255 caractères)
    void put(FILE *csv, const char *text) {
        fwrite(text, 1, strlen(text), csv);
    }

    /// L'emplacement est dans la buse ; sinon le MMU l'y charge (comme « Charger dans la buse »), buse chaude
    bool ensure_loaded(uint8_t slot) {
        if (const auto tool = VirtualToolIndex::currently_selected_opt(); tool && tool->to_raw() == slot) {
            return true;
        }
        if (!MMU2::mmu2.Enabled()) {
            return false;
        }
        st_phase = Phase::loading;
        MMU2::mmu2.load_filament_to_nozzle(slot);
        planner.synchronize();
        const auto tool = VirtualToolIndex::currently_selected_opt();
        return tool && tool->to_raw() == slot && !must_stop();
    }

    /// Écrit une ligne D par point de la trace (commande reconstituée d'après les demi-tours de X), puis les lignes B et R
    void write_block(FILE *csv, float k, const Params &params, const BlockResult &r) {
        const force::Trace t = force_probe::trace();
        const float dt = force_probe::dt();
        force_probe::csv_printf(csv, "B,%.4f,%.2f,%.6f,%u,%u\n", double(k), double(force_probe::base_force_g()), double(dt), unsigned(t.size()),
            unsigned(force_probe::overflowed()));
        std::array<Turn, 24> turns;
        const size_t n = find_turns(t, 0.25f * params.wiggle, turns);
        char line[96];
        for (size_t i = 0; i < t.size(); ++i) {
            const CsvPoint p { k, float(i) * dt, force::x_mm(t[i]), force::e_mm(t[i]), force::force_g(t[i]), command_flow(std::span(turns.data(), n), i, params) };
            if (format_point(line, sizeof(line), p) <= 0 || fwrite(line, 1, strlen(line), csv) != strlen(line)) { // snprintf : décimales comprises
                break;
            }
        }
        force_probe::csv_printf(csv, "R,%.4f,%u,%u,%u,%.1f,%.2f,%.3f,%.1f,%.1f,%.3f,%.3f,%.1f,%.4f,%.4f,%.4f,%.3f,%.4f\n", double(k), unsigned(r.saturated),
            unsigned(r.found), unsigned(r.cycles), double(r.amplitude_g), double(r.noise_g), double(r.m[rise_error]), double(r.m[overshoot]),
            double(r.m[undershoot]), double(r.m[tail]), double(r.m[plateau_slope]), double(r.m[rise_delay]), double(r.m[fall_delay]), double(r.m[settling]),
            double(r.fall_area), double(r.k_estimate));
    }

    /// Une séquence dans le vide : PA = k, lent de lancement puis cycles × (rapide, lent), X en aller-retour (sens dir) ;
    /// trace analysée et écrite. false s'il faut s'arrêter.
    bool measure_block(session::Session &session, float k, float smooth_time, const Params &params, float dir, Waveform *wave, BlockResult &result, FILE *csv) {
        const Profile prof = profile(params);
        planner.synchronize();
        if (must_stop()) {
            return false;
        }
        pressure_advance::set_axis_e_config({ .pressure_advance = k, .smooth_time = smooth_time });
        force_probe::start();
        ScopeGuard stop_probe = [] { force_probe::stop(); };
        const float dx = dir * params.wiggle;
        if (!session.extrude_by(dx, prof.e_slow * params.slow_s, params.slow_s)) {
            return false;
        }
        for (uint8_t c = 0; c < params.cycles; ++c) {
            if (!session.extrude_by(-dx, prof.e_fast * params.fast_s, params.fast_s) || !session.extrude_by(dx, prof.e_slow * params.slow_s, params.slow_s)) {
                return false;
            }
        }
        if (!wait_moves() || !wait_ms(50)) {
            return false;
        }
        force_probe::stop();
        result = analyse_block(force_probe::trace(), force_probe::dt(), params, k, wave);
        metric_record_custom(&metric_od3d_pa, " k=%.4f,amp=%.1f,noise=%.1f,cyc=%ui,sat=%ui,est=%.4f", double(k), double(result.amplitude_g), double(result.noise_g),
            unsigned(result.cycles), unsigned(result.saturated), double(result.k_estimate));
        if (csv) {
            write_block(csv, k, params, result);
        }
        return true;
    }

    /// CSV sur la clé : un fichier par emplacement, réécrit à chaque mesure
    FILE *open_csv(uint8_t slot) {
        if (!marlin_server::get_media_inserted()) {
            return nullptr;
        }
        char path[24];
        snprintf(path, sizeof(path), "/usb/OD3D_PA%u.CSV", unsigned(slot + 1));
        return fopen(path, "w");
    }

    /// Mesure dans le vide, à la manière de PrusaPATuner
    void measure_in_air(const Request &rq, const FilamentTypeParameters &material, int16_t temperature) {
        std::array<float, LastRun::max_points> ks;
        const size_t coarse = sweep_values(rq.k_min, rq.k_max, rq.k_step, std::span(ks.data(), ks.size() - 8));
        st_steps = uint8_t(coarse + rq.refine);

        // Réglages remis à la fin, même sur un arrêt de Marlin : PA, mouvements, consignes (session commune)
        session::Session session(stop_requested, rq.automatic);
        const float smooth_time = session.smooth_time();

        // Origine si nécessaire, position de purge de Prusa, buse haute : le filament tombe dans le vide
        st_phase = Phase::parking;
        float x0, y0;
        if (const session::Result r = session.go_to_park(rq.park_z, x0, y0); r != session::Result::ok) {
            fail(r == session::Result::stopped ? stop_error(rq.automatic) : error_of(r));
            return;
        }
        st_phase = Phase::heating;
        if (const session::Result r = session.heat(temperature); r != session::Result::ok) {
            fail(r == session::Result::stopped ? stop_error(rq.automatic) : error_of(r));
            return;
        }
        // L'emplacement dans la buse, chargé par le MMU s'il le faut
        if (!ensure_loaded(rq.slot)) {
            fail(must_stop() ? stop_error(rq.automatic) : Error::not_loaded);
            return;
        }
        // Créneaux nets : mode furtif coupé, accélération des mouvements et de E comme PrusaPATuner (M204, M201 E)
        planner.set_stealth_mode(false);
        {
            auto s = planner.user_settings;
            for (const AxisEnum axis : { X_AXIS, Y_AXIS }) {
                s.max_acceleration_mm_per_s2[axis] = std::max<uint32_t>(s.max_acceleration_mm_per_s2[axis], uint32_t(rq.params.accel));
            }
            s.max_acceleration_mm_per_s2[E_AXIS] = uint32_t(rq.params.accel);
            s.acceleration = rq.params.accel;
            planner.apply_settings(s); // les limites matérielles de Prusa s'appliquent encore
        }
        // Cellule en haute précision (320 Hz, sans le canal du capteur de filament) le temps de la mesure
        Loadcell::HighPrecisionEnabler high_precision(loadcell);

        // Purge lente : la pression s'établit, l'ancien filament part (le premier palier lent de PrusaPATuner, × 5)
        st_phase = Phase::priming;
        {
            const Profile prof = profile(rq.params);
            float dir = 1.f;
            for (uint8_t i = 0; i < warmup_legs; ++i, dir = -dir) {
                if (!session.extrude_by(dir * rq.params.wiggle, prof.e_slow * rq.params.slow_s, rq.params.slow_s)) {
                    fail(stop_error(rq.automatic));
                    return;
                }
            }
            if (warmup_legs % 2) {
                // Retour au départ : la première séquence part vers +X
                if (!session.extrude_by(-rq.params.wiggle, prof.e_slow * 0.2f * rq.params.slow_s, 0.2f * rq.params.slow_s)) {
                    fail(stop_error(rq.automatic));
                    return;
                }
            }
        }

        FILE *csv = rq.csv ? open_csv(rq.slot) : nullptr;
        ScopeGuard close_csv = [&] {
            if (csv) {
                fclose(csv);
            }
        };
        st_csv = csv != nullptr;
        if (csv) {
            force_probe::csv_printf(csv, "# OneDrop3D PA (F4) v3, in the air (PrusaPATuner method), slot %u, %s, nozzle %d C, park X%.1f Y%.1f Z%.1f%s\n",
                unsigned(rq.slot + 1), material.name.data(), int(temperature), double(x0), double(y0), double(current_position.z),
                rq.automatic ? ", at print start" : "");
            put(csv, "# force_g = -loadcell_value (up positive) relative to the block start (B: absolute start), x_mm and e_mm (motor, PA included) relative, cmd = commanded flow mm3/s\n");
            put(csv, "# P,slow_flow,slow_s,fast_flow,fast_s,wiggle,accel,cycles / B,k,base_force_g,dt_s,points,overflow / D,k,t_s,x_mm,e_mm,force_g,cmd / R,k,saturated,found,cycles,amplitude_g,noise_g,rise_error,overshoot,undershoot,tail,plateau_slope,rise_delay,fall_delay,settling,fall_area,k_estimate / C,k,confidence,fit,k_cost,k_segments,k_signed,depth,used\n");
            char line[96];
            if (format_params(line, sizeof(line), rq.params) > 0) {
                put(csv, line);
            }
        }

        // Balayage grossier, puis fin autour du premier choix ; une séquence par K, sens de X alterné
        st_phase = Phase::measuring;
        std::array<SweepPoint, LastRun::max_points> sweep;
        size_t n = 0;
        float best_cost = INFINITY;
        float dir = 1.f;
        Waveform wave;
        const auto run_block = [&](float k, bool first, bool keep_high) -> bool {
            st_step = uint8_t(n + 1);
            st_k = k_to_e4(k);
            BlockResult r;
            if (!measure_block(session, k, smooth_time, rq.params, dir, &wave, r, csv)) {
                return false;
            }
            dir = -dir;
            sweep[n] = { k, r };
            last.k[n] = k;
            last.amplitude[n] = r.amplitude_g;
            ++n;
            last.points = uint8_t(n);
            if (r.valid()) {
                if (first) {
                    last.low = wave;
                    last.k_low = k;
                }
                if (keep_high) {
                    last.high = wave;
                    last.k_high = k;
                }
            }
            // Coût composé (relatif au balayage) recalculé à chaque séquence, pour l'écran
            std::array<float, LastRun::max_points> costs;
            choose_k(std::span(sweep.data(), n), rq.k_step, {}, std::span(costs.data(), n));
            for (size_t i = 0; i < n; ++i) {
                last.cost[i] = costs[i];
            }
            if (r.valid() && std::isfinite(costs[n - 1]) && costs[n - 1] < best_cost) {
                best_cost = costs[n - 1];
                last.best = wave;
                last.k_best = k;
            }
            return true;
        };
        for (size_t i = 0; i < coarse; ++i) {
            if (!run_block(ks[i], i == 0, i + 1 == coarse)) {
                fail(stop_error(rq.automatic));
                return;
            }
        }
        std::array<float, LastRun::max_points> costs;
        Choice choice = choose_k(std::span(sweep.data(), n), rq.k_step, {}, std::span(costs.data(), n));
        if (choice.fit == Fit::ok && rq.refine > 0) {
            std::array<float, 8> fine;
            const size_t m = refine_values(choice.k, rq.k_step / 2.f, rq.refine, 0.f, 1.f, fine);
            st_steps = uint8_t(coarse + m);
            for (size_t i = 0; i < m && n < sweep.size(); ++i) {
                if (!run_block(fine[i], false, false)) {
                    fail(stop_error(rq.automatic));
                    return;
                }
            }
            choice = choose_k(std::span(sweep.data(), n), rq.k_step, {}, std::span(costs.data(), n));
        }
        for (size_t i = 0; i < n; ++i) {
            last.cost[i] = costs[i];
        }
        last.choice = choice;
        if (csv) {
            force_probe::csv_printf(csv, "C,%.4f,%u,%u,%.4f,%.4f,%.4f,%.3f,%u\n", double(choice.k), unsigned(choice.confidence), unsigned(choice.fit),
                double(choice.k_cost), double(choice.k_segments), double(choice.k_signed), double(choice.depth), unsigned(choice.used));
        }
        st_k = k_to_e4(choice.k);
        if (choice.fit != Fit::ok) {
            fail(error_of(choice.fit));
            return;
        }
        if (choice.confidence < min_confidence) {
            fail(Error::low_confidence);
            return;
        }
        set_record(rq.slot, { .k_e4 = k_to_e4(choice.k), .confidence = choice.confidence, .source = Record::measured });
        fail(Error::none); // done
    }

    /// Une ligne du motif à l'œil : PA = k, lancement lent puis cycles × (rapide, lent) le long de +X, puis les traits du
    /// numéro de la ligne à sa droite. false s'il faut s'arrêter.
    bool print_line(session::Session &session, float k, float smooth_time, const pattern::PaLine &line, const pattern::Lines &lines, uint8_t index) {
        const float v_lo = pattern::speed_for_flow(line.bead, line.slow_flow), v_hi = pattern::speed_for_flow(line.bead, line.fast_flow);
        const float ls = v_lo * line.slow_s, lf = v_hi * line.fast_s;
        const float e_mm = pattern::e_per_mm(line.bead);
        const float z = line.bead.height;
        const float y = lines.y(index);

        // Buse au départ de la ligne, posée à hauteur de couche (déplacement levé, rétraction ; maillage appliqué)
        if (!session.travel_to(lines.x0, y, z)) {
            return false;
        }
        planner.synchronize();
        if (must_stop()) {
            return false;
        }
        pressure_advance::set_axis_e_config({ .pressure_advance = k, .smooth_time = smooth_time });
        float x = lines.x0;
        const auto segment = [&](float length, float speed) {
            x += length;
            return session.print_to(x, y, e_mm, speed);
        };
        if (!segment(ls, v_lo)) {
            return false;
        }
        for (uint8_t c = 0; c < line.cycles; ++c) {
            if (!segment(lf, v_hi) || !segment(ls, v_lo)) {
                return false;
            }
        }
        if (!wait_moves()) {
            return false;
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

    /// Repli visuel : une ligne par K à l'avant du plateau, préparée comme une vraie impression
    void print_pattern(const Request &rq, const FilamentTypeParameters &material, int16_t temperature) {
        std::array<float, max_lines> ks;
        const size_t count = sweep_values(rq.k_min, rq.k_max, rq.k_step, ks);
        const int16_t bed = std::clamp<int16_t>(rq.bed_temperature > 0 ? rq.bed_temperature : material.heatbed_temperature, 0, session::max_bed_temperature);
        const int16_t probe = std::min<int16_t>(material.nozzle_preheat_temperature, temperature);

        pattern::Lines lines;
        lines.x0 = rq.x;
        lines.y0 = rq.y;
        lines.pitch = rq.pitch;
        lines.length = pattern::line_length(rq.line);
        lines.marks = pattern::tally_width(uint8_t(count));
        lines.count = uint8_t(count);
        lines.bead = rq.line.bead;
        if (!session::printable_zone().contains(lines.area())) {
            fail(Error::no_room);
            return;
        }
        st_steps = uint8_t(count);
        last.pattern = true;

        session::Session session(stop_requested);
        const float smooth_time = session.smooth_time();
        planner.set_stealth_mode(false);
        {
            auto s = planner.user_settings;
            for (const AxisEnum axis : { X_AXIS, Y_AXIS }) {
                s.max_acceleration_mm_per_s2[axis] = std::max<uint32_t>(s.max_acceleration_mm_per_s2[axis], uint32_t(pattern_accel));
                s.max_feedrate_mm_s[axis] = std::max(s.max_feedrate_mm_s[axis], pattern::speed_for_flow(rq.line.bead, rq.line.fast_flow) * 1.2f);
            }
            s.acceleration = pattern_accel;
            s.travel_acceleration = pattern_accel;
            planner.apply_settings(s);
        }

        // Comme le G-code de départ de la CORE One : plateau, palpage de la zone, buse chaude, purge au bord avant ; puis
        // chargement de l'emplacement s'il le faut (au-dessus de la purge)
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
        st_phase = Phase::heating;
        if (const session::Result r = session.go_to_purge(); r != session::Result::ok) {
            fail(error_of(r));
            return;
        }
        if (const session::Result r = session.heat(temperature); r != session::Result::ok) {
            fail(error_of(r));
            return;
        }
        if (!ensure_loaded(rq.slot)) {
            fail(must_stop() ? Error::stopped : Error::not_loaded);
            return;
        }
        if (const session::Result r = session.go_to_purge(); r != session::Result::ok) {
            fail(error_of(r));
            return;
        }
        st_phase = Phase::priming;
        if (!session.purge()) {
            fail(Error::stopped);
            return;
        }

        st_phase = Phase::measuring;
        for (size_t i = 0; i < count; ++i) {
            st_step = uint8_t(i + 1);
            st_k = k_to_e4(ks[i]);
            if (!print_line(session, ks[i], smooth_time, rq.line, lines, uint8_t(i))) {
                fail(Error::stopped);
                return;
            }
            last.k[i] = ks[i];
            last.points = uint8_t(i + 1);
        }
        st_k = k_none;
        fail(Error::none); // lignes imprimées : à l'utilisateur de choisir
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
        .automatic = st_automatic.load(),
        .pattern = st_pattern.load(),
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

void request_skip() {
    if (auto_id.load() != 0) {
        skip_print = true;
        skippable_gcode().request_skip();
    }
}

uint16_t auto_run() {
    return auto_id.load();
}

const LastRun &last_run() {
    return last;
}

bool choose_line(uint8_t slot, uint8_t line) {
    if (running || last.slot != slot || !last.pattern || line >= last.points || slot >= VirtualToolIndex::count) {
        return false;
    }
    const uint16_t e4 = k_to_e4(last.k[line]);
    if (e4 == k_none) {
        return false;
    }
    set_record(slot, { .k_e4 = e4, .confidence = 100, .source = Record::chosen });
    return true;
}

void on_print_init() {
    skip_print = false;
    if (!auto_measure() || !MMU2::mmu2.Enabled()) {
        return;
    }
    // Emplacements utilisés par le fichier (association des outils comprise), sans K, avec une matière connue
    std::array<bool, VirtualToolIndex::count> wanted {};
    GCodeInfo::getInstance().for_each_used_extruder([&](GcodeToolIndex, VirtualToolIndex tool, const GCodeInfo::ExtruderInfo &) {
        const uint8_t slot = tool.to_raw();
        if (slot < wanted.size() && !record(slot).has_k() && FilamentType::for_tool(tool)) {
            wanted[slot] = true;
        }
    });
    for (uint8_t slot = 0; slot < wanted.size(); ++slot) {
        if (wanted[slot]) {
            marlin_server::enqueue_gcode_printf("M1994 A P%u O1", unsigned(slot));
        }
    }
}

void calibrate(const Request &rq) {
    stop_requested = false;
    ScopeGuard count_run = [] { ++runs; }; // après l'état : l'interface lit l'état une fois runs changé
    st_slot = rq.slot;
    st_step = 0;
    st_steps = 0;
    st_k = k_none;
    st_csv = false;
    st_automatic = rq.automatic;
    st_pattern = rq.visual;
    st_error = Error::none;
    st_phase = Phase::preparing;

    if (rq.automatic) {
        // Au début d'une impression seulement, et pas après « Passer »
        if (!marlin_server::is_printing() || marlin_server::aborting_or_aborted() || rq.visual) {
            fail(Error::bad_request);
            return;
        }
        if (skip_print) {
            fail(Error::skipped);
            return;
        }
    } else if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
        fail(Error::printing);
        return;
    }
    std::array<float, LastRun::max_points> ks;
    const size_t coarse = sweep_values(rq.k_min, rq.k_max, rq.k_step, std::span(ks.data(), ks.size() - 8));
    const bool params_ok = rq.visual ? pattern::is_valid_line(rq.line) && coarse <= max_lines && rq.pitch >= 3.f && rq.pitch <= 20.f
            && std::isfinite(rq.x) && std::isfinite(rq.y)
                                      : is_valid(rq.params) && rq.refine <= 8 && rq.park_z >= 10.f && rq.park_z <= 200.f;
    if (rq.slot >= VirtualToolIndex::count || !params_ok || coarse < 4 || rq.k_min < 0.f || rq.k_max > 1.f) {
        fail(Error::bad_request);
        return;
    }
    const FilamentType type = FilamentType::for_tool(VirtualToolIndex::from_raw(rq.slot));
    const auto loaded = VirtualToolIndex::currently_selected_opt();
    // Sans MMU, la bobine doit déjà être dans la buse ; avec, elle y sera chargée
    if (!type || (!MMU2::mmu2.Enabled() && (!loaded || loaded->to_raw() != rq.slot))) {
        fail(Error::not_loaded);
        return;
    }
    const FilamentTypeParameters material = type.parameters();
    // La consigne est encore bornée par Prusa (température maximale de la buse) : on attend la consigne réelle
    const int16_t temperature = std::clamp<int16_t>(rq.temperature > 0 ? rq.temperature : material.nozzle_temperature, EXTRUDE_MINTEMP + 10, session::max_temperature);
    st_temperature = temperature;

    running = true;
    if (rq.automatic) {
        auto_id = uint16_t(++auto_count == 0 ? ++auto_count : auto_count);
    }
    ScopeGuard not_running = [] {
        running = false;
        auto_id = 0;
    };
    last = {};
    last.slot = rq.slot;
    if (rq.visual) {
        print_pattern(rq, material, temperature);
    } else {
        measure_in_air(rq, material, temperature);
    }
}

} // namespace od3d::pa
