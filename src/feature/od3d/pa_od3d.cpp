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
#include <cstdlib>
#include <cstring>
#include <feature/bedlevel/bedlevel.h>
#include <feature/print_status_message/print_status_message_mgr.hpp>
#include <feature/prusa/MMU2/mmu2_mk4.h>
#include <filament.hpp>
#include <gcode_info.hpp>
#include <i18n.h>
#include <memory>
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

void on_print_start(bool from_file) {
    // Pas de changement du PA ici (boucle du serveur : on ne vide pas les files) ; le T du début du G-code ou le
    // premier M572 posent le K de l'emplacement
    slicer_requested = NAN;
    calib::refresh_flow_limit(); // F5
    // Mesure automatique avant le G-code de départ du fichier (file vide au départ : elle passe en premier, comme les
    // G-codes que Prusa y met pour un G-code mono-outil avec le MMU) ; elle décide elle-même s'il y a quelque chose à faire
    if (from_file && auto_mode() != AutoMode::off) {
        std::ignore = marlin_server::enqueue_gcode_try("M1994 A");
    }
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
            force_probe::csv_printf(csv, "R,%.4f,%.5f,%.5f,%.3f,%.3f,%.2f,%.3f,%u\n", double(k), double(r.lag_s), double(r.cost), double(r.overshoot),
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

namespace {

    /// Réglages des mouvements de la mesure (remis par la session) : pas de mode furtif, accélérations de PrusaPATuner
    void fast_moves(const Params &params) {
        planner.set_stealth_mode(false); // ses limites fausseraient les changements de vitesse
        auto s = planner.user_settings;
        for (const AxisEnum axis : { X_AXIS, Y_AXIS }) {
            s.max_acceleration_mm_per_s2[axis] = std::max<uint32_t>(s.max_acceleration_mm_per_s2[axis], uint32_t(accel_xy));
            s.max_feedrate_mm_s[axis] = std::max(s.max_feedrate_mm_s[axis], profile(params).v_hi * 1.2f);
        }
        s.acceleration = accel_xy;
        s.travel_acceleration = accel_xy;
        planner.apply_settings(s); // les limites matérielles de Prusa s'appliquent encore
    }

    /// Lignes d'un balayage de rq, de l'avant vers l'arrière depuis (x, y)
    pattern::Lines lines_for(const Request &rq, uint8_t count, float x, float y) {
        pattern::Lines lines;
        lines.x0 = x;
        lines.y0 = y;
        lines.pitch = rq.pitch;
        lines.length = line_length(rq.params);
        lines.marks = pattern::tally_width(count);
        lines.count = count;
        lines.bead = rq.params.bead;
        return lines;
    }

    /// Le MMU peut charger : présent, actif, pas en erreur
    bool mmu_ready() {
        using Code = decltype(MMU2::mmu2.MMUCurrentErrorCode());
        const Code code = MMU2::mmu2.MMUCurrentErrorCode();
        return MMU2::mmu2.Enabled() && (code == Code::OK || code == Code::RUNNING);
    }

    /// Chemin de Prusa pour « Charger dans la buse » (MI_MMU_LOAD_TO_NOZZLE → M701 W2 P → Pause), sans la question de
    /// la couleur : le filament chargé est d'abord déchargé (pointe formée), la buse doit être à la consigne de la
    /// nouvelle matière. Une erreur du MMU ouvre son écran habituel et attend l'utilisateur.
    bool load(uint8_t slot) {
        MMU2::mmu2.load_filament_to_nozzle(slot);
        return loaded_slot() == slot;
    }

    /// Balayage grossier puis fin autour du premier choix, une ligne par K dans l'ordre ; K enregistré s'il est sûr.
    /// Buse chaude et purgée, lignes placées.
    Error sweep(session::Session &session, const Request &rq, const pattern::Lines &lines, FILE *csv) {
        std::array<float, LastRun::max_points> ks;
        const size_t coarse = sweep_values(rq.k_min, rq.k_max, rq.k_step, std::span(ks.data(), ks.size() - 8));
        const float smooth_time = session.smooth_time();
        st_phase = Phase::measuring;
        st_steps = lines.count;
        last = {};
        last.slot = rq.slot;
        std::array<SweepPoint, LastRun::max_points> points;
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
            points[n] = { k, r };
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
                return Error::stopped;
            }
        }
        Choice choice = choose_k(std::span(points.data(), n), rq.k_min, rq.k_max, rq.k_step);
        if (choice.fit == Fit::ok && rq.refine > 0) {
            std::array<float, 8> fine;
            const size_t m = refine_values(choice.k, rq.k_step / 4.f, rq.refine, 0.f, 1.f, fine);
            st_steps = uint8_t(coarse + m);
            for (size_t i = 0; i < m && n < lines.count; ++i) {
                if (!run_line(fine[i], false, false)) {
                    return Error::stopped;
                }
            }
            choice = choose_k(std::span(points.data(), n), rq.k_min, rq.k_max, rq.k_step);
        }
        last.choice = choice;
        if (csv) {
            force_probe::csv_printf(csv, "C,%.4f,%u,%u,%.3f,%.4f,%.4f\n", double(choice.k), unsigned(choice.confidence), unsigned(choice.fit), double(choice.slope),
                double(choice.k_lag), double(choice.k_cost));
        }
        st_k = k_to_e4(choice.k);
        if (choice.fit != Fit::ok) {
            return error_of(choice.fit);
        }
        if (choice.confidence < min_confidence) {
            return Error::low_confidence;
        }
        set_record(rq.slot, { .k_e4 = k_to_e4(choice.k), .confidence = choice.confidence, .source = Record::measured });
        return Error::none;
    }

    /// Une mesure commence : état remis à zéro pour l'emplacement
    void begin(uint8_t slot) {
        st_slot = slot;
        st_step = 0;
        st_steps = 0;
        st_k = k_none;
        st_csv = false;
        st_error = Error::none;
        st_phase = Phase::preparing;
    }

    struct Temperatures {
        int16_t nozzle; ///< impression
        int16_t bed;
        int16_t probe; ///< buse pendant le palpage
    };

    /// Consignes de la mesure, celles de la matière par défaut ; bornées (les maximums de Prusa s'appliquent encore)
    Temperatures temperatures_for(const FilamentTypeParameters &material, int16_t nozzle, int16_t bed) {
        const int16_t t = std::clamp<int16_t>(nozzle > 0 ? nozzle : material.nozzle_temperature, EXTRUDE_MINTEMP + 10, session::max_temperature);
        return {
            .nozzle = t,
            .bed = std::clamp<int16_t>(bed > 0 ? bed : material.heatbed_temperature, 0, session::max_bed_temperature),
            .probe = std::min<int16_t>(material.nozzle_preheat_temperature, t),
        };
    }

} // namespace

void calibrate(const Request &rq) {
    stop_requested = false;
    ScopeGuard count_run = [] { ++runs; }; // après l'état : l'interface lit l'état une fois runs changé
    begin(rq.slot);

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
    const FilamentType type = FilamentType::for_tool(VirtualToolIndex::from_raw(rq.slot));
    if (!type) {
        fail(Error::not_loaded); // matière inconnue : rien à charger ni à chauffer
        return;
    }
    // En un clic : la bobine est chargée par le MMU si elle n'est pas dans la buse ; jamais avec un MMU en erreur
    const bool in_nozzle = loaded_slot() == rq.slot;
    if (MMU2::mmu2.Enabled() && !mmu_ready()) {
        fail(Error::mmu);
        return;
    }
    if (!in_nozzle && !MMU2::mmu2.Enabled()) {
        fail(Error::not_loaded);
        return;
    }
    const FilamentTypeParameters material = type.parameters();
    const Temperatures t = temperatures_for(material, rq.temperature, rq.bed_temperature);
    st_temperature = t.nozzle;

    // Motif : une ligne par K (grossier + fin), de l'avant vers l'arrière, repères à droite ; il doit tenir dans la
    // zone imprimable (bords du plateau et course)
    const pattern::Lines lines = lines_for(rq, uint8_t(coarse + rq.refine), rq.x, rq.y);
    if (!session::printable_zone().contains(lines.area())) {
        fail(Error::no_room);
        return;
    }

    running = true;
    ScopeGuard not_running = [] { running = false; };
    st_steps = lines.count;
    last = {};
    last.slot = rq.slot;

    // Réglages remis à la fin, même sur un arrêt de Marlin : PA, mouvements, consignes, maillage (session commune)
    session::Session session(stop_requested);
    fast_moves(rq.params);

    // Plateau, puis origine et nivellement local de la zone du motif (buse à sa température de palpage)
    st_phase = Phase::bed;
    if (const session::Result r = session.heat_bed(t.bed, t.probe); r != session::Result::ok) {
        fail(error_of(r));
        return;
    }
    st_phase = Phase::leveling;
    if (const session::Result r = session.level(lines.area(), t.probe); r != session::Result::ok) {
        fail(error_of(r));
        return;
    }

    // Buse à la température de la matière au-dessus de la purge, chargement si besoin (comme le T du G-code de départ
    // du MMU), puis ligne de purge au bord avant
    st_phase = Phase::heating;
    if (const session::Result r = session.go_to_purge(); r != session::Result::ok) {
        fail(error_of(r));
        return;
    }
    if (const session::Result r = session.heat(t.nozzle); r != session::Result::ok) {
        fail(error_of(r));
        return;
    }
    if (!in_nozzle) {
        st_phase = Phase::loading;
        if (!load(rq.slot)) {
            fail(must_stop() ? Error::stopped : Error::not_loaded);
            return;
        }
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
            force_probe::csv_printf(csv, "# OneDrop3D PA (F4) slot %u, %s, nozzle %d C, bed %d C, printed lines %.2f x %.2f mm from X%.1f Y%.1f pitch %.1f, slow %.2f mm3/s %.2f s, fast %.2f mm3/s %.2f s, cycles %u\n",
                unsigned(rq.slot + 1), material.name.data(), int(t.nozzle), int(t.bed), double(rq.params.bead.width), double(rq.params.bead.height),
                double(rq.x), double(rq.y), double(rq.pitch), double(rq.params.slow_flow), double(rq.params.slow_s), double(rq.params.fast_flow),
                double(rq.params.fast_s), unsigned(rq.params.cycles));
            force_probe::csv_printf(csv, "# force up positive. D,k,t_s,x_mm,force_g / R,k,lag_s,cost,overshoot,undershoot,amplitude_g,noise,cycles / C,k,confidence,fit,slope,k_lag,k_cost\n");
        }
    }
    ScopeGuard close_csv = [&] {
        if (csv) {
            fclose(csv);
        }
    };
    st_csv = csv != nullptr;

    fail(sweep(session, rq, lines, csv)); // none : terminé
}

// ---- Mesure automatique au départ d'une impression ----

namespace {

    // Motif court : par emplacement, une ligne de purge puis 5 lignes (K de 0 à 0,08) plus courtes (2 créneaux)
    constexpr float auto_pitch = 4.f;
    constexpr float auto_purge_half = 3.f; ///< demi-largeur de la purge (cordon épais de Prusa)
    constexpr float auto_purge_gap = 6.f; ///< de la purge à la 1re ligne
    constexpr float auto_spacing = 3.f; ///< entre deux emplacements

    Request auto_request(uint8_t slot) {
        Request rq;
        rq.slot = slot;
        rq.k_min = 0.f;
        rq.k_max = 0.08f;
        rq.k_step = 0.02f;
        rq.refine = 0;
        rq.params.cycles = 2;
        rq.pitch = auto_pitch;
        return rq;
    }
    constexpr uint8_t auto_lines = 5;

    METRIC_DEF(metric_od3d_pa_auto, "od3d_pa_auto", METRIC_VALUE_CUSTOM, 0, METRIC_DISABLED);

    /// Journal de la mesure automatique d'un emplacement (metric od3d_pa_auto)
    void log_slot(uint8_t slot, Error e) {
        const Record r = record(slot);
        const bool ok = e == Error::none;
        metric_record_custom(&metric_od3d_pa_auto, ",slot=%u error=%ui,k=%.4f,confidence=%ui", unsigned(slot + 1), unsigned(e), double(ok ? k_from_e4(r.k_e4) : 0.f),
            unsigned(ok ? r.confidence : 0));
    }

    /// Message court dans la ligne d'état de l'écran Impression (gardé dans l'historique des messages)
    void tell(const string_view_utf8 &message) {
        char text[72];
        message.copyToRAM(text, sizeof(text));
        if (char *copy = strdup(text)) {
            print_status_message().show_temporary<PrintStatusMessage::custom>({ std::shared_ptr<const char[]>(copy, [](char *str) { free(str); }) });
        }
    }

    void report_slot(uint8_t slot, Error e) {
        log_slot(slot, e);
        StringViewUtf8Parameters<16> params;
        if (e == Error::none) {
            char k[12];
            snprintf(k, sizeof(k), "%.3f", double(k_from_e4(record(slot).k_e4)));
            tell(_("Slot %d PA measured: K = %s").formatted(params, int(slot + 1), k));
        } else {
            tell(_("Slot %d PA not measured, slicer PA kept").formatted(params, int(slot + 1)));
        }
    }

    /// Points majeurs de la grille, que G29 P1 palpe dans la zone d'impression agrandie (ubl_G29.cpp, dans les deux sens)
    size_t major_points(std::span<float> out, uint8_t count, float (*pos)(uint8_t)) {
        size_t n = 0;
        for (int i = GRID_BORDER; i < count - GRID_BORDER && n < out.size(); i += GRID_MAJOR_STEP) {
            out[n++] = pos(uint8_t(i));
        }
        for (int i = count - 1 - GRID_BORDER; i >= GRID_BORDER && n < out.size(); i -= GRID_MAJOR_STEP) {
            out[n++] = pos(uint8_t(i));
        }
        return n;
    }

    // L'origine Z de G28 est dans la zone palpée près de la purge, que auto_spot() laisse libre
    static_assert(Z_SAFE_HOMING_X_POINT >= 150 && Z_SAFE_HOMING_Y_POINT <= 20);

} // namespace

AutoMode auto_mode() {
    const uint8_t v = config_store().od3d_pa_auto.get();
    return v < auto_mode_count ? AutoMode(v) : AutoMode::off;
}

void set_auto_mode(AutoMode mode) {
    config_store().od3d_pa_auto.set(uint8_t(mode));
    if (mode != AutoMode::off) {
        set_printer_wins(true);
    }
}

void calibrate_auto() {
    const AutoMode mode = auto_mode();
    if (mode == AutoMode::off || running || !marlin_server::is_printing()) {
        return;
    }

    // Emplacements de l'impression (après la correspondance des outils), matière connue, K déjà là
    uint8_t used = 0, known = 0, has_k = 0;
    GCodeInfo::getInstance().for_each_used_extruder([&](GcodeToolIndex, VirtualToolIndex tool, const GCodeInfo::ExtruderInfo &) {
        if (tool.to_raw() < VirtualToolIndex::count) {
            used |= uint8_t(1u << tool.to_raw());
        }
    });
    for (uint8_t slot = 0; slot < VirtualToolIndex::count; ++slot) {
        const VirtualToolIndex tool = VirtualToolIndex::from_raw(slot);
        if (tool.is_enabled() && FilamentType::for_tool(tool)) {
            known |= uint8_t(1u << slot);
        }
        if (record(slot).has_k()) {
            has_k |= uint8_t(1u << slot);
        }
    }
    const uint8_t todo = slots_to_measure(mode, used, known, has_k);
    if (!todo) {
        return;
    }
    const auto skip_all = [&](Error e, const string_view_utf8 &message) {
        for (uint8_t slot = 0; slot < VirtualToolIndex::count; ++slot) {
            if (todo & (1u << slot)) {
                log_slot(slot, e);
            }
        }
        tell(message);
    };
    if (!mmu_ready()) {
        skip_all(Error::mmu, _("PA not measured: MMU not ready"));
        return;
    }
    const auto area = GCodeInfo::getInstance().get_bed_preheat_area(); // M555 du fichier, lu avant l'impression
    if (!area) {
        skip_all(Error::no_area, _("PA not measured: no print area in the G-code"));
        return;
    }

    // Ordre : la bobine chargée en dernier (elle reste dans la buse, rien à remettre)
    const std::optional<uint8_t> initial = loaded_slot();
    std::array<uint8_t, VirtualToolIndex::count> order;
    size_t count = 0;
    for (uint8_t slot = 0; slot < VirtualToolIndex::count; ++slot) {
        if ((todo & (1u << slot)) && initial != slot) {
            order[count++] = slot;
        }
    }
    if (initial && (todo & (1u << *initial))) {
        order[count++] = *initial;
    }

    // Place : un bloc par emplacement (purge, puis lignes), autant d'emplacements qu'il en tient à côté de la pièce
    const Request first = auto_request(order[0]);
    const pattern::Rect one = lines_for(first, auto_lines, 0.f, auto_purge_half + auto_purge_gap).area(); // bloc en (0, 0)
    const float block_w = one.width();
    const float block_h = one.y1;
    std::array<float, 2 * GRID_MAX_POINTS_X> xs;
    std::array<float, 2 * GRID_MAX_POINTS_Y> ys;
    const size_t nx = major_points(xs, GRID_MAX_POINTS_X, unified_bed_leveling::mesh_index_to_xpos);
    const size_t ny = major_points(ys, GRID_MAX_POINTS_Y, unified_bed_leveling::mesh_index_to_ypos);
    const pattern::Rect print { area->a.x, area->a.y, area->b.x, area->b.y };
    std::optional<pattern::Rect> spot;
    size_t fitting = count;
    for (; fitting > 0 && !spot; --fitting) {
        spot = auto_spot(session::printable_zone(), print, block_w, float(fitting) * (block_h + auto_spacing) - auto_spacing, std::span(xs.data(), nx),
            std::span(ys.data(), ny), MESH_X_DIST * GRID_MAJOR_STEP, MESH_Y_DIST * GRID_MAJOR_STEP);
    }
    if (!spot) {
        skip_all(Error::no_room, _("PA not measured: no room beside the print"));
        return;
    }
    ++fitting; // la boucle a compté un de trop
    for (size_t i = fitting; i < count; ++i) {
        report_slot(order[i], Error::no_room); // pas la place pour tous : à la prochaine impression
    }
    const pattern::Rect block = *spot;
    const auto block_of = [&](size_t i) {
        const float y = block.y0 + float(i) * (block_h + auto_spacing);
        return lines_for(auto_request(order[i]), auto_lines, block.x0 - one.x0, y + auto_purge_half + auto_purge_gap);
    };

    stop_requested = false;
    running = true;
    ScopeGuard not_running = [] { running = false; ++runs; };
    begin(order[0]);

    // Matières des emplacements mesurés : plateau de l'impression (M140 du fichier), buse de palpage la plus basse
    int16_t probe = session::max_temperature;
    for (size_t i = 0; i < fitting; ++i) {
        probe = std::min(probe, temperatures_for(FilamentType::for_tool(VirtualToolIndex::from_raw(order[i])).parameters(), 0, 0).probe);
    }
    const FilamentTypeParameters first_material = FilamentType::for_tool(VirtualToolIndex::from_raw(order[0])).parameters();
    const int16_t bed = temperatures_for(first_material, 0, int16_t(GCodeInfo::getInstance().get_bed_preheat_temp().value_or(0))).bed;

    {
        // Réglages remis à la fin ; « Passer » sur l'écran Impression arrête la mesure, l'impression continue
        session::Session session(stop_requested, true);
        fast_moves(first.params);
        st_phase = Phase::bed;
        session::Result r = session.heat_bed(bed, probe);
        if (r == session::Result::ok) {
            st_phase = Phase::leveling;
            r = session.level(block, probe);
        }
        if (r != session::Result::ok) {
            fail(error_of(r));
            for (size_t i = 0; i < fitting; ++i) {
                report_slot(order[i], error_of(r));
            }
        }
        size_t i = 0;
        for (; r == session::Result::ok && i < fitting; ++i) {
            const uint8_t slot = order[i];
            const Request rq = auto_request(slot);
            const pattern::Lines lines = block_of(i);
            const Temperatures t = temperatures_for(FilamentType::for_tool(VirtualToolIndex::from_raw(slot)).parameters(), 0, 0);
            begin(slot);
            st_temperature = t.nozzle;
            Error e = Error::none;

            // Comme le G-code de départ du MMU : au-dessus du début de la purge, buse chaude, chargement, purge
            st_phase = Phase::heating;
            if (session.go_to_purge(lines.x0, lines.y0 - auto_purge_gap, 1.f) != session::Result::ok || session.heat(t.nozzle) != session::Result::ok) {
                e = must_stop() ? Error::stopped : Error::heating;
            }
            if (e == Error::none && loaded_slot() != slot) {
                st_phase = Phase::loading;
                if (!load(slot)) {
                    e = must_stop() ? Error::stopped : Error::not_loaded;
                }
            }
            if (e == Error::none) {
                st_phase = Phase::priming;
                if (!session.purge(lines.x0, 1.f)) {
                    e = Error::stopped;
                }
            }
            if (e == Error::none) {
                e = sweep(session, rq, lines, nullptr);
            }
            fail(e);
            report_slot(slot, e);
            if (must_stop()) {
                ++i;
                break; // « Passer », pause ou arrêt : les emplacements restants gardent le PA du trancheur
            }
        }
        for (; r == session::Result::ok && i < fitting; ++i) {
            log_slot(order[i], Error::stopped);
        }

        // Filament remis comme au départ (déchargé en général) : le G-code de départ charge lui-même sa bobine
        if (!planner.draining() && loaded_slot() != initial) {
            st_phase = Phase::loading;
            if (initial) {
                std::ignore = load(*initial);
            } else {
                MMU2::mmu2.unload();
            }
        }
    }
}

} // namespace od3d::pa
