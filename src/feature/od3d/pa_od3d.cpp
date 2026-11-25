/// @file
/// Méthode : inspirée de PrusaPATuner (CNC Kitchen, AGPL-3.0), réimplémentée d'après sa description, sur un motif
/// imprimé (voir pa_math_od3d.hpp).
#include "pa_od3d.hpp"
#include "e_jump_od3d.hpp"
#include "live_od3d.hpp"
#include "force_probe_od3d.hpp"
#include "measure_session_od3d.hpp"
#include "spool_calib_od3d.hpp"
#include "filament_memory_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdio>
#include <tuple>
#include <utility>
#include <span>

#include <FreeRTOS.h>
#include <task.h>

#include <common/marlin_server.hpp>
#include <config_store/store_instance.hpp>
#include <cstdlib>
#include <cstring>
#include <feature/auto_retract/auto_retract.hpp>
#include <feature/bedlevel/bedlevel.h>
#include <feature/print_status_message/print_status_message_mgr.hpp>
#include <feature/prusa/e-stall_detector.h>
#include <feature/prusa/MMU2/mmu2_mk4.h>
#include <filament.hpp>
#include <gcode_info.hpp>
#include <i18n.h>
#include <memory>
#include <loadcell.hpp>
#include <mapi/feedrates/standard_feedrates.hpp>
#include <mapi/motion.hpp>
#include <Marlin.h>
#include <feature/pressure_advance/pressure_advance_config.hpp>
#include <metric.h>
#include <module/motion.h>
#include <module/planner.h>
#include <module/temperature.h>
#include <raii/scope_guard.hpp>
#include <tool/hotend/hotend.hpp>
#include <tool_index.hpp>
#include <utils/string_builder.hpp>

METRIC_DEF(metric_od3d_pa, "od3d_pa", METRIC_VALUE_CUSTOM, 0, METRIC_DISABLED);

namespace od3d::pa {

// ---- Mémoire ----

uint8_t key_for_slot(uint8_t slot) {
    const FilamentType type = FilamentType::for_tool_heuristic(VirtualToolIndex::from_raw(slot));
    if (!type) {
        return 0; // matière inconnue : pas de clé (une mesure d'avant la clé attend d'être adoptée)
    }
    const PhysicalToolIndex tool = PhysicalToolIndex::from_raw(0);
    const auto nozzle_um = uint16_t(std::lround(std::clamp(config_store().get_nozzle_diameter(tool), 0.f, 5.f) * 1000.f));
    return make_key(nozzle_um, config_store().get_nozzle_is_high_flow(tool), type.parameters().name);
}

Record record(uint8_t slot) {
    if (slot >= VirtualToolIndex::count) {
        return {};
    }
    Record r = config_store().od3d_pa.get(slot);
    const uint8_t key = key_for_slot(slot);
    if (adopt(r, key, r.has_k())) {
        config_store().od3d_pa.set(slot, r); // une seule fois : mesure d'avant la clé, gardée pour cette configuration
    }
    return if_key(r, key); // M-02 : autre buse ou autre matière : rien
}

bool measured(uint8_t slot) {
    return record(slot).has_k();
}

std::optional<float> k_for_slot(uint8_t slot) {
    const Record r = record(slot);
    if (!r.applies()) {
        return std::nullopt; // pas de K, ou trop peu sûr (M-10) : le PA du trancheur sert
    }
    return k_from_e4(r.k_e4);
}

void set_record(uint8_t slot, const Record &r) {
    if (slot >= VirtualToolIndex::count) {
        return;
    }
    Record stamped = r;
    stamped.key = key_for_slot(slot); // M-02 : valable pour la buse et la matière de maintenant
    if (config_store().od3d_pa.get(slot) != stamped) {
        config_store().od3d_pa.set(slot, stamped);
    }
}

void clear(uint8_t slot) {
    set_record(slot, Record {});
}

bool stale(uint8_t slot) {
    return slot < VirtualToolIndex::count && config_store().od3d_pa.get(slot).has_k() && !k_for_slot(slot);
}

bool printer_wins() {
    return config_store().od3d_pa_printer_wins.get();
}

void set_printer_wins(bool set) {
    config_store().od3d_pa_printer_wins.set(set);
}

bool flexible(uint8_t slot) {
    return slot < VirtualToolIndex::count && FilamentType::for_tool_heuristic(VirtualToolIndex::from_raw(slot)).parameters().is_flexible;
}

bool needs_measure(uint8_t slot) {
    return printer_wins() && !k_for_slot(slot) && !flexible(slot); // un flexible n'est jamais mesuré (M-09)
}

// ---- Application ----

namespace {

    /// Dernière valeur demandée par le trancheur pendant l'impression (M572 / M900), appliquée ou non
    float slicer_requested = NAN;

    /// F43 : bobine dans la buse pour Prusa (retenue après un redémarrage), capteurs d'accord : ni chargement ni purge
    std::optional<uint8_t> loaded_slot() {
        return filament_memory::loaded_slot();
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
    e_jump::refresh(); // M2000 : F de l'emplacement (même K : set_axis_e_config ne serait pas appelé)
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

namespace {
    StartHold start_hold; ///< mesure automatique faite : la buse garde sa consigne jusqu'à la chauffe d'impression

    // Avec le G1 E-2 du G-code de départ, la rétraction de parking de Prusa
    static_assert(start_hold_retract_mm + 2.f == float(STANDARD_RETRACT_LENGTH));

    /// Le filament peut reculer : dans la buse, buse assez chaude, pas un flexible (comme l'auto-rétraction de Prusa), pas
    /// déjà reculé par elle (elle le remet au premier mouvement d'extrusion ; un recul de plus lui en ferait perdre la trace)
    [[gnu::noinline]] bool can_retract() {
        const auto slot = loaded_slot();
        return slot && !thermalManager.tooColdToExtrude(PhysicalToolIndex::from_raw(0)) && !flexible(*slot) && !buddy::auto_retract().will_deretract();
    }

    /// Filament reculé (mm < 0) ou remis aux vitesses de parking de Prusa, sa position logique inchangée (comme
    /// AutoRetract::maybe_deretract_to_nozzle) : le G-code de départ la retrouve telle quelle, en E relatif comme absolu
    [[gnu::noinline]] void hidden_e_move(float mm) {
        if (mm == 0.f || planner.draining()) {
            return;
        }
        planner.synchronize();
        const float e = current_position.e;
        const float e_planner = planner.get_axis_position_mm(E_AXIS_N(active_extruder));
        {
            BlockEStallDetection no_stall;
            using buddy::standard_feedrates::Extruder;
            mapi::extruder_move(mm, buddy::standard_feedrates::current_extruder(mm < 0.f ? Extruder::retract : Extruder::deretract));
            planner.synchronize();
        }
        current_position.e = e;
        planner.set_e_position_mm(e_planner);
    }
} // namespace

int16_t start_nozzle_target(int16_t requested) {
    if (!start_hold.active) {
        return requested;
    }
    if (!marlin_server::is_printing()) {
        start_hold = {}; // impression finie ou arrêtée : plus rien à garder ni à remettre
        return requested;
    }
    const int16_t current = int16_t(Hotend::for_tool(PhysicalToolIndex::from_raw(0)).nozzle_target_temp());
    const HoldStep step = start_hold_step(start_hold, requested, current, can_retract());
    hidden_e_move(step.e_mm);
    return step.target;
}

void on_print_start(bool from_file) {
    start_hold = {};
    // Pas de changement du PA ici (boucle du serveur : on ne vide pas les files) ; le T du début du G-code ou le
    // premier M572 posent le K de l'emplacement
    slicer_requested = NAN;
    calib::refresh_flow_limit(); // F5
    e_jump::refresh(); // M2000 : mesure faite ou effacée depuis (sans vider les files)
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
    constexpr LastRun no_run {}; ///< copiée dans last : pas de temporaire de ~1 Ko sur la pile de Marlin

    std::atomic<bool> followup = false; ///< en un clic : M1994 J1 en file derrière le chargement de Prusa
    std::optional<std::pair<int16_t, int16_t>> before_load; ///< consignes buse et plateau d'avant ce chargement

    /// Suite de G-codes mise en file (une seule à la fois : lue dès que M1994 rend la main)
    char sequence[256];

    /// Met sequence en file avant tout le reste, par la file d'injection de Prusa (ses suites internes, M600 de
    /// l'écran...) : elle passe dès que la commande en cours rend la main, avant les lignes du fichier ; rien si vide
    bool inject_sequence() {
        return sequence[0] == '\0' || marlin_server::inject(GCodeLiteral { .gcode = ConstexprString::from_str_unsafe(sequence) });
    }

    /// Paramètre S de M701 pour la matière de l'emplacement, comme le menu « Charger dans la buse » de Prusa ; pendant
    /// l'impression, celle du fichier si l'imprimante ne la connaît pas (heuristique de Prusa, comme auto_type())
    const char *filament_param(uint8_t slot, std::span<char> out) {
        StringBuilder sb(out);
        FilamentType::for_tool_heuristic(VirtualToolIndex::from_raw(slot)).build_encoded_gcode_param(sb);
        return sb.str();
    }

    METRIC_DEF(metric_od3d_stack, "od3d_stack", METRIC_VALUE_INTEGER, 0, METRIC_DISABLED);

    /// Pile libre la plus basse de la tâche de Marlin depuis son démarrage [octets] : la marge réelle, relevée après la
    /// purge et après les lignes (les points les plus profonds de la mesure)
    uint32_t stack_free_min() {
        const uint32_t free = uint32_t(uxTaskGetStackHighWaterMark(nullptr)) * sizeof(StackType_t);
        metric_record_integer(&metric_od3d_stack, int(free));
        return free;
    }

    using session::must_stop;
    using session::wait_moves;
    using session::wait_ms;

    constexpr float accel_xy = 5000.f; ///< comme PrusaPATuner ; la limite d'accélération du filament de Prusa reste
    constexpr uint8_t min_confidence = 30; ///< en dessous, rien n'est enregistré (le choix à l'œil reste possible)
    constexpr float tick_speed = 10.f; ///< traits des repères [mm/s]

    void fail(Error e) {
        st_error = e;
        st_phase = e == Error::none ? Phase::done : Phase::failed;
    }

    /// Repères de la ligne index (grand motif seulement) : son numéro en traits vers l'arrière, à droite de la ligne, un
    /// sur cinq plus long. false s'il faut s'arrêter.
    bool print_marks(session::Session &session, const pattern::Lines &lines, uint8_t index, float e_mm, float z) {
        static std::array<pattern::Tick, max_lines> ticks; // hors de la pile de Marlin (une seule mesure à la fois)
        const size_t n = lines.marks > 0 ? pattern::tally(uint8_t(index + 1), ticks) : 0;
        const float y = lines.y(index);
        const float x_ticks = lines.x0 + lines.length + pattern::tick_gap;
        for (size_t i = 0; i < n; ++i) {
            // D'un trait au suivant : rétraction et levée (retour du 23/11 : « il n'y a pas de rétractation »)
            const float tx = x_ticks + ticks[i].dx;
            if (!session.travel_to(tx, y, z) || !session.print_to(tx, y + ticks[i].length, e_mm, tick_speed) || !wait_moves()) {
                return false;
            }
        }
        return true;
    }

    /// Une ligne du motif : PA = k, lancement lent puis cycles × (rapide, lent) le long de X, dans le sens de la ligne
    /// (lines.dir()), trace analysée (vitesse en valeur absolue : les deux sens se lisent pareil). Repères à droite :
    /// après une ligne vers +X, avant une ligne vers −X (la tête y est déjà). false s'il faut s'arrêter.
    bool measure_line(session::Session &session, float k, float smooth_time, const Params &params, const pattern::Lines &lines, uint8_t index, Waveform *wave,
        BlockResult &result, FILE *csv) {
        const Profile prof = profile(params);
        const float ls = prof.v_lo * params.slow_s, lf = prof.v_hi * params.fast_s;
        const float e_mm = pattern::e_per_mm(params.bead);
        const float z = params.bead.height;
        const float y = lines.y(index);
        const float dir = lines.dir(index);

        if (lines.reversed(index) && !print_marks(session, lines, index, e_mm, z)) {
            return false;
        }
        // Buse au départ de la ligne, posée à hauteur de couche (déplacement levé, rétraction), puis filament remis tête
        // arrêtée, comme le trancheur : la pression se rétablit avant l'enregistrement de la force (400 ms plus loin),
        // jamais pendant un créneau mesuré. Motif tourné : lines.at() donne le point du plateau
        if (const pattern::Point p = lines.at(lines.x_start(index), y); !session.travel_to(p.x, p.y, z)) {
            return false;
        }
        session.deretract();
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
            force_probe::start(lines.along_y); // vitesse lue le long des lignes
            ScopeGuard stop_probe = [] { force_probe::stop(); };
            if (!wait_ms(150)) { // tête arrêtée : zéro de la force
                return false;
            }
            float x = lines.x_start(index);
            const auto segment = [&](float length, float speed) {
                x += dir * length;
                const pattern::Point p = lines.at(x, y);
                return session.print_to(p.x, p.y, e_mm, speed);
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

        if (!lines.reversed(index) && !print_marks(session, lines, index, e_mm, z)) {
            return false;
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
    return running.load() || followup.load();
}

uint16_t run_count() {
    return runs.load();
}

void request_stop() {
    stop_requested = true; // aussi pour le M1994 J1 en file : il s'arrêtera aussitôt
    followup = false;
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
        lines.marks = rq.compact ? 0.f : pattern::tally_width(count);
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

    /// K de départ de la recherche (motif compact) : celui de l'emplacement, sinon celui d'un autre emplacement de la
    /// même matière, sinon default_prior
    float prior_k(uint8_t slot) {
        if (const auto k = k_for_slot(slot)) {
            return *k;
        }
        const FilamentType type = FilamentType::for_tool_heuristic(VirtualToolIndex::from_raw(slot));
        for (uint8_t other = 0; other < VirtualToolIndex::count; ++other) {
            if (other != slot && FilamentType::for_tool(VirtualToolIndex::from_raw(other)) == type) {
                if (const auto k = k_for_slot(other)) {
                    return *k;
                }
            }
        }
        return default_prior;
    }

    /// Grand motif : balayage grossier puis fin autour du premier choix, une ligne par K dans l'ordre ; motif compact :
    /// une ligne par K que demande next_k() (recherche adaptative). K enregistré s'il est sûr. Buse chaude et purgée,
    /// lignes placées.
    Error sweep(session::Session &session, const Request &rq, const pattern::Lines &lines, FILE *csv) {
        // Hors de la pile de Marlin (une seule mesure à la fois)
        static std::array<float, LastRun::max_points> ks;
        static std::array<SweepPoint, LastRun::max_points> points;
        static Waveform wave;
        static std::array<float, 8> fine;
        const float smooth_time = session.smooth_time();
        st_phase = Phase::measuring;
        st_steps = lines.count; // motif compact : au plus (la recherche s'arrête souvent avant)
        last = no_run;
        last.slot = rq.slot;
        last.compact = rq.compact;
        size_t n = 0;
        float best_cost = INFINITY;
        Error stop = Error::stopped; ///< raison d'un run_line() à false
        const auto run_line = [&](float k) -> bool {
            st_step = uint8_t(n + 1);
            st_k = k_to_e4(k);
            live::set(N_("Test line %d/%d, K = 0.%03d"), int(n + 1), int(lines.count), int(std::clamp<long>(std::lround(k * 1000.f), 0, 999)));
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
                // Courbes de l'écran : K le plus bas et le plus haut (lignes dans l'ordre des essais en compact)
                if (last.low.step_s == 0 || k < last.k_low) {
                    last.low = wave;
                    last.k_low = k;
                }
                if (last.high.step_s == 0 || k > last.k_high) {
                    last.high = wave;
                    last.k_high = k;
                }
                if (r.cost < best_cost) {
                    best_cost = r.cost;
                    last.best = wave;
                    last.k_best = k;
                }
            }
            // Pas de force dès les premières lignes : la buse ne touche pas le plateau, on s'arrête (retour du 24/11)
            if (in_air(std::span(points.data(), n))) {
                stop = Error::no_force;
                return false;
            }
            return true;
        };
        Choice choice;
        if (rq.compact) {
            const Search search { .prior = prior_k(rq.slot), .k_min = rq.k_min, .k_max = rq.k_max, .step = rq.k_step, .cap = lines.count, .min_confidence = min_confidence };
            while (const auto k = next_k(std::span(points.data(), n), search)) {
                if (!run_line(*k)) {
                    return stop;
                }
            }
            choice = choose_k(std::span(points.data(), n), rq.k_min, rq.k_max, rq.k_step);
        } else {
            const size_t coarse = sweep_values(rq.k_min, rq.k_max, rq.k_step, std::span(ks.data(), ks.size() - 8));
            for (size_t i = 0; i < coarse; ++i) {
                if (!run_line(ks[i])) {
                    return stop;
                }
            }
            choice = choose_k(std::span(points.data(), n), rq.k_min, rq.k_max, rq.k_step);
            if (choice.fit == Fit::ok && rq.refine > 0) {
                const size_t m = refine_values(choice.k, rq.k_step / 4.f, rq.refine, 0.f, 1.f, fine);
                st_steps = uint8_t(coarse + m);
                for (size_t i = 0; i < m && n < lines.count; ++i) {
                    if (!run_line(fine[i])) {
                        return stop;
                    }
                }
                choice = choose_k(std::span(points.data(), n), rq.k_min, rq.k_max, rq.k_step);
            }
        }
        last.choice = choice;
        const uint32_t stack_free = stack_free_min();
        if (csv) {
            force_probe::csv_printf(csv, "C,%.4f,%u,%u,%.3f,%.4f,%.4f\n", double(choice.k), unsigned(choice.confidence), unsigned(choice.fit), double(choice.slope),
                double(choice.k_lag), double(choice.k_cost));
            force_probe::csv_printf(csv, "# stack_free_min %u B after the lines\n", unsigned(stack_free));
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

    /// Consignes de la mesure, celles de la matière par défaut ; bornées (les maximums de Prusa s'appliquent encore).
    /// Palpage à la température de palpage de PrusaSlicer, la plus chaude sans suintement (F44) : la buse ne chauffe
    /// pas davantage pendant le palpage (goutte sous la buse, dilatation comptée à la consigne), elle monte juste après.
    Temperatures temperatures_for(const FilamentTypeParameters &material, int16_t nozzle, int16_t bed) {
        const int16_t t = std::clamp<int16_t>(nozzle > 0 ? nozzle : material.nozzle_temperature, EXTRUDE_MINTEMP + 10, session::max_temperature);
        // G-code de départ de PrusaSlicer pour la CORE One : M109 R175 pour les PET, la table de Prusa (170) les met à 170
        const int16_t probe = material.nozzle_preheat_temperature + (strstr(material.name.data(), "PET") && material.nozzle_preheat_temperature == 170 ? 5 : 0);
        return {
            .nozzle = t,
            .bed = std::clamp<int16_t>(bed > 0 ? bed : material.heatbed_temperature, 0, session::max_bed_temperature),
            .probe = std::min<int16_t>(probe, t),
        };
    }

    // Départ rapide (F4-b) : la purge commence pendant la montée de la buse, les lignes attendent une température stable
    // (K dépend de la viscosité : mesurer pendant la montée le fausserait).
    // Purge à purge_below °C de la consigne : presque à la température des lignes (20 °C jusqu'à la 8.0.2.27 : PETG
    // purgé à 223 °C pour des lignes à 230 °C, retour du 22/11) ; quelques secondes de plus seulement
    constexpr float purge_below = 5.f;
    constexpr float stable_tolerance = 1.5f; ///< [°C]
    constexpr uint32_t stable_ms = 5000;

    /// Plateau et buse (température de palpage) chauffent ensemble ; origine dès la buse à sa température de palpage,
    /// pendant que le plateau chauffe ; nettoyage de la buse et palpage de area une fois le plateau chaud. Comme le G-code
    /// de départ de la CORE One (M140, M109 R170, G28, M190, G29 P9, G29 P1), sans la redescente de la buse à 100 °C.
    Error prepare_bed(session::Session &session, const pattern::Rect &area, const Temperatures &t) {
        thermalManager.setTargetBed(t.bed);
        st_phase = Phase::bed;
        // Série de l'assistant, maillage repris : ni origine ni palpage ; la buse chauffe pendant l'attente du plateau et
        // le déplacement vers la purge (heat_and_purge)
        const bool reuse = session::mesh_ready();
        session::Result r = reuse ? session::Result::ok : session.heat(t.probe, 3.f, 1000, session::heat_timeout_ms, false);
        if (r == session::Result::ok && !reuse) {
            st_phase = Phase::preparing;
            r = session.home(t.probe);
        }
        if (r == session::Result::ok) {
            st_phase = Phase::bed;
            r = session.heat_bed(t.bed, t.probe);
        }
        if (r == session::Result::ok) {
            st_phase = Phase::leveling;
            r = session.level(area, t.probe);
        }
        return error_of(r);
    }

    /// Buse à sa consigne : au-dessus du départ de la purge pendant la montée, purge dès purge_below °C sous la
    /// consigne, puis attente d'une température stable avant les lignes ; along_y : purge le long de Y
    Error heat_and_purge(session::Session &session, int16_t nozzle, float x, float y, float dir, bool along_y = false) {
        st_phase = Phase::heating;
        Hotend::for_tool(PhysicalToolIndex::from_raw(0)).set_nozzle_target_temp(nozzle); // la montée commence pendant le déplacement
        if (const session::Result r = session.go_to_purge(x, y, dir, along_y); r != session::Result::ok) {
            return error_of(r);
        }
        // Jamais sous la limite d'extrusion à froid de Prusa (consigne bornée à EXTRUDE_MINTEMP + 10 au moins)
        const float purge_tolerance = std::min(purge_below, float(nozzle - EXTRUDE_MINTEMP - 5));
        if (const session::Result r = session.heat(nozzle, purge_tolerance, 0); r != session::Result::ok) {
            return error_of(r);
        }
        st_phase = Phase::priming;
        if (!session.purge(along_y ? y : x, dir, along_y)) {
            return Error::stopped;
        }
        // Buse levée, filament rétracté pendant l'attente : pas de goutte au bout de la purge
        session.retract();
        if (!session.move_z(session::hop)) {
            return Error::stopped;
        }
        st_phase = Phase::heating;
        return error_of(session.heat(nozzle, stable_tolerance, stable_ms));
    }

} // namespace

void calibrate(const Request &rq, bool load_tried) {
    followup = false;
    if (!load_tried) {
        stop_requested = false; // J1 : un « Annuler » pendant le chargement vaut pour cette suite
    }
    ScopeGuard count_run = [] { ++runs; }; // après l'état : l'interface lit l'état une fois runs changé
    begin(rq.slot);
    if (load_tried && before_load) {
        // Consignes remises comme avant le chargement de Prusa (il laisse la buse chaude) : la session les remettra à la fin
        Hotend::for_tool(PhysicalToolIndex::from_raw(0)).set_nozzle_target_temp(before_load->first);
        thermalManager.setTargetBed(before_load->second);
        before_load.reset();
    }

    if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
        fail(Error::printing);
        return;
    }
    static std::array<float, LastRun::max_points> ks; // hors de la pile de Marlin
    const size_t coarse = sweep_values(rq.k_min, rq.k_max, rq.k_step, std::span(ks.data(), ks.size() - 8));
    if (rq.slot >= VirtualToolIndex::count || !is_valid(rq.params) || coarse < 3 || coarse + rq.refine > max_lines || rq.k_min < 0.f || rq.k_max > 1.f
        || rq.refine > 8 || !(rq.pitch >= 2.f && rq.pitch <= 20.f) || !std::isfinite(rq.x) || !std::isfinite(rq.y)) {
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
    if (load_tried && stop_requested) {
        fail(Error::stopped); // annulé pendant le chargement
        return;
    }
    if (!in_nozzle && load_tried) {
        fail(Error::load_failed); // chargement déjà tenté : pas de seconde tentative
        return;
    }
    // MMU activé qui ne répond pas (à flasher, M709 S2) : rien n'est sûr dans la buse, jamais de motif dans le vide
    if (config_store().mmu2_enabled.get() && !mmu_ready()) {
        fail(Error::mmu);
        return;
    }
    // Pas dans la buse : chargement de Prusa mis en file, avec le MMU ou, sans MMU, sa demande d'insérer le filament
    // (« Arrêter » y passe la mesure : load_failed au retour)
    const FilamentTypeParameters material = type.parameters();
    if (material.is_flexible) {
        fail(Error::flexible); // M-09 : les débits fixes de la mesure (jusqu'à 10 mm³/s) pousseraient trop un TPU
        return;
    }
    Temperatures t = temperatures_for(material, rq.temperature, rq.bed_temperature);
    st_temperature = t.nozzle;

    // Motif : une ligne par K, de l'avant vers l'arrière (grand : grossier + fin, repères à droite ; compact : place
    // pour compact_cap lignes) ; il doit tenir dans la zone imprimable (bords du plateau et course)
    pattern::Lines lines = lines_for(rq, rq.compact ? compact_cap : uint8_t(coarse + rq.refine), rq.x, rq.y);
    if (!session::printable_zone().contains(lines.area())) {
        fail(Error::no_room);
        return;
    }

    if (!in_nozzle) {
        // Jamais le MMU depuis cette pile (débordement en 8.0.2.23) : « Charger dans la buse » de Prusa mis en file,
        // puis cette mesure avec J1 ; M1994 rend la main. Le plateau chauffe pendant le chargement (F4-b).
        char filament[16];
        if (!oneclick_sequence(sequence, rq, filament_param(rq.slot, filament), t.bed) || !inject_sequence()) {
            fail(Error::mmu);
            return;
        }
        before_load.emplace(Hotend::for_tool(PhysicalToolIndex::from_raw(0)).nozzle_target_temp(), thermalManager.degTargetBed());
        st_phase = Phase::loading;
        live::set(N_("Loading slot %d into the nozzle"), rq.slot + 1);
        followup = true; // « en cours » pour l'interface jusqu'à la suite
        return;
    }

    // Série de l'assistant : le motif a sa place sur le plateau, sa purge aussi (retour du 23/11 : chaque bobine
    // imprimait au même endroit) ; maillage de la série repris : la buse va directement à sa température d'impression
    if (session::in_series()) {
        const pattern::Rect a = lines.area();
        const auto spot = session::claim(a.width(), a.height());
        if (!spot) {
            fail(Error::no_room);
            return;
        }
        lines = lines_for(rq, lines.count, rq.x + spot->x0 - a.x0, rq.y + spot->y0 - a.y0);
    }
    const auto purge = session::purge_start();
    if (!purge) {
        fail(Error::no_room);
        return;
    }
    if (session::mesh_ready()) {
        t.probe = t.nozzle;
    }

    running = true;
    ScopeGuard not_running = [] { running = false; };
    st_steps = lines.count;
    last = no_run;
    last.slot = rq.slot;

    // Réglages remis à la fin, même sur un arrêt de Marlin : PA, mouvements, consignes, maillage (session commune)
    session::Session session(stop_requested);
    fast_moves(rq.params);

    // Plateau et buse ensemble, origine pendant que le plateau chauffe, puis nettoyage et palpage de la zone du motif ;
    // buse à la température de la matière, purge au bord avant pendant la montée, lignes une fois stable (F4-b)
    if (const Error e = prepare_bed(session, lines.area(), t); e != Error::none) {
        fail(e);
        return;
    }
    if (const Error e = heat_and_purge(session, t.nozzle, purge->x, purge->y, purge->dir); e != Error::none) {
        fail(e);
        return;
    }
    const uint32_t stack_free = stack_free_min();

    // CSV facultatif sur la clé : un fichier par emplacement, réécrit à chaque mesure
    FILE *csv = nullptr;
    if (rq.csv && marlin_server::get_media_inserted()) {
        char path[24];
        snprintf(path, sizeof(path), "/usb/OD3D_PA%u.CSV", unsigned(rq.slot + 1));
        csv = fopen(path, "w");
        if (csv) {
            force_probe::csv_printf(csv, "# OneDrop3D PA (F4) slot %u, %s, nozzle %d C, bed %d C, %s pattern, printed lines %.2f x %.2f mm from X%.1f Y%.1f pitch %.1f, slow %.2f mm3/s %.2f s, fast %.2f mm3/s %.2f s, cycles %u, stack_free_min %u B\n",
                unsigned(rq.slot + 1), material.name.data(), int(t.nozzle), int(t.bed), rq.compact ? "compact" : "large", double(rq.params.bead.width), double(rq.params.bead.height),
                double(rq.x), double(rq.y), double(rq.pitch), double(rq.params.slow_flow), double(rq.params.slow_s), double(rq.params.fast_flow),
                double(rq.params.fast_s), unsigned(rq.params.cycles), unsigned(stack_free));
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

    // Motif compact (F4-b) : par emplacement, un bloc : une ligne de purge puis la place de compact_cap lignes (blocs à
    // 5 mm au moins l'un de l'autre, auto_spot())
    constexpr float auto_purge_half = 3.f; ///< demi-largeur de la purge (cordon épais de Prusa)
    constexpr float auto_purge_gap = 6.f; ///< de la purge à la 1re ligne
    constexpr float auto_lead = auto_purge_half + auto_purge_gap; ///< du bord du bloc à la 1re ligne

    Request auto_request(uint8_t slot) {
        Request rq;
        rq.slot = slot;
        make_compact(rq);
        return rq;
    }

    /// Matière de l'emplacement pour la mesure automatique : celle de l'imprimante, sinon (bobine jamais chargée par
    /// l'écran : retour du 24/11, « calibrer pour les différents filaments ») celle que le fichier donne à cet outil
    /// (heuristique de Prusa, la même que pour ses propres décisions pendant l'impression)
    FilamentType auto_type(uint8_t slot) {
        return FilamentType::for_tool_heuristic(VirtualToolIndex::from_raw(slot));
    }

    /// Bloc droit d'un emplacement, en (0, 0) : lignes depuis (0, auto_lead) ; la purge est sous les lignes
    pattern::Rect auto_block(uint8_t slot) {
        return lines_for(auto_request(slot), compact_cap, 0.f, auto_lead).area();
    }

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
            tell(k_for_slot(slot) ? _("Slot %d PA measured: K = %s").formatted(params, int(slot + 1), k)
                                  : _("Slot %d PA unsure (K = %s), slicer PA kept").formatted(params, int(slot + 1), k)); // M-10
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

bool large_pattern() {
    return config_store().od3d_pa_large.get();
}

void set_large_pattern(bool set) {
    config_store().od3d_pa_large.set(set);
}

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

namespace {

    /// Mesure automatique en cours : décidée par M1994 A, faite étape par étape (chargement de Prusa, puis M1994 P<n> A1)
    struct AutoPlan {
        std::array<uint8_t, VirtualToolIndex::count> order {}; ///< emplacements à mesurer, dans l'ordre
        uint8_t count = 0;
        uint8_t next = 0; ///< prochaine étape
        std::optional<uint8_t> initial; ///< filament laissé dans la buse à la fin (mesuré en dernier s'il l'est)
        std::array<pattern::Rect, VirtualToolIndex::count> blocks {}; ///< place du bloc de chaque étape
        uint8_t along_y = 0; ///< bit i : bloc de l'étape i tourné de 90° (lignes parallèles à Y)
        pattern::Rect area {}; ///< rectangle qui contient tous les blocs : palpé une fois
        int16_t bed = 0;
        int16_t nozzle = 0; ///< buse : première couche du fichier pour une impression d'une seule matière ; 0 : celle de la matière
        int16_t probe = 0; ///< buse pendant le palpage
        bool leveled = false; ///< zone palpée par une étape précédente (maillage en mémoire)
        bool skip = false; ///< « Passer » pendant le palpage d'avant le premier chargement (M1994 A2)
        bool active = false;
    };
    AutoPlan plan;

    /// Lignes de l'étape i (un bloc par emplacement : purge, puis lignes), dans son bloc ; tourné : X et Y échangés
    /// autour du départ des lignes (pattern::Lines::at())
    pattern::Lines auto_lines_of(size_t i, uint8_t slot) {
        const pattern::Rect one = auto_block(slot);
        const pattern::Rect &b = plan.blocks[i];
        const bool along_y = plan.along_y & (1u << i);
        pattern::Lines lines = along_y ? lines_for(auto_request(slot), compact_cap, b.x0 + auto_lead, b.y0 - one.x0)
                                       : lines_for(auto_request(slot), compact_cap, b.x0 - one.x0, b.y0 + auto_lead);
        lines.along_y = along_y;
        return lines;
    }

    /// Départ de la ligne de purge d'un bloc (sous les lignes, dans leur sens)
    pattern::Point auto_purge_point(const pattern::Lines &lines) {
        return lines.at(lines.x0, lines.y0 - auto_purge_gap);
    }

    /// Place du chargement de l'étape i, dans son bloc (palpé) : au départ de sa ligne de purge, que la purge emporte en
    /// partant (buse à purge_z au-dessus, puis descente en s'en éloignant) ; after_purge (filament remis à la fin) :
    /// après la fin de la purge déjà faite, sous les lignes, dans le bloc
    session::LoadSpot auto_load_spot(size_t i, bool after_purge = false) {
        const pattern::Lines lines = auto_lines_of(i, plan.order[i]);
        const float along = after_purge ? std::min(session::purge_length + 8.f, lines.length + lines.marks - 4.f) : 0.f;
        const pattern::Point p = lines.at(lines.x0 + std::max(0.f, along), lines.y0 - auto_purge_gap);
        return { p.x, p.y, session::load_z };
    }

    /// L'impression continue (pas d'arrêt en cours) : la mesure automatique peut mettre la suite en file
    bool print_goes_on() {
        return marlin_server::is_printing() && !marlin_server::aborting_or_aborted();
    }

    /// Met en file l'étape suivante avant le reste de l'impression ; après la dernière, le filament remis comme au départ
    void auto_next() {
        char filament[16];
        bool ok;
        // Chaque chargement (et le déchargement de la fin) se pose sur une place palpée de son bloc, pas en l'air
        // (retour du 24/11) ; avant le premier, le plateau est palpé à part (M1994 A2, qui pose la place)
        std::optional<session::LoadSpot> spot;
        if (plan.next < plan.count) {
            const uint8_t slot = plan.order[plan.next];
            const bool load = loaded_slot() != slot;
            ok = auto_step_sequence(sequence, slot, load ? filament_param(slot, filament) : nullptr, load && !plan.leveled);
            if (load && plan.leveled) {
                spot = auto_load_spot(plan.next);
            }
        } else {
            plan.active = false;
            start_hold = { .active = plan.leveled }; // la buse est chaude : le G-code de départ ne la fait pas redescendre
            ok = auto_restore_sequence(sequence, plan.initial, loaded_slot(), plan.initial ? filament_param(*plan.initial, filament) : "");
            if (plan.initial != loaded_slot() && plan.leveled && plan.count) {
                spot = auto_load_spot(plan.count - 1, true);
            }
        }
        session::set_load_spot(ok ? spot : std::nullopt);
        if (!ok || !inject_sequence()) {
            session::set_load_spot(std::nullopt);
            plan.active = false; // file d'injection pleine : l'impression continue avec le PA du trancheur
            log_slot(plan.next < plan.count ? plan.order[plan.next] : 0xff, Error::mmu);
        }
    }

    /// Mesure de l'emplacement slot (étape i), filament dans la buse ; stopped : « Passer », pause ou arrêt
    Error measure_auto_slot(uint8_t slot, size_t i, bool &stopped) {
        const Request rq = auto_request(slot);
        const pattern::Lines lines = auto_lines_of(i, slot);
        Temperatures t = temperatures_for(auto_type(slot).parameters(), plan.nozzle, plan.bed);
        t.probe = plan.probe;
        st_temperature = t.nozzle;

        // Réglages remis à la fin ; « Passer » sur l'écran Impression arrête la mesure, l'impression continue
        session::Session session(stop_requested, true);
        fast_moves(rq.params);
        // Maillage palpé par une étape précédente de cette impression, gardé en mémoire : pas de palpage, la buse monte
        // déjà pendant le plateau ; sinon (1re étape, origine perdue, maillage refusé) départ complet
        Error e = Error::none;
        bool reuse = plan.leveled && !axes_need_homing();
        if (reuse) {
            st_phase = Phase::bed;
            e = error_of(session.heat_bed(t.bed, t.nozzle));
            if (e == Error::none) {
                set_bed_leveling_enabled(true);
                reuse = planner.leveling_active;
            }
        }
        if (e == Error::none && !reuse) {
            e = prepare_bed(session, plan.area, t);
            plan.leveled = e == Error::none;
        }

        // Comme le G-code de départ du MMU : au-dessus du début de la purge, purge pendant la montée de la buse ; purge
        // sous les lignes, dans leur sens (le long de Y pour un bloc tourné)
        if (e == Error::none) {
            const pattern::Point purge = auto_purge_point(lines);
            e = heat_and_purge(session, t.nozzle, purge.x, purge.y, 1.f, lines.along_y);
            if (e != Error::none && must_stop()) {
                e = Error::stopped;
            }
        }
        if (e == Error::none) {
            std::ignore = stack_free_min();
            e = sweep(session, rq, lines, nullptr);
        }
        stopped = must_stop(); // avant la fin de la session (« Passer » n'est lu que pendant)
        return e;
    }

} // namespace

void calibrate_auto() {
    plan = {};
    const AutoMode mode = auto_mode();
    if (mode == AutoMode::off || running || !marlin_server::is_printing()) {
        return;
    }

    // Emplacements de l'impression (après la correspondance des outils), matière connue, K déjà là
    uint8_t used = 0, known = 0, has_k = 0;
    // Une seule matière dans le fichier : sa température de première couche vaut pour la mesure (F4-b ; le fichier ne
    // donne pas la température de chaque outil)
    struct {
        uint8_t used = 0;
        const char *material = nullptr;
        bool single_material = true;
    } scan; // une seule capture : la place de l'inplace_function est comptée
    GCodeInfo::getInstance().for_each_used_extruder([&scan](GcodeToolIndex, VirtualToolIndex tool, const GCodeInfo::ExtruderInfo &info) {
        if (tool.to_raw() < VirtualToolIndex::count) {
            scan.used |= uint8_t(1u << tool.to_raw());
        }
        scan.single_material = scan.single_material && (!scan.material || info.filament_name == scan.material);
        scan.material = info.filament_name.data();
    });
    used = scan.used;
    const bool single_material = scan.single_material;
    for (uint8_t slot = 0; slot < VirtualToolIndex::count; ++slot) {
        if (VirtualToolIndex::from_raw(slot).is_enabled() && auto_type(slot) && !flexible(slot)) { // M-09 : jamais un flexible
            known |= uint8_t(1u << slot);
        }
        if (record(slot).has_k()) {
            has_k |= uint8_t(1u << slot);
        }
    }
    // Matière inconnue même du fichier : pas de température sûre, pas de mesure ; on le dit
    const uint8_t unknown = uint8_t(slots_to_measure(mode, used, 0xff, has_k) & ~known);
    for (uint8_t slot = 0; slot < VirtualToolIndex::count; ++slot) {
        if (unknown & (1u << slot)) {
            report_slot(slot, Error::not_loaded);
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

    // Ordre : en dernier, celle qui doit rester dans la buse : la bobine d'une impression d'une seule matière (le T du
    // G-code de départ la reprend telle quelle : ni déchargement ni rechargement après la mesure, retour du 24/11 :
    // « après le PA [...] le décharge puis le recharge »), sinon celle chargée au départ (elle est remise à la fin)
    const std::optional<uint8_t> initial = std::popcount(used) == 1 ? std::optional<uint8_t>(uint8_t(std::countr_zero(used))) : loaded_slot();
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

    // Place : un bloc par emplacement (purge, puis lignes), chacun à sa place libre autour de la pièce (tourné de 90°
    // s'il ne tient pas droit), autant d'emplacements qu'il en tient (demande du 24/11)
    const pattern::Rect one = auto_block(order[0]);
    std::array<float, 2 * GRID_MAX_POINTS_X> xs; // M1994 A n'attend rien : pas besoin de mémoire statique
    std::array<float, 2 * GRID_MAX_POINTS_Y> ys;
    const size_t nx = major_points(xs, GRID_MAX_POINTS_X, unified_bed_leveling::mesh_index_to_xpos);
    const size_t ny = major_points(ys, GRID_MAX_POINTS_Y, unified_bed_leveling::mesh_index_to_ypos);
    const pattern::Rect print { area->a.x, area->a.y, area->b.x, area->b.y };
    const size_t fitting = auto_spots(std::span(plan.blocks.data(), count), plan.along_y, session::printable_zone(), print, one.width(), one.y1,
        std::span(xs.data(), nx), std::span(ys.data(), ny), MESH_X_DIST * GRID_MAJOR_STEP, MESH_Y_DIST * GRID_MAJOR_STEP);
    if (fitting == 0) {
        skip_all(Error::no_room, _("PA not measured: no room beside the print"));
        return;
    }
    for (size_t i = fitting; i < count; ++i) {
        report_slot(order[i], Error::no_room); // pas la place pour tous : à la prochaine impression
    }

    // Températures de l'impression : plateau du fichier (M140), buse du fichier pour une seule matière (sinon celle de
    // chaque matière), buse de palpage la plus basse ; un seul palpage pour tous les blocs
    plan.probe = session::max_temperature;
    plan.area = plan.blocks[0];
    for (size_t i = 0; i < fitting; ++i) {
        plan.order[i] = order[i];
        plan.probe = std::min(plan.probe, temperatures_for(auto_type(order[i]).parameters(), 0, 0).probe);
        const pattern::Rect &b = plan.blocks[i];
        plan.area = { std::min(plan.area.x0, b.x0), std::min(plan.area.y0, b.y0), std::max(plan.area.x1, b.x1), std::max(plan.area.y1, b.y1) };
    }
    const FilamentTypeParameters first_material = auto_type(order[0]).parameters();
    plan.bed = temperatures_for(first_material, 0, int16_t(GCodeInfo::getInstance().get_bed_preheat_temp().value_or(0))).bed;
    plan.nozzle = single_material ? int16_t(GCodeInfo::getInstance().get_hotend_preheat_temp().value_or(0)) : 0;
    plan.count = uint8_t(fitting);
    plan.initial = initial;
    plan.active = true;
    thermalManager.setTargetBed(plan.bed); // chaud entre les étapes : chacune remet cette consigne à sa fin
    auto_next();
}

void calibrate_auto_level() {
    if (!plan.active || plan.leveled || plan.next >= plan.count || !print_goes_on()) {
        return;
    }
    const uint8_t slot = plan.order[plan.next];
    Temperatures t = temperatures_for(auto_type(slot).parameters(), plan.nozzle, plan.bed);
    t.probe = plan.probe;
    stop_requested = false;
    running = true;
    ScopeGuard not_running = [] { running = false; };
    begin(slot);
    {
        session::Session session(stop_requested, true);
        plan.leveled = prepare_bed(session, plan.area, t) == Error::none;
        plan.skip = must_stop(); // avant la fin de la session (« Passer » n'est lu que pendant)
    }
    // Palpage raté : le chargement se fait à la position de Prusa, l'étape refera le palpage complet
    if (plan.leveled && !plan.skip) {
        session::set_load_spot(auto_load_spot(plan.next));
    }
}

void calibrate_auto_step(uint8_t slot) {
    if (!plan.active || plan.next >= plan.count || plan.order[plan.next] != slot) {
        return; // pas l'étape attendue (G-code tapé à la main, impression d'avant)
    }
    const size_t i = plan.next++;
    if (!print_goes_on()) {
        // Impression arrêtée pendant le chargement : plus rien (Prusa s'occupe de la fin)
        for (size_t j = i; j < plan.count; ++j) {
            log_slot(plan.order[j], Error::stopped);
        }
        plan.active = false;
        return;
    }

    session::set_load_spot(std::nullopt); // pas prise (chargement sauté) : ne servira pas à un chargement plus tard
    stop_requested = false;
    running = true;
    ScopeGuard not_running = [] { running = false; ++runs; };
    begin(slot);
    // « Passer » pendant le palpage d'avant le premier chargement : comme pendant la mesure
    bool stopped = plan.skip;
    // Chargement de Prusa déjà tenté : sans la bobine dans la buse, pas de seconde tentative
    const Error e = stopped ? Error::stopped : loaded_slot() == slot ? measure_auto_slot(slot, i, stopped) : Error::load_failed;
    fail(e);
    report_slot(slot, e);

    // Mesure ratée sur ses lignes : on passe à la suivante ; sinon (« Passer », chauffe, palpage, chargement) les
    // emplacements restants gardent le PA du trancheur, et le filament est remis comme au départ
    const bool next_slot = !stopped
        && (e == Error::none || e == Error::weak_signal || e == Error::no_crossing || e == Error::too_few_points || e == Error::out_of_range
            || e == Error::low_confidence);
    if (!next_slot) {
        for (size_t j = plan.next; j < plan.count; ++j) {
            log_slot(plan.order[j], Error::stopped);
        }
        plan.next = plan.count;
    }
    if (print_goes_on()) {
        auto_next();
    } else {
        plan.active = false;
    }
}

} // namespace od3d::pa
