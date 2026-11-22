/// @file
#include "measure_session_od3d.hpp"
#include "flow_limit_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>

#include <common/marlin_server.hpp>
#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <Marlin.h>
#include <feature/bedlevel/bedlevel.h>
#include <feature/pressure_advance/pressure_advance_config.hpp>
#include <feature/prusa/e-stall_detector.h>
#include <gcode/gcode.h>
#include <marlin_stubs/skippable_gcode.hpp>
#include <module/motion.h>
#include <module/planner.h>
#include <module/temperature.h>
#include <timing.h>
#include <tool/hotend/hotend.hpp>
#include <option/has_crash_detection.h>
#include <option/has_emergency_stop.h>
#if HAS_EMERGENCY_STOP()
    #include <feature/emergency_stop/emergency_stop.hpp>
#endif
#if HAS_CRASH_DETECTION()
    #include <feature/prusa/crash_recovery.hpp>
#endif

namespace od3d::session {

namespace {

    constexpr float retract_mm = 0.8f; ///< contre le suintement, aux déplacements et à la fin
    constexpr float retract_speed = 35.f;
    constexpr float default_smooth_time = 0.04f;

    /// Drapeau d'arrêt de la session en cours (nullptr sans session)
    const std::atomic<bool> *active_stop = nullptr;

    // État d'avant la session (une seule à la fois, fil de Marlin)
    pressure_advance::Config saved_pa;
    user_planner_settings_t saved_settings;
    bool saved_stealth = false;
    Hotend::TargetTemperature saved_target = 0;
    int16_t saved_bed = 0;
    bool saved_leveling = false;
    bool retracted = false;
    std::optional<BlockEStallDetection> block_estall; ///< la détection de filament coincé lirait nos paliers comme des à-coups
    std::optional<SkippableGCode::Guard> skip_guard; ///< en impression : « Passer » sur l'écran Impression
#if HAS_CRASH_DETECTION()
    std::optional<Crash_Temporary_Deactivate> no_crash; ///< en impression : nos accélérations ne sont pas des collisions
#endif

    Hotend &hotend() {
        return Hotend::for_tool(PhysicalToolIndex::from_raw(0));
    }

    bool hot() {
        return !thermalManager.tooColdToExtrude(PhysicalToolIndex::from_raw(0));
    }

    /// Arrêt demandé, ou Marlin qui vide ses files (arrêt d'urgence, G29 abandonné)
    bool interrupted() {
        return planner.draining() || must_stop();
    }

    /// Mouvement mis en file comme G1 : maillage du plateau appliqué (to_machine_pos) et découpé. Le planificateur de Prusa
    /// prend la position de line_to_current_position telle quelle, sans le maillage : les lignes du motif étaient posées à
    /// une hauteur qui ne suivait pas le plateau (cordons trop hauts, « des fils » ; trouvé par la version dans le vide, e49fff00)
    ///
    /// Prusa découpe le mouvement (maillage tous les LEVELED_SEGMENT_LENGTH, arrêt d'urgence tous les 0,05 mm en Z et
    /// toutes les 0,1 s) : la descente de 15 mm de la purge en faisait ~300, et buffer_segment attendait ses places avec
    /// idle() au fond de la pile de la mesure (débordement de la pile de Marlin en 8.0.2.24, juste après la purge). Ici,
    /// des morceaux d'au plus `pieces` segments, chacun après une place dans la file : buffer_segment n'attend jamais.
    void queue_to(const xyze_pos_t &target, float speed) {
        constexpr float pieces = 16.f; // + queue_depth déjà en file < BLOCK_BUFFER_SIZE
        static_assert(pieces + queue_depth + 2 < BLOCK_BUFFER_SIZE);
        const xyze_pos_t start = current_position;
        const xyze_pos_t diff = target - start;
        const float xy = std::hypot(diff.x, diff.y);
        float n = xy / (float(LEVELED_SEGMENT_LENGTH) * pieces);
#if HAS_EMERGENCY_STOP()
        n = std::max({ n, std::fabs(diff.z) / (buddy::EmergencyStop::max_segment_z_mm * pieces),
            speed > 0.f ? xy / speed / (buddy::EmergencyStop::max_segment_time_s * pieces) : 0.f });
#endif
        const int count = std::max(1, int(std::ceil(n)));
        for (int i = 1; i <= count; ++i) {
            // Même après un arrêt demandé (levée finale) : la file se vide d'elle-même ; vidée par Marlin, le mouvement est écarté
            while (planner.movesplanned() >= queue_depth && !planner.draining()) {
                idle(true);
            }
            destination = i == count ? target : start + diff * (float(i) / float(count));
            prepare_internal_move_to_destination(speed);
        }
    }
} // namespace

bool must_stop() {
    if ((active_stop && active_stop->load()) || planner.draining()) {
        return true;
    }
    if (skip_guard) {
        return skip_guard->is_skip_requested() || marlin_vars().print_state.get() != marlin_server::State::Printing;
    }
    return marlin_server::is_printing() || !marlin_server::printer_idle();
}

bool wait_ms(uint32_t ms) {
    const uint32_t start = ticks_ms();
    while (ticks_ms() - start < ms) {
        if (must_stop()) {
            return false;
        }
        idle(true);
    }
    return true;
}

bool wait_moves() {
    while (planner.processing()) {
        if (must_stop()) {
            return false;
        }
        idle(true);
    }
    return !must_stop();
}

bool wait_queue(uint8_t depth) {
    while (planner.movesplanned() >= depth) {
        if (must_stop()) {
            return false;
        }
        idle(true);
    }
    return !must_stop();
}

pattern::Rect printable_zone() {
    const pattern::Rect r {
        std::max(0.f, float(soft_endstop.min.x)),
        std::max(0.f, float(soft_endstop.min.y)),
        std::min(float(X_BED_SIZE), float(soft_endstop.max.x)),
        std::min(float(Y_BED_SIZE), float(soft_endstop.max.y)),
    };
    return r.grown(-edge_margin, -edge_margin);
}

Session::Session(const std::atomic<bool> &stop_requested, bool in_print) {
    active_stop = &stop_requested;
    planner.synchronize();
    if (in_print) {
        skip_guard.emplace();
#if HAS_CRASH_DETECTION()
        no_crash.emplace(); // file vide : Prusa ne change cet état qu'à l'arrêt
#endif
    }
    saved_pa = pressure_advance::get_axis_e_config();
    saved_settings = planner.user_settings;
    saved_stealth = config_store().stealth_mode.get();
    saved_target = hotend().nozzle_target_temp();
    saved_bed = thermalManager.degTargetBed();
    saved_leveling = planner.leveling_active;
    retracted = false;
    block_estall.emplace();
    flow_limit::suspended = true;
}

Session::~Session() {
    if (!planner.draining()) {
        // Un peu de filament en arrière contre le suintement, buse levée au-dessus du motif
        retract();
        if (!axes_need_homing()) {
            xyze_pos_t target = current_position;
            target.z = std::min(current_position.z + end_lift, float(soft_endstop.max.z));
            queue_to(target, travel_speed_z);
        }
        planner.synchronize();
    }
    if (!planner.draining()) {
        pressure_advance::set_axis_e_config(saved_pa);
        if (planner.leveling_active != saved_leveling) {
            set_bed_leveling_enabled(saved_leveling);
        }
    }
    planner.apply_settings(saved_settings);
    planner.set_stealth_mode(saved_stealth);
    hotend().set_nozzle_target_temp(saved_target);
    thermalManager.setTargetBed(saved_bed); // plateau laissé à sa consigne d'avant
    block_estall.reset();
    flow_limit::suspended = false;
#if HAS_CRASH_DETECTION()
    no_crash.reset(); // après planner.synchronize() (ou l'arrêt de Marlin)
#endif
    skip_guard.reset();
    active_stop = nullptr;
}

float Session::smooth_time() const {
    return saved_pa.smooth_time > 0.f ? saved_pa.smooth_time : default_smooth_time;
}

Result Session::heat_bed(int16_t bed, int16_t nozzle_probe) {
    hotend().set_nozzle_target_temp(nozzle_probe); // la buse va à sa température de palpage pendant ce temps
    thermalManager.setTargetBed(bed);
    const uint32_t start = ticks_ms();
    std::optional<uint32_t> inside_since;
    while (true) {
        if (must_stop()) {
            return Result::stopped;
        }
        // La consigne réelle (Prusa peut la borner) ; une consigne retirée entre-temps : échec
        const int16_t target = thermalManager.degTargetBed();
        if (target < bed - 5) {
            return Result::heating;
        }
        const bool inside = std::fabs(thermalManager.degBed() - float(target)) <= 2.f;
        const uint32_t now = ticks_ms();
        if (!inside) {
            inside_since.reset();
        } else if (!inside_since) {
            inside_since = now;
        } else if (now - *inside_since >= 2000) {
            return Result::ok;
        }
        if (now - start > bed_timeout_ms) {
            return Result::heating;
        }
        idle(true);
    }
}

Result Session::level(const pattern::Rect &area, int16_t nozzle_probe) {
    // Buse chaude mais sous la température d'impression : pas de suintement pendant le palpage (Prusa : 170 °C)
    if (const Result r = heat(nozzle_probe, 3.f, 1000, heat_timeout_ms, false); r != Result::ok) {
        return r;
    }
    if (axes_need_homing()) {
        GcodeSuite::G28_no_parser(true, true, true, { .only_if_needed = true, .precise = false });
    }
    if (interrupted()) {
        return Result::stopped;
    }
    if (axes_need_homing()) {
        return Result::homing;
    }

    // Comme le G-code de départ de PrusaSlicer pour la CORE One : moteur E coupé, nettoyage de la buse par
    // tapotements sur la bande avant droite, palpage de la zone imprimée (agrandie d'un point majeur de la grille,
    // comme Prusa le fait de la zone déclarée par M555), puis près de la purge, interpolation, extrapolation, maillage
    // activé
    GcodeSuite::process_subcommands_now_P("M84 E");
    GcodeSuite::process_subcommands_now_P("G29 P9 X208 Y-2.5 W32 H4");
    if (interrupted()) {
        return Result::stopped;
    }
    const pattern::Rect bed { 0.f, 0.f, float(X_BED_SIZE), float(Y_BED_SIZE) };
    const pattern::Rect probe = pattern::probe_rect(area, MESH_X_DIST * GRID_MAJOR_STEP, MESH_Y_DIST * GRID_MAJOR_STEP, bed);
    if (probe.empty()) {
        return Result::no_room;
    }
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "G29 P1 X%.1f Y%.1f W%.1f H%.1f", double(probe.x0), double(probe.y0), double(probe.width()), double(probe.height()));
    GcodeSuite::process_subcommands_now(cmd);
    if (interrupted()) {
        return Result::stopped;
    }
    if (ubl.g29_probing_failed) {
        return Result::leveling;
    }
    GcodeSuite::process_subcommands_now_P("G29 P1 X150 Y0 W100 H20 C");
    if (interrupted()) {
        return Result::stopped;
    }
    if (ubl.g29_probing_failed) {
        return Result::leveling;
    }
    GcodeSuite::process_subcommands_now_P("G29 P3.2");
    GcodeSuite::process_subcommands_now_P("G29 P3.13");
    GcodeSuite::process_subcommands_now_P("G29 A");
    if (interrupted()) {
        return Result::stopped;
    }
    return planner.leveling_active ? Result::ok : Result::leveling;
}

Result Session::heat(int16_t temperature, float tolerance, uint32_t hold_ms, uint32_t timeout_ms, bool extrude) {
    Hotend &h = hotend();
    h.set_nozzle_target_temp(temperature);
    const uint32_t start = ticks_ms();
    std::optional<uint32_t> inside_since;
    while (true) {
        if (must_stop()) {
            return Result::stopped;
        }
        // La consigne réelle (Prusa peut la borner) ; une consigne retirée entre-temps (refroidissement) : échec
        const Hotend::TargetTemperature target = h.nozzle_target_temp();
        if (extrude ? target < EXTRUDE_MINTEMP : target < temperature - 20) {
            return Result::heating;
        }
        const bool inside = std::fabs(h.nozzle_temp().value_or(0.f) - float(target)) <= tolerance;
        const uint32_t now = ticks_ms();
        if (!inside) {
            inside_since.reset();
        } else if (!inside_since) {
            inside_since = now;
        } else if (now - *inside_since >= hold_ms) {
            break;
        }
        if (now - start > timeout_ms) {
            return Result::heating;
        }
        idle(true);
    }
    return extrude && !hot() ? Result::heating : Result::ok;
}

Result Session::go_to_purge(float x, float y, float dir) {
    const float x_end = x + dir * purge_length;
    if (std::max(x, x_end) > soft_endstop.max.x || std::min(x, x_end) < soft_endstop.min.x || y < soft_endstop.min.y || y > soft_endstop.max.y) {
        return Result::no_room;
    }
    if (current_position.z < purge_z) {
        do_blocking_move_to_z(purge_z, travel_speed_z);
    }
    do_blocking_move_to_xy(x, y, travel_speed_xy);
    return must_stop() ? Result::stopped : Result::ok;
}

bool Session::purge(float x, float dir) {
    deretract();
    // Depuis X249 : G0 E5 X235 Z0.2 F500, X225 E4 F500, X215 E4 F650, X205 E4 F800, puis essuyage X202 Z0.05 et
    // X199 Z0.2 à F8000 ; ici, les mêmes tronçons depuis x dans le sens dir
    struct Segment {
        float dx, z, e, feedrate_mm_min;
    };
    constexpr Segment segments[] = {
        { 14.f, 0.2f, 5.f, 500.f },
        { 24.f, 0.2f, 4.f, 500.f },
        { 34.f, 0.2f, 4.f, 650.f },
        { 44.f, 0.2f, 4.f, 800.f },
        { 47.f, 0.05f, 0.f, 8000.f },
        { purge_length, 0.2f, 0.f, 8000.f },
    };
    for (const Segment &s : segments) {
        if (!wait_queue()) {
            return false;
        }
        xyze_pos_t target = current_position;
        target.x = x + dir * s.dx;
        target.z = s.z;
        target.e += s.e;
        queue_to(target, s.feedrate_mm_min / 60.f);
    }
    return wait_moves();
}

bool Session::travel_to(float x, float y, float z, bool lower) {
    if (must_stop()) {
        return false;
    }
    retract();
    if (current_position.z < z + hop) {
        do_blocking_move_to_z(z + hop, travel_speed_z);
    }
    do_blocking_move_to_xy(x, y, travel_speed_xy);
    if (lower) {
        do_blocking_move_to_z(z, travel_speed_z);
    }
    return !must_stop();
}

bool Session::move_z(float z) {
    if (must_stop()) {
        return false;
    }
    do_blocking_move_to_z(z, travel_speed_z);
    return !must_stop();
}

bool Session::print_to(float x, float y, float e_per_mm, float speed) {
    if (!wait_queue()) {
        return false;
    }
    deretract();
    const float d = std::hypot(x - current_position.x, y - current_position.y);
    xyze_pos_t target = current_position;
    target.x = x;
    target.y = y;
    target.e += d * e_per_mm;
    queue_to(target, speed);
    return true;
}

void Session::retract() {
    if (retracted || !hot()) {
        return;
    }
    xyze_pos_t target = current_position;
    target.e -= retract_mm;
    queue_to(target, retract_speed);
    retracted = true;
}

void Session::deretract() {
    if (!retracted) {
        return;
    }
    xyze_pos_t target = current_position;
    target.e += retract_mm;
    queue_to(target, retract_speed);
    retracted = false;
}

} // namespace od3d::session
