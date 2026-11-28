/// @file
#include "clean_resume_od3d.hpp"
#include "feature_marks_od3d.hpp"

#include <algorithm>
#include <cmath>

#include <config_store/store_instance.hpp>
#include <mapi/feedrates/standard_feedrates.hpp>
#include <mapi/motion.hpp>
#include <mapi/parking.hpp>
#include <Marlin.h>
#include <module/motion.h>
#include <module/planner.h>
#include <module/temperature.h>
#include <option/has_cancel_object.h>
#include <tool_index.hpp>
#if HAS_CANCEL_OBJECT()
    #include <feature/cancel_object/cancel_object.hpp>
#endif

namespace od3d::clean_resume {

namespace {

    Tracker tracker; ///< fil de Marlin
    InfillPlan plan; ///< hors de la pile de Marlin

    /// Marge au-dessus de la pièce pour les déplacements XY, comme unpark_head_XY de Prusa
    constexpr float traverse_clearance_mm = 5.0f;

    bool object_cancelled([[maybe_unused]] int16_t object) {
#if HAS_CANCEL_OBJECT()
        return buddy::cancel_object().is_object_cancelled(object);
#else
        return false;
#endif
    }

    /// Outil actif, axes connus, buse assez chaude pour extruder
    bool can_extrude() {
        const auto tool = PhysicalToolIndex::currently_selected_opt();
        return tool.has_value() && all_axes_homed() && !thermalManager.tooColdToExtrude(*tool);
    }

} // namespace

Settings settings() {
    return Settings {
        .enabled = config_store().od3d_clean_resume.get(),
        .purge_tenths = config_store().od3d_clean_resume_purge.get(),
        .retract_tenths = config_store().od3d_clean_resume_retract.get(),
    }
        .clamped();
}

void set_settings(const Settings &s) {
    const Settings c = s.clamped();
    config_store().od3d_clean_resume.set(c.enabled);
    config_store().od3d_clean_resume_purge.set(c.purge_tenths);
    config_store().od3d_clean_resume_retract.set(c.retract_tenths);
    feature_marks::enabled = c.enabled;
}

void load() {
    feature_marks::enabled = config_store().od3d_clean_resume.get();
}

void on_print_start() {
    tracker.reset();
    feature_marks::enabled = config_store().od3d_clean_resume.get();
}

void set_feature(uint8_t feature) {
    tracker.set_feature(feature);
}

void on_tool_change() {
    tracker.forget_runs();
}

void on_print_move(float x0, float y0, float z0, float e0, float x1, float y1, float z1, float e1) {
    if (!feature_marks::enabled.load(std::memory_order_relaxed)) {
        return;
    }
    if (tracker.move({ x0, y0 }, z0, e0, { x1, y1 }, z1, e1)) {
#if HAS_CANCEL_OBJECT()
        tracker.set_object(buddy::cancel_object().current_object());
#endif
    }
}

void before_unpark(float resume_z, float resume_e) {
    const Settings s = settings();
    if (!s.enabled || !can_extrude()) {
        return;
    }
    const Refusal refusal = plan_infill(s, tracker, resume_z, resume_e, current_position.e,
        Box { X_MIN_POS, X_MAX_POS, Y_MIN_POS, Y_MAX_POS }, object_cancelled, plan);
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPAIR("Clean resume (0: infill prime, else Prusa resume): ", int(refusal));
    if (refusal != Refusal::none) {
        return;
    }

    namespace fr = buddy::standard_feedrates;
    const float traverse_z = std::max(resume_z, planner.max_printed_z) + traverse_clearance_mm;

    // 1. Comme unpark_head_XY : le long retour en Z au-dessus du parcage, puis au-dessus du remplissage
    do_blocking_move_to_z(std::min(current_position.z, traverse_z), NOZZLE_PARK_Z_FEEDRATE, Segmented::yes);
    if (planner.draining()) {
        return;
    }
    // IF-20 : le chemin du parcage de Prusa (INDX : hors du bac en Y, puis autour de la brosse), pas une diagonale
    if (!mapi::park({ .x = plan.purge.x, .y = plan.purge.y })) {
        return;
    }

    // 2. Descente sur le remplissage, filament toujours rétracté
    do_blocking_move_to_z(plan.z, NOZZLE_PARK_Z_FEEDRATE);
    if (planner.draining()) {
        return;
    }

    // 3. Une seule fois en bas : désrétraction de la pause et petite purge
    mapi::extruder_move(plan.purge_e, fr::current_extruder(fr::Extruder::advanced_pause_purge));

    // 4. Court cordon le long du remplissage déjà tracé, au débit de ce remplissage
    for (size_t i = 0; i < plan.line_count; ++i) {
        destination = current_position;
        destination.x = plan.line[i].x;
        destination.y = plan.line[i].y;
        destination.e += std::hypot(destination.x - current_position.x, destination.y - current_position.y) * plan.e_per_mm;
        prepare_move_to(destination, prime_line_mm_s, { .move = { .is_printing_move = true } });
    }

    // 5. Rétraction, remontée : unpark_head_XY rejoint le point de reprise, unpark_head_ZE descend et désrétracte
    mapi::extruder_move(-plan.retract, fr::current_extruder(fr::Extruder::retract));
    do_blocking_move_to_z(traverse_z, NOZZLE_PARK_Z_FEEDRATE);
    planner.synchronize();
    if (planner.draining()) {
        return;
    }
    // La désrétraction de Prusa vise E de la pièce : elle pousse exactement la rétraction faite ici
    sync_e_position_to(plan.e_after);
}

} // namespace od3d::clean_resume
