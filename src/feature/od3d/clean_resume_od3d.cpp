/// @file
#include "clean_resume_od3d.hpp"

#include <variant>

#include <config_store/store_instance.hpp>
#include <mapi/feedrates/standard_feedrates.hpp>
#include <mapi/motion.hpp>
#include <mapi/parking.hpp>
#include <module/motion.h>
#include <module/planner.h>
#include <module/temperature.h>
#include <tool_index.hpp>

namespace od3d::clean_resume {

namespace {

    bool approach_pending = false; ///< before_unpark a purgé : la descente suivante est la nôtre (thread de Marlin)

    /// Position de purge de Prusa (M600), dans les limites de la machine
    std::optional<Plan> purge_position(auto &&make_plan) {
        const mapi::ParkingPosition purge = mapi::get_parking_position(mapi::ParkPosition::purge);
        const float *x = std::get_if<float>(&purge.x);
        const float *y = std::get_if<float>(&purge.y);
        if (!x || !y) {
            return std::nullopt;
        }
        return make_plan(*x, *y, Box { X_MIN_POS, X_MAX_POS, Y_MIN_POS, Y_MAX_POS });
    }

    /// Outil actif, axes connus, buse assez chaude pour extruder
    bool can_extrude() {
        const auto tool = PhysicalToolIndex::currently_selected_opt();
        return tool.has_value() && all_axes_homed() && !thermalManager.tooColdToExtrude(*tool);
    }

    /// Purge, rétraction, essuyage (aller-retour en X), recalage de E. \returns false si l'impression s'arrête
    bool run(const Plan &p) {
        namespace fr = buddy::standard_feedrates;
        do_blocking_move_to_xy(p.purge_x, p.purge_y, NOZZLE_PARK_XY_FEEDRATE);
        mapi::extruder_move(p.extrude_mm, fr::current_extruder(fr::Extruder::advanced_pause_purge));
        if (p.retract_mm > 0) {
            mapi::extruder_move(-p.retract_mm, fr::current_extruder(fr::Extruder::retract));
        }
        do_blocking_move_to_x(p.wipe_x, NOZZLE_PARK_XY_FEEDRATE);
        do_blocking_move_to_x(p.purge_x, NOZZLE_PARK_XY_FEEDRATE);
        planner.synchronize();
        // Toujours recalé : la désrétraction de Prusa vise la position E de la pièce
        sync_e_position_to(p.e_after);
        return !planner.draining();
    }

} // namespace

Settings settings() {
    return Settings {
        .enabled = config_store().od3d_clean_resume.get(),
        .purge_mm = config_store().od3d_clean_resume_purge.get(),
        .retract_tenths = config_store().od3d_clean_resume_retract.get(),
        .approach_mm_s = config_store().od3d_clean_resume_approach.get(),
    }
        .clamped();
}

void set_settings(const Settings &s) {
    const Settings c = s.clamped();
    config_store().od3d_clean_resume.set(c.enabled);
    config_store().od3d_clean_resume_purge.set(c.purge_mm);
    config_store().od3d_clean_resume_retract.set(c.retract_tenths);
    config_store().od3d_clean_resume_approach.set(c.approach_mm_s);
}

void before_unpark(float resume_e) {
    approach_pending = false;
    const Settings s = settings();
    if (!s.enabled || !can_extrude()) {
        return;
    }
    const auto plan = purge_position([&](float x, float y, const Box &box) {
        return plan_pause(s, x, y, resume_e, current_position.e, box);
    });
    if (!plan) {
        return;
    }
    // La tête est garée haut (au moins 50 mm au-dessus de la pièce) : les mouvements XY restent sans Z, comme le
    // parcage de la pause ; ensuite unpark_head_XY de Prusa revient au point
    approach_pending = run(*plan);
}

std::optional<float> take_approach(float prusa_z_mm_s) {
    if (!approach_pending) {
        return std::nullopt;
    }
    approach_pending = false;
    return approach_feedrate(settings(), prusa_z_mm_s);
}

void before_power_panic_unpark(float resume_z, float pause_retract_mm) {
    const Settings s = settings();
    if (!s.enabled || !can_extrude()) {
        return;
    }
    const auto plan = purge_position([&](float x, float y, const Box &box) {
        return plan_power_panic(s, x, y, pause_retract_mm, current_position.e, box);
    });
    if (!plan) {
        return;
    }
    // Au-dessus de la pièce avant tout déplacement XY (Z monte = le plateau descend : sens toujours sûr)
    const float lift = power_panic_lift_z(current_position.z, resume_z, Z_MAX_POS);
    if (lift > current_position.z) {
        do_blocking_move_to_z(lift, NOZZLE_PARK_Z_FEEDRATE);
    }
    run(*plan);
}

float power_panic_z_feedrate(float prusa_mm_s) {
    const Settings s = settings();
    return s.enabled ? approach_feedrate(s, prusa_mm_s) : prusa_mm_s;
}

} // namespace od3d::clean_resume
