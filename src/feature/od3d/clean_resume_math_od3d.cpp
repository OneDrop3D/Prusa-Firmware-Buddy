/// @file
#include "clean_resume_math_od3d.hpp"

#include <algorithm>
#include <cmath>

namespace od3d::clean_resume {

namespace {

    /// Purge et essuyage dans les limites ; l'essuyage part vers la gauche, ou vers la droite contre la butée
    std::optional<Plan> place(float purge_x, float purge_y, const Box &limits) {
        if (!std::isfinite(purge_x) || !std::isfinite(purge_y) || limits.x_max - limits.x_min < 2 * wipe_mm) {
            return std::nullopt;
        }
        Plan p {};
        p.purge_x = std::clamp(purge_x, limits.x_min, limits.x_max);
        p.purge_y = std::clamp(purge_y, limits.y_min, limits.y_max);
        p.wipe_x = p.purge_x - wipe_mm;
        if (p.wipe_x < limits.x_min) {
            p.wipe_x = p.purge_x + wipe_mm;
        }
        return p;
    }

} // namespace

Settings Settings::clamped() const {
    return Settings {
        .enabled = enabled,
        .purge_mm = std::min(purge_mm, purge_max),
        .retract_tenths = std::min(retract_tenths, retract_max),
        .approach_mm_s = std::clamp(approach_mm_s, approach_min, approach_max),
    };
}

std::optional<Plan> plan_pause(const Settings &settings, float purge_x, float purge_y, float resume_e, float current_e, const Box &limits) {
    const Settings s = settings.clamped();
    if (!s.enabled || !std::isfinite(resume_e) || !std::isfinite(current_e)) {
        return std::nullopt;
    }
    const float unretract = resume_e - current_e;
    if (unretract < 0 || unretract > max_unretract_mm) {
        return std::nullopt; // E en avance sur la pièce, ou très loin derrière : on ne devine pas
    }
    auto p = place(purge_x, purge_y, limits);
    if (!p) {
        return std::nullopt;
    }
    p->extrude_mm = unretract + s.purge_mm;
    p->retract_mm = s.retract_tenths / 10.0f;
    p->e_after = resume_e - p->retract_mm;
    return p;
}

std::optional<Plan> plan_power_panic(const Settings &settings, float purge_x, float purge_y, float pause_retract_mm, float current_e, const Box &limits) {
    const Settings s = settings.clamped();
    if (!s.enabled || !std::isfinite(current_e) || !(pause_retract_mm >= 0) || pause_retract_mm > max_unretract_mm) {
        return std::nullopt;
    }
    auto p = place(purge_x, purge_y, limits);
    if (!p) {
        return std::nullopt;
    }
    p->extrude_mm = pause_retract_mm + s.purge_mm;
    p->retract_mm = pause_retract_mm; // Prusa désrétracte cette longueur au retour
    p->e_after = current_e;
    return p;
}

float power_panic_lift_z(float current_z, float resume_z, float z_max) {
    return std::min(std::max(current_z, resume_z + power_panic_clearance_mm), std::max(current_z, z_max));
}

float approach_feedrate(const Settings &settings, float prusa_mm_s) {
    return std::min<float>(settings.clamped().approach_mm_s, prusa_mm_s);
}

} // namespace od3d::clean_resume
