/// @file
#include "chamber_climate_math_od3d.hpp"

#include <algorithm>
#include <cmath>

namespace od3d::chamber_climate {

std::optional<uint8_t> ceiling_target(Recipe recipe, std::optional<uint8_t> material_max_c, bool chamber_has_target, bool can_cool) {
    if (!has_ceiling(recipe) || !material_max_c.has_value() || *material_max_c == 0 || chamber_has_target || !can_cool) {
        return std::nullopt;
    }
    return material_max_c;
}

std::optional<uint8_t> lower_ceiling(std::optional<uint8_t> so_far, Recipe recipe, std::optional<uint8_t> material_max_c) {
    if (!has_ceiling(recipe) || !material_max_c.has_value() || *material_max_c == 0) {
        return so_far;
    }
    return so_far.has_value() ? std::min(*so_far, *material_max_c) : *material_max_c;
}

CooldownParams CooldownParams::clamped() const {
    return CooldownParams {
        .step_c = std::clamp(step_c, step_min, step_max),
        .interval_min = std::clamp(interval_min, interval_min_min, interval_min_max),
        .floor_c = std::clamp(floor_c, floor_min, floor_max),
    };
}

std::optional<int16_t> Cooldown::start(const CooldownParams &params, int16_t print_bed_target, float bed_current, uint32_t now_ms) {
    active_ = false;
    params_ = params.clamped();
    if (!std::isfinite(bed_current) || print_bed_target <= 0) {
        return std::nullopt;
    }
    // Jamais au-dessus de la mesure (on ne réchauffe pas) ni de la consigne de l'impression
    const int16_t first = std::min<int16_t>(print_bed_target, static_cast<int16_t>(std::floor(bed_current)));
    if (first <= params_.floor_c) {
        return std::nullopt;
    }
    active_ = true;
    target_ = first;
    start_ms_ = now_ms;
    last_step_ms_ = now_ms;
    return first;
}

Cooldown::Step Cooldown::step(const Inputs &in) {
    if (!active_) {
        return {};
    }
    if (in.bed_target != target_) {
        // M140, « Refroidir », préchauffage, minuteur de sécurité, erreur thermique : on lâche tout de suite
        active_ = false;
        return { Action::abort, in.bed_target };
    }
    if (static_cast<uint32_t>(in.now_ms - start_ms_) >= max_duration_ms) {
        active_ = false;
        return { Action::finish, 0 };
    }
    if (static_cast<uint32_t>(in.now_ms - last_step_ms_) < uint32_t(params_.interval_min) * 60 * 1000) {
        return {};
    }
    if (in.nozzle_heating) {
        // Une buse chauffe : l'imprimante sert à autre chose. On ne pose plus rien (chaque consigne relance le minuteur
        // de sécurité de Prusa, qui doit garder la main sur une buse chaude) ; le plateau reste à sa consigne
        active_ = false;
        return { Action::abort, in.bed_target };
    }
    last_step_ms_ = in.now_ms;
    int16_t next = static_cast<int16_t>(target_ - params_.step_c);
    if (std::isfinite(in.bed_current)) {
        next = std::min<int16_t>(next, static_cast<int16_t>(std::floor(in.bed_current)));
    }
    if (next <= params_.floor_c) {
        active_ = false;
        return { Action::finish, 0 };
    }
    return { Action::set_target, next };
}

uint32_t Cooldown::ms_to_next_step(uint32_t now_ms) const {
    if (!active_) {
        return 0;
    }
    const uint32_t interval = uint32_t(params_.interval_min) * 60 * 1000;
    const uint32_t elapsed = now_ms - last_step_ms_;
    return elapsed >= interval ? 0 : interval - elapsed;
}

uint16_t Cooldown::steps_left() const {
    if (!active_ || target_ <= params_.floor_c) {
        return 0;
    }
    return static_cast<uint16_t>((target_ - params_.floor_c + params_.step_c - 1) / params_.step_c);
}

} // namespace od3d::chamber_climate
