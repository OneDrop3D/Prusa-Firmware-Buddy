/// @file
#include "e_jump_math_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace od3d::e_jump {

size_t jump_steps(std::span<float> out) {
    size_t n = 0;
    for (float j = first_jump; j <= max_jump * 1.001f && n < std::min(out.size(), max_steps); j *= step_ratio) {
        out[n++] = j;
    }
    return n;
}

Pulse pulse(float jump, float smooth_time) {
    Pulse p;
    p.base_speed = base_speed;
    p.peak_speed = base_speed + jump;
    p.accel = jump / smooth_time;
    p.before_mm = base_speed * 0.25f;
    // Montée et descente : smooth_time × (base + jump / 2) chacune ; palier : smooth_time × (base + jump)
    p.jump_mm = smooth_time * (3.f * base_speed + 2.f * jump);
    p.after_mm = base_speed * 0.15f;
    return p;
}

StepState state(const Step &s) {
    return s.stalls >= 2 ? StepState::stalled : (s.stalls == 1 ? StepState::doubtful : StepState::clean);
}

bool must_stop(std::span<const Step> steps) {
    const size_t n = steps.size();
    return n && (state(steps[n - 1]) == StepState::stalled
                    || (n >= 2 && state(steps[n - 1]) == StepState::doubtful && state(steps[n - 2]) == StepState::doubtful));
}

Verdict classify(std::span<const Step> steps, bool complete) {
    Verdict v;
    size_t stop = steps.size();
    for (size_t i = 0; i < steps.size(); ++i) {
        if (must_stop(steps.first(i + 1))) {
            stop = state(steps[i]) == StepState::stalled ? i : i - 1; // deux douteux : le calage commence au premier
            break;
        }
    }
    if (stop == steps.size() && !complete) {
        return v; // arrêtée avant la fin, sans calage
    }
    for (size_t i = stop; i-- > 0;) {
        if (state(steps[i]) == StepState::clean) {
            v.safe_index = uint8_t(i);
            v.safe_jump = steps[i].jump;
            break;
        }
    }
    v.stop_index = stop < steps.size() ? uint8_t(stop) : 0xff;
    if (v.safe_index == 0xff) {
        v.fit = Fit::too_low;
    } else {
        v.fit = stop < steps.size() ? Fit::found : Fit::not_reached;
    }
    return v;
}

float filament_accel(float jump, float k) {
    if (!(jump > 0.f) || !(k > 0.f)) {
        return NAN;
    }
    return std::clamp(jump / k, prusa_filament_accel, max_filament_accel);
}

float line_accel(float f, float width, float height) {
    const float area = (width - height * (1.f - std::numbers::pi_v<float> / 4.f)) * height;
    if (!(f > 0.f) || !(area > 0.f)) {
        return NAN;
    }
    return std::min(f * filament_area_mm2 / area, xy_accel_cap);
}

uint16_t jump_to_e1(float jump) {
    return std::isfinite(jump) && jump > 0.f ? uint16_t(std::lround(std::min(jump, 6000.f) * 10.f)) : 0;
}

} // namespace od3d::e_jump
