/// @file
#include "force_trace_od3d.hpp"

#include <algorithm>
#include <cmath>

namespace od3d::force {

namespace {
    int16_t clamp16(float v) {
        if (!std::isfinite(v)) {
            return 0;
        }
        return int16_t(std::clamp(std::lround(v), -32767L, 32767L));
    }
} // namespace

TracePoint make_point(float force_g, float x_mm, float e_mm) {
    return { clamp16(force_g / force_unit_g), clamp16(x_mm / x_unit_mm), clamp16(e_mm / e_unit_mm) };
}

} // namespace od3d::force
