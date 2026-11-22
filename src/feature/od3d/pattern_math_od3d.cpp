/// @file
#include "pattern_math_od3d.hpp"

#include <algorithm>
#include <cmath>

namespace od3d::pattern {

Rect Rect::clipped(const Rect &to) const {
    return { std::max(x0, to.x0), std::max(y0, to.y0), std::min(x1, to.x1), std::min(y1, to.y1) };
}

float e_per_mm(const Bead &b) {
    return b.width * b.height / filament_area_mm2;
}

float speed_for_flow(const Bead &b, float flow) {
    const float section = b.width * b.height;
    return section > 0 ? flow / section : 0.f;
}

bool is_valid(const Bead &b) {
    const auto in = [](float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; };
    return in(b.width, 0.3f, 1.2f) && in(b.height, 0.1f, 0.4f) && b.width >= 1.2f * b.height;
}

size_t tally(uint8_t number, std::span<Tick> out) {
    size_t n = 0;
    for (uint8_t i = 1; i <= number && n < out.size(); ++i) {
        out[n++] = { float(i - 1) * tick_pitch, i % 5 == 0 ? tick_long : tick_short };
    }
    return n;
}

float tally_width(uint8_t number) {
    return number == 0 ? 0.f : tick_gap + float(number - 1) * tick_pitch;
}

Rect Lines::area() const {
    if (count == 0) {
        return { x0, y0, x0, y0 };
    }
    const float half = bead.width / 2.f;
    // Les traits partent de la ligne vers l'arrière (y croissant)
    const float top = y(uint8_t(count - 1)) + (marks > 0 ? std::max(tick_long, half) : half);
    return { x0 - half, y0 - half, x0 + length + marks + half, top };
}

Rect probe_rect(const Rect &pattern, float grow_x, float grow_y, const Rect &bed) {
    return pattern.grown(grow_x, grow_y).clipped(bed);
}

} // namespace od3d::pattern
