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

TracePoint make_point(float force_g, float x_mm) {
    return { clamp16(force_g / force_unit_g), clamp16(x_mm / x_unit_mm) };
}

float force_at(Trace t, size_t i) {
    if (t.empty()) {
        return NAN;
    }
    const size_t i0 = i > 0 ? i - 1 : 0;
    const size_t i1 = std::min(i + 1, t.size() - 1);
    int32_t sum = 0;
    for (size_t k = i0; k <= i1; ++k) {
        sum += t[k].force;
    }
    return float(sum) * force_unit_g / float(i1 - i0 + 1);
}

float speed_at(Trace t, size_t i, float dt) {
    if (t.size() < 2 || dt <= 0) {
        return 0;
    }
    const size_t i0 = i >= 2 ? i - 2 : 0;
    const size_t i1 = std::min(i + 2, t.size() - 1);
    if (i1 == i0) {
        return 0;
    }
    return std::fabs(float(t[i1].x - t[i0].x)) * x_unit_mm / (float(i1 - i0) * dt);
}

float mean_force(Trace t, size_t i0, size_t i1) {
    i1 = std::min(i1, t.size());
    if (i0 >= i1) {
        return NAN;
    }
    int32_t sum = 0;
    for (size_t k = i0; k < i1; ++k) {
        sum += t[k].force;
    }
    return float(sum) * force_unit_g / float(i1 - i0);
}

float noise_force(Trace t, size_t i0, size_t i1) {
    i1 = std::min(i1, t.size());
    if (i0 + 4 > i1) {
        return NAN;
    }
    const size_t mid = i0 + (i1 - i0) / 2;
    const float m0 = mean_force(t, i0, mid);
    const float m1 = mean_force(t, mid, i1);
    // Centres des deux moitiés : la droite qui passe par (c0, m0) et (c1, m1)
    const float c0 = float(i0 + mid - 1) / 2.f;
    const float c1 = float(mid + i1 - 1) / 2.f;
    const float slope = (m1 - m0) / (c1 - c0);
    float sum2 = 0;
    for (size_t k = i0; k < i1; ++k) {
        const float d = force_g(t[k]) - (m0 + slope * (float(k) - c0));
        sum2 += d * d;
    }
    return std::sqrt(sum2 / float(i1 - i0));
}

size_t find_edges(Trace t, float dt, float threshold, uint8_t min_run, std::span<Edge> out) {
    size_t n = 0;
    if (t.size() < 5 || out.empty()) {
        return 0;
    }
    min_run = std::max<uint8_t>(min_run, 1);
    bool high = speed_at(t, 0, dt) > threshold;
    size_t candidate = 0;
    uint8_t run = 0;
    for (size_t i = 1; i < t.size(); ++i) {
        const bool now = speed_at(t, i, dt) > threshold;
        if (now == high) {
            run = 0;
            continue;
        }
        if (run == 0) {
            candidate = i;
        }
        if (++run >= min_run) {
            high = now;
            run = 0;
            if (n < out.size()) {
                out[n++] = { uint16_t(std::min<size_t>(candidate, UINT16_MAX)), now };
            } else {
                break;
            }
        }
    }
    return n;
}

} // namespace od3d::force
