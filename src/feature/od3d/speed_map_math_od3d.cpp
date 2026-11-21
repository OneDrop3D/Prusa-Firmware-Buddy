/// @file
#include "speed_map_math_od3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace od3d::speed_map {

VibrationMeter::VibrationMeter(float sample_period, float tau)
    : alpha(std::clamp(tau > 0.f ? sample_period / tau : 1.f, 0.f, 1.f))
    , settling(uint16_t(std::clamp(tau > 0.f && sample_period > 0.f ? 5.f * tau / sample_period : 0.f, 0.f, 10000.f))) {}

void VibrationMeter::add(float x, float y, float z, bool counted) {
    const float v[3] = { x, y, z };
    if (!started) {
        // Le filtre part de la première valeur : pas de grand saut à l'établissement
        for (int i = 0; i < 3; ++i) {
            mean[i] = v[i];
        }
        started = true;
    }
    float square = 0.f;
    for (int i = 0; i < 3; ++i) {
        const float r = v[i] - mean[i]; // écart à la moyenne d'avant cet échantillon
        mean[i] += alpha * r;
        square += r * r;
    }
    if (settling > 0) {
        --settling;
        return;
    }
    if (counted && std::isfinite(square)) {
        sum += square;
        ++n;
    }
}

float VibrationMeter::mean_square() const {
    return n ? float(sum / double(n)) : 0.f;
}

Pass pass_for(float v, float accel, float cruise_s, float margin_mm, float delay_s, float max_length) {
    Pass p {};
    if (!(v > 0.f) || !(accel > 0.f)) {
        return p;
    }
    p.ramp = v * v / (2.f * accel);
    p.margin = margin_mm + v * delay_s;
    p.length = std::min(2.f * (p.ramp + p.margin) + v * cruise_s, max_length);
    return p;
}

bool in_cruise(const Pass &pass, float d) {
    const float from = pass.ramp + pass.margin;
    const float to = pass.length - pass.ramp - pass.margin;
    return to > from && d >= from && d <= to;
}

float median(std::span<const float> values) {
    std::array<float, 64> buf;
    size_t n = 0;
    for (const float v : values) {
        if (std::isfinite(v) && n < buf.size()) {
            buf[n++] = v;
        }
    }
    if (n == 0) {
        return 0.f;
    }
    std::sort(buf.begin(), buf.begin() + n);
    return n % 2 ? buf[n / 2] : (buf[n / 2 - 1] + buf[n / 2]) / 2.f;
}

size_t find_critical(std::span<const float> values, float factor, float min_value, std::span<uint8_t> out) {
    const size_t n = std::min<size_t>(values.size(), 64);
    if (n == 0 || out.empty()) {
        return 0;
    }
    const float threshold = std::max(factor * median(values.first(n)), min_value);
    std::array<uint8_t, 64> peaks;
    size_t count = 0;
    for (size_t i = 0; i < n; ++i) {
        const float v = values[i];
        if (!std::isfinite(v) || !(v > threshold)) {
            continue;
        }
        // Sommet : plus haut que le voisin de gauche, au moins autant que celui de droite (un plateau compte une fois)
        const bool left_ok = i == 0 || !(values[i - 1] >= v);
        const bool right_ok = i + 1 == n || !(values[i + 1] > v);
        if (left_ok && right_ok) {
            peaks[count++] = uint8_t(i);
        }
    }
    // Du plus fort au plus faible (à valeur égale, la vitesse la plus basse d'abord)
    std::stable_sort(peaks.begin(), peaks.begin() + count, [&](uint8_t a, uint8_t b) { return values[a] > values[b]; });
    count = std::min(count, out.size());
    std::copy_n(peaks.begin(), count, out.begin());
    return count;
}

void format_speeds(std::span<const uint8_t> indices, float start, float step, char *out, size_t size) {
    if (size == 0) {
        return;
    }
    out[0] = '\0';
    std::array<uint8_t, 16> sorted {};
    const size_t n = std::min(indices.size(), sorted.size());
    std::copy_n(indices.begin(), n, sorted.begin());
    std::sort(sorted.begin(), sorted.begin() + n);
    size_t used = 0;
    for (size_t i = 0; i < n && used + 1 < size; ++i) {
        const int written = snprintf(out + used, size - used, i ? ", %d" : "%d", int(std::lround(start + float(sorted[i]) * step)));
        if (written < 0) {
            break;
        }
        used = std::min(size - 1, used + size_t(written));
    }
}

int16_t bar_height(float value, float scale, int16_t h) {
    if (h <= 0 || !(scale > 0.f) || !std::isfinite(value) || !(value > 0.f)) {
        return 0;
    }
    return int16_t(std::lround(std::clamp(value / scale, 0.f, 1.f) * float(h)));
}

} // namespace od3d::speed_map
