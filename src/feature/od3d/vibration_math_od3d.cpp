/// @file
#include "vibration_math_od3d.hpp"
#include "calib_registry_od3d.hpp" // no_value
#include "speed_map_math_od3d.hpp" // median

#include <algorithm>
#include <cmath>

namespace od3d::vibration {

void WindowMeter::add(float x, float y, float z) {
    const float v[3] = { x, y, z };
    for (int i = 0; i < 3; ++i) {
        sum[i] += v[i];
        sum_sq[i] += double(v[i]) * v[i];
    }
    ++n;
}

float WindowMeter::variance() const {
    if (!n) {
        return 0.f;
    }
    double var = 0;
    for (int i = 0; i < 3; ++i) {
        const double mean = sum[i] / n;
        var += std::max(0.0, sum_sq[i] / n - mean * mean);
    }
    return float(var);
}

Limit find_limit(std::span<const float> residual, std::span<const uint16_t> accel, float noise) {
    Limit l;
    const size_t n = std::min(residual.size(), accel.size());
    const float floor = std::isfinite(noise) && noise > 0.f ? noise : 0.f;
    // Résidu par unité d'accélération, bruit retiré (les variances s'ajoutent)
    const auto per_accel = [&](size_t i) {
        return std::sqrt(std::max(0.f, residual[i] - floor)) / float(accel[i]);
    };
    size_t i = 0;
    for (; i < n && std::isfinite(residual[i]); ++i) {
        if (residual[i] > 0.f && residual[i] >= reference_ratio * floor && accel[i] > 0) {
            break;
        }
    }
    if (i >= n || !std::isfinite(residual[i])) {
        return l; // tout dans le bruit, ou rien de mesuré
    }
    l.found = true;
    l.reference = uint8_t(i);
    l.last_ok = uint8_t(i);
    const float limit = rise_ratio * per_accel(i);
    for (size_t j = i + 1; j < n && std::isfinite(residual[j]); ++j) {
        if (per_accel(j) > limit) {
            l.rose = true;
            break;
        }
        l.last_ok = uint8_t(j);
    }
    return l;
}

std::array<float, fan_bins> bin_frequencies(float f0) {
    std::array<float, fan_bins> f {};
    for (size_t k = 0; k < peak_bins; ++k) {
        f[k] = f0 * (1.f + peak_step * (float(k) - float(peak_bins / 2)));
    }
    for (size_t k = 0; k < noise_factors.size(); ++k) {
        f[peak_bins + k] = f0 * noise_factors[k];
    }
    return f;
}

Tone tone(std::span<const float> magnitudes) {
    Tone t;
    if (magnitudes.size() < fan_bins) {
        return t;
    }
    for (size_t k = 0; k < peak_bins; ++k) {
        if (std::isfinite(magnitudes[k])) {
            t.amplitude = std::max(t.amplitude, magnitudes[k]);
        }
    }
    t.noise = speed_map::median(magnitudes.subspan(peak_bins, noise_factors.size()));
    t.detected = t.amplitude > 0.f && t.amplitude >= fan_snr_min * t.noise;
    return t;
}

FanLevel fan_level(std::span<const Tone> tones) {
    FanLevel f;
    double sum = 0;
    size_t n = 0;
    for (const Tone &t : tones) {
        if (std::isfinite(t.amplitude)) {
            sum += double(t.amplitude) * t.amplitude;
            ++n;
        }
        f.detected = f.detected || t.detected;
    }
    f.value = n ? float(std::sqrt(sum / double(n))) : 0.f;
    return f;
}

Verdict compare(FanLevel now, std::optional<float> reference) {
    if (!now.detected) {
        return Verdict::not_detected;
    }
    if (!reference || !(*reference > 0.f)) {
        return Verdict::reference;
    }
    return now.value >= worse_ratio * *reference ? Verdict::worse : Verdict::ok;
}

uint8_t encode(float amplitude) {
    if (!(amplitude > 0.f) || !std::isfinite(amplitude)) {
        return 0;
    }
    return uint8_t(std::clamp(std::lround(8.f * std::log2(amplitude / code_min)), 0L, 126L) + 1);
}

std::optional<float> decode(uint8_t code) {
    if (code == 0 || code > 127) {
        return std::nullopt;
    }
    return code_min * std::exp2(float(code - 1) / 8.f);
}

int16_t pack(uint8_t low, uint8_t high) {
    return int16_t(uint16_t((high & 0x7F) << 8 | (low & 0x7F)));
}

uint8_t unpack(int16_t packed, bool high) {
    const uint16_t u = uint16_t(packed);
    return uint8_t((high ? u >> 8 : u) & 0x7F);
}

std::array<uint8_t, 4> unpack_references(int16_t v2, int16_t v3) {
    if (v3 == calib_registry::no_value) {
        return {};
    }
    return { unpack(v2, false), unpack(v2, true), unpack(v3, false), unpack(v3, true) };
}

} // namespace od3d::vibration
