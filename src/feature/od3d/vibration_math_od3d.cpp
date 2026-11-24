/// @file
#include "vibration_math_od3d.hpp"
#include "speed_map_math_od3d.hpp" // median

#include <algorithm>
#include <cmath>
#include <numbers>

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

float ramp_for(float frequency) {
    return std::isfinite(frequency) && frequency > 0.f ? std::min(0.5f / frequency, ramp_s) : ramp_s;
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

void ToneBank::start(const std::array<float, fan_bins> &frequencies, float period) {
    for (size_t k = 0; k < fan_bins; ++k) {
        const float w = 2.f * std::numbers::pi_v<float> * frequencies[k] * period;
        bins[k] = Bin { .step_re = std::cos(w), .step_im = std::sin(w) };
    }
    n = 0;
}

void ToneBank::add(float x, float y, float z) {
    // shortcut: phaseurs jamais renormalisés (en float, ~1e-4 d'écart après les 1 344 pas d'une fenêtre de 1 s) ; à
    // renormaliser pour des fenêtres bien plus longues
    const float a[3] = { x, y, z };
    for (Bin &b : bins) {
        for (int i = 0; i < 3; ++i) {
            b.sum[2 * i] += a[i] * b.re;
            b.sum[2 * i + 1] += a[i] * b.im;
        }
        const float re = b.re * b.step_re - b.im * b.step_im;
        b.im = b.re * b.step_im + b.im * b.step_re;
        b.re = re;
    }
    ++n;
}

std::array<float, fan_bins> ToneBank::magnitudes() const {
    std::array<float, fan_bins> m {};
    for (size_t k = 0; k < fan_bins && n; ++k) {
        float s = 0.f;
        for (const float v : bins[k].sum) {
            s += v * v;
        }
        m[k] = 2.f * std::sqrt(s) / float(n);
    }
    return m;
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
    if (!(v3 & calibrated_mark)) { // no_value (0x8000) n'a pas la marque
        return {};
    }
    return { unpack(v2, false), unpack(v2, true), unpack(v3, false), unpack(v3, true) };
}

// ---- Pose de l'accéléromètre (M2005) ----

float tilt_degrees(float gx, float gy, float gz) {
    const float norm = std::sqrt(gx * gx + gy * gy + gz * gz);
    if (!(norm > 0.f)) {
        return NAN;
    }
    return std::acos(std::clamp(std::abs(gz) / norm, 0.f, 1.f)) * 180.f / 3.14159265f;
}

float plane_share(std::span<const float, 3> sums) {
    const float total = sums[0] + sums[1] + sums[2];
    return total > 0.f ? (sums[0] + sums[1]) / total : NAN;
}

Mount mount_verdict(float tilt_deg, float share_x, float share_y) {
    if (!(tilt_deg <= mount_tilt_max_deg)) { // NAN : pas de pesanteur, compté de travers
        return Mount::tilted;
    }
    if (!(share_x >= mount_share_min) || !(share_y >= mount_share_min)) {
        return Mount::loose;
    }
    return Mount::ok;
}

// ---- Vérification de l'input shaper (M2006) ----

float shaper_reduction(float with_filter, float without_filter, float noise) {
    if (!std::isfinite(with_filter) || !std::isfinite(without_filter) || !std::isfinite(noise) || noise < 0.f) {
        return NAN;
    }
    if (without_filter < reference_ratio * noise || !(without_filter > noise)) {
        return NAN; // même sans filtre, dans le bruit
    }
    const float net_with = std::max(with_filter - noise, 0.f);
    const float net_without = without_filter - noise;
    return std::clamp(1.f - net_with / net_without, -1.f, 1.f);
}

ShaperCheck shaper_check(bool has_filter, float reduction) {
    if (!has_filter) {
        return ShaperCheck::no_filter;
    }
    if (std::isnan(reduction)) {
        return ShaperCheck::quiet;
    }
    return reduction >= check_min_reduction ? ShaperCheck::ok : ShaperCheck::weak;
}

bool shaper_check_passed(std::span<const ShaperCheck> axes) {
    bool filtered = false;
    for (const ShaperCheck c : axes) {
        if (c == ShaperCheck::weak) {
            return false;
        }
        filtered = filtered || c != ShaperCheck::no_filter;
    }
    return filtered;
}

} // namespace od3d::vibration
