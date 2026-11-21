/// @file
#include "spool_calib_math_od3d.hpp"
#include "pa_math_od3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace od3d::calib {

// ---- F5 ----

float Knee::reference(float flow) const {
    return flow > 0 ? a * std::pow(flow, n) : 0.f;
}

Knee find_knee(std::span<const StepPoint> pts, bool final, const KneeThresholds &th) {
    Knee k;
    const size_t count = pts.size();
    constexpr size_t max_ref = 16;
    const size_t ref = std::min<size_t>(th.ref_points, max_ref);
    k.index = uint8_t(std::min<size_t>(count, 255));
    if (ref < 2 || count < ref + 1) {
        k.fit = KneeFit::too_few_points;
        return k;
    }

    // Signe de la force : celui du plus fort palier
    float peak = 0;
    for (const StepPoint &p : pts) {
        if (std::isfinite(p.force_g) && std::fabs(p.force_g) > std::fabs(peak)) {
            peak = p.force_g;
        }
    }
    if (!(std::fabs(peak) >= th.min_force_g)) {
        k.fit = KneeFit::weak_signal;
        return k;
    }
    k.sign = peak < 0 ? -1.f : 1.f;
    const auto force = [&](size_t i) { return k.sign * pts[i].force_g; };

    // Loi de référence sur les premiers paliers : log f = log a + n·log Q
    std::array<float, max_ref> lx, ly;
    for (size_t i = 0; i < ref; ++i) {
        const float q = pts[i].x, f = force(i);
        if (!(q > 0) || !(f > 0) || !std::isfinite(f)) {
            k.fit = KneeFit::weak_signal;
            return k;
        }
        lx[i] = std::log(q);
        ly[i] = std::log(f);
    }
    const pa::LineFit line = pa::fit_line(std::span<const float>(lx.data(), ref), std::span<const float>(ly.data(), ref));
    if (!line.ok) {
        k.fit = KneeFit::too_few_points;
        return k;
    }
    k.n = std::clamp(line.b, th.min_exponent, th.max_exponent);
    float log_a = line.a;
    if (k.n != line.b) {
        // Exposant borné : l'ordonnée à l'origine est refaite avec lui (moyenne des écarts)
        float sum = 0;
        for (size_t i = 0; i < ref; ++i) {
            sum += ly[i] - k.n * lx[i];
        }
        log_a = sum / float(ref);
    }
    k.a = std::exp(log_a);

    const auto excess = [&](size_t i) { return force(i) / k.reference(pts[i].x) - 1.f; };
    // L'extrudeur saute : la force chute nettement depuis le dernier palier fiable, ou devient très bruitée
    const auto dropped = [&](size_t from, size_t i) { return force(from) >= th.min_force_g && force(i) < force(from) * (1.f - th.drop); };
    const auto noisy = [&](size_t i) { return force(i) >= th.min_force_g && pts[i].noise_g > th.max_noise * force(i); };

    size_t good = ref - 1; ///< dernier palier fiable (un point isolé au-dessus de la loi n'en est pas un)
    for (size_t i = ref; i < count; ++i) {
        if (dropped(good, i) || noisy(i)) {
            k.fit = KneeFit::found;
            k.skipping = true;
            k.flow = pts[i - 1].x;
            k.index = uint8_t(i);
            return k;
        }
        const float e = excess(i);
        if (!(e > th.excess)) {
            good = i;
            continue;
        }
        if (i + 1 < count) {
            // Point isolé (bruit) : le palier suivant est revenu sur la loi, sans chute ni bruit ; on continue
            const size_t j = i + 1;
            if (!(excess(j) > th.excess) && !noisy(j) && force(j) >= k.reference(pts[j].x) * (1.f - th.drop)) {
                continue;
            }
        } else if (!final) {
            k.fit = KneeFit::not_reached;
            k.pending = true;
            k.flow = pts[i - 1].x;
            k.index = uint8_t(i);
            return k;
        }
        // Débit où le dépassement vaut le seuil, entre le palier précédent et celui-ci
        const float e_prev = excess(i - 1);
        float t = e - e_prev > 1e-6f ? (th.excess - e_prev) / (e - e_prev) : 1.f;
        t = std::clamp(t, 0.f, 1.f);
        k.fit = KneeFit::found;
        k.flow = pts[i - 1].x + t * (pts[i].x - pts[i - 1].x);
        k.index = uint8_t(i);
        return k;
    }
    k.fit = KneeFit::not_reached;
    k.flow = pts[count - 1].x;
    return k;
}

// ---- F6 ----

MinTemp find_min_temperature(std::span<const StepPoint> pts, float floor, const MinTempThresholds &th) {
    MinTemp m;
    const size_t count = pts.size();
    m.index = uint8_t(std::min<size_t>(count, 255));
    if (count < 2) {
        m.fit = MinTempFit::too_few_points;
        return m;
    }
    const float f0 = pts[0].force_g;
    if (!std::isfinite(f0) || !(std::fabs(f0) >= th.min_force_g)) {
        m.fit = MinTempFit::weak_signal;
        return m;
    }
    m.sign = f0 < 0 ? -1.f : 1.f;
    m.reference_g = std::fabs(f0);
    for (size_t i = 1; i < count; ++i) {
        const float f = m.sign * pts[i].force_g;
        const bool too_hard = !std::isfinite(f) || f > th.max_ratio * m.reference_g;
        const bool too_noisy = pts[i].noise_g > th.max_noise * std::max(f, m.reference_g);
        if (too_hard || too_noisy) {
            m.fit = MinTempFit::found;
            m.temperature = pts[i - 1].x;
            m.index = uint8_t(i);
            return m;
        }
    }
    m.temperature = pts[count - 1].x;
    m.fit = pts[count - 1].x <= floor + 0.5f ? MinTempFit::floor_reached : MinTempFit::not_reached;
    return m;
}

size_t temperature_steps(int16_t start, int16_t floor, int16_t step, std::span<int16_t> out) {
    size_t n = 0;
    if (step <= 0 || start <= floor) {
        return 0;
    }
    for (int t = start; t > floor && n < out.size(); t -= step) {
        out[n++] = int16_t(t);
    }
    if (n < out.size()) {
        out[n++] = floor;
    }
    return n;
}

// ---- Mémoire ----

uint16_t flow_to_e1(float flow) {
    if (!std::isfinite(flow) || flow < 0) {
        return flow_none;
    }
    return uint16_t(std::min(std::lround(flow * 10.f), long(flow_none - 1)));
}

float flow_from_e1(uint16_t e1) {
    return e1 == flow_none ? NAN : float(e1) * 0.1f;
}

float e_speed_limit(float max_flow, float floor, float ceiling, float filament_area_mm2) {
    if (!std::isfinite(max_flow) || !(max_flow > 0) || !(filament_area_mm2 > 0)) {
        return 0.f;
    }
    return std::clamp(max_flow, floor, ceiling) / filament_area_mm2;
}

} // namespace od3d::calib
