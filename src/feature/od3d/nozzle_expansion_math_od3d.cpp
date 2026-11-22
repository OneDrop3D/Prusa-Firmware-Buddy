/// @file
#include "nozzle_expansion_math_od3d.hpp"

#include <algorithm>

namespace od3d::nozzle_expansion {

Stage stage_of(std::span<const float> z_mm) {
    if (z_mm.empty()) {
        return {};
    }
    float sum = 0.f, lo = z_mm[0], hi = z_mm[0];
    for (const float z : z_mm) {
        sum += z;
        lo = std::min(lo, z);
        hi = std::max(hi, z);
    }
    return { sum / float(z_mm.size()), hi - lo };
}

Line fit(std::span<const float> x, std::span<const float> y) {
    const size_t n = std::min(x.size(), y.size());
    if (n < 2) {
        return {};
    }
    float mx = 0.f, my = 0.f;
    for (size_t i = 0; i < n; ++i) {
        mx += x[i];
        my += y[i];
    }
    mx /= float(n);
    my /= float(n);
    float sxx = 0.f, sxy = 0.f;
    for (size_t i = 0; i < n; ++i) {
        sxx += (x[i] - mx) * (x[i] - mx);
        sxy += (x[i] - mx) * (y[i] - my); // centré : pas de perte de précision sur z ~ 0,3 mm et T ~ 200
    }
    if (!(sxx > 0.f)) {
        return {};
    }
    const float slope = sxy / sxx;
    return { slope, my - slope * mx, true };
}

Outcome evaluate(const std::array<Stage, stages> &s) {
    Outcome o;
    std::array<float, stages> t, z;
    for (size_t i = 0; i < stages; ++i) {
        t[i] = float(temperatures[i]);
        z[i] = s[i].z_mm;
        o.spread_mm = std::max(o.spread_mm, s[i].spread_mm);
    }
    const Line line = fit(t, z);
    o.slope_mm_per_c = line.slope;
    o.intercept_mm = line.intercept;
    for (size_t i = 0; i < stages; ++i) {
        o.residual_mm = std::max(o.residual_mm, std::fabs(z[i] - (line.slope * t[i] + line.intercept)));
    }
    o.hysteresis_mm = ((z[4] - z[0]) + (z[3] - z[1])) / 2.f;

    // NaN (palier manquant) : jamais accepté
    if (!(o.spread_mm <= max_spread_mm)) {
        o.verdict = Verdict::spread;
    } else if (!line.valid || !(o.residual_mm <= max_residual_mm)) {
        o.verdict = Verdict::residual;
    } else if (!(line.slope >= min_slope_mm_per_c && line.slope <= max_slope_mm_per_c)) {
        o.verdict = Verdict::slope;
    } else {
        o.verdict = Verdict::ok;
    }
    return o;
}

} // namespace od3d::nozzle_expansion
