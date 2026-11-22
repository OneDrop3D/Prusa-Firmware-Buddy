/// @file
#include "damping_math_od3d.hpp"
#include "belt_tension_math_od3d.hpp" // parabolic_offset

#include <cmath>

namespace od3d::damping {

Resonance analyse(std::span<const float> psd, float start_hz, float step_hz) {
    Resonance r;
    const size_t n = psd.size();
    if (n < 3 || !(step_hz > 0.f)) {
        return r;
    }
    size_t top = 0;
    for (size_t i = 0; i < n; ++i) {
        if (!std::isfinite(psd[i])) {
            return r; // mesure incomplète
        }
        if (psd[i] > psd[top]) {
            top = i;
        }
    }
    if (top == 0 || top + 1 == n || !(psd[top] > 0.f)) {
        return r; // sommet au bord : le pic dépasse le balayage
    }

    const float left = psd[top - 1], centre = psd[top], right = psd[top + 1];
    const float offset = belts::parabolic_offset(left, centre, right);
    const float half = 0.5f * (centre - 0.25f * (left - right) * offset); // moitié du sommet de la parabole

    // Coupure de gauche : dernier point sous la moitié avant le sommet, interpolée avec le suivant
    size_t j = top;
    while (j > 0 && psd[j - 1] >= half) {
        --j;
    }
    if (j == 0) {
        return r;
    }
    const float f_left = start_hz + step_hz * (float(j - 1) + (half - psd[j - 1]) / (psd[j] - psd[j - 1]));

    size_t k = top;
    while (k + 1 < n && psd[k + 1] >= half) {
        ++k;
    }
    if (k + 1 == n) {
        return r;
    }
    const float f_right = start_hz + step_hz * (float(k) + (psd[k] - half) / (psd[k] - psd[k + 1]));

    r.frequency = start_hz + step_hz * (float(top) + offset);
    r.width = f_right - f_left;
    if (!(r.width > 0.f) || !(r.frequency > 0.f)) {
        return r;
    }
    r.q = r.frequency / r.width;
    r.zeta = 0.5f / r.q;
    r.found = true;
    return r;
}

} // namespace od3d::damping
