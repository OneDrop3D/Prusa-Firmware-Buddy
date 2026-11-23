/// @file
#include "belt_tension_math_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace od3d::belts {

float parabolic_offset(float left, float centre, float right) {
    const float denominator = left - 2.f * centre + right;
    if (!(denominator < 0.f)) {
        return 0.f; // pas un sommet (droite ou creux)
    }
    const float offset = 0.5f * (left - right) / denominator;
    return std::clamp(offset, -0.5f, 0.5f);
}

Peak find_peak(std::span<const float> values, float start_hz, float step_hz, float window_min_hz, float window_max_hz, float min_relative) {
    Peak best;
    const size_t n = values.size();
    if (n < 3 || !(step_hz > 0.f)) {
        return best;
    }
    constexpr float epsilon = 1e-3f;
    float best_prominence = -1.f;
    size_t best_index = 0;
    for (size_t i = 1; i + 1 < n; ++i) {
        const float frequency = start_hz + float(i) * step_hz;
        if (frequency < window_min_hz - epsilon || frequency > window_max_hz + epsilon) {
            continue;
        }
        const float v = values[i];
        // Sommet : plus haut que le point de gauche, au moins aussi haut que celui de droite (un plateau compte une fois)
        if (!std::isfinite(v) || !(v > 0.f) || !(v > values[i - 1]) || !(v >= values[i + 1])) {
            continue;
        }
        float left_min = v;
        for (size_t j = i; j-- > 0;) {
            if (values[j] > v) {
                break;
            }
            left_min = std::min(left_min, values[j]);
        }
        float right_min = v;
        for (size_t j = i + 1; j < n; ++j) {
            if (values[j] > v) {
                break;
            }
            right_min = std::min(right_min, values[j]);
        }
        const float prominence = v - std::max(left_min, right_min);
        if (prominence > best_prominence) {
            best_prominence = prominence;
            best_index = i;
        }
    }
    if (best_prominence < 0.f) {
        return best;
    }
    const float v = values[best_index];
    const float offset = parabolic_offset(values[best_index - 1], v, values[best_index + 1]);
    best.frequency = start_hz + (float(best_index) + offset) * step_hz;
    best.height = v;
    best.relative_prominence = std::clamp(best_prominence / v, 0.f, 1.f);
    best.found = best.relative_prominence >= min_relative;
    return best;
}

Peak belt_peak(std::span<const float> values, float start_hz, float step_hz) {
    const float end_hz = start_hz + float(values.size() ? values.size() - 1 : 0) * step_hz;
    Peak p = find_peak(values, start_hz, step_hz, start_hz, end_hz, min_peak_relative);
    if (p.found && (p.frequency < start_hz + edge_guard_hz || p.frequency > end_hz - edge_guard_hz)) {
        p.found = false; // collé à un bord : la vraie résonance est peut-être au-delà
    }
    return p;
}

Zone zone(float frequency, const Targets &targets) {
    if (frequency < targets.result_min) {
        return Zone::low;
    }
    if (frequency > targets.result_max) {
        return Zone::high;
    }
    return Zone::ok;
}

float revs_from_freq(float df1, float df2, float k1, float k2) {
    // manual_belt_tuning::calc_revs_from_freq de Prusa
    return ((df2 * k2) - (df1 * k1)) / ((k2 * k2) - (k1 * k1));
}

Advice advise(float freq_a, float freq_b, const Targets &t) {
    // Même enchaînement que ManualBeltTuningWizard::run_from_gantry() de Prusa
    if (std::fabs(freq_a - freq_b) > t.diff_limit) {
        return { Verdict::misaligned, 0 };
    }
    const float freq_higher = std::fmax(freq_a, freq_b);
    const float freq_lower = std::fmin(freq_a, freq_b);
    int eighths = 0;
    if (freq_higher > t.result_max || freq_lower < t.result_min) {
        const float dfh = t.higher_optimal - freq_higher;
        const float dfl = t.lower_optimal - freq_lower;
        const float drh = revs_from_freq(dfh, dfl, t.hz_per_rev, t.hz_per_rev2);
        const float drl = revs_from_freq(dfl, dfh, t.hz_per_rev, t.hz_per_rev2);
        const float dr = (drh + drl) / 2.f;
        eighths = int(std::lround(dr * 8.f));
    }
    eighths = std::clamp(eighths, -64, 64); // 8 tours au plus : au-delà, la mesure est sans doute fausse
    return { eighths == 0 ? Verdict::ok : Verdict::adjust, int8_t(eighths) };
}

float live_start(float center_hz, float lo_hz, float hi_hz, float step_hz) {
    const float span_hz = float(live_points - 1) * step_hz;
    const float start = std::round((center_hz - span_hz / 2.f - lo_hz) / step_hz) * step_hz + lo_hz;
    return std::isfinite(start) ? std::clamp(start, lo_hz, std::max(lo_hz, hi_hz - span_hz)) : lo_hz;
}

float live_peak(std::span<const float> values, float start_hz, float step_hz) {
    if (values.empty()) {
        return NAN;
    }
    const size_t top = size_t(std::max_element(values.begin(), values.end()) - values.begin());
    if (top == 0 || top + 1 == values.size()) {
        return start_hz + float(top) * step_hz; // au bord : la fenêtre suivante s'y recentre
    }
    return start_hz + (float(top) + parabolic_offset(values[top - 1], values[top], values[top + 1])) * step_hz;
}

uint8_t live_streak(uint8_t streak, float freq_a, float freq_b, const Targets &targets) {
    const bool ok = zone(freq_a, targets) == Zone::ok && zone(freq_b, targets) == Zone::ok;
    return ok ? uint8_t(std::min(streak + 1, 255)) : 0;
}

void format_turns(int eighths, char *out, size_t size) {
    if (size == 0) {
        return;
    }
    const int a = eighths < 0 ? -eighths : eighths;
    const int turns = a / 8;
    int rest = a % 8;
    int denominator = 8;
    while (rest != 0 && rest % 2 == 0) { // 4/8 → 1/2, 2/8 → 1/4
        rest /= 2;
        denominator /= 2;
    }
    if (rest == 0) {
        snprintf(out, size, "%d", turns);
    } else if (turns == 0) {
        snprintf(out, size, "%d/%d", rest, denominator);
    } else {
        snprintf(out, size, "%d %d/%d", turns, rest, denominator);
    }
}

int16_t gauge_x(float frequency, float lo, float hi, int16_t width) {
    if (width <= 1 || !(hi > lo) || !std::isfinite(frequency)) {
        return 0;
    }
    const float t = std::clamp((frequency - lo) / (hi - lo), 0.f, 1.f);
    return int16_t(std::lround(t * float(width - 1)));
}

int16_t curve_height(std::span<const float> values, size_t valid, int x, int w, int h, float scale) {
    valid = std::min(valid, values.size());
    if (valid == 0 || w <= 0 || h <= 0 || x < 0 || x >= w) {
        return -1;
    }
    const size_t n = values.size();
    // Colonne x → position dans les points (0 à n - 1), les deux bords du tracé sur le premier et le dernier point
    const float pos = n > 1 && w > 1 ? float(x) * float(n - 1) / float(w - 1) : 0.f;
    if (pos > float(valid - 1) + 1e-4f) {
        return -1;
    }
    const size_t i = std::min(size_t(pos), valid - 1);
    const float frac = pos - float(i);
    const float a = values[i];
    const float b = i + 1 < valid ? values[i + 1] : a;
    float v = a + (b - a) * frac;
    if (!std::isfinite(v) || !(scale > 0.f)) {
        v = 0.f;
    }
    const float t = std::clamp(v / (scale > 0.f ? scale : 1.f), 0.f, 1.f);
    return int16_t(std::lround(t * float(h - 1)));
}

int16_t plot_x(float frequency, float start_hz, float end_hz, int w) {
    return gauge_x(frequency, start_hz, end_hz, int16_t(w));
}

} // namespace od3d::belts
