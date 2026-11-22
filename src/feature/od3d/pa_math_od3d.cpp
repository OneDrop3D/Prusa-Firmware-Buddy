/// @file
/// Méthode de PrusaPATuner (CNC Kitchen, AGPL-3.0), réécrite d'après son étude : aucun code repris.
#include "pa_math_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace od3d::pa {

using force::Trace;

Profile profile(const Params &p) {
    return {
        .e_slow = p.slow_flow / filament_area_mm2,
        .e_fast = p.fast_flow / filament_area_mm2,
        .v_slow = p.slow_s > 0 ? p.wiggle / p.slow_s : 0.f,
        .v_fast = p.fast_s > 0 ? p.wiggle / p.fast_s : 0.f,
        .slow_s = p.slow_s,
        .fast_s = p.fast_s,
    };
}

float block_duration_s(const Params &p) {
    return p.slow_s + float(p.cycles) * (p.fast_s + p.slow_s);
}

bool is_valid(const Params &p) {
    const auto in = [](float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; };
    return in(p.slow_flow, 0.5f, 10.f) && in(p.fast_flow, 2.f, 40.f) && p.fast_flow >= 2.f * p.slow_flow && in(p.slow_s, 0.6f, 4.f)
        && in(p.fast_s, 0.2f, 2.f) && in(p.wiggle, 0.2f, 5.f) && in(p.accel, 100.f, 10000.f) && p.cycles >= 3 && p.cycles <= 8
        && block_duration_s(p) <= max_block_s;
}

float median(std::span<float> v) {
    if (v.empty()) {
        return NAN;
    }
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

// ---- Fronts ----

size_t find_turns(Trace t, float threshold_mm, std::span<Turn> out) {
    size_t n = 0;
    if (t.size() < 3 || out.empty() || !(threshold_mm > 0)) {
        return 0;
    }
    const int32_t thr = std::max<int32_t>(1, int32_t(std::lround(threshold_mm / force::x_unit_mm)));
    // Hystérésis : sens engagé une fois X parti de thr ; un demi-tour est l'extrême atteint avant un retour de thr
    int dir = 0;
    size_t extreme = 0;
    for (size_t i = 1; i < t.size(); ++i) {
        const int32_t x = t[i].x;
        if (dir == 0) {
            if (x - t[extreme].x >= thr) {
                dir = 1;
            } else if (t[extreme].x - x >= thr) {
                dir = -1;
            } else {
                continue;
            }
            extreme = i;
            continue;
        }
        if ((dir > 0 && x >= t[extreme].x) || (dir < 0 && x <= t[extreme].x)) {
            extreme = i;
        } else if (std::abs(x - t[extreme].x) >= thr) {
            if (n >= out.size()) {
                break;
            }
            out[n] = { uint16_t(std::min<size_t>(extreme, UINT16_MAX)), n % 2 == 0 };
            ++n;
            dir = -dir;
            extreme = i;
        }
    }
    return n;
}

float command_flow(std::span<const Turn> turns, size_t i, const Params &p) {
    bool fast = false;
    for (const Turn &turn : turns) {
        if (turn.index > i) {
            break;
        }
        fast = turn.rise;
    }
    return fast ? p.fast_flow : p.slow_flow;
}

// ---- Analyse d'un créneau ----

namespace {

    constexpr size_t max_cycles = 8;
    constexpr size_t median_points = 128; ///< fenêtres plus longues sous-échantillonnées pour la médiane (pile)

    /// La trace vue comme force orientée (sens de la cellule) et tarée par une droite (base au point base_at, pente
    /// base_slope par point)
    struct View {
        Trace t;
        float dt;
        float sign;
        float base = 0;
        float base_slope = 0;
        long base_at = 0;

        long size() const { return long(t.size()); }
        float raw(long i) const { return sign * force::force_g(t[size_t(i)]); }
        float y(long i) const { return raw(i) - base - base_slope * float(i - base_at); }
        void untare() {
            base = 0;
            base_slope = 0;
            base_at = 0;
        }
        long points(float seconds) const { return long(std::lround(seconds / dt)); }

        /// Médiane de y sur [a, b) borné à la trace ; NAN si moins de min_n points
        float median_of(long a, long b, long min_n = 3) const {
            a = std::max(a, 0L);
            b = std::min(b, size());
            if (b - a < min_n) {
                return NAN;
            }
            std::array<float, median_points> buf;
            const long stride = (b - a + long(median_points) - 1) / long(median_points);
            size_t n = 0;
            for (long i = a; i < b && n < buf.size(); i += stride) {
                buf[n++] = y(i);
            }
            return median(std::span(buf.data(), n));
        }

        /// Écart type de y sur [a, b)
        float sd_of(long a, long b) const {
            a = std::max(a, 0L);
            b = std::min(b, size());
            if (b - a < 2) {
                return NAN;
            }
            double s = 0, s2 = 0;
            for (long i = a; i < b; ++i) {
                const double v = double(y(i));
                s += v;
                s2 += v * v;
            }
            const double n = double(b - a), m = s / n;
            return float(std::sqrt(std::max(0.0, s2 / n - m * m)));
        }

        /// Pente (moindres carrés) de y sur [a, b) [g/s] et écart type des résidus
        void slope_of(long a, long b, float &slope, float &resid_sd) const {
            slope = NAN;
            resid_sd = NAN;
            if (b - a < 4) {
                return;
            }
            double st = 0, sy = 0;
            for (long i = a; i < b; ++i) {
                st += double(i);
                sy += double(y(i));
            }
            const double n = double(b - a), mt = st / n, my = sy / n;
            double stt = 0, sty = 0;
            for (long i = a; i < b; ++i) {
                stt += (double(i) - mt) * (double(i) - mt);
                sty += (double(i) - mt) * (double(y(i)) - my);
            }
            if (stt <= 0) {
                return;
            }
            const double b1 = sty / stt;
            double r2 = 0;
            for (long i = a; i < b; ++i) {
                const double r = double(y(i)) - (my + b1 * (double(i) - mt));
                r2 += r * r;
            }
            slope = float(b1 / double(dt));
            resid_sd = float(std::sqrt(r2 / n));
        }

        /// ∫ y dt (abs : ∫ |y − level| dt) sur [a, b)
        float integral(long a, long b, bool abs = false, float level = 0) const {
            a = std::max(a, 0L);
            b = std::min(b, size());
            double s = 0;
            for (long i = a; i < b; ++i) {
                const float v = y(i) - level;
                s += double(abs ? std::fabs(v) : v);
            }
            return float(s * double(dt));
        }
    };

    /// Pic attendu du bruit seul sur n points (espérance du maximum d'un bruit gaussien) : σ·√(2 ln n)
    float noise_peak(float sigma, long n) {
        return sigma * std::sqrt(2.f * std::log(float(std::max(n, 2L))));
    }

    /// Un créneau : montée r, descente f, fin de la fenêtre end (exclue) ; v.base est mis à la tare locale
    Segment analyse_segment(View &v, long r, long f, long end, const Profile &p, float k) {
        Segment s;
        const float S = p.slow_s, F = p.fast_s;
        end = std::min(end, v.size());
        if (end - f < 4) {
            return s;
        }

        // Tare locale : médiane du palier lent avant la montée (R1, sans ses bords), comme PrusaPATuner ; ici en plus
        // une droite jusqu'à la fin du palier lent suivant (réponse établie), qui retire la dérive de la cellule au
        // sein du créneau : sans elle, quelques g/s de dérive faussent l'aire de la descente
        v.untare();
        const long b0 = r - v.points(0.85f * S), b1 = r - v.points(0.05f * S);
        if (b0 < 0 || b1 - b0 < 4) {
            return s;
        }
        const float before = v.median_of(b0, b1);
        const long a0 = f + v.points(0.55f * S), a1 = std::min(f + v.points(0.95f * S), end);
        const float after = v.median_of(a0, a1, 4);
        v.base = before;
        v.base_at = (b0 + b1 - 1) / 2;
        if (std::isfinite(after)) {
            v.base_slope = (after - before) / float((a0 + a1 - 1) / 2 - v.base_at);
        }
        const float sigma = v.sd_of(b0, b1);
        s.noise_g = sigma;

        // Palier rapide grossier (seconde moitié), pour placer les seuils de 90 % et 10 %
        const float coarse = v.median_of(r + v.points(0.5f * F), f - v.points(0.05f * F));
        if (!(coarse > 0)) {
            return s;
        }
        const float hi90 = 0.9f * coarse, lo10 = 0.1f * coarse;

        // Fin de la montée : 90 % atteints (au plus la moitié du palier rapide)
        long rise_end = -1;
        for (long i = r; i <= std::min(r + v.points(0.5f * F), f); ++i) {
            if (v.y(i) >= hi90) {
                rise_end = i;
                break;
            }
        }
        // Début réel de la descente : premier passage sous 90 % qui dure 50 ms (avec un PA faible, la force quitte le
        // palier avant la descente commandée)
        long fall_start = -1;
        {
            const long lo = std::max(rise_end >= 0 ? rise_end + v.points(0.05f) : r + v.points(0.3f * F), f - v.points(0.3f * F));
            const long hi = std::min(f + v.points(0.1f * F), end - 1);
            const long sustain = std::max(1L, v.points(0.05f));
            for (long i = std::max(lo, r); i <= hi; ++i) {
                if (v.y(i) >= hi90) {
                    continue;
                }
                bool held = true;
                for (long j = i + 1; j <= std::min(i + sustain, end - 1); ++j) {
                    if (v.y(j) >= hi90) {
                        held = false;
                        break;
                    }
                }
                if (held) {
                    fall_start = i;
                    break;
                }
            }
        }
        const long fa = fall_start >= 0 ? fall_start : f;
        // Fin de la descente : retour sous 10 % (au plus la moitié du palier lent)
        long fall_end = -1;
        for (long i = fa; i <= std::min(fa + v.points(0.5f * S), end - 1); ++i) {
            if (v.y(i) <= lo10) {
                fall_end = i;
                break;
            }
        }

        // Pic juste après la montée (un vrai dépassement retombe en 50 à 100 ms)
        long peak = r;
        const long peak_hi = std::min(rise_end >= 0 ? rise_end + v.points(0.1f * F) : r + v.points(0.2f * F), f);
        for (long i = r; i <= peak_hi; ++i) {
            if (v.y(i) > v.y(peak)) {
                peak = i;
            }
        }

        // Palier rapide : de la fin de la montée (plus une marge) au début de la descente (moins une marge)
        const long pl_lo = rise_end >= 0 ? rise_end + v.points(std::max(0.03f, 0.1f * F)) : r + v.points(0.5f * F);
        const long pl_hi = fa - v.points(std::max(0.02f, 0.1f * F));
        const float high = v.median_of(pl_lo, pl_hi);
        if (!(high > 0)) {
            return s;
        }
        s.high_g = high;
        float slope, resid;
        v.slope_of(pl_lo, pl_hi, slope, resid);
        s.m[plateau_slope] = std::isfinite(slope) ? slope : 0.f;
        const float sigma_plateau = std::max(pl_hi - pl_lo >= 6 && std::isfinite(resid) ? resid : sigma, sigma);

        s.m[overshoot] = v.y(peak) - high - noise_peak(sigma_plateau, peak_hi - r + 1);
        s.m[rise_delay] = float(peak - r) * v.dt;
        s.m[rise_error] = v.integral(r, fa, true, high);

        // Creux après la descente
        long trough = fa;
        const long trough_hi = std::min(fall_end >= 0 ? fall_end + v.points(0.3f * S) : f + v.points(0.5f * S), end - 1);
        for (long i = fa; i <= trough_hi; ++i) {
            if (v.y(i) < v.y(trough)) {
                trough = i;
            }
        }
        s.m[fall_delay] = float(trough - fa) * v.dt;
        s.m[undershoot] = -v.y(trough) - noise_peak(sigma, trough_hi - fa + 1);
        s.fall_area = v.integral(fa, end);
        s.k_estimate = k + s.fall_area / high;

        // Traîne : de la fin de la descente (plus une marge) à la fin de la fenêtre
        const long tail_lo = fall_end >= 0 ? fall_end + v.points(std::max(0.03f, 0.05f * S)) : trough + v.points(0.1f * S);
        s.m[tail] = v.integral(tail_lo, end - v.points(0.05f * S), true);

        // Retour au calme : premier séjour de 0,1 s à ±3 σ après le creux ; jamais : toute la fenêtre (valeur bornée,
        // là où PrusaPATuner écarterait le K)
        {
            const float tol = std::max(3.f * sigma, 1e-3f);
            const long hold = std::max(1L, v.points(0.1f));
            long run_start = -1;
            float settle = float(end - f) * v.dt;
            for (long i = trough; i < end; ++i) {
                if (std::fabs(v.y(i)) >= tol) {
                    run_start = -1;
                    continue;
                }
                if (run_start < 0) {
                    run_start = i;
                }
                if (i - run_start >= hold) {
                    settle = float(run_start - f) * v.dt;
                    break;
                }
            }
            s.m[settling] = settle;
        }

        // Écarté : créneau noyé dans le bruit, palier lent trop agité, creux démesuré (pointe parasite)
        s.ok = high >= 3.f * sigma && sigma <= 0.3f * high && std::fabs(v.y(trough)) <= 5.f * high;
        for (const float m : s.m) {
            s.ok = s.ok && std::isfinite(m);
        }
        return s;
    }

    int8_t to_q6(float v) {
        if (!std::isfinite(v)) {
            return 0;
        }
        return int8_t(std::clamp(std::lround(v * 64.f), -127L, 127L));
    }

} // namespace

BlockResult analyse_block(Trace t, float dt, const Params &p, float k, Waveform *wave) {
    BlockResult out;
    if (wave) {
        *wave = {};
    }
    {
        size_t clipped = 0;
        for (const force::TracePoint &point : t) {
            clipped += point.force >= 32767 || point.force <= -32767;
        }
        out.saturated = clipped * 100 > t.size();
    }
    if (t.size() < 32 || !(dt > 0) || !is_valid(p)) {
        return out;
    }
    const Profile prof = profile(p);
    const long S = std::lround(p.slow_s / dt), F = std::lround(p.fast_s / dt);

    // 1. Créneaux : montée puis descente, durées proches de celles commandées
    std::array<Turn, 2 * max_cycles + 4> turns;
    const size_t n_turns = find_turns(t, 0.25f * p.wiggle, turns);
    struct Pair {
        long rise, fall, end;
    };
    std::array<Pair, max_cycles> pairs;
    size_t n_pairs = 0;
    for (size_t i = 0; i + 1 < n_turns && n_pairs < pairs.size(); i += 2) {
        const long r = turns[i].index, f = turns[i + 1].index;
        if (!turns[i].rise || f - r < F / 2 || f - r > F * 8 / 5 + 2) {
            break; // le motif n'est plus celui attendu : on garde ce qui précède
        }
        if (n_pairs > 0 && (r - pairs[n_pairs - 1].fall < S / 2 || r - pairs[n_pairs - 1].fall > S * 8 / 5 + 2)) {
            break;
        }
        pairs[n_pairs++] = { r, f, 0 };
    }
    out.found = uint8_t(n_pairs);
    if (n_pairs == 0) {
        return out;
    }
    for (size_t i = 0; i < n_pairs; ++i) {
        // Fenêtre jusqu'à la montée suivante (moins 30 ms : l'avance lissée du PA y commence), ou un palier lent
        const long next = i + 1 < n_pairs ? pairs[i + 1].rise : pairs[i].fall + S;
        pairs[i].end = std::min<long>(next - std::lround(0.03f / dt), long(t.size()));
    }

    // 2. Sens de la cellule : palier rapide au-dessus ou au-dessous du palier lent
    View v { t, dt, 1.f };
    {
        std::array<float, max_cycles> d;
        size_t n = 0;
        for (size_t i = 0; i < n_pairs; ++i) {
            const float lo = v.median_of(pairs[i].rise - S / 2, pairs[i].rise - S / 20);
            const float hi = v.median_of(pairs[i].rise + F / 2, pairs[i].fall - F / 20);
            if (std::isfinite(lo) && std::isfinite(hi)) {
                d[n++] = hi - lo;
            }
        }
        const float delta = median(std::span(d.data(), n));
        if (!std::isfinite(delta) || delta == 0.f) {
            return out;
        }
        v.sign = delta > 0 ? 1.f : -1.f;
    }

    // 3. Créneaux analysés ; le premier est écarté (il garde l'histoire du K précédent), sauf s'il n'y en a que 2
    std::array<Segment, max_cycles> segs;
    size_t used = 0;
    const size_t first = n_pairs >= 3 ? 1 : 0;
    std::array<float, Waveform::points> wave_force {}, wave_command {};
    const float wave_step = (0.1f + p.fast_s + 0.45f * p.slow_s) / float(Waveform::points);
    for (size_t i = first; i < n_pairs; ++i) {
        const Segment s = analyse_segment(v, pairs[i].rise, pairs[i].fall, pairs[i].end, prof, k);
        if (!s.ok) {
            continue;
        }
        segs[used++] = s;
        if (wave) {
            // v.base vaut la tare de ce créneau
            for (size_t j = 0; j < Waveform::points; ++j) {
                const long idx = pairs[i].rise + std::lround((float(j) * wave_step - 0.1f) / dt);
                if (idx >= 0 && idx < long(t.size())) {
                    wave_force[j] += v.y(idx) / s.high_g;
                    wave_command[j] += (idx >= pairs[i].rise && idx < pairs[i].fall) ? 1.f : 0.f;
                }
            }
        }
    }
    out.cycles = uint8_t(used);
    if (used == 0) {
        return out;
    }

    // 4. Médianes
    std::array<float, max_cycles> buf;
    const auto med = [&](auto get) {
        for (size_t i = 0; i < used; ++i) {
            buf[i] = get(segs[i]);
        }
        return median(std::span(buf.data(), used));
    };
    for (uint8_t m = 0; m < metric_count; ++m) {
        out.m[m] = med([m](const Segment &s) { return s.m[m]; });
    }
    out.amplitude_g = v.sign * med([](const Segment &s) { return s.high_g; });
    out.noise_g = med([](const Segment &s) { return s.noise_g; });
    out.fall_area = med([](const Segment &s) { return s.fall_area; });
    out.k_estimate = med([](const Segment &s) { return s.k_estimate; });
    if (wave) {
        wave->step_s = wave_step;
        wave->pre_s = 0.1f;
        for (size_t j = 0; j < Waveform::points; ++j) {
            wave->force[j] = to_q6(wave_force[j] / float(used));
            wave->command[j] = to_q6(wave_command[j] / float(used));
        }
    }
    return out;
}

// ---- Choix de K ----

LineFit fit_line(std::span<const float> xs, std::span<const float> ys) {
    LineFit f;
    const size_t n = std::min(xs.size(), ys.size());
    if (n < 2) {
        return f;
    }
    double sx = 0, sy = 0;
    for (size_t i = 0; i < n; ++i) {
        sx += double(xs[i]);
        sy += double(ys[i]);
    }
    const double mx = sx / double(n), my = sy / double(n);
    double sxx = 0, sxy = 0, syy = 0;
    for (size_t i = 0; i < n; ++i) {
        const double dx = double(xs[i]) - mx, dy = double(ys[i]) - my;
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
    }
    if (sxx <= 0) {
        return f;
    }
    f.b = float(sxy / sxx);
    f.a = float(my - double(f.b) * mx);
    f.r2 = syy > 0 ? float((sxy * sxy) / (sxx * syy)) : 1.f;
    f.ok = true;
    return f;
}

float parabola_vertex(float x0, float y0, float x1, float y1, float x2, float y2) {
    // Différences divisées : y = y0 + d1·(x − x0) + d2·(x − x0)(x − x1)
    const float d01 = (y1 - y0) / (x1 - x0);
    const float d12 = (y2 - y1) / (x2 - x1);
    const float d2 = (d12 - d01) / (x2 - x0);
    if (!std::isfinite(d2) || std::fabs(d2) < 1e-12f) {
        return x1;
    }
    // Dérivée nulle : d01 + d2·(2x − x0 − x1) = 0
    return 0.5f * (x0 + x1) - d01 / (2.f * d2);
}

namespace {

    constexpr size_t max_sweep = 32;

    /// Centile q (0 à 1) par interpolation linéaire, comme numpy (tableau trié)
    float percentile_sorted(std::span<const float> v, float q) {
        if (v.empty()) {
            return NAN;
        }
        const float pos = q * float(v.size() - 1);
        const size_t i = size_t(pos);
        if (i + 1 >= v.size()) {
            return v.back();
        }
        return v[i] + (pos - float(i)) * (v[i + 1] - v[i]);
    }

    /// Zéro de y(x) au changement de signe + → − le plus central, par une droite de Theil-Sen (médiane des pentes) sur
    /// au plus 5 points de chaque côté ; NAN sans changement de signe
    float signed_crossing(std::span<const float> xs, std::span<const float> ys) {
        const size_t n = xs.size();
        std::array<size_t, max_sweep> flips;
        size_t n_flips = 0;
        for (size_t i = 0; i + 1 < n; ++i) {
            if (ys[i] > 0 && ys[i + 1] <= 0) {
                flips[n_flips++] = i;
            }
        }
        if (n_flips == 0) {
            return NAN;
        }
        std::array<float, max_sweep> fk;
        for (size_t i = 0; i < n_flips; ++i) {
            fk[i] = xs[flips[i]];
        }
        const float mid = median(std::span(fk.data(), n_flips));
        size_t cross = flips[0];
        for (size_t i = 0; i < n_flips; ++i) {
            if (std::fabs(xs[flips[i]] - mid) < std::fabs(xs[cross] - mid)) {
                cross = flips[i];
            }
        }
        const size_t lo = cross >= 4 ? cross - 4 : 0;
        const size_t hi = std::min(n, cross + 6);
        std::array<float, 64> slopes; // au plus 10 points : 45 pentes
        size_t m = 0;
        for (size_t i = lo; i < hi; ++i) {
            for (size_t j = i + 1; j < hi && m < slopes.size(); ++j) {
                if (xs[j] != xs[i]) {
                    slopes[m++] = (ys[j] - ys[i]) / (xs[j] - xs[i]);
                }
            }
        }
        const float slope = median(std::span(slopes.data(), m));
        if (!std::isfinite(slope) || slope == 0.f) {
            return NAN;
        }
        std::array<float, max_sweep> resid;
        for (size_t i = lo; i < hi; ++i) {
            resid[i - lo] = ys[i] - slope * xs[i];
        }
        const float intercept = median(std::span(resid.data(), hi - lo));
        return std::clamp(-intercept / slope, xs[lo], xs[hi - 1]);
    }

} // namespace

Choice choose_k(std::span<const SweepPoint> sweep, float k_step, const Thresholds &th, std::span<float> costs) {
    Choice c;
    c.k_segments = NAN;
    c.k_signed = NAN;
    for (float &x : costs) {
        x = NAN;
    }
    for (const SweepPoint &s : sweep) {
        if (s.r.saturated) {
            c.fit = Fit::saturated; // un créneau écrêté fausserait toutes les grandeurs : refus explicite
            return c;
        }
    }

    // 1. K exploitables, dans le sens majoritaire de la cellule
    std::array<size_t, max_sweep> idx;
    size_t n = 0, weak = 0;
    int polarity = 0;
    for (const SweepPoint &s : sweep) {
        if (s.r.valid() && s.r.cycles >= th.min_segments) {
            polarity += s.r.amplitude_g > 0 ? 1 : -1;
        }
    }
    for (size_t i = 0; i < sweep.size() && n < idx.size(); ++i) {
        const BlockResult &r = sweep[i].r;
        if (!r.valid() || r.cycles < th.min_segments) {
            continue;
        }
        if (std::fabs(r.amplitude_g) < th.min_amplitude_g || r.noise_g > th.max_noise * std::fabs(r.amplitude_g) || (r.amplitude_g > 0) != (polarity >= 0)) {
            ++weak;
            continue;
        }
        idx[n++] = i;
    }
    if (n < 4) {
        c.fit = weak > 0 ? Fit::weak_signal : Fit::too_few_points;
        return c;
    }
    // Tri par K (insertion : peu de points)
    for (size_t i = 1; i < n; ++i) {
        for (size_t j = i; j > 0 && sweep[idx[j]].k < sweep[idx[j - 1]].k; --j) {
            std::swap(idx[j], idx[j - 1]);
        }
    }

    // 2. Coût composé : chaque critère ramené à son échelle robuste (90e centile des valeurs absolues, ou le maximum si
    // le critère est presque partout nul), pondéré ; dépassement et creux négatifs comptés nuls, pente en valeur absolue
    std::array<float, max_sweep> ks, cost {}, tmp;
    for (size_t i = 0; i < n; ++i) {
        ks[i] = sweep[idx[i]].k;
    }
    for (uint8_t m = 0; m < metric_count; ++m) {
        for (size_t i = 0; i < n; ++i) {
            tmp[i] = std::fabs(sweep[idx[i]].r.m[m]);
        }
        std::sort(tmp.begin(), tmp.begin() + n);
        const float p90 = percentile_sorted(std::span(tmp.data(), n), 0.9f), mx = tmp[n - 1];
        const float scale = p90 > 0.05f * mx ? p90 : mx;
        if (!(scale > 0)) {
            continue;
        }
        for (size_t i = 0; i < n; ++i) {
            float v = sweep[idx[i]].r.m[m] / scale;
            if (m == overshoot || m == undershoot) {
                v = std::max(0.f, v);
            } else if (m == plateau_slope) {
                v = std::fabs(v);
            }
            cost[i] += weights[m] * v;
        }
    }
    for (size_t i = 0; i < n; ++i) {
        if (idx[i] < costs.size()) {
            costs[idx[i]] = cost[i];
        }
    }

    // 3. Minimum, affiné par la parabole de ses deux voisins si elle est convexe (bornée à eux)
    size_t best = 0;
    for (size_t i = 1; i < n; ++i) {
        if (cost[i] < cost[best]) {
            best = i;
        }
    }
    float k_cost = ks[best];
    if (best > 0 && best + 1 < n) {
        const float x0 = ks[best - 1], x1 = ks[best], x2 = ks[best + 1];
        const float d2 = ((cost[best + 1] - cost[best]) / (x2 - x1) - (cost[best] - cost[best - 1]) / (x1 - x0)) / (x2 - x0);
        if (d2 > 0 && x0 < x1 && x1 < x2) {
            k_cost = std::clamp(parabola_vertex(x0, cost[best - 1], x1, cost[best], x2, cost[best + 1]), x0, x2);
        }
    }
    std::copy_n(cost.begin(), n, tmp.begin());
    const float cost_median = median(std::span(tmp.data(), n));
    c.depth = cost_median > 0 ? (cost_median - cost[best]) / cost_median : 0.f;

    // 4. Contrôles croisés : estimations par créneau, changement de signe de l'aire de la descente
    std::array<float, max_sweep> areas;
    for (size_t i = 0; i < n; ++i) {
        tmp[i] = sweep[idx[i]].r.k_estimate;
        areas[i] = sweep[idx[i]].r.fall_area;
    }
    c.k_segments = median(std::span(tmp.data(), n));
    c.k_signed = signed_crossing(std::span(ks.data(), n), std::span(areas.data(), n));

    c.k_cost = k_cost;
    c.k = std::max(0.f, k_cost);
    c.used = uint8_t(n);

    // 5. Confiance : creux net du coût, accord des contrôles croisés, minimum à l'intérieur du balayage, assez de K
    const float step = k_step > 0 ? k_step : 0.01f;
    float confidence = 100.f;
    confidence -= std::max(0.f, 0.35f - c.depth) * 150.f;
    for (const float other : { c.k_segments, c.k_signed }) {
        if (std::isfinite(other)) {
            confidence -= std::min(25.f, std::max(0.f, std::fabs(other - c.k) / step - 1.5f) * 10.f);
        } else {
            confidence -= 10.f;
        }
    }
    if (best == 0 && ks[0] > 0.f) {
        confidence -= 25.f; // minimum au bord bas : le vrai K est peut-être plus bas
    }
    if (n < 6) {
        confidence -= 10.f;
    }
    c.confidence = uint8_t(std::clamp(confidence, 0.f, 100.f));
    if (c.depth < th.min_depth) {
        c.fit = Fit::no_crossing;
    } else if (best + 1 == n) {
        c.fit = Fit::out_of_range;
    } else {
        c.fit = Fit::ok;
    }
    return c;
}

size_t sweep_values(float lo, float hi, float step, std::span<float> out) {
    size_t n = 0;
    if (!(step > 0) || !(hi >= lo)) {
        return 0;
    }
    for (int i = 0; n < out.size(); ++i) {
        const float k = lo + float(i) * step;
        if (k > hi + 1e-6f) {
            break;
        }
        out[n++] = k;
    }
    return n;
}

size_t refine_values(float center, float step, uint8_t count, float lo, float hi, std::span<float> out) {
    size_t n = 0;
    if (!(step > 0) || count == 0) {
        return 0;
    }
    // Les points restent dans [lo, hi] : la grille glisse si le centre est près d'un bord
    const float span = step * float(count - 1);
    float first = center - 0.5f * span;
    first = std::min(first, hi - span);
    first = std::max(first, lo);
    for (uint8_t i = 0; i < count && n < out.size(); ++i) {
        const float k = std::round((first + float(i) * step) * 1e4f) / 1e4f;
        if (k > hi + 1e-6f) {
            break;
        }
        if (n == 0 || k > out[n - 1] + 1e-6f) {
            out[n++] = k;
        }
    }
    return n;
}

float interpolate(std::span<const float> xs, std::span<const float> ys, float x) {
    const size_t n = std::min(xs.size(), ys.size());
    if (n == 0) {
        return NAN;
    }
    if (n == 1 || x <= xs[0]) {
        return ys[0];
    }
    if (x >= xs[n - 1]) {
        return ys[n - 1];
    }
    for (size_t i = 1; i < n; ++i) {
        if (x <= xs[i]) {
            const float w = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
            return ys[i - 1] + w * (ys[i] - ys[i - 1]);
        }
    }
    return ys[n - 1];
}

// ---- CSV ----

namespace {

    /// Lit les nombres séparés par des virgules après le préfixe « X, » (au plus out.size()) ; rend leur nombre, 0 si
    /// la ligne ne convient pas (préfixe, champ vide ou non numérique, texte en trop)
    size_t parse_numbers(const char *line, char tag, std::span<float> out) {
        if (!line || line[0] != tag || line[1] != ',') {
            return 0;
        }
        const char *p = line + 2;
        size_t n = 0;
        while (n < out.size()) {
            char *end = nullptr;
            out[n] = strtof(p, &end);
            if (end == p || !std::isfinite(out[n])) {
                return 0;
            }
            ++n;
            p = end;
            if (*p != ',') {
                break;
            }
            ++p;
        }
        while (*p == '\r' || *p == '\n' || *p == ' ') {
            ++p;
        }
        return *p == '\0' ? n : 0;
    }

} // namespace

int format_params(char *buf, size_t size, const Params &p) {
    return snprintf(buf, size, "P,%.3f,%.3f,%.3f,%.3f,%.3f,%.0f,%u\n", double(p.slow_flow), double(p.slow_s), double(p.fast_flow), double(p.fast_s),
        double(p.wiggle), double(p.accel), unsigned(p.cycles));
}

bool parse_params(const char *line, Params &p) {
    std::array<float, 7> v;
    if (parse_numbers(line, 'P', v) != v.size() || v[6] < 1.f || v[6] > 255.f) {
        return false;
    }
    p = { .slow_flow = v[0], .fast_flow = v[2], .slow_s = v[1], .fast_s = v[3], .wiggle = v[4], .accel = v[5], .cycles = uint8_t(std::lround(v[6])) };
    return true;
}

int format_point(char *buf, size_t size, const CsvPoint &p) {
    return snprintf(buf, size, "D,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f\n", double(p.k), double(p.t_s), double(p.x_mm), double(p.e_mm), double(p.force_g), double(p.command));
}

bool parse_point(const char *line, CsvPoint &p) {
    std::array<float, 6> v;
    switch (parse_numbers(line, 'D', v)) {
    case 6:
        p = { .k = v[0], .t_s = v[1], .x_mm = v[2], .e_mm = v[3], .force_g = v[4], .command = v[5] };
        return true;
    case 4: // ancien format (motif imprimé)
        p = { .k = v[0], .t_s = v[1], .x_mm = v[2], .e_mm = 0.f, .force_g = v[3], .command = NAN };
        return true;
    default:
        return false;
    }
}

uint16_t k_to_e4(float k) {
    if (!std::isfinite(k) || k < 0) {
        return k_none;
    }
    return uint16_t(std::min(std::lround(k * 1e4f), long(k_none - 1)));
}

float k_from_e4(uint16_t e4) {
    return e4 == k_none ? NAN : float(e4) * 1e-4f;
}

} // namespace od3d::pa
