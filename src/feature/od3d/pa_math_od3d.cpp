/// @file
/// Méthode inspirée de PrusaPATuner (CNC Kitchen, AGPL-3.0), réimplémentée d'après sa description : aucun code repris.
#include "pa_math_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace od3d::pa {

using force::Trace;

Profile profile(const Params &p) {
    return {
        .v_lo = pattern::speed_for_flow(p.bead, p.slow_flow),
        .v_hi = pattern::speed_for_flow(p.bead, p.fast_flow),
        .slow_s = p.slow_s,
        .fast_s = p.fast_s,
    };
}

float line_length(const Params &p) {
    const Profile prof = profile(p);
    return prof.v_lo * p.slow_s * float(p.cycles + 1) + prof.v_hi * p.fast_s * float(p.cycles);
}

bool is_valid(const Params &p) {
    const auto in = [](float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; };
    if (!pattern::is_valid(p.bead) || !in(p.slow_flow, 0.3f, 10.f) || !in(p.fast_flow, 2.f, 40.f) || p.fast_flow < 2.f * p.slow_flow
        || !in(p.slow_s, 0.4f, 3.f) || !in(p.fast_s, 0.1f, 1.f) || p.cycles < 2 || p.cycles > 8) {
        return false;
    }
    return profile(p).v_hi <= max_line_speed && line_length(p) <= max_line_length;
}

float block_duration_s(const Params &p) {
    return p.slow_s + float(p.cycles) * (p.fast_s + p.slow_s);
}

float median(std::span<float> v) {
    if (v.empty()) {
        return NAN;
    }
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

namespace {

    constexpr size_t max_cycles = 8;

    /// Indice borné, depuis un indice de base et un décalage en points (arrondi)
    long offset(size_t base, float points) {
        return long(base) + std::lround(points);
    }

    struct Cycle {
        size_t rise;
        size_t fall;
    };

    int8_t to_q6(float v) {
        if (!std::isfinite(v)) {
            return 0;
        }
        return int8_t(std::clamp(std::lround(v * 64.f), -127L, 127L));
    }

} // namespace

// Jamais incorporée à la mesure d'une ligne : son cadre (~1,3 Ko) ne vit que le temps de l'analyse, pas pendant les
// attentes de la ligne (idle(), au fond de la pile de Marlin : débordement en 8.0.2.24)
[[gnu::noinline]] BlockResult analyse_block(Trace t, float dt, const Profile &p, Waveform *wave) {
    BlockResult out;
    if (t.size() < 16 || !(dt > 0) || !(p.v_hi > p.v_lo) || !(p.slow_s > 0) || !(p.fast_s > 0)) {
        return out;
    }
    const long n = long(t.size());
    const float n_slow = p.slow_s / dt; // points d'un palier lent

    // 1. Fronts de vitesse, puis créneaux : une montée suivie d'une descente, d'une durée proche du palier rapide
    std::array<force::Edge, 2 * max_cycles + 6> edges;
    const uint8_t min_run = uint8_t(std::clamp(std::lround(0.02f / dt), 2L, 20L));
    const size_t n_edges = force::find_edges(t, dt, 0.5f * (p.v_lo + p.v_hi), min_run, edges);
    std::array<Cycle, max_cycles> cycles;
    size_t n_cycles = 0;
    for (size_t e = 0; e + 1 < n_edges && n_cycles < max_cycles; ++e) {
        if (!edges[e].rising || edges[e + 1].rising) {
            continue;
        }
        const float fast = float(edges[e + 1].index - edges[e].index) * dt;
        if (fast >= 0.5f * p.fast_s && fast <= 2.f * p.fast_s) {
            cycles[n_cycles++] = { edges[e].index, edges[e + 1].index };
        }
    }

    // 2. Niveaux de chaque créneau : palier lent avant la montée et avant la montée suivante (dérive lente retirée par
    // une droite entre les deux), fin du palier rapide
    struct Level {
        float b0, c0, b1, c1; // ligne de base : (c0, b0) — (c1, b1)
        float amplitude;
        float noise;
        bool ok;
        float base(float k) const { return b0 + (b1 - b0) * (k - c0) / (c1 - c0); }
    };
    std::array<Level, max_cycles> levels {};
    std::array<float, max_cycles> amplitudes;
    std::array<float, max_cycles> noises;
    size_t n_levels = 0;
    for (size_t c = 0; c < n_cycles; ++c) {
        Level &l = levels[c];
        const Cycle &cy = cycles[c];
        const long pre0 = offset(cy.rise, -0.35f * n_slow), pre1 = offset(cy.rise, -0.05f * n_slow);
        // Après la descente : le tronçon lent dure slow_s, puis vient la montée suivante (ou la fin de la ligne)
        const long post0 = offset(cy.fall, 0.65f * n_slow), post1 = offset(cy.fall, 0.95f * n_slow);
        const long hi0 = offset(cy.fall, -0.35f * float(cy.fall - cy.rise)), hi1 = offset(cy.fall, -0.03f * float(cy.fall - cy.rise));
        if (pre0 < 0 || post1 > n || hi1 <= hi0) {
            continue;
        }
        l.b0 = force::mean_force(t, pre0, pre1);
        l.c0 = 0.5f * float(pre0 + pre1 - 1);
        l.b1 = force::mean_force(t, post0, post1);
        l.c1 = 0.5f * float(post0 + post1 - 1);
        const float h = force::mean_force(t, hi0, hi1);
        l.amplitude = h - l.base(0.5f * float(hi0 + hi1 - 1));
        l.noise = force::noise_force(t, pre0, pre1);
        l.ok = std::isfinite(l.amplitude) && std::isfinite(l.b0) && std::isfinite(l.b1);
        if (l.ok) {
            amplitudes[n_levels] = l.amplitude;
            noises[n_levels] = std::isfinite(l.noise) ? l.noise : 0.f;
            ++n_levels;
        }
    }
    if (n_levels == 0) {
        return out;
    }
    const float amplitude = median(std::span(amplitudes.data(), n_levels));
    if (!(std::fabs(amplitude) > 1e-3f)) {
        return out;
    }

    // 3. Aires, coût, dépassements, avec la force et la commande normalisées (0 : palier lent, 1 : palier rapide)
    const auto command = [&](long k) { return (force::speed_at(t, size_t(k), dt) - p.v_lo) / (p.v_hi - p.v_lo); };
    const auto response = [&](const Level &l, long k) { return (force::force_at(t, size_t(k)) - l.base(float(k))) / amplitude; };

    std::array<float, max_cycles> lags, costs, overs, unders;
    size_t used = 0;
    if (wave) {
        static constexpr Waveform empty_wave {};
        *wave = empty_wave; // copie : pas de temporaire sur la pile de Marlin
    }
    std::array<float, Waveform::points> wave_force {}, wave_command {};
    const float wave_step = (0.1f + p.fast_s + 0.45f * p.slow_s) / float(Waveform::points);

    for (size_t c = 0; c < n_cycles; ++c) {
        const Level &l = levels[c];
        if (!l.ok) {
            continue;
        }
        const Cycle &cy = cycles[c];
        // Montée : de 0,1 tronçon lent avant le front jusqu'au début de la fin du tronçon rapide (la force y est
        // établie) ; descente : de là jusqu'à la moitié du tronçon lent. Couper au front de descente compterait le
        // début de la décélération (avant le front) avec le signe de la montée.
        const long r0 = std::max(0L, offset(cy.rise, -0.1f * n_slow));
        const long f0 = offset(cy.fall, -0.35f * float(cy.fall - cy.rise));
        const long f1 = std::min(n, offset(cy.fall, 0.45f * n_slow));
        float area_rise = 0, area_fall = 0, cost = 0, peak = -1e9f, dip = 1e9f;
        for (long k = r0; k < f0; ++k) {
            const float d = command(k) - response(l, k);
            area_rise += d;
            cost += d * d;
            if (k >= long(cy.rise)) {
                peak = std::max(peak, response(l, k));
            }
        }
        for (long k = f0; k < f1; ++k) {
            const float r = response(l, k);
            const float d = r - command(k);
            area_fall += d;
            cost += d * d;
            if (k >= long(cy.fall)) {
                dip = std::min(dip, r);
            }
        }
        lags[used] = 0.5f * (area_rise + area_fall) * dt;
        costs[used] = cost * dt;
        overs[used] = std::max(0.f, peak - 1.f);
        unders[used] = std::max(0.f, -dip);
        ++used;

        if (wave) {
            for (size_t j = 0; j < Waveform::points; ++j) {
                const long k = offset(cy.rise, (float(j) * wave_step - 0.1f) / dt);
                if (k >= 0 && k < n) {
                    wave_force[j] += response(l, k);
                    wave_command[j] += command(k);
                }
            }
        }
    }
    if (used == 0) {
        return out;
    }

    out.cycles = uint8_t(used);
    out.lag_s = median(std::span(lags.data(), used));
    out.cost = median(std::span(costs.data(), used));
    out.overshoot = median(std::span(overs.data(), used));
    out.undershoot = median(std::span(unders.data(), used));
    out.amplitude_g = amplitude;
    out.noise = median(std::span(noises.data(), n_levels)) / std::fabs(amplitude);
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

// Jamais incorporée au balayage, pour la même raison (~0,7 Ko)
[[gnu::noinline]] Choice choose_k(std::span<const SweepPoint> sweep, float k_min, float k_max, float k_step, const Thresholds &th) {
    Choice c;
    constexpr size_t max_points = 32;
    std::array<float, max_points> ks, lags, cost_k, costs;
    size_t n = 0, weak = 0;
    for (const SweepPoint &s : sweep) {
        if (!s.r.valid()) {
            continue;
        }
        if (std::fabs(s.r.amplitude_g) < th.min_amplitude_g || s.r.noise > th.max_noise) {
            ++weak;
            continue;
        }
        if (n < max_points) {
            ks[n] = s.k;
            lags[n] = s.r.lag_s;
            ++n;
        }
    }
    if (n < 3) {
        c.fit = weak > 0 ? Fit::weak_signal : Fit::too_few_points;
        return c;
    }

    // Zéro de la droite lag(K), puis second ajustement sur les K proches de ce zéro (la réponse n'est linéaire que
    // près de l'optimum)
    LineFit f = fit_line(std::span(ks.data(), n), std::span(lags.data(), n));
    if (!f.ok || f.b > th.max_slope || f.b < th.min_slope) {
        c.fit = Fit::no_crossing;
        c.slope = f.b;
        return c;
    }
    float k_lag = -f.a / f.b;
    {
        std::array<float, max_points> lk, ll;
        size_t m = 0;
        for (size_t i = 0; i < n; ++i) {
            if (std::fabs(ks[i] - k_lag) <= th.local_span) {
                lk[m] = ks[i];
                ll[m] = lags[i];
                ++m;
            }
        }
        if (m >= 3) {
            const LineFit local = fit_line(std::span(lk.data(), m), std::span(ll.data(), m));
            if (local.ok && local.b <= th.max_slope && local.b >= th.min_slope) {
                f = local;
                k_lag = -f.a / f.b;
            }
        }
    }

    // Zéro encadré : deux K voisins dont le retard change de signe, interpolés linéairement. Plus sûr que la droite
    // quand la réponse n'est plus linéaire loin de l'optimum (écrasement du cordon qui sature) ; on garde l'encadrement
    // le plus proche du zéro de la droite, s'il en est assez près.
    {
        std::array<float, max_points> sk, sl;
        std::copy_n(ks.begin(), n, sk.begin());
        std::copy_n(lags.begin(), n, sl.begin());
        for (size_t i = 1; i < n; ++i) {
            for (size_t j = i; j > 0 && sk[j] < sk[j - 1]; --j) {
                std::swap(sk[j], sk[j - 1]);
                std::swap(sl[j], sl[j - 1]);
            }
        }
        float bracket = NAN, distance = INFINITY;
        for (size_t i = 1; i < n; ++i) {
            const float dk = sk[i] - sk[i - 1];
            if (dk < 1e-6f || !(sl[i - 1] >= 0.f && sl[i] < 0.f)) {
                continue;
            }
            const float z = sk[i - 1] + sl[i - 1] * dk / (sl[i - 1] - sl[i]);
            if (std::fabs(z - k_lag) < distance) {
                distance = std::fabs(z - k_lag);
                bracket = z;
            }
        }
        if (std::isfinite(bracket) && distance <= th.local_span) {
            k_lag = bracket;
        }
    }

    // Contrôle croisé : minimum du coût, affiné par une parabole sur ses voisins
    size_t m = 0;
    for (const SweepPoint &s : sweep) {
        if (s.r.valid() && std::fabs(s.r.amplitude_g) >= th.min_amplitude_g && s.r.noise <= th.max_noise && m < max_points) {
            cost_k[m] = s.k;
            costs[m] = s.r.cost;
            ++m;
        }
    }
    // Tri par K (insertion : peu de points)
    for (size_t i = 1; i < m; ++i) {
        for (size_t j = i; j > 0 && cost_k[j] < cost_k[j - 1]; --j) {
            std::swap(cost_k[j], cost_k[j - 1]);
            std::swap(costs[j], costs[j - 1]);
        }
    }
    size_t best = 0;
    for (size_t i = 1; i < m; ++i) {
        if (costs[i] < costs[best]) {
            best = i;
        }
    }
    float k_cost = cost_k[best];
    if (best > 0 && best + 1 < m && cost_k[best - 1] < cost_k[best] && cost_k[best] < cost_k[best + 1]) {
        k_cost = std::clamp(parabola_vertex(cost_k[best - 1], costs[best - 1], cost_k[best], costs[best], cost_k[best + 1], costs[best + 1]), cost_k[best - 1], cost_k[best + 1]);
    }

    c.slope = f.b;
    c.k_lag = k_lag;
    c.k_cost = k_cost;
    c.used = uint8_t(n);
    c.k = std::max(0.f, k_lag);

    // Confiance : pente nette (ni plate, ni démesurée), droite bien ajustée, accord avec le minimum du coût. Sur une
    // ligne imprimée, l'écrasement du cordon s'ajoute à la pression : la pente dépasse −1 sans que la mesure soit moins
    // sûre.
    const float step = k_step > 0 ? k_step : 0.01f;
    float confidence = 100.f;
    confidence -= 80.f * std::max(0.f, 0.6f + f.b); // pente moins raide que −0,6
    confidence -= 8.f * std::max(0.f, -f.b - 5.f); // plus raide que −5
    confidence -= std::max(0.f, 0.95f - f.r2) * 200.f;
    confidence -= std::max(0.f, std::fabs(k_lag - k_cost) / step - 1.f) * 20.f;
    if (k_lag < 0) {
        confidence -= 30.f;
    }
    c.confidence = uint8_t(std::clamp(confidence, 0.f, 100.f));
    c.fit = (k_lag < k_min - step || k_lag > k_max + step) ? Fit::out_of_range : Fit::ok;
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

uint16_t k_to_e4(float k) {
    if (!std::isfinite(k) || k < 0) {
        return k_none;
    }
    return uint16_t(std::min(std::lround(k * 1e4f), long(k_none - 1)));
}

float k_from_e4(uint16_t e4) {
    return e4 == k_none ? NAN : float(e4) * 1e-4f;
}

uint8_t slots_to_measure(AutoMode mode, uint8_t used, uint8_t known, uint8_t has_k) {
    const uint8_t candidates = used & known;
    switch (mode) {
    case AutoMode::off:
        return 0;
    case AutoMode::first_print:
        return candidates & uint8_t(~has_k);
    case AutoMode::every_print:
        return candidates;
    }
    return 0;
}

std::optional<pattern::Rect> auto_spot(const pattern::Rect &zone, const pattern::Rect &print, float w, float h, std::span<const float> xs,
    std::span<const float> ys, float grow_x, float grow_y, float clearance) {
    constexpr pattern::Rect purge_probe { 150.f, 0.f, 250.f, 20.f };
    const auto overlaps = [](const pattern::Rect &a, const pattern::Rect &b) {
        return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1;
    };
    const pattern::Rect kept = print.grown(clearance, clearance);
    const pattern::Rect kept_purge = purge_probe.grown(clearance, clearance);
    const pattern::Rect probed = print.grown(grow_x, grow_y);
    const auto is_free = [&](const pattern::Rect &r) {
        if (overlaps(r, kept) || overlaps(r, kept_purge)) {
            return false;
        }
        for (const float x : xs) {
            for (const float y : ys) {
                // Comme Prusa (PrintArea::rect_t::contains) : bornes comprises
                const bool tapped = x >= probed.x0 && x <= probed.x1 && y >= probed.y0 && y <= probed.y1;
                if (tapped && overlaps(r, pattern::Rect { x, y, x, y }.grown(clearance, clearance))) {
                    return false;
                }
            }
        }
        return true;
    };
    if (!(w > 0.f) || !(h > 0.f) || w > zone.width() || h > zone.height()) {
        return std::nullopt;
    }
    for (float y = zone.y0; y + h <= zone.y1; y += 1.f) {
        for (const float x : { zone.x0, zone.x1 - w }) {
            if (const pattern::Rect r { x, y, x + w, y + h }; is_free(r)) {
                return r;
            }
        }
    }
    return std::nullopt;
}

namespace {

    /// Ajoute à out depuis n ; false si out est trop court
    [[gnu::format(printf, 3, 4)]] bool append(std::span<char> out, size_t &n, const char *format, ...) {
        if (n >= out.size()) {
            return false;
        }
        va_list args;
        va_start(args, format);
        const int w = vsnprintf(out.data() + n, out.size() - n, format, args);
        va_end(args);
        if (w < 0 || size_t(w) >= out.size() - n) {
            return false;
        }
        n += size_t(w);
        return true;
    }

    bool load_line(std::span<char> out, size_t &n, uint8_t slot, const char *filament) {
        return append(out, n, "M701 S\"%s\" W2 P%u", filament, unsigned(slot));
    }

} // namespace

bool oneclick_sequence(std::span<char> out, const Request &rq, const char *filament) {
    size_t n = 0;
    bool ok = load_line(out, n, rq.slot, filament) && append(out, n, "\nM1994 P%u", unsigned(rq.slot));
    // Seuls les paramètres changés, pour une ligne courte
    const Request d;
    const auto real = [&](char c, float v, float def) {
        ok = ok && (v == def || append(out, n, " %c%.4f", c, double(v)));
    };
    const auto whole = [&](char c, int v, int def) {
        ok = ok && (v == def || append(out, n, " %c%d", c, v));
    };
    real('L', rq.k_min, d.k_min);
    real('H', rq.k_max, d.k_max);
    real('S', rq.k_step, d.k_step);
    whole('R', rq.refine, d.refine);
    real('B', rq.params.slow_flow, d.params.slow_flow);
    real('F', rq.params.fast_flow, d.params.fast_flow);
    real('U', rq.params.slow_s, d.params.slow_s);
    real('D', rq.params.fast_s, d.params.fast_s);
    real('W', rq.params.bead.width, d.params.bead.width);
    real('Z', rq.params.bead.height, d.params.bead.height);
    whole('C', rq.params.cycles, d.params.cycles);
    whole('T', rq.temperature, d.temperature);
    whole('Q', rq.bed_temperature, d.bed_temperature);
    real('X', rq.x, d.x);
    real('Y', rq.y, d.y);
    real('I', rq.pitch, d.pitch);
    whole('O', rq.csv, d.csv);
    return ok && append(out, n, " J1");
}

bool auto_step_sequence(std::span<char> out, uint8_t slot, const char *filament) {
    size_t n = 0;
    return (!filament || (load_line(out, n, slot, filament) && append(out, n, "\n"))) && append(out, n, "M1994 P%u A1", unsigned(slot));
}

bool auto_restore_sequence(std::span<char> out, std::optional<uint8_t> initial, std::optional<uint8_t> loaded, const char *filament) {
    size_t n = 0;
    if (initial == loaded) {
        return append(out, n, "%s", "");
    }
    return initial ? load_line(out, n, *initial, filament) : append(out, n, "M702 W2");
}

} // namespace od3d::pa
