// OneDrop3D (F4) : analyse de la réponse en force et choix du pressure advance, sur des courbes synthétiques de
// lignes imprimées sur le plateau (motif de mesure : lancement lent, puis rapide / lent le long de +X).
// La buse est simulée par le modèle du premier ordre τ·dp/dt + p = débit entrant, le PA de Prusa par
// e_moteur = e + K·(vitesse moyenne de E sur une fenêtre). La force verticale (positive vers le haut) a deux parts :
// la pression dans la buse (réponse non linéaire) et l'écrasement du cordon, qui suit le filament déposé par mm
// (débit sorti / vitesse de la tête, borné : un cordon ne peut pas être négatif) ; plus bruit et dérive lente.
#include <feature/od3d/pa_math_od3d.hpp>
#include <feature/od3d/pattern_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace od3d;
using Catch::Matchers::WithinAbs;

namespace {

struct Nozzle {
    float tau = 0.04f; ///< constante de temps de la buse [s] = K idéal
    float gain_g = 30.f; ///< force de la pression au palier rapide [g] (positive vers le haut ; négative : autre montage)
    float squish_g = 6.f; ///< force d'écrasement par unité d'écart du filament déposé par mm [g]
    float exponent = 0.8f; ///< force ∝ pression^exponent (réponse sous-linéaire des vraies buses)
    float noise_g = 0.4f; ///< bruit blanc [g]
    float drift_g_per_s = 1.5f; ///< dérive lente (cellule qui chauffe, câbles)
    float pa_effect = 1.f; ///< 0 : le PA n'agit pas
    float smooth_s = 0.04f; ///< fenêtre de lissage du PA de Prusa (M572 W)
};

/// Générateur pseudo-aléatoire reproductible (LCG) et loi normale de Box-Muller
struct Random {
    uint32_t state;
    float uniform() {
        state = state * 1664525u + 1013904223u;
        return (float(state >> 8) + 0.5f) / 16777216.f;
    }
    float normal() {
        const float u1 = uniform(), u2 = uniform();
        return std::sqrt(-2.f * std::log(u1)) * std::cos(6.2831853f * u2);
    }
};

/// Simule une ligne de mesure (un K), comme le firmware : 0,3 s d'arrêt, lancement lent, puis cycles × (rapide, lent),
/// dans le sens dir (1 : vers +X, −1 : vers −X), 0,3 s d'arrêt ; échantillonné au pas dt (moyenne de la force et de X
/// sur chaque pas, comme l'interruption de la cellule de charge)
std::vector<force::TracePoint> simulate(const pa::Params &params, const Nozzle &nozzle, float k, float dt, uint32_t seed, float dir = 1.f) {
    const pa::Profile p = pa::profile(params);
    constexpr float h = 0.00025f; // pas de la simulation
    constexpr float accel = 5000.f;
    const float ls = p.v_lo * p.slow_s, lf = p.v_hi * p.fast_s;
    const float e_mm = pattern::e_per_mm(params.bead);

    struct Segment {
        float dist;
        float speed;
    };
    std::vector<Segment> segments { { ls, p.v_lo } };
    for (int c = 0; c < params.cycles; ++c) {
        segments.push_back({ lf, p.v_hi });
        segments.push_back({ ls, p.v_lo });
    }

    // Cinématique : vitesse limitée par l'accélération, jonction à la plus petite des deux vitesses, arrêt à la fin
    std::vector<float> xs, vs;
    float x = 0;
    const auto idle = [&](float s) {
        for (float t = 0; t < s; t += h) {
            xs.push_back(x);
            vs.push_back(0);
        }
    };
    idle(0.3f);
    float v = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        const Segment &s = segments[i];
        const float v_end = i + 1 < segments.size() ? std::min(s.speed, segments[i + 1].speed) : 0.f;
        float done = 0;
        while (done < s.dist) {
            const float remaining = s.dist - done;
            v = std::min({ s.speed, v + accel * h, std::sqrt(v_end * v_end + 2.f * accel * remaining) });
            v = std::max(v, 0.5f);
            done += v * h;
            x += dir * v * h;
            xs.push_back(x);
            vs.push_back(v);
        }
        v = v_end;
    }
    idle(0.3f);

    // PA : vitesse de E moyennée sur smooth_s, puis e_moteur' = e' + K·d(moyenne)/dt ; buse du premier ordre
    const size_t n = vs.size();
    const size_t half = size_t(nozzle.smooth_s / 2 / h);
    std::vector<double> prefix(n + 1, 0);
    for (size_t i = 0; i < n; ++i) {
        prefix[i + 1] = prefix[i] + double(e_mm * vs[i]);
    }
    std::vector<float> mean(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t a = i > half ? i - half : 0, b = std::min(n, i + half + 1);
        mean[i] = float((prefix[b] - prefix[a]) / double(b - a));
    }
    const float ve_hi = e_mm * p.v_hi;
    Random rnd { seed };
    std::vector<force::TracePoint> out;
    float pressure = 0;
    const size_t per_sample = size_t(dt / h + 0.5f);
    double sum_f = 0, sum_x = 0;
    size_t count = 0;
    float f0 = NAN, x0 = NAN;
    for (size_t i = 0; i < n; ++i) {
        const float dmean = i > 0 ? (mean[i] - mean[i - 1]) / h : 0;
        const float inflow = e_mm * vs[i] + nozzle.pa_effect * k * dmean;
        pressure += h / nozzle.tau * (inflow - pressure);
        const float rel = pressure / ve_hi;
        const float shaped = (rel >= 0 ? 1.f : -1.f) * std::pow(std::fabs(rel), nozzle.exponent);
        // Filament déposé par mm, relatif au cordon nominal : 1 quand la sortie suit la tête ; 0 à 3
        const float deposit = vs[i] > 1.f ? std::clamp(pressure / (e_mm * vs[i]), 0.f, 3.f) : 1.f;
        const float t = float(i) * h;
        const float sign = nozzle.gain_g < 0 ? -1.f : 1.f;
        sum_f += nozzle.gain_g * shaped + sign * nozzle.squish_g * (deposit - 1.f) + nozzle.drift_g_per_s * t;
        sum_x += xs[i];
        if (++count == per_sample) {
            const float f = float(sum_f / double(count)) + nozzle.noise_g * rnd.normal();
            const float xm = float(sum_x / double(count));
            if (std::isnan(f0)) {
                f0 = f;
                x0 = xm;
            }
            out.push_back(force::make_point(f - f0, xm - x0));
            sum_f = sum_x = 0;
            count = 0;
        }
    }
    return out;
}

constexpr float dt = 1.f / 160.f; // 320 Hz décimé par 2

/// Balayage complet comme le firmware : grossier, puis fin autour du premier choix
pa::Choice sweep(const pa::Params &params, const Nozzle &nozzle, std::vector<pa::SweepPoint> *points = nullptr) {
    const pa::Profile p = pa::profile(params);
    std::vector<pa::SweepPoint> all;
    std::array<float, 32> ks;
    const size_t n = pa::sweep_values(0.f, 0.10f, 0.02f, ks);
    uint32_t seed = 1;
    for (size_t i = 0; i < n; ++i) {
        const auto trace = simulate(params, nozzle, ks[i], dt, seed++);
        all.push_back({ ks[i], pa::analyse_block(trace, dt, p) });
    }
    const pa::Choice coarse = pa::choose_k(all, 0.f, 0.10f, 0.02f);
    if (coarse.fit == pa::Fit::ok) {
        std::array<float, 8> fine;
        const size_t m = pa::refine_values(coarse.k, 0.005f, 5, 0.f, 0.3f, fine);
        for (size_t i = 0; i < m; ++i) {
            const auto trace = simulate(params, nozzle, fine[i], dt, seed++);
            all.push_back({ fine[i], pa::analyse_block(trace, dt, p) });
        }
    }
    if (points) {
        *points = all;
    }
    return pa::choose_k(all, 0.f, 0.10f, 0.02f);
}

} // namespace

TEST_CASE("PA : profil et bornes de la ligne imprimée") {
    const pa::Params params;
    CHECK(pa::is_valid(params));
    const pa::Profile p = pa::profile(params);
    // 1,5 mm³/s et 10 mm³/s sur un cordon de 0,45 × 0,2 mm : ~16,7 mm/s et ~111 mm/s
    CHECK_THAT(p.v_lo, WithinAbs(16.67, 0.05));
    CHECK_THAT(p.v_hi, WithinAbs(111.1, 0.1));
    CHECK_THAT(pa::block_duration_s(params), WithinAbs(4.1, 1e-4));
    // 4 tronçons lents de 13,3 mm et 3 rapides de 33,3 mm
    CHECK_THAT(pa::line_length(params), WithinAbs(153.3, 0.1));

    pa::Params bad = params;
    bad.fast_flow = 2.5f; // moins de 2 × le débit lent
    CHECK_FALSE(pa::is_valid(bad));
    bad = params;
    bad.cycles = 1;
    CHECK_FALSE(pa::is_valid(bad));
    bad = params;
    bad.bead.height = 0.6f; // pas une première couche
    CHECK_FALSE(pa::is_valid(bad));
    bad = params;
    bad.fast_flow = 30.f; // 333 mm/s : trop vite
    CHECK_FALSE(pa::is_valid(bad));
    bad = params;
    bad.cycles = 6; // ligne de plus de 200 mm
    CHECK_FALSE(pa::is_valid(bad));
}

TEST_CASE("PA : bloc analysé, retard ≈ τ − K") {
    const pa::Params params;
    const pa::Profile p = pa::profile(params);
    Nozzle nozzle;
    nozzle.noise_g = 0.f;
    nozzle.drift_g_per_s = 0.f;
    nozzle.exponent = 1.f;
    nozzle.squish_g = 0.f; // pression seule : retard = τ − K
    for (float k : { 0.f, 0.02f, 0.04f, 0.06f, 0.08f }) {
        pa::Waveform wave;
        const auto trace = simulate(params, nozzle, k, dt, 7);
        const pa::BlockResult r = pa::analyse_block(trace, dt, p, &wave);
        INFO("K = " << k << ", lag = " << r.lag_s);
        CHECK(r.cycles == params.cycles);
        // Créneau de force, vers le haut : tronçon rapide (30 g) moins tronçon lent (1,5/10 de 30 g, réponse linéaire ici)
        CHECK_THAT(r.amplitude_g, WithinAbs(25.5, 1.0));
        CHECK_THAT(r.lag_s, WithinAbs(nozzle.tau - k, 0.006));
        CHECK(wave.step_s > 0);
        // Palier rapide de la commande moyenne à 1 (×64)
        CHECK(std::abs(wave.command[40] - 64) <= 4);
    }
    // Trop peu de PA : pas de dépassement ; trop : dépassement net
    const auto low = pa::analyse_block(simulate(params, nozzle, 0.f, dt, 3), dt, p);
    const auto high = pa::analyse_block(simulate(params, nozzle, 0.10f, dt, 3), dt, p);
    CHECK(low.overshoot < 0.05f);
    CHECK(high.overshoot > 0.25f);
    CHECK(high.undershoot > 0.25f);
}

TEST_CASE("PA : l'écrasement du cordon raidit le retard sans déplacer son zéro") {
    const pa::Params params;
    const pa::Profile p = pa::profile(params);
    Nozzle nozzle;
    nozzle.noise_g = 0.f;
    nozzle.drift_g_per_s = 0.f;
    const auto lag = [&](float squish, float k) {
        Nozzle n = nozzle;
        n.squish_g = squish;
        return pa::analyse_block(simulate(params, n, k, dt, 5), dt, p).lag_s;
    };
    // Même signe des deux côtés du bon K, nul (à peu près) au bon K, plus fort avec l'écrasement
    CHECK(lag(6.f, 0.02f) > lag(0.f, 0.02f));
    CHECK(lag(6.f, 0.06f) < lag(0.f, 0.06f));
    CHECK(lag(0.f, 0.02f) > 0.f);
    CHECK(lag(6.f, 0.06f) < 0.f);
    CHECK_THAT(lag(6.f, nozzle.tau), WithinAbs(0.0, 0.004));
}

TEST_CASE("PA : K retrouvé sur lignes imprimées malgré écrasement, bruit, dérive, non-linéarité et signe inversé") {
    const pa::Params params;
    for (float tau : { 0.025f, 0.04f, 0.065f }) {
        for (float gain : { 30.f, -30.f }) {
            Nozzle nozzle;
            nozzle.tau = tau;
            nozzle.gain_g = gain;
            std::vector<pa::SweepPoint> points;
            const pa::Choice c = sweep(params, nozzle, &points);
            INFO("τ = " << tau << ", gain = " << gain << ", K = " << c.k << ", K coût = " << c.k_cost << ", pente = " << c.slope << ", confiance = " << int(c.confidence));
            CHECK(c.fit == pa::Fit::ok);
            CHECK_THAT(c.k, WithinAbs(tau, 0.003));
            CHECK_THAT(c.k_cost, WithinAbs(tau, 0.006));
            CHECK(c.slope < -0.6f);
            CHECK(c.slope > -8.f);
            CHECK(c.confidence >= 60);
            CHECK(points.size() >= 9);
        }
    }
}

TEST_CASE("PA : refus quand le signal est trop faible ou ne dépend pas de K") {
    const pa::Params params;
    Nozzle weak;
    weak.gain_g = 1.f;
    CHECK(sweep(params, weak).fit == pa::Fit::weak_signal);

    Nozzle deaf;
    deaf.pa_effect = 0.f; // le PA n'agit pas : le retard ne bouge pas avec K
    CHECK(sweep(params, deaf).fit == pa::Fit::no_crossing);

    // Une trace sans mouvement ne donne aucun créneau
    std::vector<force::TracePoint> still(800, force::make_point(0, 0));
    CHECK_FALSE(pa::analyse_block(still, dt, pa::profile(params)).valid());
    CHECK(pa::choose_k({}, 0, 0.1f, 0.02f).fit == pa::Fit::too_few_points);
}

TEST_CASE("PA : K hors de la plage balayée") {
    const pa::Params params;
    Nozzle slow;
    slow.tau = 0.16f;
    slow.squish_g = 0.f; // loin de l'optimum, l'écrasement sature : la droite ne vaut que pour la pression
    const pa::Choice c = sweep(params, slow);
    CHECK(c.fit == pa::Fit::out_of_range);
    CHECK(c.k > 0.12f);
}

TEST_CASE("PA : grilles de K") {
    std::array<float, 32> out;
    REQUIRE(pa::sweep_values(0.f, 0.1f, 0.02f, out) == 6);
    CHECK_THAT(out[5], WithinAbs(0.1, 1e-6));
    CHECK(pa::sweep_values(0.1f, 0.f, 0.02f, out) == 0);
    CHECK(pa::sweep_values(0.f, 0.1f, 0.f, out) == 0);

    REQUIRE(pa::refine_values(0.043f, 0.005f, 5, 0.f, 0.3f, out) == 5);
    CHECK_THAT(out[0], WithinAbs(0.033, 1e-6));
    CHECK_THAT(out[4], WithinAbs(0.053, 1e-6));
    // Près du bord : la grille glisse et reste dans [lo, hi]
    REQUIRE(pa::refine_values(0.002f, 0.005f, 5, 0.f, 0.3f, out) == 5);
    CHECK_THAT(out[0], WithinAbs(0.0, 1e-6));
    CHECK_THAT(out[4], WithinAbs(0.02, 1e-6));
}

TEST_CASE("PA : droite, parabole, interpolation, médiane, mémoire") {
    const float xs[] = { 0.f, 1.f, 2.f, 3.f };
    const float ys[] = { 1.f, 3.f, 5.f, 7.f };
    const pa::LineFit f = pa::fit_line(xs, ys);
    CHECK(f.ok);
    CHECK_THAT(f.a, WithinAbs(1.0, 1e-5));
    CHECK_THAT(f.b, WithinAbs(2.0, 1e-5));
    CHECK_THAT(f.r2, WithinAbs(1.0, 1e-5));
    const float one[] = { 1.f };
    CHECK_FALSE(pa::fit_line(one, one).ok);

    // y = (x − 0,037)² + 2 sur trois K non équidistants
    const auto y = [](float x) { return (x - 0.037f) * (x - 0.037f) + 2.f; };
    CHECK_THAT(pa::parabola_vertex(0.02f, y(0.02f), 0.03f, y(0.03f), 0.05f, y(0.05f)), WithinAbs(0.037, 1e-4));
    CHECK_THAT(pa::parabola_vertex(0.f, 1.f, 1.f, 2.f, 2.f, 3.f), WithinAbs(1.0, 1e-6)); // alignés

    // PA adaptatif (étape suivante) : K selon le débit, borné aux extrémités
    const float flows[] = { 5.f, 10.f, 20.f };
    const float ks[] = { 0.05f, 0.04f, 0.03f };
    CHECK_THAT(pa::interpolate(flows, ks, 7.5f), WithinAbs(0.045, 1e-6));
    CHECK_THAT(pa::interpolate(flows, ks, 15.f), WithinAbs(0.035, 1e-6));
    CHECK_THAT(pa::interpolate(flows, ks, 1.f), WithinAbs(0.05, 1e-6));
    CHECK_THAT(pa::interpolate(flows, ks, 30.f), WithinAbs(0.03, 1e-6));

    float v[] = { 5.f, 1.f, 3.f, 2.f };
    CHECK_THAT(pa::median(v), WithinAbs(2.5, 1e-6));
    CHECK(std::isnan(pa::median({})));

    CHECK(pa::k_to_e4(0.0375f) == 375);
    CHECK_THAT(pa::k_from_e4(375), WithinAbs(0.0375, 1e-6));
    CHECK(pa::k_to_e4(-0.01f) == pa::k_none);
    CHECK(pa::k_to_e4(NAN) == pa::k_none);
    CHECK(std::isnan(pa::k_from_e4(pa::k_none)));
}

TEST_CASE("trace de force : points, vitesse, fronts, bruit") {
    std::vector<force::TracePoint> t;
    // 1 s à 10 mm/s puis 1 s à 50 mm/s, pas de 10 ms
    float x = 0;
    for (int i = 0; i < 200; ++i) {
        x += (i < 100 ? 10.f : 50.f) * 0.01f;
        t.push_back(force::make_point(i < 100 ? 0.f : 20.f, x));
    }
    CHECK_THAT(force::speed_at(t, 50, 0.01f), WithinAbs(10.0, 0.01));
    CHECK_THAT(force::speed_at(t, 150, 0.01f), WithinAbs(50.0, 0.01));
    std::array<force::Edge, 4> edges;
    REQUIRE(force::find_edges(t, 0.01f, 30.f, 3, edges) == 1);
    CHECK(edges[0].rising);
    CHECK(std::abs(int(edges[0].index) - 100) <= 2);
    CHECK_THAT(force::mean_force(t, 120, 180), WithinAbs(20.0, 1e-4));
    CHECK_THAT(force::noise_force(t, 120, 180), WithinAbs(0.0, 1e-4));
    CHECK(std::isnan(force::mean_force(t, 10, 10)));

    // Bornes des unités : saturation au lieu du débordement
    CHECK(force::make_point(10000.f, 0).force == 32767);
    CHECK(force::make_point(-10000.f, 0).force == -32767);
    CHECK(force::make_point(NAN, NAN).x == 0);
}

TEST_CASE("motif : cordon, repères, lignes, palpage") {
    const pattern::Bead bead { 0.45f, 0.2f };
    CHECK(pattern::is_valid(bead));
    CHECK_FALSE(pattern::is_valid({ 0.2f, 0.2f })); // plus étroit que haut
    CHECK_FALSE(pattern::is_valid({ NAN, 0.2f }));
    CHECK_THAT(pattern::e_per_mm(bead), WithinAbs(0.09 / 2.4053, 1e-5));
    CHECK_THAT(pattern::speed_for_flow(bead, 9.f), WithinAbs(100.0, 1e-3));
    CHECK(pattern::speed_for_flow({ 0.f, 0.f }, 9.f) == 0.f);

    // Numéro 7 : sept traits, le 5e long
    std::array<pattern::Tick, 16> ticks;
    REQUIRE(pattern::tally(7, ticks) == 7);
    CHECK(ticks[4].length == pattern::tick_long);
    CHECK(ticks[3].length == pattern::tick_short);
    CHECK(ticks[6].length == pattern::tick_short);
    CHECK_THAT(ticks[6].dx, WithinAbs(6 * pattern::tick_pitch, 1e-6));
    CHECK(pattern::tally(0, ticks) == 0);
    CHECK(pattern::tally(20, std::span(ticks.data(), 3)) == 3); // borné à la place donnée
    CHECK(pattern::tally_width(0) == 0.f);
    CHECK_THAT(pattern::tally_width(11), WithinAbs(pattern::tick_gap + 10 * pattern::tick_pitch, 1e-6));

    // Motif par défaut de F4 (11 lignes) : il tient à l'avant du plateau de la CORE One (250 × 220, 10 mm de marge)
    const pa::Params params;
    pattern::Lines lines;
    lines.x0 = 20.f;
    lines.y0 = 12.f;
    lines.pitch = 5.f;
    lines.length = pa::line_length(params);
    lines.count = 11;
    lines.marks = pattern::tally_width(lines.count);
    lines.bead = params.bead;
    const pattern::Rect area = lines.area();
    const pattern::Rect zone { 10.f, 10.f, 240.f, 210.f };
    CHECK(zone.contains(area));
    CHECK_THAT(area.y1, WithinAbs(12.0 + 50.0 + pattern::tick_long, 1e-4));
    CHECK_THAT(area.x1, WithinAbs(20.0 + 153.33 + 19.0 + 0.225, 0.05));
    // Trop à gauche ou trop de lignes : refus
    lines.x0 = 5.f;
    CHECK_FALSE(zone.contains(lines.area()));
    lines.x0 = 20.f;
    lines.count = 40;
    CHECK_FALSE(zone.contains(lines.area()));

    // Palpage : la zone agrandie, bornée au plateau
    const pattern::Rect bed { 0.f, 0.f, 250.f, 220.f };
    const pattern::Rect probe = pattern::probe_rect(area, 22.f, 22.f, bed);
    CHECK(probe.x0 == 0.f);
    CHECK(probe.y0 == 0.f);
    CHECK_THAT(probe.x1, WithinAbs(area.x1 + 22.0, 1e-4));
    CHECK(bed.contains(probe));
    CHECK(pattern::Rect { 0.f, 0.f, 10.f, 10.f }.clipped({ 20.f, 20.f, 30.f, 30.f }).empty());
}

TEST_CASE("od3d pa: emplacements de la mesure automatique", "[od3d][pa]") {
    // Impression : emplacements 1 et 3 (bits 0 et 2) ; matières connues : 1, 2, 3 ; K déjà mesuré : 1
    const uint8_t used = 0b00101, known = 0b00111, has_k = 0b00001;
    CHECK(pa::slots_to_measure(pa::AutoMode::off, used, known, has_k) == 0);
    CHECK(pa::slots_to_measure(pa::AutoMode::first_print, used, known, has_k) == 0b00100);
    CHECK(pa::slots_to_measure(pa::AutoMode::every_print, used, known, has_k) == 0b00101);
    // Matière inconnue : jamais mesurée
    CHECK(pa::slots_to_measure(pa::AutoMode::every_print, 0b10000, known, 0) == 0);
    // Tout est déjà mesuré : rien à la 1re impression
    CHECK(pa::slots_to_measure(pa::AutoMode::first_print, used, known, 0b11111) == 0);
}

namespace {

/// Points majeurs de la grille de la CORE One (21 × 21 points, bord 1, un point majeur sur 3)
struct Grid {
    std::vector<float> xs, ys;
    float grow_x, grow_y;
    Grid() {
        const float min_x = -(250.f / 18.f) + 15.f, dist_x = (250.f - 2.f * min_x) / 20.f;
        const float min_y = -(220.f / 18.f) + 15.f, dist_y = (220.f - 2.f * min_y) / 20.f;
        for (int i = 1; i <= 19; i += 3) {
            xs.push_back(min_x + float(i) * dist_x);
            ys.push_back(min_y + float(i) * dist_y);
        }
        grow_x = 3.f * dist_x;
        grow_y = 3.f * dist_y;
    }
};

bool overlap(const pattern::Rect &a, const pattern::Rect &b) {
    return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1;
}

/// La place ne gêne pas l'impression : hors de sa zone, du palpage près de la purge et des points palpés autour
void check_free(const pattern::Rect &r, const pattern::Rect &zone, const pattern::Rect &print, const Grid &g) {
    CHECK(zone.contains(r));
    CHECK_FALSE(overlap(r, print.grown(5.f, 5.f)));
    CHECK_FALSE(overlap(r, pattern::Rect { 145.f, -5.f, 255.f, 25.f }));
    const pattern::Rect probed = print.grown(g.grow_x, g.grow_y);
    for (const float x : g.xs) {
        for (const float y : g.ys) {
            if (x >= probed.x0 && x <= probed.x1 && y >= probed.y0 && y <= probed.y1) {
                CHECK_FALSE(overlap(r, pattern::Rect { x - 5.f, y - 5.f, x + 5.f, y + 5.f }));
            }
        }
    }
}

} // namespace

TEST_CASE("od3d pa: place du motif automatique à côté de l'impression", "[od3d][pa]") {
    const Grid g;
    const pattern::Rect zone { 10.f, 10.f, 240.f, 210.f };
    const float w = 120.f, h = 28.f; // un emplacement : purge et 5 lignes courtes

    SECTION("pièce de 80 mm au centre : devant, à gauche") {
        const pattern::Rect print { 85.f, 70.f, 165.f, 150.f };
        const auto r = pa::auto_spot(zone, print, w, h, g.xs, g.ys, g.grow_x, g.grow_y);
        REQUIRE(r.has_value());
        check_free(*r, zone, print, g);
        CHECK(r->y1 <= print.y0);
        CHECK(r->x0 == zone.x0);
    }
    SECTION("pièce à l'avant : derrière") {
        const pattern::Rect print { 20.f, 0.f, 230.f, 60.f };
        const auto r = pa::auto_spot(zone, print, w, h, g.xs, g.ys, g.grow_x, g.grow_y);
        REQUIRE(r.has_value());
        check_free(*r, zone, print, g);
        CHECK(r->y0 >= print.y1);
    }
    SECTION("pièce sur tout le plateau : pas de place") {
        const pattern::Rect print { 0.f, 0.f, 250.f, 220.f };
        CHECK_FALSE(pa::auto_spot(zone, print, w, h, g.xs, g.ys, g.grow_x, g.grow_y).has_value());
    }
    SECTION("pas de M555 (zone maximale de Prusa) : pas de place") {
        const pattern::Rect print { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max() };
        CHECK_FALSE(pa::auto_spot(zone, print, w, h, g.xs, g.ys, g.grow_x, g.grow_y).has_value());
    }
    SECTION("motif plus large que le plateau : pas de place") {
        const pattern::Rect print { 100.f, 100.f, 110.f, 110.f };
        CHECK_FALSE(pa::auto_spot(zone, print, 300.f, h, g.xs, g.ys, g.grow_x, g.grow_y).has_value());
    }
    SECTION("motif large devant à droite : la zone palpée près de la purge reste libre") {
        const pattern::Rect print { 10.f, 150.f, 60.f, 200.f }; // petite pièce au fond à gauche
        const auto r = pa::auto_spot(zone, print, 200.f, h, g.xs, g.ys, g.grow_x, g.grow_y);
        REQUIRE(r.has_value());
        check_free(*r, zone, print, g);
        CHECK(r->y0 >= 25.f); // au-dessus du palpage de la purge (x 150 à 250, y 0 à 20)
    }
    SECTION("plusieurs emplacements : la hauteur réduit les places") {
        const pattern::Rect print { 85.f, 70.f, 165.f, 150.f };
        CHECK_FALSE(pa::auto_spot(zone, print, w, 3.f * h, g.xs, g.ys, g.grow_x, g.grow_y).has_value());
    }
    SECTION("rien contre un bord : la place libre entre deux blocs déjà pris") {
        const pattern::Rect band { 10.f, 10.f, 240.f, 50.f }; // bande étroite : un bloc par rangée
        const pattern::Rect print { 100.f, 180.f, 150.f, 200.f }; // loin : aucun point palpé dans la bande
        const pattern::Rect taken[] { { 10.f, 10.f, 70.f, 50.f }, { 170.f, 10.f, 240.f, 50.f } };
        const auto r = pa::auto_spot(band, print, 60.f, 27.f, g.xs, g.ys, g.grow_x, g.grow_y, 5.f, taken);
        REQUIRE(r.has_value());
        CHECK(r->x0 == 75.f); // contre le bloc de gauche, à 5 mm
        CHECK(r->y0 == 10.f);
        CHECK(r->x1 <= 145.f); // hors de la zone palpée près de la purge
        for (const pattern::Rect &t : taken) {
            CHECK_FALSE(overlap(*r, t.grown(5.f, 5.f)));
        }
    }
}

namespace {

/// Bloc d'un emplacement de la mesure automatique (comme pa_od3d.cpp) : purge (demi-largeur 3, à 6 mm de la 1re
/// ligne), puis compact_cap lignes du motif compact ; lead : du bord du bloc à la 1re ligne
constexpr float lead = 3.f + 6.f;

pattern::Lines compact_lines(float x, float y) {
    pa::Request rq;
    pa::make_compact(rq);
    pattern::Lines l;
    l.x0 = x;
    l.y0 = y;
    l.pitch = rq.pitch;
    l.length = pa::line_length(rq.params);
    l.count = pa::compact_cap;
    l.bead = rq.params.bead;
    return l;
}

/// Blocs placés : chacun libre (check_free) et à 5 mm au moins des autres
void check_blocks(std::span<const pattern::Rect> blocks, const pattern::Rect &zone, const pattern::Rect &print, const Grid &g) {
    for (size_t i = 0; i < blocks.size(); ++i) {
        check_free(blocks[i], zone, print, g);
        for (size_t j = 0; j < i; ++j) {
            CHECK_FALSE(overlap(blocks[i], blocks[j].grown(5.f, 5.f)));
        }
    }
}

} // namespace

TEST_CASE("od3d pa: un bloc par emplacement, chacun à sa place, tourné s'il le faut", "[od3d][pa]") {
    const Grid g;
    const pattern::Rect zone { 10.f, 10.f, 240.f, 210.f };
    const pattern::Rect one = compact_lines(0.f, lead).area(); // bloc droit en (0, 0)
    const float w = one.width(), h = one.y1;
    REQUIRE(w > 80.f);
    REQUIRE(w < 95.f);
    REQUIRE(h > 20.f);
    REQUIRE(h < 30.f);
    std::array<pattern::Rect, 5> out;
    uint8_t along_y = 0xff;

    SECTION("pièce à l'avant : trois blocs derrière elle, couchés") {
        const pattern::Rect print { 20.f, 0.f, 230.f, 60.f };
        const size_t n = pa::auto_spots(std::span(out.data(), 3), along_y, zone, print, w, h, g.xs, g.ys, g.grow_x, g.grow_y);
        REQUIRE(n == 3);
        CHECK(along_y == 0);
        check_blocks(std::span(out.data(), n), zone, print, g);
        for (size_t i = 0; i < n; ++i) {
            CHECK(out[i].y0 >= print.y1);
            CHECK_THAT(out[i].width(), WithinAbs(w, 1e-4));
        }
        // Le 1er comme avant (une seule bobine : même place que l'ancien bloc unique)
        const auto first = pa::auto_spot(zone, print, w, h, g.xs, g.ys, g.grow_x, g.grow_y);
        REQUIRE(first.has_value());
        CHECK(out[0].x0 == first->x0);
        CHECK(out[0].y0 == first->y0);
    }
    SECTION("pièce large au milieu : devant et derrière, pas une colonne") {
        const pattern::Rect print { 20.f, 60.f, 230.f, 150.f };
        // Une colonne de deux blocs ne tient nulle part (ancien comportement : une seule bobine mesurée)
        CHECK_FALSE(pa::auto_spot(zone, print, w, 2.f * h + 5.f, g.xs, g.ys, g.grow_x, g.grow_y).has_value());
        const size_t n = pa::auto_spots(std::span(out.data(), 3), along_y, zone, print, w, h, g.xs, g.ys, g.grow_x, g.grow_y);
        REQUIRE(n == 3);
        check_blocks(std::span(out.data(), n), zone, print, g);
        CHECK(out[0].y1 <= print.y0);
        CHECK(out[1].y0 >= print.y1);
        CHECK(out[2].y0 >= print.y1);
    }
    SECTION("pièce sur toute la profondeur : blocs debout dans la bande de gauche, le 3e reporté") {
        const pattern::Rect print { 80.f, 0.f, 240.f, 220.f }; // reste 65 mm à gauche : un bloc couché n'y tient pas
        CHECK_FALSE(pa::auto_spot(zone, print, w, h, g.xs, g.ys, g.grow_x, g.grow_y).has_value());
        const size_t n = pa::auto_spots(std::span(out.data(), 3), along_y, zone, print, w, h, g.xs, g.ys, g.grow_x, g.grow_y);
        REQUIRE(n == 2);
        CHECK(along_y == 0b11);
        check_blocks(std::span(out.data(), n), zone, print, g);
        for (size_t i = 0; i < n; ++i) {
            CHECK_THAT(out[i].width(), WithinAbs(h, 1e-4)); // tourné : h × w
            CHECK_THAT(out[i].height(), WithinAbs(w, 1e-4));
            CHECK(out[i].x1 <= print.x0);
        }
    }
    SECTION("rien ne tient : aucun bloc") {
        const pattern::Rect print { 0.f, 0.f, 250.f, 220.f };
        CHECK(pa::auto_spots(out, along_y, zone, print, w, h, g.xs, g.ys, g.grow_x, g.grow_y) == 0);
        CHECK(along_y == 0);
    }
}

TEST_CASE("motif : bloc tourné de 90°, lignes et purge le long de Y", "[od3d][pattern]") {
    const pattern::Rect one = compact_lines(0.f, lead).area();
    // Bloc debout (comme pa_od3d.cpp, auto_lines_of()) : X et Y échangés autour du départ des lignes
    const pattern::Rect b { 20.f, 30.f, 20.f + one.y1, 30.f + one.width() };
    pattern::Lines lines = compact_lines(b.x0 + lead, b.y0 - one.x0);
    lines.along_y = true;
    const pattern::Rect a = lines.area();
    CHECK(b.grown(1e-3f, 1e-3f).contains(a)); // à l'arrondi des flottants près
    CHECK_THAT(a.width(), WithinAbs(one.height(), 1e-4));
    CHECK_THAT(a.height(), WithinAbs(one.width(), 1e-4));
    // Ligne 0 le long de +Y, ligne 1 en sens inverse, 2,5 mm plus loin en X
    const pattern::Point s0 = lines.at(lines.x_start(0), lines.y(0));
    const pattern::Point e0 = lines.at(lines.x_start(0) + lines.dir(0) * lines.length, lines.y(0));
    CHECK(s0.x == e0.x);
    CHECK_THAT(e0.y - s0.y, WithinAbs(lines.length, 1e-4));
    const pattern::Point s1 = lines.at(lines.x_start(1), lines.y(1));
    CHECK_THAT(s1.x - s0.x, WithinAbs(lines.pitch, 1e-4));
    CHECK_THAT(s1.y, WithinAbs(e0.y, 1e-4)); // part d'où la précédente s'arrête
    // Purge (à 6 mm sous la 1re ligne, 50 mm le long des lignes) : dans le bloc, à côté des lignes
    const pattern::Point p = lines.at(lines.x0, lines.y0 - 6.f);
    CHECK(p.x - 3.f >= b.x0);
    CHECK(p.x + 3.f < a.x0);
    CHECK(p.y >= b.y0);
    CHECK(p.y + 50.f <= b.y1);
    // Droit : rien ne change
    lines.along_y = false;
    const pattern::Point q = lines.at(12.f, 34.f);
    CHECK(q.x == 12.f);
    CHECK(q.y == 34.f);
}

TEST_CASE("od3d pa: G-codes du chargement par le MMU, hors de la pile de M1994", "[od3d][pa]") {
    char out[256];
    const auto text = [&] { return std::string(out); };

    SECTION("en un clic : « Charger dans la buse » de Prusa, puis la même mesure avec J1") {
        pa::Request rq;
        rq.slot = 2;
        rq.csv = true;
        REQUIRE(pa::oneclick_sequence(out, rq, "#5"));
        CHECK(text() == "G28 O I\nG27 W2\nM701 S\"#5\" W2 P2\nM1994 P2 V0 O1 J1");

        rq.slot = 0;
        rq.csv = false;
        rq.k_max = 0.2f;
        rq.refine = 0;
        rq.x = 30.f;
        rq.temperature = 230;
        REQUIRE(pa::oneclick_sequence(out, rq, "#12"));
        CHECK(text() == "G28 O I\nG27 W2\nM701 S\"#12\" W2 P0\nM1994 P0 V0 H0.2000 R0 T230 X30.0000 J1");
    }
    SECTION("en un clic, motif compact : plateau chauffé pendant le chargement, réglages du motif compact sous-entendus") {
        pa::Request rq;
        rq.slot = 1;
        pa::make_compact(rq);
        REQUIRE(pa::oneclick_sequence(out, rq, "#5", 60));
        CHECK(text() == "M140 S60\nG28 O I\nG27 W2\nM701 S\"#5\" W2 P1\nM1994 P1 V1 J1");
        rq.params.cycles = 3;
        REQUIRE(pa::oneclick_sequence(out, rq, "#5", 0));
        CHECK(text() == "G28 O I\nG27 W2\nM701 S\"#5\" W2 P1\nM1994 P1 V1 C3 J1");
    }
    SECTION("automatique : chargement si besoin, puis M1994 P<n> A1") {
        REQUIRE(pa::auto_step_sequence(out, 3, "#5"));
        CHECK(text() == "G28 O I\nG27 W2\nM701 S\"#5\" W2 P3\nM1994 P3 A1");
        REQUIRE(pa::auto_step_sequence(out, 3, nullptr));
        CHECK(text() == "M1994 P3 A1");
        // Premier chargement : le plateau palpé avant (M1994 A2), le chargement se pose ensuite sur une place palpée
        REQUIRE(pa::auto_step_sequence(out, 3, "#5", true));
        CHECK(text() == "M1994 A2\nG28 O I\nG27 W2\nM701 S\"#5\" W2 P3\nM1994 P3 A1");
        REQUIRE(pa::auto_step_sequence(out, 3, nullptr, true)); // rien à charger : pas de palpage à part
        CHECK(text() == "M1994 P3 A1");
    }
    SECTION("fin automatique : filament remis comme au départ") {
        REQUIRE(pa::auto_restore_sequence(out, std::nullopt, std::nullopt, ""));
        CHECK(text().empty());
        REQUIRE(pa::auto_restore_sequence(out, 1, 1, "#7"));
        CHECK(text().empty());
        REQUIRE(pa::auto_restore_sequence(out, std::nullopt, 2, ""));
        CHECK(text() == "G28 O I\nG27 W2\nM702 W2");
        REQUIRE(pa::auto_restore_sequence(out, 1, 3, "#7"));
        CHECK(text() == "G28 O I\nG27 W2\nM701 S\"#7\" W2 P1");
        REQUIRE(pa::auto_restore_sequence(out, 1, std::nullopt, "#7"));
        CHECK(text() == "G28 O I\nG27 W2\nM701 S\"#7\" W2 P1");
    }
    SECTION("jamais de M701 ni de M702 avant l'origine et la position de chargement levée") {
        pa::Request rq;
        REQUIRE(pa::oneclick_sequence(out, rq, "#5"));
        CHECK(text().rfind("G28 O I\nG27 W2\nM701 ", 0) == 0);
        REQUIRE(pa::auto_step_sequence(out, 4, "#5"));
        CHECK(text().rfind("G28 O I\nG27 W2\nM701 ", 0) == 0);
        REQUIRE(pa::auto_restore_sequence(out, std::nullopt, 0, ""));
        CHECK(text().rfind("G28 O I\nG27 W2\nM702 ", 0) == 0);
        // Tous les paramètres changés : la ligne tient encore dans le tampon de M1994 (256)
        rq.slot = 4;
        rq.k_min = 0.01f;
        rq.k_max = 0.12f;
        rq.k_step = 0.015f;
        rq.refine = 4;
        rq.params = { .slow_flow = 2.f, .fast_flow = 12.f, .slow_s = 0.9f, .fast_s = 0.35f, .bead = { 0.5f, 0.25f }, .cycles = 4 };
        rq.temperature = 245;
        rq.bed_temperature = 85;
        rq.x = 25.f;
        rq.y = 14.f;
        rq.pitch = 6.f;
        rq.csv = true;
        CHECK(pa::oneclick_sequence(out, rq, "#65535", 110));
        // Le plateau d'abord (pas un chargement), puis l'origine et la position levée avant M701
        CHECK(text().rfind("M140 S110\nG28 O I\nG27 W2\nM701 ", 0) == 0);
    }
    SECTION("tampon trop court : refusé, jamais tronqué") {
        char small[20];
        CHECK_FALSE(pa::oneclick_sequence(small, pa::Request {}, "#5"));
        CHECK_FALSE(pa::auto_step_sequence(small, 1, "#5"));
    }
    SECTION("aucun % : la suite sert de format à GCodeLiteral") {
        pa::Request rq;
        rq.params.slow_flow = 2.f;
        rq.y = 15.5f;
        REQUIRE(pa::oneclick_sequence(out, rq, "#5"));
        CHECK(text().find('%') == std::string::npos);
    }
}

// ---- Motif compact et recherche adaptative (F4-b) ----

namespace {

/// Réponse synthétique d'une ligne : retard = pente·(K − K*) (positif : K trop faible), coût minimal en K*, bruit
struct Synthetic {
    float k_star = 0.045f;
    float slope = -1.5f;
    float noise = 0.f; ///< bruit du retard [s] (le coût a le sien, proportionnel)
    float amplitude_g = 25.f;
    bool flat = false; ///< le retard ne dépend pas de K
};

/// Lignes de la recherche, une par K demandé par next_k(), jusqu'à la fin
std::vector<pa::SweepPoint> run_search(const pa::Search &s, const Synthetic &m, uint32_t seed = 1) {
    Random rnd { seed };
    std::vector<pa::SweepPoint> done;
    while (const auto k = pa::next_k(done, s)) {
        REQUIRE(done.size() < s.cap);
        REQUIRE(*k >= s.k_min - 1e-6f);
        REQUIRE(*k <= s.k_max + 1e-6f);
        for (const pa::SweepPoint &p : done) {
            REQUIRE(std::fabs(p.k - *k) > 1e-5f); // jamais deux fois le même K
        }
        pa::BlockResult r;
        r.cycles = 2;
        r.amplitude_g = m.amplitude_g;
        r.noise = 0.05f;
        const float d = *k - m.k_star;
        r.lag_s = (m.flat ? 0.02f : m.slope * d) + m.noise * rnd.normal();
        r.cost = 0.002f + (m.flat ? 0.f : 3.f * d * d) + 0.2f * m.noise * std::fabs(rnd.normal());
        done.push_back({ *k, r });
    }
    return done;
}

std::string ks_of(const std::vector<pa::SweepPoint> &done) {
    std::string s;
    for (const pa::SweepPoint &p : done) {
        s += std::to_string(p.k) + " ";
    }
    return s;
}

} // namespace

TEST_CASE("PA compact : ligne plus courte, motif serré sans repères") {
    pa::Request rq;
    pa::make_compact(rq);
    CHECK(rq.compact);
    CHECK(rq.refine == 0);
    CHECK(pa::is_valid(rq.params));
    // 3 tronçons lents de 10 mm et 2 rapides de 27,8 mm, contre 153 mm pour le grand motif
    CHECK_THAT(pa::line_length(rq.params), WithinAbs(85.6, 0.1));

    pattern::Lines lines;
    lines.x0 = rq.x;
    lines.y0 = rq.y;
    lines.pitch = rq.pitch;
    lines.length = pa::line_length(rq.params);
    lines.count = pa::compact_cap;
    lines.bead = rq.params.bead;
    const pattern::Rect area = lines.area();
    // ~86 × 18 mm (grand motif : ~173 × 54 mm)
    CHECK_THAT(area.width(), WithinAbs(86.0, 0.2));
    CHECK_THAT(area.height(), WithinAbs(7 * 2.5 + 0.45, 1e-4));
}

TEST_CASE("PA compact : recherche adaptative, minimum au milieu") {
    pa::Search s;
    for (const float k_star : { 0.031f, 0.045f, 0.058f, 0.07f }) {
        const auto done = run_search(s, { .k_star = k_star });
        const pa::Choice c = pa::choose_k(done, s.k_min, s.k_max, s.step);
        INFO("K* = " << k_star << ", lignes : " << ks_of(done) << ", K = " << c.k);
        CHECK(done.size() >= 4);
        CHECK(done.size() <= 6);
        CHECK(c.fit == pa::Fit::ok);
        CHECK(c.confidence >= 60);
        CHECK_THAT(c.k, WithinAbs(k_star, 0.002));
    }
}

TEST_CASE("PA compact : bon K de départ, 4 lignes (monter, voir que c'est moins bien, redescendre)") {
    pa::Search s;
    s.prior = 0.0365f; // dernier K de l'emplacement : départ à 0,04
    const auto done = run_search(s, { .k_star = 0.037f });
    INFO("lignes : " << ks_of(done));
    REQUIRE(done.size() == 4);
    CHECK_THAT(done[0].k, WithinAbs(0.04, 1e-6));
    // 0,04 : un peu trop fort, on descend à 0,02 (encadré), confirmation à 0,00, puis le demi-pas 0,03
    CHECK_THAT(done[1].k, WithinAbs(0.02, 1e-6));
    CHECK_THAT(done[2].k, WithinAbs(0.0, 1e-6));
    CHECK_THAT(done[3].k, WithinAbs(0.03, 1e-6));
    CHECK_THAT(pa::choose_k(done, s.k_min, s.k_max, s.step).k, WithinAbs(0.037, 0.002));
}

TEST_CASE("PA compact : minimum à un bord de la plage") {
    pa::Search s;
    for (const float k_star : { 0.f, 0.004f, 0.096f, 0.1f }) {
        const auto done = run_search(s, { .k_star = k_star });
        const pa::Choice c = pa::choose_k(done, s.k_min, s.k_max, s.step);
        INFO("K* = " << k_star << ", lignes : " << ks_of(done) << ", K = " << c.k << ", ajustement " << int(c.fit));
        CHECK(done.size() <= 6);
        CHECK(c.fit == pa::Fit::ok);
        CHECK_THAT(c.k, WithinAbs(k_star, 0.004));
    }
    // Au-delà de la plage : on s'arrête au bord, K hors plage signalé
    const auto done = run_search(s, { .k_star = 0.16f });
    CHECK(done.size() <= s.cap);
    CHECK(pa::choose_k(done, s.k_min, s.k_max, s.step).fit == pa::Fit::out_of_range);
}

TEST_CASE("PA compact : bruit sur le retard") {
    pa::Search s;
    for (uint32_t seed = 1; seed <= 40; ++seed) {
        const float k_star = 0.02f + 0.0015f * float(seed);
        const auto done = run_search(s, { .k_star = k_star, .noise = 0.004f }, seed);
        const pa::Choice c = pa::choose_k(done, s.k_min, s.k_max, s.step);
        INFO("graine " << seed << ", K* = " << k_star << ", lignes : " << ks_of(done) << ", K = " << c.k);
        CHECK(done.size() <= s.cap);
        CHECK(c.fit == pa::Fit::ok);
        CHECK_THAT(c.k, WithinAbs(k_star, 0.008));
    }
}

TEST_CASE("PA compact : réponse plate ou trop faible, balayage élargi borné") {
    pa::Search s;
    const auto flat = run_search(s, { .flat = true });
    INFO("plat : " << ks_of(flat));
    CHECK(flat.size() <= s.cap);
    CHECK(flat.size() >= 5); // élargi au-delà de la montée
    CHECK(pa::choose_k(flat, s.k_min, s.k_max, s.step).fit == pa::Fit::no_crossing);

    const auto weak = run_search(s, { .amplitude_g = 1.f });
    INFO("faible : " << ks_of(weak));
    CHECK(weak.size() == 6); // toute la grille 0 à 0,10, puis fini
    CHECK(pa::choose_k(weak, s.k_min, s.k_max, s.step).fit == pa::Fit::weak_signal);

    s.cap = 3;
    CHECK(run_search(s, { .flat = true }).size() == 3);
}

TEST_CASE("PA compact : K retrouvé sur lignes imprimées simulées, en 4 à 6 lignes") {
    pa::Request rq;
    pa::make_compact(rq);
    const pa::Profile p = pa::profile(rq.params);
    for (float tau : { 0.025f, 0.04f, 0.065f }) {
        for (float gain : { 30.f, -30.f }) {
            Nozzle nozzle;
            nozzle.tau = tau;
            nozzle.gain_g = gain;
            pa::Search s;
            std::vector<pa::SweepPoint> done;
            uint32_t seed = 1;
            while (const auto k = pa::next_k(done, s)) {
                done.push_back({ *k, pa::analyse_block(simulate(rq.params, nozzle, *k, dt, seed++), dt, p) });
            }
            const pa::Choice c = pa::choose_k(done, s.k_min, s.k_max, s.step);
            INFO("τ = " << tau << ", gain = " << gain << ", lignes : " << ks_of(done) << ", K = " << c.k << ", confiance = " << int(c.confidence));
            CHECK(done.size() >= 4);
            CHECK(done.size() <= 6);
            CHECK(c.fit == pa::Fit::ok);
            CHECK(c.confidence >= 30);
            CHECK_THAT(c.k, WithinAbs(tau, 0.004));
        }
    }
}

// ---- Lignes dans les deux sens (demande du 22/11 : plus de retour à gauche entre deux lignes) ----

TEST_CASE("PA : ligne vers −X analysée comme vers +X") {
    const pa::Params params;
    const pa::Profile p = pa::profile(params);
    for (float k : { 0.f, 0.04f, 0.08f }) {
        const Nozzle nozzle; // écrasement, bruit, dérive, non-linéarité
        const pa::BlockResult forward = pa::analyse_block(simulate(params, nozzle, k, dt, 11, 1.f), dt, p);
        const pa::BlockResult backward = pa::analyse_block(simulate(params, nozzle, k, dt, 11, -1.f), dt, p);
        INFO("K = " << k << ", retard +X " << forward.lag_s << ", −X " << backward.lag_s);
        REQUIRE(forward.valid());
        CHECK(backward.cycles == forward.cycles);
        CHECK_THAT(backward.lag_s, WithinAbs(forward.lag_s, 1e-5));
        CHECK_THAT(backward.cost, WithinAbs(forward.cost, 1e-5));
        CHECK_THAT(backward.amplitude_g, WithinAbs(forward.amplitude_g, 1e-3));
        CHECK_THAT(backward.overshoot, WithinAbs(forward.overshoot, 1e-4));
    }
    // Trace vers −X : X décroît, fronts de vitesse trouvés quand même
    const auto t = simulate(params, Nozzle {}, 0.04f, dt, 3, -1.f);
    CHECK(force::x_mm(t.back()) < -100.f);
    std::array<force::Edge, 16> edges;
    CHECK(force::find_edges(t, dt, 0.5f * (p.v_lo + p.v_hi), 3, edges) == 2u * params.cycles);
}

TEST_CASE("PA : K retrouvé avec des lignes alternées (grand motif et compact)") {
    const pa::Params params;
    for (float tau : { 0.025f, 0.04f, 0.065f }) {
        Nozzle nozzle;
        nozzle.tau = tau;
        // Grand motif : balayage grossier puis fin, une ligne sur deux vers −X
        std::vector<pa::SweepPoint> all;
        std::array<float, 32> ks;
        const size_t n = pa::sweep_values(0.f, 0.10f, 0.02f, ks);
        uint32_t seed = 1;
        const auto line = [&](float k) {
            const float dir = all.size() % 2 ? -1.f : 1.f;
            all.push_back({ k, pa::analyse_block(simulate(params, nozzle, k, dt, seed++, dir), dt, pa::profile(params)) });
        };
        for (size_t i = 0; i < n; ++i) {
            line(ks[i]);
        }
        const pa::Choice coarse = pa::choose_k(all, 0.f, 0.10f, 0.02f);
        REQUIRE(coarse.fit == pa::Fit::ok);
        std::array<float, 8> fine;
        const size_t m = pa::refine_values(coarse.k, 0.005f, 5, 0.f, 0.3f, fine);
        for (size_t i = 0; i < m; ++i) {
            line(fine[i]);
        }
        const pa::Choice c = pa::choose_k(all, 0.f, 0.10f, 0.02f);
        INFO("grand, τ = " << tau << ", K = " << c.k);
        CHECK(c.fit == pa::Fit::ok);
        CHECK_THAT(c.k, WithinAbs(tau, 0.003));

        // Compact : recherche adaptative, une ligne sur deux vers −X
        pa::Request rq;
        pa::make_compact(rq);
        pa::Search s;
        std::vector<pa::SweepPoint> done;
        while (const auto k = pa::next_k(done, s)) {
            const float dir = done.size() % 2 ? -1.f : 1.f;
            done.push_back({ *k, pa::analyse_block(simulate(rq.params, nozzle, *k, dt, seed++, dir), dt, pa::profile(rq.params)) });
        }
        const pa::Choice cc = pa::choose_k(done, s.k_min, s.k_max, s.step);
        INFO("compact, τ = " << tau << ", lignes : " << ks_of(done) << ", K = " << cc.k);
        CHECK(done.size() <= 6);
        CHECK(cc.fit == pa::Fit::ok);
        CHECK_THAT(cc.k, WithinAbs(tau, 0.004));
    }
}

TEST_CASE("motif : lignes alternées, chaque ligne part de la fin de la précédente") {
    for (const bool compact : { true, false }) {
        pa::Request rq;
        if (compact) {
            pa::make_compact(rq);
        }
        pattern::Lines lines;
        lines.x0 = rq.x;
        lines.y0 = rq.y;
        lines.pitch = rq.pitch;
        lines.length = pa::line_length(rq.params);
        lines.count = compact ? pa::compact_cap : 11;
        lines.marks = compact ? 0.f : pattern::tally_width(lines.count);
        lines.bead = rq.params.bead;
        const pattern::Rect area = lines.area();
        // Même emprise qu'avant : il tient toujours à l'avant du plateau (250 × 220, 10 mm de marge)
        CHECK(pattern::Rect { 10.f, 10.f, 240.f, 210.f }.contains(area));
        for (uint8_t i = 0; i < lines.count; ++i) {
            const float start = lines.x_start(i), end = start + lines.dir(i) * lines.length;
            INFO((compact ? "compact" : "grand") << ", ligne " << int(i));
            CHECK(lines.dir(i) == (i % 2 ? -1.f : 1.f));
            CHECK(std::min(start, end) >= area.x0);
            CHECK(std::max(start, end) <= area.x1);
            CHECK_THAT(std::min(start, end), WithinAbs(lines.x0, 1e-4));
            if (i + 1 < lines.count) {
                CHECK_THAT(lines.x_start(uint8_t(i + 1)), WithinAbs(end, 1e-4)); // pas de retour en X
            }
        }
    }
}

namespace {

/// Plateau de la CORE One (250 × 220, 10 mm de marge), comme session::series_begin() : origine Z (240, 10) et centre
/// du plateau interdits à 8 mm près, 4 mm entre deux motifs
constexpr pattern::Rect shelf_zone { 10.f, 10.f, 240.f, 210.f };
constexpr float shelf_gap = 4.f;
const std::array<pattern::Rect, 2> shelf_keep_out {
    pattern::Rect { 232.f, 2.f, 248.f, 18.f },
    pattern::Rect { 117.f, 102.f, 133.f, 118.f },
};

/// Tout est dans la zone, hors des zones interdites, et à shelf_gap au moins des autres
void check_layout(const std::vector<pattern::Rect> &placed) {
    for (size_t i = 0; i < placed.size(); ++i) {
        INFO("rectangle " << i);
        CHECK(shelf_zone.contains(placed[i]));
        for (const pattern::Rect &k : shelf_keep_out) {
            CHECK_FALSE(pattern::overlaps(placed[i], k));
        }
        for (size_t j = 0; j < i; ++j) {
            INFO("et " << j);
            CHECK_FALSE(pattern::overlaps(placed[i].grown(shelf_gap / 2.f - 1e-3f, shelf_gap / 2.f - 1e-3f), placed[j].grown(shelf_gap / 2.f - 1e-3f, shelf_gap / 2.f - 1e-3f)));
        }
    }
}

} // namespace

TEST_CASE("série : chaque motif a sa place sur le plateau, sans chevauchement", "[od3d][pattern]") {
    pattern::Shelf shelf(shelf_zone, shelf_keep_out, shelf_gap);
    // Rangée : de gauche à droite, à l'avant
    const auto a = shelf.place(52.f, 6.f);
    REQUIRE(a);
    CHECK(a->x0 == shelf_zone.x0);
    CHECK(a->y0 == shelf_zone.y0);
    const auto b = shelf.place(90.f, 18.f);
    REQUIRE(b);
    CHECK_THAT(b->x0, WithinAbs(a->x1 + shelf_gap, 1e-4));
    CHECK(b->y0 == shelf_zone.y0);
    // Trop large pour le reste de la rangée (et l'origine Z à droite) : rangée suivante, derrière la plus haute
    const auto c = shelf.place(120.f, 10.f);
    REQUIRE(c);
    CHECK(c->x0 == shelf_zone.x0);
    CHECK_THAT(c->y0, WithinAbs(b->y1 + shelf_gap, 1e-4));
    // Trop grand pour le plateau : refusé sans rien changer
    CHECK_FALSE(shelf.place(500.f, 5.f));
    CHECK_FALSE(shelf.place(10.f, 500.f));
    CHECK_FALSE(shelf.place(0.f, 5.f));
    const auto d = shelf.place(20.f, 5.f);
    REQUIRE(d);
    CHECK_THAT(d->x0, WithinAbs(c->x1 + shelf_gap, 1e-4));
    check_layout({ *a, *b, *c, *d });
}

TEST_CASE("série : zones interdites (origine Z, centre du plateau) évitées", "[od3d][pattern]") {
    pattern::Shelf shelf(shelf_zone, shelf_keep_out, shelf_gap);
    std::vector<pattern::Rect> placed;
    // Des carrés jusqu'à ce que le plateau soit plein : jamais sur un point palpé
    while (const auto r = shelf.place(30.f, 30.f)) {
        placed.push_back(*r);
        REQUIRE(placed.size() < 100);
    }
    CHECK(placed.size() >= 20); // 230 × 200 mm : 6 rangées de 5 à 6 carrés, moins les zones interdites
    check_layout(placed);
    // Plein pour ces carrés : un plus petit peut encore tenir au bout d'une rangée ; un nouvel objet repart de zéro
    pattern::Shelf clean(shelf_zone, shelf_keep_out, shelf_gap);
    const auto first = clean.place(30.f, 30.f);
    REQUIRE(first);
    CHECK(first->x0 == shelf_zone.x0);
    CHECK(first->y0 == shelf_zone.y0);
}

TEST_CASE("série : motifs de l'assistant par bobine, une demande de nettoyage quand le plateau est plein", "[od3d][pattern]") {
    // Par bobine : purge (52 × 6), PA compact, purge, [débit F5 (~181 × 46), purge,] température F6 (~84 × 40)
    pa::Request rq;
    pa::make_compact(rq);
    pattern::Lines pa_lines;
    pa_lines.pitch = rq.pitch;
    pa_lines.length = pa::line_length(rq.params);
    pa_lines.count = pa::compact_cap;
    pa_lines.bead = rq.params.bead;
    const pattern::Rect pa_area = pa_lines.area();
    const std::pair<float, float> purge { 52.f, 6.f }, pa { pa_area.width(), pa_area.height() }, flow { 180.6f, 45.6f }, temp { 83.9f, 39.6f };
    for (const bool flow_limit : { false, true }) {
        std::vector<std::pair<float, float>> per_spool { purge, pa, purge };
        if (flow_limit) {
            per_spool.insert(per_spool.end(), { flow, purge });
        }
        per_spool.push_back(temp);
        pattern::Shelf shelf(shelf_zone, shelf_keep_out, shelf_gap);
        std::vector<pattern::Rect> placed;
        size_t spools = 0;
        bool full = false;
        while (!full && spools < 5) {
            for (const auto &[w, h] : per_spool) {
                const auto r = shelf.place(w, h);
                if (!r) {
                    full = true;
                    break;
                }
                placed.push_back(*r);
            }
            spools += full ? 0 : 1;
        }
        INFO((flow_limit ? "avec F5" : "sans F5") << ", bobines complètes sur une plaque : " << spools);
        CHECK(spools >= (flow_limit ? 1u : 3u));
        CHECK(full); // 5 bobines ne tiennent pas : l'assistant demande de nettoyer, puis repart de zéro
        check_layout(placed);
    }
}

TEST_CASE("série : rétraction par matière (constante prudente)", "[od3d][pattern]") {
    CHECK(pattern::retract_mm(false) > 0.5f);
    CHECK(pattern::retract_mm(false) <= 1.f); // PrusaSlicer pour la CORE One : 0,7 à 0,8 mm
    CHECK(pattern::retract_mm(true) == 0.f); // flexibles : pas de rétraction
    CHECK(pattern::retract_speed > 0.f);
}
