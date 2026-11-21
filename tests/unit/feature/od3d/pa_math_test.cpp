// OneDrop3D (F4) : analyse de la réponse en force et choix du pressure advance, sur des courbes synthétiques.
// La buse est simulée par le modèle du premier ordre τ·dp/dt + p = débit entrant, le PA de Prusa par
// e_moteur = e + K·(vitesse moyenne de E sur une fenêtre), la cellule de charge par une force non linéaire en p,
// avec bruit, dérive lente et signe quelconque.
#include <feature/od3d/pa_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstdint>
#include <vector>

using namespace od3d;
using Catch::Matchers::WithinAbs;

namespace {

struct Nozzle {
    float tau = 0.04f; ///< constante de temps de la buse [s] = K idéal
    float gain_g = 30.f; ///< force au palier rapide [g] (signe : sens de la cellule)
    float exponent = 0.8f; ///< force ∝ pression^exponent (réponse sous-linéaire des vraies buses)
    float noise_g = 0.4f; ///< bruit blanc [g]
    float drift_g_per_s = 1.5f; ///< dérive lente (filament qui pend, cellule qui chauffe)
    float pa_effect = 1.f; ///< 0 : le PA n'agit pas (buse sans XY, par exemple)
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

/// Simule un bloc de mesure (un K), comme la séquence du firmware : 0,3 s d'arrêt, lancement lent (aller-retour),
/// créneaux (rapide, demi-lent, demi-tour, demi-lent), 0,3 s d'arrêt ; échantillonné au pas dt (moyenne de la force et
/// de X sur chaque pas, comme l'interruption de la cellule de charge)
std::vector<force::TracePoint> simulate(const pa::Params &params, const Nozzle &nozzle, float k, float dt, uint32_t seed) {
    const pa::Profile p = pa::profile(params);
    constexpr float h = 0.00025f; // pas de la simulation
    constexpr float accel = 5000.f;
    const float ls = p.v_lo * p.slow_s, lf = p.v_hi * p.fast_s;

    struct Segment {
        float dist; // signée
        float speed;
    };
    std::vector<Segment> segments { { -ls / 2, p.v_lo }, { ls / 2, p.v_lo } };
    float d = 1;
    for (int c = 0; c < params.cycles; ++c) {
        segments.push_back({ d * lf, p.v_hi });
        segments.push_back({ d * ls / 2, p.v_lo });
        segments.push_back({ -d * ls / 2, p.v_lo });
        d = -d;
    }

    // Cinématique : vitesse limitée par l'accélération, jonction à la plus petite des deux vitesses (0 au demi-tour)
    std::vector<float> xs, ev;
    float x = 0;
    const auto idle = [&](float s) {
        for (float t = 0; t < s; t += h) {
            xs.push_back(x);
            ev.push_back(0);
        }
    };
    idle(0.3f);
    float v = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        const Segment &s = segments[i];
        const float dir = s.dist > 0 ? 1.f : -1.f;
        const float len = std::fabs(s.dist);
        float v_end = 0;
        if (i + 1 < segments.size() && (segments[i + 1].dist > 0) == (s.dist > 0)) {
            v_end = std::min(s.speed, segments[i + 1].speed);
        }
        float done = 0;
        while (done < len) {
            const float remaining = len - done;
            v = std::min({ s.speed, v + accel * h, std::sqrt(v_end * v_end + 2.f * accel * remaining) });
            v = std::max(v, 0.5f);
            done += v * h;
            x += dir * v * h;
            xs.push_back(x);
            ev.push_back(params.e_per_mm * v);
        }
        v = v_end;
    }
    idle(0.3f);

    // PA : vitesse de E moyennée sur smooth_s, puis e_moteur' = e' + K·d(moyenne)/dt ; buse du premier ordre
    const size_t n = ev.size();
    const size_t half = size_t(nozzle.smooth_s / 2 / h);
    std::vector<float> mean(n);
    double run = 0;
    std::vector<double> prefix(n + 1, 0);
    for (size_t i = 0; i < n; ++i) {
        run += ev[i];
        prefix[i + 1] = run;
    }
    for (size_t i = 0; i < n; ++i) {
        const size_t a = i > half ? i - half : 0, b = std::min(n, i + half + 1);
        mean[i] = float((prefix[b] - prefix[a]) / double(b - a));
    }
    const float ve_hi = params.e_per_mm * p.v_hi;
    Random rnd { seed };
    std::vector<force::TracePoint> out;
    float pressure = 0;
    const size_t per_sample = size_t(dt / h + 0.5f);
    double sum_f = 0, sum_x = 0;
    size_t count = 0;
    float f0 = NAN, x0 = NAN;
    for (size_t i = 0; i < n; ++i) {
        const float dmean = i > 0 ? (mean[i] - mean[i - 1]) / h : 0;
        const float inflow = ev[i] + nozzle.pa_effect * k * dmean;
        pressure += h / nozzle.tau * (inflow - pressure);
        const float rel = pressure / ve_hi;
        const float shaped = (rel >= 0 ? 1.f : -1.f) * std::pow(std::fabs(rel), nozzle.exponent);
        const float t = float(i) * h;
        sum_f += nozzle.gain_g * shaped + nozzle.drift_g_per_s * t;
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

TEST_CASE("PA : profil et bornes de la séquence") {
    const pa::Params params;
    CHECK(pa::is_valid(params));
    const pa::Profile p = pa::profile(params);
    // 2 mm³/s et 15 mm³/s avec 0,08 mm de filament par mm : ~10 mm/s et ~78 mm/s
    CHECK_THAT(p.v_lo, WithinAbs(10.39, 0.05));
    CHECK_THAT(p.v_hi, WithinAbs(77.95, 0.1));
    CHECK_THAT(pa::block_duration_s(params), WithinAbs(7.5, 1e-4));

    pa::Params bad = params;
    bad.fast_flow = 3.f; // moins de 2 × le débit lent
    CHECK_FALSE(pa::is_valid(bad));
    bad = params;
    bad.cycles = 1;
    CHECK_FALSE(pa::is_valid(bad));
}

TEST_CASE("PA : bloc analysé, retard ≈ τ − K") {
    const pa::Params params;
    const pa::Profile p = pa::profile(params);
    Nozzle nozzle;
    nozzle.noise_g = 0.f;
    nozzle.drift_g_per_s = 0.f;
    nozzle.exponent = 1.f;
    for (float k : { 0.f, 0.02f, 0.04f, 0.06f, 0.08f }) {
        pa::Waveform wave;
        const auto trace = simulate(params, nozzle, k, dt, 7);
        const pa::BlockResult r = pa::analyse_block(trace, dt, p, &wave);
        INFO("K = " << k << ", lag = " << r.lag_s);
        CHECK(r.cycles == params.cycles);
        // Créneau de force : palier rapide (30 g) moins palier lent (2/15 de 30 g, réponse linéaire ici)
        CHECK_THAT(r.amplitude_g, WithinAbs(26.0, 1.0));
        CHECK_THAT(r.lag_s, WithinAbs(nozzle.tau - k, 0.006));
        CHECK(wave.step_s > 0);
        // Palier rapide de la commande moyenne à 1 (×64)
        CHECK(std::abs(wave.command[40] - 64) <= 4);
    }
    // Trop peu de PA : pas de dépassement ; trop : dépassement net
    const auto low = pa::analyse_block(simulate(params, nozzle, 0.f, dt, 3), dt, p);
    const auto high = pa::analyse_block(simulate(params, nozzle, 0.10f, dt, 3), dt, p);
    CHECK(low.overshoot < 0.05f);
    CHECK(high.overshoot > 0.3f);
    CHECK(high.undershoot > 0.3f);
}

TEST_CASE("PA : K retrouvé malgré bruit, dérive, non-linéarité et signe inversé") {
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
            CHECK(c.slope > -1.4f);
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
    CHECK(force::make_point(1000.f, 0).force == 32767);
    CHECK(force::make_point(-1000.f, 0).force == -32767);
    CHECK(force::make_point(NAN, NAN).x == 0);
}
