// OneDrop3D (F4) : analyse de la réponse en force et choix du pressure advance, sur des signaux synthétiques réalistes
// d'extrusion dans le vide (protocole de PrusaPATuner : créneaux lent 3 mm³/s × 2 s / rapide 14 mm³/s × 0,714 s,
// aller-retour de 1 mm en X, 500 mm/s²), et relecture d'un CSV de notre format.
//
// Simulation, comme la machine :
// - commande : vitesse nominale de E, chaque tronçon étant un mouvement XY + E séparé (X fait demi-tour à chaque
//   changement de débit, donc la vitesse passe par zéro), accélération de E bornée à 500 mm/s² ;
// - PA de Prusa : moteur E = E nominal + K × vitesse nominale lissée sur une fenêtre centrée de 40 ms (M572 W) ;
// - buse : pression du premier ordre τ·dP/dt + P = vitesse du moteur E (le K idéal vaut τ), force non linéaire
//   (∝ P^0,9, comme la loi de puissance du débit de PrusaPATuner) ;
// - cellule : retard pur et filtre du convertisseur (premier ordre), bruit blanc, dérive lente, échantillons à 320 Hz
//   moyennés par 3 comme l'interruption (≈107 points/s), force relative au premier point, sens au choix ;
// - X et E au même instant (positionLookback), quantifiés comme la trace.
// Amplitudes de l'ordre de celles de PrusaPATuner : palier rapide de 450 à 2 500 g au-dessus du palier lent, bruit de
// 5 à 60 g (test « plateau noise » de PrusaPATuner : ~60 unités).
//
// Variable d'environnement OD3D_PA_CSV : chemin d'un OD3D_PA<n>.CSV de la machine, rejoué et résumé (sans contrôle du
// résultat) pour régler les seuils sur les vraies courbes.
#include <feature/od3d/pa_math_od3d.hpp>
#include <feature/od3d/pattern_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace od3d;
using Catch::Matchers::WithinAbs;

namespace {

struct Machine {
    float tau = 0.035f; ///< constante de temps de la buse [s] = K idéal
    float delta_g = 480.f; ///< palier rapide au-dessus du palier lent [g]
    float offset_g = 150.f; ///< palier lent au-dessus de la buse au repos [g] (n'entre pas dans le créneau)
    float exponent = 0.9f; ///< force ∝ P^exponent
    float noise_g = 6.f; ///< bruit blanc par échantillon de la cellule [g]
    float drift_g_per_s = 3.f; ///< dérive lente (cellule qui chauffe)
    float delay_s = 0.015f; ///< retard pur de la force sur la commande
    float filter_s = 0.006f; ///< filtre du convertisseur
    float cell_sign = -1.f; ///< la trace est −loadcell_value (force vers le haut positive), qui monte avec le débit
    float smooth_s = 0.04f; ///< fenêtre de lissage du PA (M572 W)
    float hx_hz = 320.f;
    uint8_t decimation = 3;
};

/// Générateur reproductible (LCG) et loi normale de Box-Muller
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

/// Une séquence (un K) comme le firmware : enregistrement lancé à l'arrêt, 50 ms avant le premier mouvement, lent de
/// lancement, cycles × (rapide, lent), puis 0,2 s ; direction de X donnée (alternée d'un K au suivant)
std::vector<force::TracePoint> simulate(const pa::Params &params, const Machine &m, float k, float direction, uint32_t seed, float *dt_out = nullptr) {
    const pa::Profile p = pa::profile(params);
    constexpr float h = 1.f / 3200.f; // pas de la simulation
    struct Leg {
        float duration, e_speed, x_speed;
    };
    std::vector<Leg> legs;
    legs.push_back({ 0.05f, 0.f, 0.f });
    legs.push_back({ params.slow_s, p.e_slow, direction * p.v_slow });
    for (int c = 0; c < params.cycles; ++c) {
        legs.push_back({ params.fast_s, p.e_fast, -direction * p.v_fast });
        legs.push_back({ params.slow_s, p.e_slow, direction * p.v_slow });
    }
    legs.push_back({ 0.2f, 0.f, 0.f });
    float total = 0;
    for (const Leg &l : legs) {
        total += l.duration;
    }
    const size_t n = size_t(total / h);

    // Commande nominale : chaque tronçon part de zéro et y revient, E accéléré à params.accel
    std::vector<float> u(n), vx(n);
    {
        float t0 = 0;
        size_t i = 0;
        for (const Leg &l : legs) {
            const float ramp = l.e_speed / params.accel;
            for (; i < n && float(i) * h < t0 + l.duration; ++i) {
                const float tl = float(i) * h - t0;
                float f = 1.f;
                if (ramp > 0) {
                    f = std::min({ 1.f, tl / ramp, (l.duration - tl) / ramp });
                    f = std::max(f, 0.f);
                }
                u[i] = l.e_speed * f;
                vx[i] = l.x_speed * f;
            }
            t0 += l.duration;
        }
    }
    // PA : vitesse du moteur = u + K × d(u lissé)/dt (fenêtre centrée)
    const long w = std::max(1L, std::lround(m.smooth_s / h / 2.f));
    std::vector<float> smooth(n), motor(n);
    {
        std::vector<double> cum(n + 1, 0.0);
        for (size_t i = 0; i < n; ++i) {
            cum[i + 1] = cum[i] + u[i];
        }
        for (size_t i = 0; i < n; ++i) {
            const long a = std::max(0L, long(i) - w), b = std::min(long(n), long(i) + w + 1);
            smooth[i] = float((cum[b] - cum[a]) / double(b - a));
        }
        for (size_t i = 0; i < n; ++i) {
            const float d = i > 0 ? (smooth[i] - smooth[i - 1]) / h : 0.f;
            motor[i] = u[i] + k * d;
        }
    }
    // Buse, puis cellule
    const float gain = m.delta_g / (std::pow(p.e_fast, m.exponent) - std::pow(p.e_slow, m.exponent));
    Random rng { seed };
    float pressure = 0.6f * p.e_slow; // pression qui retombe depuis le K précédent
    std::vector<float> force(n);
    for (size_t i = 0; i < n; ++i) {
        pressure += h / m.tau * (motor[i] - pressure);
        force[i] = gain * std::pow(std::max(pressure, 0.f), m.exponent) + m.offset_g - gain * std::pow(p.e_slow, m.exponent);
    }
    const long delay = std::lround(m.delay_s / h);
    float filtered = force[0];
    float x = 0, e = 0;
    std::vector<float> xs(n), es(n);
    for (size_t i = 0; i < n; ++i) {
        x += vx[i] * h;
        e += motor[i] * h;
        xs[i] = x;
        es[i] = e;
    }

    // Échantillons de la cellule, moyennés comme l'interruption, relatifs au premier
    std::vector<force::TracePoint> out;
    const size_t per_sample = size_t(std::lround(1.f / (m.hx_hz * h)));
    float base_f = NAN, base_x = 0, base_e = 0, sum_f = 0, sum_x = 0, sum_e = 0;
    int summed = 0;
    for (size_t i = 0; i < n; i += per_sample) {
        // Filtre du convertisseur sur la force retardée, évalué à la cadence de la simulation
        for (size_t j = i; j < std::min(n, i + per_sample); ++j) {
            const long src = std::max(0L, long(j) - delay);
            filtered += h / m.filter_s * (force[size_t(src)] - filtered);
        }
        const float t = float(i) * h;
        const float sample = m.cell_sign * (filtered + m.drift_g_per_s * t + m.noise_g * rng.normal());
        if (std::isnan(base_f)) {
            base_f = sample;
            base_x = xs[i];
            base_e = es[i];
        }
        sum_f += sample;
        sum_x += xs[i];
        sum_e += es[i];
        if (++summed == m.decimation) {
            out.push_back(force::make_point(sum_f / summed - base_f, sum_x / summed - base_x, sum_e / summed - base_e));
            sum_f = sum_x = sum_e = 0;
            summed = 0;
        }
    }
    if (dt_out) {
        *dt_out = float(m.decimation) / m.hx_hz;
    }
    return out;
}

/// Balayage complet (grossier, puis fin autour du premier choix), comme M1994
pa::Choice sweep(const Machine &m, const pa::Params &params = {}, float k_min = 0.f, float k_max = 0.10f, float k_step = 0.01f, uint8_t refine = 4, uint32_t seed = 1) {
    std::vector<pa::SweepPoint> points;
    std::array<float, 32> ks;
    const size_t coarse = pa::sweep_values(k_min, k_max, k_step, ks);
    float direction = 1.f;
    const auto run = [&](float k) {
        float dt;
        const auto trace = simulate(params, m, k, direction, seed++, &dt);
        direction = -direction;
        points.push_back({ k, pa::analyse_block(trace, dt, params, k) });
    };
    for (size_t i = 0; i < coarse; ++i) {
        run(ks[i]);
    }
    pa::Choice c = pa::choose_k(points, k_step);
    if (c.fit == pa::Fit::ok && refine > 0) {
        std::array<float, 8> fine;
        const size_t n = pa::refine_values(c.k, k_step / 2.f, refine, 0.f, 1.f, fine);
        for (size_t i = 0; i < n; ++i) {
            run(fine[i]);
        }
        c = pa::choose_k(points, k_step);
    }
    return c;
}

/// Écrit une séquence au format du CSV de la machine (lignes D), commande reconstituée comme le firmware
void write_block(std::string &csv, const std::vector<force::TracePoint> &trace, float dt, float k, const pa::Params &params) {
    std::array<pa::Turn, 24> turns;
    const size_t n = pa::find_turns(trace, 0.25f * params.wiggle, turns);
    char line[96];
    for (size_t i = 0; i < trace.size(); ++i) {
        const pa::CsvPoint p { k, float(i) * dt, force::x_mm(trace[i]), force::e_mm(trace[i]), force::force_g(trace[i]), pa::command_flow(std::span(turns.data(), n), i, params) };
        REQUIRE(pa::format_point(line, sizeof(line), p) > 0);
        csv += line;
    }
}

/// Relit un CSV (P, D) et rejoue l'analyse : séquences regroupées par K dans l'ordre du fichier
struct Replay {
    pa::Params params;
    bool has_params = false;
    std::vector<pa::SweepPoint> points;
    size_t lines = 0;
};

Replay replay(std::istream &in) {
    Replay r;
    std::string line;
    std::vector<force::TracePoint> trace;
    float k = NAN, t_first = 0, t_last = 0;
    const auto flush = [&] {
        if (trace.size() >= 2) {
            const float dt = (t_last - t_first) / float(trace.size() - 1);
            r.points.push_back({ k, pa::analyse_block(trace, dt, r.params, k) });
        }
        trace.clear();
    };
    while (std::getline(in, line)) {
        ++r.lines;
        pa::CsvPoint p;
        if (pa::parse_params(line.c_str(), r.params)) {
            r.has_params = true;
        } else if (pa::parse_point(line.c_str(), p)) {
            if (p.k != k) {
                flush();
                k = p.k;
                t_first = p.t_s;
            }
            t_last = p.t_s;
            // Ancien format (jusqu'à la 8.0.2.21) : points en 0,02 g, bornés à ±655,34 g ; un point à la borne est écrêté
            const bool old_clip = std::isnan(p.command) && std::fabs(p.force_g) >= 655.3f;
            trace.push_back(force::make_point(old_clip ? std::copysign(1e9f, p.force_g) : p.force_g, p.x_mm, p.e_mm));
        }
    }
    flush();
    return r;
}

} // namespace

TEST_CASE("PA : protocole de PrusaPATuner et bornes") {
    const pa::Params params;
    CHECK(pa::is_valid(params));
    const pa::Profile p = pa::profile(params);
    // 3 et 14 mm³/s de filament de 1,75 mm : ~1,25 et ~5,82 mm/s ; X : 1 mm en 2 s, puis en 0,714 s
    CHECK_THAT(p.e_slow, WithinAbs(1.247, 0.002));
    CHECK_THAT(p.e_fast, WithinAbs(5.820, 0.002));
    CHECK_THAT(p.v_slow, WithinAbs(0.5, 1e-5));
    CHECK_THAT(p.v_fast, WithinAbs(1.4, 0.001));
    // 6 mm³ lents et 10 mm³ rapides
    CHECK_THAT(params.slow_flow * params.slow_s, WithinAbs(6.0, 1e-4));
    CHECK_THAT(params.fast_flow * params.fast_s, WithinAbs(10.0, 0.01));
    CHECK_THAT(pa::block_duration_s(params), WithinAbs(2.0 + 5 * 2.714, 1e-3));

    pa::Params bad = params;
    bad.fast_flow = 5.f; // moins de 2 × le débit lent
    CHECK_FALSE(pa::is_valid(bad));
    bad = params;
    bad.cycles = 2;
    CHECK_FALSE(pa::is_valid(bad));
    bad = params;
    bad.wiggle = 0.f; // sans XY, Prusa n'applique pas le PA
    CHECK_FALSE(pa::is_valid(bad));
    bad = params;
    bad.cycles = 8; // séquence trop longue pour la mémoire de la trace
    CHECK_FALSE(pa::is_valid(bad));
}

TEST_CASE("PA : demi-tours de X et commande reconstituée") {
    const pa::Params params;
    const Machine m;
    for (const float direction : { 1.f, -1.f }) {
        float dt;
        const auto trace = simulate(params, m, 0.03f, direction, 3, &dt);
        std::array<pa::Turn, 24> turns;
        const size_t n = pa::find_turns(trace, 0.25f * params.wiggle, turns);
        REQUIRE(n == 2 * params.cycles);
        for (size_t i = 0; i < n; ++i) {
            CHECK(turns[i].rise == (i % 2 == 0));
            // Montée i/2 : 50 ms + lent + (i/2) cycles ; descente : + le rapide
            const float expected = 0.05f + params.slow_s + float(i / 2) * (params.slow_s + params.fast_s) + (i % 2 ? params.fast_s : 0.f);
            CHECK_THAT(float(turns[i].index) * dt, WithinAbs(expected, 0.015));
        }
        const std::span<const pa::Turn> span(turns.data(), n);
        CHECK(pa::command_flow(span, 0, params) == params.slow_flow);
        CHECK(pa::command_flow(span, turns[0].index + 5, params) == params.fast_flow);
        CHECK(pa::command_flow(span, turns[1].index + 5, params) == params.slow_flow);
    }
    // Sans mouvement de X : aucun front
    std::vector<force::TracePoint> still(1000, force::make_point(0, 0));
    std::array<pa::Turn, 4> turns;
    CHECK(pa::find_turns(still, 0.25f, turns) == 0);
}

TEST_CASE("PA : une séquence analysée, critères qui suivent K") {
    const pa::Params params;
    const Machine m;
    // La trace d'abord, puis l'analyse : l'ordre d'évaluation des arguments n'est pas fixé (dt serait lu avant d'être écrit)
    const auto block = [&](float k, float direction, uint32_t seed, pa::Waveform *wave = nullptr) {
        float dt;
        const auto trace = simulate(params, m, k, direction, seed, &dt);
        return pa::analyse_block(trace, dt, params, k, wave);
    };
    const pa::BlockResult low = block(0.f, 1.f, 5);
    const pa::BlockResult good = block(m.tau, -1.f, 6);
    const pa::BlockResult high = block(0.10f, 1.f, 7);
    for (const pa::BlockResult *r : { &low, &good, &high }) {
        CHECK(r->found == params.cycles);
        CHECK(r->cycles == params.cycles - 1); // le premier créneau est écarté
        // La trace est −loadcell_value : créneau négatif, d'amplitude proche de celle simulée
        CHECK(r->amplitude_g < 0);
        CHECK_THAT(-r->amplitude_g, WithinAbs(m.delta_g, 0.15 * m.delta_g));
    }
    // PA trop faible : montée lente, traîne positive ; trop fort : dépassement et creux
    CHECK(low.m[pa::rise_error] > good.m[pa::rise_error]);
    CHECK(low.m[pa::rise_delay] > good.m[pa::rise_delay]);
    CHECK(low.fall_area > 0);
    CHECK(high.fall_area < 0);
    CHECK(high.m[pa::overshoot] > good.m[pa::overshoot]);
    CHECK(high.m[pa::undershoot] > good.m[pa::undershoot]);
    // L'estimation par créneau (K + aire / amplitude, théorie du premier ordre) tombe vers τ des deux côtés, décalée par
    // le retard de la cellule et le lissage du PA : elle ne sert que de contrôle croisé
    CHECK_THAT(low.k_estimate, WithinAbs(m.tau + 0.02, 0.03));
    CHECK_THAT(high.k_estimate, WithinAbs(m.tau + 0.02, 0.03));

    pa::Waveform wave;
    block(m.tau, 1.f, 8, &wave);
    CHECK(wave.step_s > 0);
    CHECK(wave.command[pa::Waveform::points / 4] == 64); // dans le palier rapide
    CHECK(std::abs(wave.force[pa::Waveform::points / 4] - 64) < 12);
}

TEST_CASE("PA : K retrouvé dans le vide, cas réalistes") {
    SECTION("amplitude modérée (450 g), bruit 6 g, retard 15 ms") {
        const Machine m;
        const pa::Choice c = sweep(m);
        INFO("k=" << c.k << " conf=" << int(c.confidence) << " seg=" << c.k_segments << " signed=" << c.k_signed << " depth=" << c.depth);
        REQUIRE(c.fit == pa::Fit::ok);
        CHECK_THAT(c.k, WithinAbs(m.tau, 0.006));
        CHECK(c.confidence >= 50);
    }
    SECTION("forte amplitude (2,5 kg), bruit de 60 g comme les mesures de PrusaPATuner") {
        Machine m;
        m.delta_g = 2500.f;
        m.offset_g = 700.f;
        m.noise_g = 60.f;
        const pa::Choice c = sweep(m);
        INFO("k=" << c.k << " conf=" << int(c.confidence) << " seg=" << c.k_segments << " signed=" << c.k_signed);
        REQUIRE(c.fit == pa::Fit::ok);
        CHECK_THAT(c.k, WithinAbs(m.tau, 0.006));
    }
    SECTION("buse plus lente (τ = 0,06 s), cellule dans l'autre sens, retard 30 ms, dérive forte") {
        Machine m;
        m.tau = 0.06f;
        m.cell_sign = 1.f;
        m.delay_s = 0.03f;
        m.drift_g_per_s = 15.f;
        const pa::Choice c = sweep(m);
        INFO("k=" << c.k << " conf=" << int(c.confidence));
        REQUIRE(c.fit == pa::Fit::ok);
        CHECK_THAT(c.k, WithinAbs(m.tau, 0.008));
    }
    SECTION("buse rapide (τ = 0,02 s)") {
        Machine m;
        m.tau = 0.02f;
        const pa::Choice c = sweep(m);
        INFO("k=" << c.k << " conf=" << int(c.confidence));
        REQUIRE(c.fit == pa::Fit::ok);
        CHECK_THAT(c.k, WithinAbs(m.tau, 0.006));
    }
}

TEST_CASE("PA : refus quand le signal est trop faible, ne dépend pas de K ou sort de la plage") {
    SECTION("créneau noyé dans le bruit") {
        Machine m;
        m.delta_g = 4.f;
        m.offset_g = 2.f;
        m.noise_g = 6.f;
        const pa::Choice c = sweep(m, {}, 0.f, 0.10f, 0.01f, 0);
        CHECK((c.fit == pa::Fit::weak_signal || c.fit == pa::Fit::too_few_points));
    }
    SECTION("la tête ne bouge pas en X : pas de créneau") {
        std::vector<pa::SweepPoint> points;
        for (int i = 0; i < 8; ++i) {
            std::vector<force::TracePoint> t(1500, force::make_point(0, 0));
            points.push_back({ 0.01f * float(i), pa::analyse_block(t, 0.0094f, {}, 0.01f * float(i)) });
            CHECK(points.back().r.found == 0);
        }
        CHECK(pa::choose_k(points, 0.01f).fit == pa::Fit::too_few_points);
    }
    SECTION("la force ne dépend pas de K (PA sans effet)") {
        // Même réponse à tous les K : le coût est plat
        std::vector<pa::SweepPoint> points;
        const pa::Params params;
        Machine m;
        for (int i = 0; i <= 10; ++i) {
            float dt;
            const auto trace = simulate(params, m, 0.f, i % 2 ? -1.f : 1.f, 40 + i, &dt);
            points.push_back({ 0.01f * float(i), pa::analyse_block(trace, dt, params, 0.01f * float(i)) });
        }
        const pa::Choice c = pa::choose_k(points, 0.01f);
        INFO("fit=" << int(c.fit) << " k=" << c.k << " depth=" << c.depth << " conf=" << int(c.confidence));
        CHECK((c.fit == pa::Fit::no_crossing || c.confidence < 30));
    }
    SECTION("K idéal au-delà du balayage") {
        Machine m;
        m.tau = 0.09f;
        const pa::Choice c = sweep(m, {}, 0.f, 0.05f, 0.005f, 0);
        CHECK(c.fit == pa::Fit::out_of_range);
    }
}

TEST_CASE("PA : CSV écrit puis relu, analyse rejouée à l'identique") {
    const pa::Params params;
    const Machine m;
    std::string csv = "# OneDrop3D PA (F4) test\n";
    char line[96];
    REQUIRE(pa::format_params(line, sizeof(line), params) > 0);
    csv += line;
    std::vector<pa::SweepPoint> direct;
    float direction = 1.f;
    for (int i = 0; i <= 10; ++i) {
        const float k = 0.01f * float(i);
        float dt;
        const auto trace = simulate(params, m, k, direction, 100 + i, &dt);
        direction = -direction;
        direct.push_back({ k, pa::analyse_block(trace, dt, params, k) });
        csv += "B,0.0100,123.40,0.009375,1500,0\n"; // lignes sans D : ignorées
        write_block(csv, trace, dt, k, params);
        csv += "R,ignored\n";
    }
    const pa::Choice expected = pa::choose_k(direct, 0.01f);
    REQUIRE(expected.fit == pa::Fit::ok);

    std::istringstream in(csv);
    const Replay r = replay(in);
    REQUIRE(r.has_params);
    CHECK(r.params.cycles == params.cycles);
    CHECK_THAT(r.params.fast_s, WithinAbs(params.fast_s, 1e-3));
    REQUIRE(r.points.size() == direct.size());
    const pa::Choice again = pa::choose_k(r.points, 0.01f);
    CHECK(again.fit == pa::Fit::ok);
    // Arrondis du CSV (0,01 mm, 0,01 g, pas de temps reconstitué) : même K à moins d'un dixième de pas
    CHECK_THAT(again.k, WithinAbs(expected.k, 0.001));

    // Lignes mal formées refusées
    pa::CsvPoint p;
    CHECK_FALSE(pa::parse_point("D,1,2,3", p));
    CHECK_FALSE(pa::parse_point("R,1,2,3,4,5,6", p));
    CHECK_FALSE(pa::parse_point("D,1,2,3,4,5,x", p));
    pa::Params q;
    CHECK_FALSE(pa::parse_params("P,1,2,3", q));
    CHECK(pa::parse_point("D,0.0300,1.2500,0.40,3.10,-512.25,14.00", p));
    CHECK_THAT(p.force_g, WithinAbs(-512.25, 1e-3));
    CHECK(pa::parse_point("D,0.0200,0.4684,0.53,-655.34\r", p)); // ancien format
    CHECK_THAT(p.force_g, WithinAbs(-655.34, 1e-3));
    CHECK(std::isnan(p.command));
    CHECK_FALSE(pa::parse_point("D,0.0200,0.4684,0.53,-655.34,", p));
}

TEST_CASE("PA : relevé de la machine du 22/11 (OD3D_PA3.CSV, motif imprimé) : signal saturé reconnu") {
    // Extrait du vrai fichier (K = 0 et K = 0,10) : dès que l'extrudeur pousse, la force reste à −655 g, la borne de
    // l'ancienne unité ; « la force ne suit pas K » venait de là. Refus explicite au lieu d'un faux diagnostic.
    std::FILE *f = std::fopen(OD3D_PA_DATA_DIR "/OD3D_PA3_220.CSV", "r");
    REQUIRE(f);
    std::string text;
    char buf[4096];
    for (size_t n; (n = std::fread(buf, 1, sizeof(buf), f)) > 0;) {
        text.append(buf, n);
    }
    std::fclose(f);
    std::istringstream in(text);
    const Replay r = replay(in);
    CHECK_FALSE(r.has_params); // ancien format : pas de ligne P
    REQUIRE(r.points.size() == 2);
    CHECK(r.points[0].k == 0.f);
    CHECK_THAT(r.points[1].k, WithinAbs(0.10, 1e-6));
    for (const pa::SweepPoint &p : r.points) {
        CHECK(p.r.saturated);
    }
    CHECK(pa::choose_k(r.points, 0.02f).fit == pa::Fit::saturated);
}

TEST_CASE("PA : rejouer un CSV de la machine (OD3D_PA_CSV)") {
    const char *path = std::getenv("OD3D_PA_CSV");
    if (!path || !*path) {
        SUCCEED("OD3D_PA_CSV non défini : rien à rejouer");
        return;
    }
    FILE *f = std::fopen(path, "r");
    REQUIRE(f);
    std::string text;
    char buf[4096];
    for (size_t n; (n = std::fread(buf, 1, sizeof(buf), f)) > 0;) {
        text.append(buf, n);
    }
    std::fclose(f);
    std::istringstream in(text);
    const Replay r = replay(in);
    CHECK(r.has_params);
    std::vector<float> costs(r.points.size());
    const pa::Choice c = pa::choose_k(r.points, 0.01f, {}, costs);
    std::printf("%s : %zu lignes, %zu K\n", path, r.lines, r.points.size());
    for (size_t i = 0; i < r.points.size(); ++i) {
        const pa::BlockResult &b = r.points[i].r;
        std::printf("K=%.4f sature=%d trouves=%u retenus=%u amplitude=%.1f g bruit=%.1f g cout=%.3f aire=%.3f k_est=%.4f\n", double(r.points[i].k), int(b.saturated), b.found, b.cycles,
            double(b.amplitude_g), double(b.noise_g), double(costs[i]), double(b.fall_area), double(b.k_estimate));
    }
    std::printf("choix : fit=%d K=%.4f confiance=%u creux=%.2f k_creneaux=%.4f k_signe=%.4f\n", int(c.fit), double(c.k), c.confidence, double(c.depth),
        double(c.k_segments), double(c.k_signed));
}

TEST_CASE("PA : grilles de K") {
    std::array<float, 32> out;
    REQUIRE(pa::sweep_values(0.f, 0.1f, 0.01f, out) == 11);
    CHECK_THAT(out[10], WithinAbs(0.1, 1e-6));
    CHECK(pa::sweep_values(0.1f, 0.f, 0.02f, out) == 0);
    CHECK(pa::sweep_values(0.f, 0.1f, 0.f, out) == 0);

    REQUIRE(pa::refine_values(0.043f, 0.005f, 4, 0.f, 0.3f, out) == 4);
    CHECK_THAT(out[0], WithinAbs(0.0355, 1e-6));
    CHECK_THAT(out[3], WithinAbs(0.0505, 1e-6));
    // Près du bord : la grille glisse et reste dans [lo, hi]
    REQUIRE(pa::refine_values(0.002f, 0.005f, 5, 0.f, 0.3f, out) == 5);
    CHECK_THAT(out[0], WithinAbs(0.0, 1e-6));
    CHECK_THAT(out[4], WithinAbs(0.02, 1e-6));
}

TEST_CASE("PA : choix sur des coûts connus, parabole, interpolation, médiane, mémoire") {
    // Un seul critère qui dépend de K : rise_error = (K − 0,037)², les autres nuls ; minimum interpolé
    std::vector<pa::SweepPoint> points;
    for (int i = 0; i <= 10; ++i) {
        pa::BlockResult r;
        r.found = 5;
        r.cycles = 4;
        r.amplitude_g = 300.f;
        r.noise_g = 5.f;
        const float k = 0.01f * float(i);
        r.m[pa::rise_error] = (k - 0.037f) * (k - 0.037f) + 1e-4f;
        r.fall_area = 0.037f - k;
        r.k_estimate = 0.037f;
        points.push_back({ k, r });
    }
    std::array<float, 11> costs;
    const pa::Choice c = pa::choose_k(points, 0.01f, {}, costs);
    REQUIRE(c.fit == pa::Fit::ok);
    CHECK_THAT(c.k, WithinAbs(0.037, 0.0015));
    CHECK_THAT(c.k_signed, WithinAbs(0.037, 1e-4));
    CHECK_THAT(c.k_segments, WithinAbs(0.037, 1e-6));
    CHECK(c.confidence >= 80);
    CHECK(c.used == 11);
    CHECK(costs[4] < costs[0]);
    CHECK(costs[4] < costs[10]);
    // Un K au signal trop faible est écarté (coût NAN)
    points[2].r.amplitude_g = 3.f;
    const pa::Choice d = pa::choose_k(points, 0.01f, {}, costs);
    CHECK(d.used == 10);
    CHECK(std::isnan(costs[2]));
    // Minimum au dernier K : hors plage
    for (pa::SweepPoint &p : points) {
        p.r.m[pa::rise_error] = (0.2f - p.k) * (0.2f - p.k) + 1e-4f;
    }
    CHECK(pa::choose_k(points, 0.01f).fit == pa::Fit::out_of_range);

    const float lx[] = { 0.f, 1.f, 2.f, 3.f };
    const float ly[] = { 1.f, 3.f, 5.f, 7.f };
    const pa::LineFit f = pa::fit_line(lx, ly);
    CHECK(f.ok);
    CHECK_THAT(f.b, WithinAbs(2.0, 1e-5));
    CHECK_THAT(f.r2, WithinAbs(1.0, 1e-5));

    // y = (x − 0,037)² + 2 sur trois K non équidistants
    const auto y = [](float x) { return (x - 0.037f) * (x - 0.037f) + 2.f; };
    CHECK_THAT(pa::parabola_vertex(0.02f, y(0.02f), 0.03f, y(0.03f), 0.05f, y(0.05f)), WithinAbs(0.037, 1e-4));
    CHECK_THAT(pa::parabola_vertex(0.f, 1.f, 1.f, 2.f, 2.f, 3.f), WithinAbs(1.0, 1e-6)); // alignés

    // PA adaptatif (étape suivante) : K selon le débit, borné aux extrémités
    const float flows[] = { 5.f, 10.f, 20.f };
    const float ks[] = { 0.05f, 0.04f, 0.03f };
    CHECK_THAT(pa::interpolate(flows, ks, 7.5f), WithinAbs(0.045, 1e-6));
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

TEST_CASE("trace de force : unités et bornes") {
    const force::TracePoint p = force::make_point(-512.3f, 1.234f, 35.5f);
    CHECK_THAT(force::force_g(p), WithinAbs(-512.25, 1e-3)); // pas de 0,25 g
    CHECK_THAT(force::x_mm(p), WithinAbs(1.23, 1e-4));
    CHECK_THAT(force::e_mm(p), WithinAbs(35.5, 1e-4));
    // Un palier de 3 kg n'est plus écrêté (l'ancienne unité s'arrêtait à ±655 g)
    CHECK_THAT(force::force_g(force::make_point(3000.f, 0)), WithinAbs(3000.0, 1e-3));
    // Saturation au lieu du débordement ; une séquence écrêtée est refusée
    CHECK(force::make_point(1e6f, 0).force == 32767);
    {
        const pa::Params params;
        float dt;
        auto trace = simulate(params, Machine {}, 0.03f, 1.f, 9, &dt);
        for (size_t i = trace.size() / 4; i < trace.size() / 2; ++i) {
            trace[i].force = -32767;
        }
        const pa::BlockResult r = pa::analyse_block(trace, dt, params, 0.03f);
        CHECK(r.saturated);
        std::vector<pa::SweepPoint> points(6, { 0.03f, r });
        CHECK(pa::choose_k(points, 0.01f).fit == pa::Fit::saturated);
    }
    CHECK(force::make_point(-1e6f, 0).force == -32767);
    CHECK(force::make_point(NAN, NAN, NAN).x == 0);
}

TEST_CASE("motif à l'œil : cordon, repères, lignes, palpage") {
    const pattern::Bead bead { 0.45f, 0.2f };
    CHECK(pattern::is_valid(bead));
    CHECK_FALSE(pattern::is_valid({ 0.2f, 0.2f })); // plus étroit que haut
    CHECK_FALSE(pattern::is_valid({ NAN, 0.2f }));
    CHECK_THAT(pattern::e_per_mm(bead), WithinAbs(0.09 / 2.4053, 1e-5));
    CHECK_THAT(pattern::speed_for_flow(bead, 9.f), WithinAbs(100.0, 1e-3));
    CHECK(pattern::speed_for_flow({ 0.f, 0.f }, 9.f) == 0.f);

    // Ligne par défaut : 4 tronçons lents de 13,3 mm et 3 rapides de 33,3 mm
    const pattern::PaLine line;
    CHECK(pattern::is_valid_line(line));
    CHECK_THAT(pattern::line_length(line), WithinAbs(153.3, 0.1));
    pattern::PaLine bad = line;
    bad.fast_flow = 2.5f;
    CHECK_FALSE(pattern::is_valid_line(bad));
    bad = line;
    bad.bead.height = 0.6f; // pas une première couche
    CHECK_FALSE(pattern::is_valid_line(bad));
    bad = line;
    bad.fast_flow = 30.f; // 333 mm/s : trop vite
    CHECK_FALSE(pattern::is_valid_line(bad));
    bad = line;
    bad.cycles = 6; // ligne de plus de 200 mm
    CHECK_FALSE(pattern::is_valid_line(bad));

    // Numéro 7 : sept traits, le 5e long
    std::array<pattern::Tick, 16> ticks;
    REQUIRE(pattern::tally(7, ticks) == 7);
    CHECK(ticks[4].length == pattern::tick_long);
    CHECK(ticks[3].length == pattern::tick_short);
    CHECK_THAT(ticks[6].dx, WithinAbs(6 * pattern::tick_pitch, 1e-6));
    CHECK(pattern::tally(0, ticks) == 0);
    CHECK(pattern::tally(20, std::span(ticks.data(), 3)) == 3);
    CHECK_THAT(pattern::tally_width(11), WithinAbs(pattern::tick_gap + 10 * pattern::tick_pitch, 1e-6));

    // 11 lignes : le motif tient à l'avant du plateau de la CORE One (250 × 220, 10 mm de marge)
    pattern::Lines lines;
    lines.x0 = 20.f;
    lines.y0 = 12.f;
    lines.pitch = 5.f;
    lines.length = pattern::line_length(line);
    lines.count = 11;
    lines.marks = pattern::tally_width(lines.count);
    lines.bead = line.bead;
    const pattern::Rect area = lines.area();
    const pattern::Rect zone { 10.f, 10.f, 240.f, 210.f };
    CHECK(zone.contains(area));
    CHECK_THAT(area.y1, WithinAbs(12.0 + 50.0 + pattern::tick_long, 1e-4));
    lines.x0 = 5.f;
    CHECK_FALSE(zone.contains(lines.area()));

    const pattern::Rect bed { 0.f, 0.f, 250.f, 220.f };
    const pattern::Rect probe = pattern::probe_rect(area, 22.f, 22.f, bed);
    CHECK(probe.x0 == 0.f);
    CHECK(bed.contains(probe));
    CHECK(pattern::Rect { 0.f, 0.f, 10.f, 10.f }.clipped({ 20.f, 20.f, 30.f, 30.f }).empty());
}
