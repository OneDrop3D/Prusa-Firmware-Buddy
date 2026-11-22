// OneDrop3D (F33) : amortissement réel ζ et Q par la bande à −3 dB, sur des courbes de résonance synthétiques ;
// (ACC-1) pesanteur attendue de l'accéléromètre à l'arrêt.
#include <feature/od3d/accelerometer_od3d.hpp>
#include <feature/od3d/damping_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <vector>

using namespace od3d::damping;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

/// Gain² d'un oscillateur à un degré de liberté : excité en force (|H|²) ou, comme la tête sur ses courroies, par un
/// déplacement (transmissibilité T² = (1 + (2ζr)²) |H|²), sur la grille de M1997
std::vector<float> resonance(float f0, float zeta, bool transmissibility, float height = 1.f) {
    std::vector<float> v;
    for (size_t i = 0; i < sweep_points; ++i) {
        const double r = (sweep_start_hz + i * sweep_step_hz) / f0;
        const double z2r2 = 4. * zeta * zeta * r * r;
        const double h2 = 1. / ((1. - r * r) * (1. - r * r) + z2r2);
        v.push_back(float(height * (transmissibility ? (1. + z2r2) * h2 : h2)));
    }
    return v;
}

} // namespace

TEST_CASE("amortissement : la grille de M1997") {
    CHECK(sweep_end_hz == 90.f);
    CHECK(resonance(50, 0.1f, false).size() == sweep_points);
}

TEST_CASE("amortissement : ζ et f0 retrouvés sur un oscillateur excité en force") {
    for (const float f0 : { 40.f, 55.f, 70.f }) {
        for (const float zeta : { 0.03f, 0.05f, 0.1f, 0.15f }) {
            CAPTURE(f0, zeta);
            const Resonance r = analyse(resonance(f0, zeta, false), sweep_start_hz, sweep_step_hz);
            REQUIRE(r.found);
            // Le sommet de |H|² est à f0 √(1 − 2ζ²)
            CHECK_THAT(r.frequency, WithinAbs(f0 * std::sqrt(1.f - 2.f * zeta * zeta), 0.2));
            CHECK_THAT(r.zeta, WithinRel(zeta, 0.1f));
            CHECK_THAT(r.q, WithinRel(0.5f / r.zeta, 1e-4f));
            CHECK_THAT(r.width, WithinRel(r.frequency / r.q, 1e-4f));
        }
    }
}

TEST_CASE("amortissement : ζ retrouvé sur la transmissibilité (excitation en déplacement, comme M958)") {
    for (const float zeta : { 0.05f, 0.08f, 0.12f }) {
        CAPTURE(zeta);
        const Resonance r = analyse(resonance(52, zeta, true), sweep_start_hz, sweep_step_hz);
        REQUIRE(r.found);
        CHECK_THAT(r.zeta, WithinRel(zeta, 0.1f));
        CHECK(applicable(r));
    }
}

TEST_CASE("amortissement : l'échelle du gain ne compte pas") {
    const Resonance a = analyse(resonance(48, 0.07f, true, 1.f), sweep_start_hz, sweep_step_hz);
    const Resonance b = analyse(resonance(48, 0.07f, true, 0.02f), sweep_start_hz, sweep_step_hz);
    REQUIRE(a.found);
    REQUIRE(b.found);
    CHECK_THAT(a.zeta, WithinRel(b.zeta, 1e-3f));
}

TEST_CASE("amortissement : mesure bruitée") {
    std::vector<float> v = resonance(60, 0.1f, true);
    for (size_t i = 0; i < v.size(); ++i) {
        v[i] *= 1.f + 0.05f * std::sin(float(i) * 2.3f); // ±5 %, déterministe
    }
    const Resonance r = analyse(v, sweep_start_hz, sweep_step_hz);
    REQUIRE(r.found);
    CHECK_THAT(r.zeta, WithinRel(0.1f, 0.15f));
}

TEST_CASE("amortissement : le pic le plus haut est le pic principal") {
    std::vector<float> v = resonance(42, 0.05f, true);
    const std::vector<float> second = resonance(72, 0.05f, true, 0.3f);
    for (size_t i = 0; i < v.size(); ++i) {
        v[i] += second[i];
    }
    const Resonance r = analyse(v, sweep_start_hz, sweep_step_hz);
    REQUIRE(r.found);
    CHECK_THAT(r.frequency, WithinAbs(42, 0.5));
    CHECK_THAT(r.zeta, WithinRel(0.05f, 0.15f));
}

TEST_CASE("amortissement : pas de résultat quand le pic sort du balayage ou n'existe pas") {
    // Coupure de droite au-delà de 90 Hz
    CHECK_FALSE(analyse(resonance(86, 0.1f, true), sweep_start_hz, sweep_step_hz).found);
    // Sommet au bord
    CHECK_FALSE(analyse(resonance(95, 0.05f, true), sweep_start_hz, sweep_step_hz).found);
    CHECK_FALSE(analyse(resonance(28, 0.05f, true), sweep_start_hz, sweep_step_hz).found);
    // Plat (tête rigide) : rien ne retombe sous la moitié
    CHECK_FALSE(analyse(std::vector<float>(sweep_points, 1.f), sweep_start_hz, sweep_step_hz).found);
    // Mesure incomplète
    std::vector<float> v = resonance(50, 0.1f, true);
    v.back() = NAN;
    CHECK_FALSE(analyse(v, sweep_start_hz, sweep_step_hz).found);
    CHECK_FALSE(analyse({}, sweep_start_hz, sweep_step_hz).found);
}

TEST_CASE("amortissement : valeurs proposées à l'input shaper") {
    Resonance r { .found = true, .frequency = 50, .width = 10, .q = 5, .zeta = 0.1f };
    CHECK(applicable(r));
    r.zeta = 0.01f;
    CHECK_FALSE(applicable(r));
    r.zeta = 0.4f;
    CHECK_FALSE(applicable(r));
    r = {};
    CHECK_FALSE(applicable(r));
}

TEST_CASE("accéléromètre (ACC-1) : pesanteur à l'arrêt") {
    using od3d::accel::gravity_plausible;
    CHECK(gravity_plausible(0, 0, 9.81f));
    CHECK(gravity_plausible(-6.9f, 6.9f, 0.4f)); // puce inclinée, comme sur la tête de la CORE One
    CHECK_FALSE(gravity_plausible(0, 0, 0)); // bus à 0
    constexpr float lsb = 2.f * 9.80665f / 32767.f; // un pas de ±2 g (raw_to_accel de Prusa)
    CHECK_FALSE(gravity_plausible(-lsb, -lsb, -lsb)); // bus à 1 : 0xFFFF = -1
    CHECK_FALSE(gravity_plausible(0, 0, 19.6f)); // saturé à 2 g
    CHECK_FALSE(gravity_plausible(0, 3.f, 0));
}
