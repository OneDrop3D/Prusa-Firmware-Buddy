// OneDrop3D (M2003) : pente de la dilatation de la buse tirée des paliers palpés. Régression linéaire, dérive régulière
// sans effet sur la pente, refus d'une mesure trop dispersée ou hors de la droite, pente impossible ; et sans mesure,
// la compensation garde le modèle de Prusa.
#include <feature/nozzle_thermal_compensation/nozzle_thermal_compensation.hpp>
#include <feature/od3d/nozzle_expansion_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cstdlib>

using namespace od3d::nozzle_expansion;
namespace comp = buddy::nozzle_thermal_compensation;
using Catch::Matchers::WithinAbs;

namespace {

constexpr float um = 0.001f;

/// Paliers d'une buse de pente slope [mm/°C], plateau à z0, plus une dérive de drift par palier (le temps)
std::array<Stage, stages> synthetic(float slope, float z0 = 0.25f, float drift = 0.f, float spread = 0.004f) {
    std::array<Stage, stages> s;
    for (size_t i = 0; i < stages; ++i) {
        s[i] = Stage { z0 + slope * (temperatures[i] - 170.f) + drift * float(i), spread };
    }
    return s;
}

} // namespace

TEST_CASE("nozzle_expansion: least squares line") {
    const std::array<float, 4> x { 0, 1, 2, 3 };
    const std::array<float, 4> y { 1, 3, 5, 7 };
    const Line l = fit(x, y);
    REQUIRE(l.valid);
    CHECK_THAT(l.slope, WithinAbs(2.f, 1e-6f));
    CHECK_THAT(l.intercept, WithinAbs(1.f, 1e-6f));

    // Bruit symétrique : la droite passe au milieu
    const std::array<float, 4> noisy { 1.1f, 2.9f, 5.1f, 6.9f };
    CHECK_THAT(fit(x, noisy).slope, WithinAbs(1.96f, 1e-5f));

    // Moins de deux x différents : pas de droite
    const std::array<float, 2> same { 4, 4 };
    CHECK_FALSE(fit(same, std::array<float, 2> { 1, 2 }).valid);
    CHECK_FALSE(fit(std::span<const float>(x).first(1), std::span<const float>(y).first(1)).valid);
}

TEST_CASE("nozzle_expansion: taps of a stage") {
    const std::array<float, taps> z { 0.250f, 0.254f, 0.252f };
    const Stage s = stage_of(z);
    CHECK_THAT(s.z_mm, WithinAbs(0.252f, 1e-6f));
    CHECK_THAT(s.spread_mm, WithinAbs(0.004f, 1e-6f));
}

TEST_CASE("nozzle_expansion: measured slope") {
    SECTION("Prusa's brass nozzle: 0.41 um/C, 37 um from 170 to 260") {
        const Outcome o = evaluate(synthetic(comp::prusa_slope_mm_per_c));
        CHECK(o.verdict == Verdict::ok);
        CHECK_THAT(o.slope_mm_per_c, WithinAbs(comp::prusa_slope_mm_per_c, 1e-7f));
        CHECK_THAT(o.residual_mm, WithinAbs(0.f, 1e-6f));
        CHECK_THAT(o.hysteresis_mm, WithinAbs(0.f, 1e-6f));
    }

    SECTION("A longer nozzle (0.55 um/C) is 11 um past Prusa's model at 250 C") {
        const Outcome o = evaluate(synthetic(0.55f * um));
        REQUIRE(o.verdict == Verdict::ok);
        const float delta_250 = (o.slope_mm_per_c - comp::prusa_slope_mm_per_c) * (250.f - comp::reference_temperature_c);
        CHECK_THAT(delta_250, WithinAbs(0.0112f, 0.0001f));
    }

    SECTION("A steady drift in time (bed, frame, load cell warming up) leaves the slope unchanged") {
        // 3 um par palier : 12 um du premier au dernier, plus qu'il n'en faut pour fausser un simple aller
        const Outcome o = evaluate(synthetic(0.45f * um, 0.25f, 3.f * um));
        CHECK(o.verdict == Verdict::ok);
        CHECK_THAT(o.slope_mm_per_c, WithinAbs(0.45f * um, 1e-7f));
        CHECK_THAT(o.hysteresis_mm, WithinAbs(9.f * um, 1e-6f)); // retour − aller : (12 + 6) / 2
    }
}

TEST_CASE("nozzle_expansion: rejected measurements") {
    SECTION("Scattered taps") {
        auto s = synthetic(comp::prusa_slope_mm_per_c);
        s[2].spread_mm = 0.025f;
        const Outcome o = evaluate(s);
        CHECK(o.verdict == Verdict::spread);
        CHECK_THAT(o.spread_mm, WithinAbs(0.025f, 1e-6f));
    }

    SECTION("Back and forth disagree (hysteresis): a stage far from the line") {
        auto s = synthetic(comp::prusa_slope_mm_per_c);
        s[4].z_mm += 0.030f; // retour à 170 °C : 30 µm plus haut
        const Outcome o = evaluate(s);
        CHECK(o.verdict == Verdict::residual);
        CHECK(o.residual_mm > max_residual_mm);
        CHECK_THAT(o.hysteresis_mm, WithinAbs(0.015f, 1e-6f));
    }

    SECTION("A small disagreement is kept") {
        auto s = synthetic(comp::prusa_slope_mm_per_c);
        s[3].z_mm += 0.004f;
        CHECK(evaluate(s).verdict == Verdict::ok);
    }

    SECTION("Implausible slopes") {
        CHECK(evaluate(synthetic(0.f)).verdict == Verdict::slope);
        CHECK(evaluate(synthetic(-0.4f * um)).verdict == Verdict::slope);
        // 3 µm/°C : 270 µm sur l'aller, la droite reste droite mais la pente est impossible
        CHECK(evaluate(synthetic(3.f * um)).verdict == Verdict::slope);
    }

    SECTION("A missing stage is never accepted") {
        auto s = synthetic(comp::prusa_slope_mm_per_c);
        s[3] = Stage {};
        CHECK(evaluate(s).verdict != Verdict::ok);
    }
}

TEST_CASE("nozzle_expansion: back to Prusa's model without a measurement") {
    // Rien de gardé (0) : la pente de Prusa, et la compensation à l'identique de la sienne
    CHECK(comp::slope_from_nm(0) == comp::prusa_slope_mm_per_c);
    for (const float t : { 150.f, 170.f, 215.f, 250.f, 290.f }) {
        CHECK(comp::elongation_vs_reference_mm(t, comp::slope_from_nm(0)) == comp::elongation_vs_reference_mm(t));
    }

    // Une pente gardée remplace la sienne ; l'aller-retour en nm/°C est exact au nm près
    CHECK_THAT(comp::slope_from_nm(550), WithinAbs(0.55f * um, 1e-9f));
    CHECK(comp::slope_to_nm(0.55f * um) == 550);
    CHECK(std::abs(int(comp::slope_to_nm(comp::prusa_slope_mm_per_c)) - 410) <= 1); // 0,4095 µm/°C
    CHECK_THAT(comp::elongation_vs_reference_mm(250.f, comp::slope_from_nm(550)), WithinAbs(0.044f, 1e-6f));
    CHECK(comp::slope_to_nm(-1.f) == 0); // jamais gardée : modèle de Prusa
    CHECK(comp::slope_to_nm(NAN) == 0);

    // Une pente par buse réglée dans Prusa
    CHECK(comp::nozzle_key(0, false, false) == 0);
    CHECK(comp::nozzle_key(0, true, false) != comp::nozzle_key(0, false, false)); // buse HF
    CHECK(comp::nozzle_key(1, false, false) != comp::nozzle_key(0, false, false)); // chaussette
    CHECK(comp::nozzle_key(3, true, true) < comp::nozzle_keys); // hotend haute température, HF, trempée
}
