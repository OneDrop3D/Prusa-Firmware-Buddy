// OneDrop3D (F15) : facteur d'équerrage XY d'après les diagonales AC, BD et le côté AD (formule de Marlin).
#include <feature/od3d/skew_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <limits>

using namespace od3d::skew;
using Catch::Matchers::WithinAbs;

namespace {

/// Mesures d'un parallélogramme de côté AD = side, de hauteur height, dont le haut (B, C) est décalé de shift en X
struct Measures {
    float ac, bd, ad;
};

Measures piece(double side, double height, double shift) {
    return { float(std::hypot(side + shift, height)), float(std::hypot(side - shift, height)), float(side) };
}

} // namespace

TEST_CASE("équerrage : pièce d'équerre") {
    const Result r = factor_from_measures(141.42f, 141.42f, 100.0f);
    CHECK(r.status == Status::ok);
    CHECK_THAT(r.factor, WithinAbs(0.0, 1e-4));

    // Valeurs par défaut de Marlin (Configuration.h, SKEW_CORRECTION)
    const Result marlin = factor_from_measures(282.8427124746f, 282.8427124746f, 200.0f);
    CHECK(marlin.status == Status::ok);
    CHECK_THAT(marlin.factor, WithinAbs(0.0, 1e-5));
}

TEST_CASE("équerrage : facteur = décalage / hauteur, signe selon la diagonale la plus longue") {
    for (const double shift : { 0.05, 0.2, -0.2, 0.5, -0.9 }) {
        CAPTURE(shift);
        const Measures m = piece(100, 100, shift);
        const Result r = factor_from_measures(m.ac, m.bd, m.ad);
        REQUIRE(r.status == Status::ok);
        CHECK_THAT(r.factor, WithinAbs(shift / 100, 2e-5)); // diagonales arrondies en float : ~1e-5 près
    }

    // Rectangle (AB différent de AD) : seule la hauteur compte
    const Measures m = piece(100, 150, 0.3);
    const Result r = factor_from_measures(m.ac, m.bd, m.ad);
    REQUIRE(r.status == Status::ok);
    CHECK_THAT(r.factor, WithinAbs(0.3 / 150, 2e-5));

    // AC > BD : B penche vers +X, facteur positif
    CHECK(factor_from_measures(141.60f, 141.24f, 100.0f).factor > 0);
    CHECK(factor_from_measures(141.24f, 141.60f, 100.0f).factor < 0);
}

TEST_CASE("équerrage : mesures refusées") {
    // Écart au-delà de 0,57° (k > 0,01)
    const Measures big = piece(100, 100, 2);
    CHECK(factor_from_measures(big.ac, big.bd, big.ad).status == Status::too_large);

    // Juste sous la limite
    const Measures edge = piece(100, 100, 0.99);
    CHECK(factor_from_measures(edge.ac, edge.bd, edge.ad).status == Status::ok);

    // Incohérentes : côté faux (10 au lieu de 100), diagonales trop courtes pour le côté, hors limites, non finies
    CHECK(factor_from_measures(141.42f, 141.42f, 10.0f).status == Status::inconsistent);
    CHECK(factor_from_measures(100.0f, 100.0f, 100.0f).status == Status::inconsistent);
    CHECK(factor_from_measures(60.0f, 60.0f, 100.0f).status == Status::inconsistent);
    CHECK(factor_from_measures(0.0f, 141.42f, 100.0f).status == Status::inconsistent);
    CHECK(factor_from_measures(141.42f, 1000.0f, 100.0f).status == Status::inconsistent);
    CHECK(factor_from_measures(std::numeric_limits<float>::quiet_NaN(), 141.42f, 100.0f).status == Status::inconsistent);
    CHECK(factor_from_measures(141.42f, std::numeric_limits<float>::infinity(), 100.0f).status == Status::inconsistent);
    CHECK(factor_from_measures(141.42f, 141.42f, 100.0f).status == Status::ok);

    // Un refus rend toujours 0
    CHECK(factor_from_measures(big.ac, big.bd, big.ad).factor == 0.0f);
}

TEST_CASE("équerrage : degrés et facteur valide") {
    CHECK_THAT(factor_to_degrees(0.0f), WithinAbs(0.0, 1e-9));
    CHECK_THAT(factor_to_degrees(0.002f), WithinAbs(0.114591, 1e-5));
    CHECK_THAT(factor_to_degrees(-0.01f), WithinAbs(-0.572939, 1e-5));

    CHECK(is_valid_factor(0.0f));
    CHECK(is_valid_factor(0.01f));
    CHECK(is_valid_factor(-0.01f));
    CHECK_FALSE(is_valid_factor(0.0101f));
    CHECK_FALSE(is_valid_factor(std::numeric_limits<float>::quiet_NaN()));
}
