// OneDrop3D (F10 réduite) : moyenne, écart et alerte de dérive de la pointe du filament par emplacement.
#include <feature/od3d/tip_math_od3d.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace od3d::tip;
using Catch::Approx;

namespace {

/// Pointe habituelle : 14 mm ± 0,5 (alternance, pas de hasard dans un test)
float usual(int i) {
    return 14.f + (i % 2 ? 0.5f : -0.5f);
}

} // namespace

TEST_CASE("tip: exact mean and spread for the first changes") {
    Stats s;
    for (float mm : { 13.f, 15.f, 14.f, 14.f }) {
        s = add(s, mm);
    }
    CHECK(s.count == 4);
    CHECK(s.mean == Approx(14.f));
    CHECK(s.range == Approx(1.f)); // écarts 2, 1, 0
    CHECK(sigma(s) == Approx(1.f / d2));
    CHECK(s.last == 14.f);
}

TEST_CASE("tip: readings where the sensor never switched are ignored") {
    Stats s = add(Stats {}, 14.f);
    CHECK(add(s, 0.f) == s);
    CHECK(add(s, -3.f) == s);
    CHECK(add(s, max_mm + 1.f) == s);
    CHECK(add(s, NAN) == s);
}

TEST_CASE("tip: steady tips never alert") {
    Stats s;
    for (int i = 0; i < 500; ++i) {
        s = add(s, usual(i));
        CHECK_FALSE(drifting(s));
    }
    CHECK(s.mean == Approx(14.f).margin(0.05f));
    CHECK(sigma(s) == Approx(1.f / d2)); // écart de 1 mm à chaque changement
}

TEST_CASE("tip: no alert before 10 changes") {
    Stats s;
    for (int i = 0; i < min_count - 1; ++i) {
        s = add(s, i < 5 ? 14.f : 8.f);
    }
    CHECK_FALSE(drifting(s));
}

TEST_CASE("tip: a shorter tip (filament left in the melt zone) alerts within a few changes") {
    Stats s;
    for (int i = 0; i < 40; ++i) {
        s = add(s, usual(i));
    }
    int seen = -1;
    for (int i = 0; i < 10 && seen < 0; ++i) {
        s = add(s, usual(i) - 2.f); // 2 mm plus court
        seen = drifting(s) ? i + 1 : -1;
    }
    CHECK(seen > 0);
    CHECK(seen <= 5);
    CHECK(sigma(s) < 1.2f); // la dérive n'a élargi l'écart qu'une fois
}

TEST_CASE("tip: small changes under 1 mm never alert, even with a tiny spread") {
    Stats s;
    for (int i = 0; i < 40; ++i) {
        s = add(s, 14.f + (i % 2 ? 0.05f : -0.05f));
    }
    for (int i = 0; i < 40; ++i) {
        s = add(s, 14.8f);
        CHECK_FALSE(drifting(s));
    }
}

TEST_CASE("tip: the alert stops once the new tip is the usual one") {
    Stats s;
    for (int i = 0; i < 40; ++i) {
        s = add(s, usual(i));
    }
    for (int i = 0; i < 200; ++i) {
        s = add(s, usual(i) + 3.f);
    }
    CHECK_FALSE(drifting(s));
    CHECK(s.mean == Approx(17.f).margin(0.1f));
}
