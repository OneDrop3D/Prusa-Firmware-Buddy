// OneDrop3D (F10 réduite) : moyenne, écart et alerte de dérive de la pointe du filament par emplacement.
#include <feature/od3d/tip_math_od3d.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace od3d::tip;
using Catch::Approx;

namespace {

/// Pointe habituelle : 14 mm ± 0,5 (alternance, pas de hasard dans un test)
float usual(int i) {
    return 14.f + (i % 2 ? 0.5f : -0.5f);
}

/// Une suite de mesures, la précédente gardée comme dans tip_od3d.cpp
struct Feed {
    Stats s;
    float last = NAN;
    Feed &operator<<(float mm) {
        s = add(s, mm, last);
        last = valid(mm) ? mm : last;
        return *this;
    }
};

} // namespace

TEST_CASE("tip: exact mean and spread for the first changes") {
    Feed f;
    f << 13.f << 15.f << 14.f << 14.f;
    CHECK(f.s.count == 4);
    CHECK(mm_of(f.s.mean) == Approx(14.f).margin(unit_mm));
    CHECK(mm_of(f.s.range) == Approx(1.f).margin(unit_mm)); // écarts 2, 1, 0
    CHECK(sigma(f.s) == Approx(1.f / d2).margin(0.01f));
}

TEST_CASE("tip: readings where the sensor never switched are ignored") {
    const Stats s = add(Stats {}, 14.f, NAN);
    CHECK(s.count == 1);
    CHECK(add(s, 0.f, 14.f) == s);
    CHECK(add(s, -3.f, 14.f) == s);
    CHECK(add(s, max_mm + 1.f, 14.f) == s);
    CHECK(add(s, NAN, 14.f) == s);
}

TEST_CASE("tip: after a restart the first jump is not counted") {
    Feed f;
    f << 14.f << 15.f; // écart 1
    const Stats before = f.s;
    const Stats after = add(before, 20.f, NAN); // redémarrage : la mesure d'avant est perdue
    CHECK(after.range == before.range);
    CHECK(after.count == 3);
}

TEST_CASE("tip: 16-bit storage") {
    CHECK(units_of(14.f) == 7000);
    CHECK(mm_of(units_of(13.37f)) == Approx(13.37f).margin(unit_mm));
    CHECK(units_of(-1.f) == 0);
    CHECK(units_of(1000.f) == 65535);
}

TEST_CASE("tip: steady tips never alert") {
    Feed f;
    for (int i = 0; i < 500; ++i) {
        f << usual(i);
        CHECK_FALSE(drifting(f.s));
    }
    CHECK(f.s.count == 255); // plafonné
    CHECK(mm_of(f.s.mean) == Approx(14.f).margin(0.1f));
    CHECK(sigma(f.s) == Approx(1.f / d2).margin(0.01f)); // écart de 1 mm à chaque changement
}

TEST_CASE("tip: no alert before 10 changes") {
    Feed f;
    for (int i = 0; i < min_count - 1; ++i) {
        f << (i < 5 ? 14.f : 8.f);
    }
    CHECK_FALSE(drifting(f.s));
}

TEST_CASE("tip: a shorter tip (filament left in the melt zone) alerts within a few changes") {
    Feed f;
    for (int i = 0; i < 40; ++i) {
        f << usual(i);
    }
    int seen = -1;
    for (int i = 0; i < 10 && seen < 0; ++i) {
        f << usual(i) - 2.f; // 2 mm plus court
        seen = drifting(f.s) ? i + 1 : -1;
    }
    CHECK(seen > 0);
    CHECK(seen <= 5);
    CHECK(sigma(f.s) < 1.2f); // la dérive n'a élargi l'écart qu'une fois
}

TEST_CASE("tip: small changes under 1 mm never alert, even with a tiny spread") {
    Feed f;
    for (int i = 0; i < 40; ++i) {
        f << 14.f + (i % 2 ? 0.05f : -0.05f);
    }
    for (int i = 0; i < 40; ++i) {
        f << 14.8f;
        CHECK_FALSE(drifting(f.s));
    }
}

TEST_CASE("tip: the alert stops once the new tip is the usual one") {
    Feed f;
    for (int i = 0; i < 40; ++i) {
        f << usual(i);
    }
    for (int i = 0; i < 200; ++i) {
        f << usual(i) + 3.f;
    }
    CHECK_FALSE(drifting(f.s));
    CHECK(mm_of(f.s.mean) == Approx(17.f).margin(0.1f));
}
