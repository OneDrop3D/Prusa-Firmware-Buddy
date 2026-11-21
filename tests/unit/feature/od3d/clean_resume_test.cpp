// OneDrop3D (F30) : reprise sans cicatrice. Option coupée = rien ; purge dans les limites ; désrétraction de la
// pause comptée ; E recalé pour que la désrétraction de Prusa pousse exactement la rétraction ; coupure de courant.
#include <feature/od3d/clean_resume_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

using namespace od3d::clean_resume;
using Catch::Matchers::WithinAbs;

namespace {
// CORE One : X -1 à 252, Y -19 à 220 ; purge de Prusa en (202, -13)
constexpr Box core_one { -1, 252, -19, 220 };
constexpr Settings on { .enabled = true, .purge_mm = 8, .retract_tenths = 8, .approach_mm_s = 2 };
} // namespace

TEST_CASE("reprise : option coupée, rien ne change", "[od3d][resume]") {
    const Settings off {};
    CHECK_FALSE(off.enabled); // coupée par défaut
    CHECK_FALSE(plan_pause(off, 202, -13, 100, 92, core_one).has_value());
    CHECK_FALSE(plan_power_panic(off, 202, -13, 8, 50, core_one).has_value());
}

TEST_CASE("reprise après pause", "[od3d][resume]") {
    // Pause : E de la pièce 1234,5 ; la pause a rétracté 8 mm (STANDARD_RETRACT_LENGTH)
    const auto p = plan_pause(on, 202, -13, 1234.5f, 1226.5f, core_one);
    REQUIRE(p.has_value());
    CHECK(p->purge_x == 202);
    CHECK(p->purge_y == -13);
    CHECK_THAT(p->extrude_mm, WithinAbs(8 + 8, 1e-4)); // désrétraction de la pause + purge
    CHECK_THAT(p->retract_mm, WithinAbs(0.8, 1e-6));
    // Recalage : Prusa désrétracte vers 1234,5, soit exactement les 0,8 mm rétractés après la purge
    CHECK_THAT(1234.5f - p->e_after, WithinAbs(0.8, 1e-4));
    CHECK_THAT(p->wipe_x, WithinAbs(192, 1e-6));
    CHECK(std::abs(p->wipe_x - p->purge_x) == wipe_mm);
}

TEST_CASE("reprise : position de E incohérente, on laisse Prusa", "[od3d][resume]") {
    CHECK_FALSE(plan_pause(on, 202, -13, 100, 101, core_one).has_value()); // E en avance
    CHECK_FALSE(plan_pause(on, 202, -13, 100, 80, core_one).has_value()); // 20 mm derrière
    CHECK_FALSE(plan_pause(on, 202, -13, NAN, 80, core_one).has_value());
    CHECK(plan_pause(on, 202, -13, 100, 100, core_one).has_value()); // pas de rétraction de pause : purge seule
}

TEST_CASE("reprise : purge et essuyage dans les limites", "[od3d][resume]") {
    const auto right = plan_pause(on, 400, -50, 10, 2, core_one);
    REQUIRE(right.has_value());
    CHECK(right->purge_x == 252);
    CHECK(right->purge_y == -19);
    CHECK(right->wipe_x == 242);
    // Contre la butée gauche, l'essuyage part à droite
    const auto left = plan_pause(on, 0, 0, 10, 2, core_one);
    REQUIRE(left.has_value());
    CHECK(left->wipe_x == 10);
    CHECK(left->wipe_x <= core_one.x_max);
    CHECK_FALSE(plan_pause(on, NAN, 0, 10, 2, core_one).has_value());
}

TEST_CASE("reprise : réglages bornés", "[od3d][resume]") {
    const Settings wild { .enabled = true, .purge_mm = 200, .retract_tenths = 99, .approach_mm_s = 0 };
    const Settings s = wild.clamped();
    CHECK(s.purge_mm == Settings::purge_max);
    CHECK(s.retract_tenths == Settings::retract_max);
    CHECK(s.approach_mm_s == Settings::approach_min);
    const auto p = plan_pause(wild, 202, -13, 100, 92, core_one);
    REQUIRE(p.has_value());
    CHECK_THAT(p->retract_mm, WithinAbs(2.0, 1e-6));
    CHECK_THAT(p->extrude_mm, WithinAbs(8 + 30, 1e-4));
    // Jamais plus vite que la descente de Prusa
    CHECK(approach_feedrate(Settings { .enabled = true, .approach_mm_s = 5 }, 5) == 5);
    CHECK(approach_feedrate(Settings { .enabled = true, .approach_mm_s = 5 }, 3) == 3);
    CHECK(approach_feedrate(on, 5) == 2);
}

TEST_CASE("reprise après coupure", "[od3d][resume]") {
    const auto p = plan_power_panic(on, 202, -13, 8, 500, core_one);
    REQUIRE(p.has_value());
    CHECK_THAT(p->extrude_mm, WithinAbs(8 + 8, 1e-4));
    CHECK_THAT(p->retract_mm, WithinAbs(8, 1e-6)); // Prusa désrétracte 8 mm ensuite
    CHECK(p->e_after == 500); // E inchangé pour la suite de Prusa
    CHECK_FALSE(plan_power_panic(on, 202, -13, 50, 500, core_one).has_value());

    // Monter d'au moins 5 mm au-dessus du point de reprise, jamais descendre, jamais au-delà de Z max
    CHECK(power_panic_lift_z(10.2f, 10.2f, 270) == 10.2f + power_panic_clearance_mm);
    CHECK(power_panic_lift_z(40, 10, 270) == 40);
    CHECK(power_panic_lift_z(268, 266, 270) == 270);
    CHECK(power_panic_lift_z(275, 10, 270) == 275);
}
