// OneDrop3D (F11) : assistance du MMU, vitesse envoyée et échanges avec le registre 0x25.
#include <feature/od3d/lookahead_math_od3d.hpp>
#include <feature/od3d/mmu_assist_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <initializer_list>

using namespace od3d::mmu_assist;
using od3d::lookahead::BlockInfo;
using od3d::lookahead::FlowScan;

namespace {

/// Vitesse moyenne du filament des blocs à venir, comme planner_e_speed_ahead
float mean_ahead(std::initializer_list<BlockInfo> blocks) {
    FlowScan scan(od3d::lookahead::flow_window_s, max_e_speed);
    for (const BlockInfo &b : blocks) {
        if (!scan.add(b)) {
            break;
        }
    }
    return scan.mean_e_speed();
}

BlockInfo move(float duration_s, float e_mm) {
    return BlockInfo { .duration_s = duration_s, .e_mm = e_mm, .fan = 0, .is_move = true };
}

} // namespace

TEST_CASE("mmu_assist: the pulley runs a bit faster than the filament") {
    CHECK(ratio >= 1.05f);
    CHECK(ratio <= 1.10f);
    CHECK(pulley_value(true, 4.0f) == 428); // 4 mm/s × 1,07 = 4,28 mm/s
    CHECK(pulley_value(true, 1.0f) == 107);
    // borné à la vitesse maximale du MMU
    CHECK(pulley_value(true, 40.0f) == max_value);
}

TEST_CASE("mmu_assist: cut outside a running print, below the minimal flow, on nonsense") {
    CHECK(pulley_value(false, 4.0f) == 0); // pause, fin, assistance non choisie
    CHECK(pulley_value(true, 0.0f) == 0); // déplacements, attente de chauffe
    CHECK(pulley_value(true, min_e_speed / 2) == 0);
    CHECK(pulley_value(true, -2.0f) == 0);
    CHECK(pulley_value(true, NAN) == 0);
    CHECK(pulley_value(true, min_e_speed) > 0);
}

TEST_CASE("mmu_assist: short gaps in the flow keep the idler pressed, long ones and pauses cut") {
    Speed speed;
    CHECK(speed.next(0, true, 0.0f) == 0); // début d'impression, chauffe : rien
    CHECK(speed.next(500, true, 4.0f) == 428);
    // recul et déplacement : la poulie ralentit, le galet reste serré
    CHECK(speed.next(1000, true, 0.0f) == creep_value);
    CHECK(speed.next(500 + hold_ms - 1, true, 0.0f) == creep_value);
    CHECK(speed.next(1500 + hold_ms, true, 4.0f) == 428);
    // débit nul plus long que hold_ms : coupée, et le reste
    CHECK(speed.next(1500 + 2 * hold_ms, true, 0.0f) == 0);
    CHECK(speed.next(2000 + 2 * hold_ms, true, 0.0f) == 0);
    // pause ou fin : coupée tout de suite, sans maintien ensuite
    CHECK(speed.next(10000, true, 2.0f) == 214);
    CHECK(speed.next(10500, false, 2.0f) == 0);
    CHECK(speed.next(11000, true, 0.0f) == 0);
}

TEST_CASE("mmu_assist: mean flow of the next blocks, retractions and E-only moves count 0") {
    // 0,25 s à 4 mm/s puis 0,25 s de déplacement : 2 mm/s en moyenne
    CHECK(std::abs(mean_ahead({ move(0.25f, 1.0f), move(0.25f, 0.0f) }) - 2.0f) < 1e-4f);
    CHECK(pulley_value(true, mean_ahead({ move(0.25f, 1.0f), move(0.25f, 0.0f) })) == 214);
    // un recul (e négatif) ne retire rien et ne pousse pas
    CHECK(std::abs(mean_ahead({ move(0.1f, -0.8f), move(0.4f, 1.6f) }) - 3.2f) < 1e-4f);
    // file vide ou seulement des reculs : coupée
    CHECK(pulley_value(true, mean_ahead({})) == 0);
    CHECK(pulley_value(true, mean_ahead({ move(0.2f, -0.8f) })) == 0);
    // chargement (au-delà de max_e_speed) : compte 0
    CHECK(pulley_value(true, mean_ahead({ move(0.5f, 50.0f) })) == 0);
}

TEST_CASE("mmu_assist: link detects the register, then writes at most every period") {
    Link link;
    // MMU qui démarre : d'abord une lecture
    REQUIRE(link.next(1000, 0));
    REQUIRE(link.detecting());
    link.read_back(present_bit);
    CHECK(link.support() == Support::yes);
    CHECK(link.applied() == 0);

    // rien à envoyer à l'arrêt
    CHECK_FALSE(link.next(2000, 0));
    // une vitesse : écrite, puis pas avant la période suivante
    REQUIRE(link.next(2600, 428));
    CHECK_FALSE(link.detecting());
    CHECK(link.value() == 428);
    link.read_back(present_bit | 428);
    CHECK(link.applied() == 428);
    CHECK_FALSE(link.due(2600 + period_ms - 1));
    CHECK_FALSE(link.next(2600 + period_ms - 1, 500));
    // même vitesse : renvoyée à chaque période (le MMU s'arrête seul sans ordre)
    REQUIRE(link.next(2600 + period_ms, 428));
    CHECK(link.value() == 428);
    // refusée par le MMU (relue sans vitesse) : renvoyée ensuite
    link.read_back(present_bit);
    CHECK(link.applied() == 0);
    REQUIRE(link.next(2600 + 2 * period_ms, 428));
    // arrêt : envoyé une fois
    REQUIRE(link.next(2600 + 3 * period_ms, 0));
    CHECK(link.value() == 0);
    link.read_back(present_bit);
    CHECK_FALSE(link.next(2600 + 4 * period_ms, 0));
}

TEST_CASE("mmu_assist: a MMU without the register is never written") {
    Link link;
    REQUIRE(link.next(0, 428));
    REQUIRE(link.detecting());
    link.read_back(0); // firmware Prusa ou ancien OneDrop3D : lecture refusée, 0
    CHECK(link.support() == Support::no);
    for (uint32_t t = 0; t < 10 * period_ms; t += 100) {
        CHECK_FALSE(link.due(t));
        CHECK_FALSE(link.next(t, 428));
    }
    // MMU redémarré (flashé) : reconnu de nouveau
    link = Link {};
    CHECK(link.next(0, 428));
    CHECK(link.detecting());
}
