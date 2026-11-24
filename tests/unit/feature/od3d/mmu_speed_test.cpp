// OneDrop3D (N4) : vitesse de la poulie du MMU choisie d'après les cycles à vitesse croissante.
#include <feature/od3d/mmu_speed_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>

#include <vector>

using namespace od3d::mmu_speed;

namespace {

/// Cycles sans erreur : 30 s à la vitesse de Prusa, 0,5 s de moins par palier, 1,2 s d'avance lente
std::vector<Cycle> clean(size_t n = levels.size()) {
    std::vector<Cycle> c;
    for (size_t i = 0; i < n; ++i) {
        c.push_back(Cycle { .error = false, .total_ms = uint32_t(30'000 - 500 * i), .slow_ms = 1200 });
    }
    return c;
}

} // namespace

TEST_CASE("mmu_speed: levels never above 120 mm/s (U-03)") {
    CHECK(levels.front() == prusa_speed);
    for (const uint8_t v : levels) {
        CHECK(v <= 120);
    }
}

TEST_CASE("mmu_speed: every level passed keeps the one before the last") {
    const Choice c = choose(clean());
    CHECK(c.passed == levels.size());
    CHECK(c.speed == 115); // marge d'un palier sous 120
}

TEST_CASE("mmu_speed: an error stops at the previous level") {
    auto cycles = clean();
    cycles[4].error = true; // 115 mm/s
    const Choice c = choose(cycles);
    CHECK(c.passed == 4);
    CHECK(c.speed == 105); // 110 passé, le palier d'avant gardé
    // Les paliers après l'échec ne comptent pas, même réussis
    cycles.resize(5);
    CHECK(choose(cycles).speed == 105);
}

TEST_CASE("mmu_speed: never below Prusa's speed") {
    auto cycles = clean(2);
    cycles[1].error = true; // 100 : seul le palier de Prusa est passé
    CHECK(choose(cycles).speed == prusa_speed);
    cycles = clean(3);
    cycles[2].error = true; // 105 : 100 passé, le palier d'avant est celui de Prusa
    CHECK(choose(cycles).speed == prusa_speed);
}

TEST_CASE("mmu_speed: an error at Prusa's speed changes nothing") {
    auto cycles = clean(1);
    cycles[0].error = true;
    const Choice c = choose(cycles);
    CHECK(c.passed == 0);
    CHECK(c.speed == 0);
    CHECK(choose({}).speed == 0);
}

TEST_CASE("mmu_speed: slipping pulley (longer slow feed) fails the level") {
    auto cycles = clean();
    cycles[3].slow_ms = cycles[0].slow_ms + slip_ms; // à la limite : passé
    CHECK(choose(cycles).passed == levels.size());
    cycles[3].slow_ms = cycles[0].slow_ms + slip_ms + 1; // 110 : 20 mm de trop
    const Choice c = choose(cycles);
    CHECK(c.passed == 3);
    CHECK(c.speed == 100); // 105 passé, le palier d'avant gardé
}

TEST_CASE("mmu_speed: a slower cycle (hidden retry) fails the level") {
    auto cycles = clean();
    cycles[2].total_ms = cycles[1].total_ms + slower_ms; // bruit : passé
    CHECK(choose(cycles).passed == levels.size());
    cycles[2].total_ms = cycles[1].total_ms + 12'000; // déchargé et rechargé
    CHECK(choose(cycles).passed == 2);
}

TEST_CASE("mmu_speed: saved speed sent to the MMU, within 95 to 120 mm/s") {
    CHECK(speed_of(Saved {}) == prusa_speed);
    CHECK(speed_of(Saved { .speed = 110 }) == 110);
    CHECK(speed_of(Saved { .speed = 152 }) == 120); // enregistrée avant le plafond (U-03)
    CHECK(speed_of(Saved { .speed = 50 }) == prusa_speed);
}
