// OneDrop3D : bobine de l'accueil calculée pixel par pixel (SPEC_G.md §1.12).
#include <gui/od3d/spool_draw.hpp>

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <vector>

using namespace od3d::spool;

namespace {

constexpr Color orange = Color::from_raw(0xF8651B);
constexpr Color dark_orange = Color::from_raw(0xB34913); // la maquette : chaque canal × 0,72

/// Couleurs du rayon horizontal d'une bobine Ø40, du centre (rayon 0) au bord (rayon 19)
std::vector<Color> radius_colors(const Look &look) {
    const Disc disc(look, 40);
    std::vector<Color> out;
    for (int r = 0; r < 20; ++r) {
        // Ligne 20, colonne 20 + r : le centre du pixel est à r + 0,5 px du centre
        out.push_back(disc.pixel(20 + r, 20).value());
    }
    return out;
}

} // namespace

TEST_CASE("bobine : bandes d'une bobine pleine, comme la maquette") {
    const auto c = radius_colors({ orange, 100 });
    const Color hole = Color::from_raw(0x111215), hub_ring = Color::from_raw(0x7A7E85), hub = Color::from_raw(0x26282C);
    const Color hub_edge = Color::from_raw(0x44474D), flange_edge = Color::from_raw(0x55595F);
    const Color flange = Color::from_raw(0x2E3035), rim = Color::from_raw(0x5F636A);
    const std::vector<Color> expected {
        hole, hole, hole, hub_ring, hub, hub, hub_edge, // moyeu : 0 à 6,6 px
        dark_orange, orange, orange, dark_orange, orange, orange, dark_orange, orange, orange, // enroulement : 2 px, 1 px sombre
        flange_edge, flange, flange, rim, // flasque et jante
    };
    CHECK(c == expected);
}

TEST_CASE("bobine : hors du disque, rien n'est dessiné") {
    const Disc disc({ orange, 100 }, 40);
    CHECK_FALSE(disc.pixel(0, 0).has_value());
    CHECK_FALSE(disc.pixel(39, 39).has_value());
    CHECK_FALSE(disc.pixel(-1, 20).has_value());
    CHECK_FALSE(disc.pixel(40, 20).has_value());
    CHECK(disc.pixel(0, 20).has_value()); // bord gauche, au milieu
    CHECK(disc.pixel(39, 19).has_value());
}

TEST_CASE("bobine : le niveau de remplissage déplace le bord de l'enroulement") {
    const auto count = [](uint8_t fill) {
        const auto c = radius_colors({ orange, fill });
        return std::count_if(c.begin(), c.end(), [](Color x) { return x == orange || x == dark_orange; });
    };
    CHECK(count(100) == 9);
    CHECK(count(35) < count(100));
    CHECK(count(8) > 0); // presque vide : il reste un anneau
    CHECK(count(0) == count(8)); // la maquette garde au moins 8 %
}

TEST_CASE("bobine : un emplacement vide n'a pas d'enroulement") {
    const auto c = radius_colors({ std::nullopt, 100 });
    const Color empty = Color::from_raw(0x17181B);
    for (int r = 7; r < 16; ++r) {
        CHECK(c[r] == empty);
    }
}

TEST_CASE("bobine : les segments couvrent exactement le disque") {
    for (uint8_t d : { 14, 40, 64, 104 }) {
        const Disc disc({ orange, 60 }, d);
        for (int y = 0; y < d; ++y) {
            int covered = 0, last_end = -1;
            disc.for_each_run(y, [&](int x, int len, Color color) {
                CHECK(x > last_end - 1);
                CHECK(len > 0);
                for (int i = x; i < x + len; ++i) {
                    CHECK(disc.pixel(i, y) == color);
                }
                covered += len;
                last_end = x + len;
            });
            int inside = 0;
            for (int x = 0; x < d; ++x) {
                inside += disc.pixel(x, y).has_value();
            }
            CHECK(covered == inside);
        }
    }
}

TEST_CASE("bobine : couleurs par défaut") {
    CHECK(shade(orange) == dark_orange);
    CHECK(default_color("PLA") == orange);
    CHECK(default_color("PETG") != default_color("PLA"));
    CHECK(default_color("INCONNU") == default_color("autre"));
}
