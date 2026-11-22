// OneDrop3D : bobine de l'accueil calculée pixel par pixel (SPEC_G.md §1.12).
#include <gui/od3d/spool_draw.hpp>

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <vector>

using namespace od3d::spool;

namespace {

constexpr Color orange = Color::from_raw(0xF8651B);
constexpr Color dark_orange = Color::from_raw(0xB34913); // la maquette : chaque canal × 0,72
constexpr Color back = Color::from_raw(0x1B1C1E); // fond de l'écran

/// Couleurs du rayon horizontal d'une bobine Ø40, du centre (rayon 0) au bord (rayon 19)
std::vector<Color> radius_colors(const Look &look) {
    const Disc disc(look, 40);
    std::vector<Color> out;
    for (int r = 0; r < 20; ++r) {
        // Ligne 20, colonne 20 + r : le centre du pixel est à r + 0,5 px du centre
        out.push_back(disc.pixel(20 + r, 20, back));
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

TEST_CASE("bobine : hors du disque, le fond") {
    const Disc disc({ orange, 100 }, 40);
    CHECK(disc.pixel(0, 0, back) == back);
    CHECK(disc.pixel(39, 39, back) == back);
    CHECK(disc.pixel(-1, 20, back) == back);
    CHECK(disc.pixel(40, 20, back) == back);
    CHECK(disc.pixel(0, 20, back) != back); // bord gauche, au milieu
    CHECK(disc.pixel(39, 19, back) != back);
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

TEST_CASE("anticrénelage : part du pixel au-delà d'un bord") {
    CHECK(beyond_quarters(10.0f, 10) == 2); // centre du pixel sur le bord : moitié de chaque côté
    CHECK(beyond_quarters(9.5f, 10) == 0); // fenêtre [9 ; 10] : toute dedans
    CHECK(beyond_quarters(9.75f, 10) == 1);
    CHECK(beyond_quarters(10.25f, 10) == 3);
    CHECK(beyond_quarters(10.5f, 10) == 4);
    CHECK(beyond_quarters(3.0f, 10) == 0); // loin des bords
    CHECK(beyond_quarters(30.0f, 10) == 4);
    for (float r = 0; r < 20; r += 0.01f) { // croissante avec r
        CHECK(beyond_quarters(r + 0.01f, 10) >= beyond_quarters(r, 10));
    }
}

TEST_CASE("anticrénelage : mélange en quarts") {
    const Color black = Color::from_rgb(0, 0, 0), white = Color::from_rgb(255, 255, 255);
    CHECK(mix_quarters(black, white, 0) == black);
    CHECK(mix_quarters(black, white, 4) == white);
    CHECK(mix_quarters(black, white, 2) == Color::from_rgb(128, 128, 128));
    CHECK(mix_quarters(black, white, 1) == Color::from_rgb(64, 64, 64));
    CHECK(mix_quarters(black, white, 3) == Color::from_rgb(191, 191, 191));
    CHECK(mix_quarters(orange, orange, 2) == orange);
}

TEST_CASE("anticrénelage : la couverture d'un disque a l'aire du cercle") {
    // Somme des parts dedans de chaque pixel ≈ π R² (à 1,5 % près)
    for (int d : { 14, 16, 26, 40, 44, 64, 104 }) {
        int quarters = 0;
        for (int y = 0; y < d; ++y) {
            for (int x = 0; x < d; ++x) {
                quarters += 4 - beyond_quarters(radius_in_square(x, y, d), d / 2.0f);
            }
        }
        const float area = 3.14159265f * d * d / 4;
        CHECK(quarters / 4.0f > area * 0.985f);
        CHECK(quarters / 4.0f < area * 1.015f);
    }
}

TEST_CASE("anticrénelage : le bord de la jante est un dégradé vers le fond") {
    const Color rim = Color::from_raw(0x5F636A), black = Color::from_rgb(0, 0, 0);
    const Disc disc({ orange, 100 }, 40);
    // Les pixels que le cercle extérieur coupe : ni jante ni fond, plus sombres que la jante
    int blended = 0;
    for (int y = 0; y < 40; ++y) {
        for (int x = 0; x < 40; ++x) {
            const uint8_t q = beyond_quarters(radius_in_square(x, y, 40), 20);
            if (q == 0 || q == 4) {
                continue;
            }
            const Color c = disc.pixel(x, y, black);
            CHECK(c != black);
            CHECK(c.r < rim.r);
            CHECK(c.g < rim.g);
            CHECK(c.b < rim.b);
            ++blended;
        }
    }
    CHECK(blended >= 40); // tout le tour, sauf aux axes où le bord passe entre deux pixels
}

TEST_CASE("anticrénelage : la bobine reste symétrique") {
    for (uint8_t d : { 14, 26, 40, 64 }) {
        const Disc disc({ orange, 60 }, d);
        for (int y = 0; y < d; ++y) {
            for (int x = 0; x < d; ++x) {
                const Color c = disc.pixel(x, y, back);
                CHECK(c == disc.pixel(d - 1 - x, y, back));
                CHECK(c == disc.pixel(x, d - 1 - y, back));
                CHECK(c == disc.pixel(y, x, back));
            }
        }
    }
}

TEST_CASE("bobine : couleurs par défaut") {
    CHECK(shade(orange) == dark_orange);
    CHECK(default_color("PLA") == orange);
    CHECK(default_color("PETG") != default_color("PLA"));
    CHECK(default_color("INCONNU") == default_color("autre"));
}
