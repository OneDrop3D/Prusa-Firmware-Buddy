/// OneDrop3D (interface G) : bobines selon le nombre d'outils et défilement de la grille des fichiers (layout_od3d.hpp)
#include <gui/od3d/layout_od3d.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace od3d;

TEST_CASE("od3d layout: tool count gives the spool grid") {
    const auto check = [](uint8_t tools, uint8_t cols, uint8_t rows) {
        const layout::Grid g = layout::spool_grid(tools);
        INFO("tools " << int(tools));
        CHECK(g.cols == cols);
        CHECK(g.rows == rows);
    };
    check(0, 1, 1); // sans outil activé : la case de la bobine reste
    check(1, 1, 1); // sans MMU
    check(3, 3, 1); // INDX 3 têtes
    check(5, 5, 1); // MMU3
    check(6, 3, 2);
    check(8, 4, 2); // INDX 8 têtes : 2 rangées de 4
}

TEST_CASE("od3d layout: spool cells fill the area without overlap") {
    // La bande de l'accueil (x 8 à 397) et l'étagère de l'Atelier (x 8 à 471)
    for (const Rect16 area : { Rect16(8, 46, 390, 56), Rect16(8, 46, 464, 133) }) {
        for (uint8_t tools = 1; tools <= 8; ++tools) {
            const layout::Grid g = layout::spool_grid(tools);
            for (uint8_t i = 0; i < tools; ++i) {
                const Rect16 c = layout::spool_cell(area, tools, 5, i);
                INFO("tools " << int(tools) << " cell " << int(i));
                CHECK(c.Left() >= area.Left());
                CHECK(c.Right() <= area.Right());
                CHECK(c.Top() >= area.Top());
                CHECK(c.Bottom() <= area.Bottom());
                if (i % g.cols != 0) {
                    CHECK(c.Left() == layout::spool_cell(area, tools, 5, i - 1).Right() + 1 + 5); // écart régulier
                }
                if (i >= g.cols) {
                    CHECK(c.Top() == layout::spool_cell(area, tools, 5, i - g.cols).Bottom() + 1 + 5);
                }
            }
            // La dernière colonne et la dernière rangée touchent le bord : toute la place est prise
            const Rect16 last = layout::spool_cell(area, tools, 5, g.cols * g.rows - 1);
            CHECK(last.Right() == area.Right());
            CHECK(last.Bottom() == area.Bottom());
        }
    }
    // Les 5 bobines du MMU gardent la géométrie de la maquette (SPEC_G.md §3.1, §3.2)
    CHECK(layout::spool_cell(Rect16(8, 46, 390, 56), 5, 5, 4) == Rect16(324, 46, 74, 56));
    CHECK(layout::spool_cell(Rect16(8, 46, 464, 133), 5, 6, 4) == Rect16(384, 46, 88, 133));
}

TEST_CASE("od3d grid: the knob moves one file, the grid one row") {
    constexpr int cols = 2, rows = 2;
    using grid::View;
    // 7 fichiers : rangées 0-1, 2-3, 4-5, 6
    CHECK(grid::step(7, cols, rows, View { 0, 0 }, 1) == View { 0, 1 });
    CHECK(grid::step(7, cols, rows, View { 0, 3 }, 1) == View { 2, 4 }); // sort en bas : une rangée de plus
    CHECK(grid::step(7, cols, rows, View { 2, 5 }, 1) == View { 4, 6 });
    CHECK_FALSE(grid::step(7, cols, rows, View { 4, 6 }, 1).has_value()); // fin : vers les onglets
    CHECK(grid::step(7, cols, rows, View { 4, 4 }, -1) == View { 2, 3 }); // sort en haut : une rangée de moins
    CHECK(grid::step(7, cols, rows, View { 2, 2 }, -1) == View { 0, 1 });
    CHECK_FALSE(grid::step(7, cols, rows, View { 0, 0 }, -1).has_value()); // début : vers l'en-tête
    CHECK_FALSE(grid::step(0, cols, rows, View { 0, 0 }, 1).has_value());
}

TEST_CASE("od3d grid: the last page keeps its last row at the bottom") {
    CHECK(grid::max_first(0, 2, 2) == 0);
    CHECK(grid::max_first(4, 2, 2) == 0);
    CHECK(grid::max_first(5, 2, 2) == 2);
    CHECK(grid::max_first(7, 2, 2) == 4);
    CHECK(grid::max_first(8, 2, 2) == 4);
    CHECK(grid::clamp_first(7, 2, 2, 5) == 4); // début de rangée
    CHECK(grid::clamp_first(7, 2, 2, 40) == 4);
    CHECK(grid::clamp_first(7, 2, 2, -3) == 0);
}

TEST_CASE("od3d grid: the content follows the finger row by row") {
    constexpr int pitch = 116;
    CHECK(grid::drag(20, 2, 2, 4, 0, pitch) == 4);
    CHECK(grid::drag(20, 2, 2, 4, -57, pitch) == 4); // moins d'une demi-rangée : rien
    CHECK(grid::drag(20, 2, 2, 4, -58, pitch) == 6); // doigt vers le haut : la suite
    CHECK(grid::drag(20, 2, 2, 4, -200, pitch) == 8);
    CHECK(grid::drag(20, 2, 2, 4, 58, pitch) == 2); // vers le bas : le début
    CHECK(grid::drag(20, 2, 2, 4, 400, pitch) == 0); // borné au début
    CHECK(grid::drag(20, 2, 2, 14, -400, pitch) == 16); // et à la dernière page
}

TEST_CASE("od3d grid: a fling adds a few rows") {
    CHECK(grid::fling_rows(0) == 0);
    CHECK(grid::fling_rows(-499) == 0); // glissement lent : pas d'inertie
    CHECK(grid::fling_rows(-500) == 1);
    CHECK(grid::fling_rows(-1200) == 2);
    CHECK(grid::fling_rows(-9000) == 3); // inertie légère
    CHECK(grid::fling_rows(1200) == -2); // doigt vers le bas : retour au début
}
