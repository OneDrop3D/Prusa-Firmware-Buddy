// OneDrop3D (F8) : lecture des mouvements à venir sur une file du planificateur simulée (tampon circulaire de
// 32 cases, comme BLOCK_BUFFER_SIZE de la CORE One).
#include <feature/od3d/lookahead_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

using namespace od3d::lookahead;
using Catch::Matchers::WithinAbs;

namespace {

constexpr uint8_t size = 32;
constexpr float max_e = 20; ///< max_melt_e_speed de F7

BlockInfo move(float duration_s, float e_mm) {
    return BlockInfo { .duration_s = duration_s, .e_mm = e_mm, .is_move = true };
}

BlockInfo sync_block() {
    return BlockInfo { .duration_s = 0, .e_mm = 0, .is_move = false };
}

/// File simulée : les blocs sont posés à partir de tail, en faisant le tour du tampon
struct Ring {
    std::array<BlockInfo, size> cells {};
    uint8_t tail = 0;
    uint8_t head = 0;
    std::vector<uint8_t> visited;

    Ring(uint8_t start, std::initializer_list<BlockInfo> blocks)
        : tail(start)
        , head(start) {
        for (const BlockInfo &b : blocks) {
            cells[head] = b;
            head = uint8_t((head + 1) % size);
        }
    }

    float e_speed_ahead(float window_s = flow_window_s) {
        FlowScan scan(window_s, max_e);
        scan_ring(tail, head, size, scan, [this](uint8_t i) {
            visited.push_back(i);
            return cells[i];
        });
        return scan.mean_e_speed();
    }
};

} // namespace

TEST_CASE("lookahead : durée d'un bloc") {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float inf = std::numeric_limits<float>::infinity();
    CHECK_THAT(block_duration(10, 100), WithinAbs(0.1f, 1e-6f));
    CHECK(block_duration(0, 100) == 0);
    CHECK(block_duration(10, 0) == 0);
    CHECK(block_duration(-1, 100) == 0);
    CHECK(block_duration(10, -5) == 0);
    CHECK(block_duration(nan, 100) == 0);
    CHECK(block_duration(10, nan) == 0);
    CHECK(block_duration(inf, 100) == 0);
    CHECK(block_duration(10, 1e-40f) == 0); // quotient infini
}

TEST_CASE("lookahead F8 : débit moyen des ~500 ms à venir") {
    SECTION("file vide : 0") {
        Ring r(5, {});
        CHECK(r.e_speed_ahead() == 0);
        CHECK(r.visited.empty());
    }

    SECTION("un bloc") {
        Ring r(0, { move(0.2f, 1.0f) });
        CHECK_THAT(r.e_speed_ahead(), WithinAbs(5.0f, 1e-4f));
    }

    SECTION("moyenne pondérée par la durée, coupée à la fenêtre") {
        // 0,4 s à 2 mm/s puis 0,4 s à 10 mm/s : la fenêtre de 0,5 s n'en prend que 0,1 s
        Ring r(0, { move(0.4f, 0.8f), move(0.4f, 4.0f), move(0.4f, 8.0f) });
        CHECK_THAT(r.e_speed_ahead(), WithinAbs((2 * 0.4f + 10 * 0.1f) / 0.5f, 1e-4f));
        CHECK(r.visited.size() == 2); // le 3e bloc n'est pas lu
    }

    SECTION("reculs, mouvements du seul extrudeur, vitesses aberrantes : débit nul, temps compté") {
        Ring r(0, {
                      move(0.1f, -2.0f), // recul
                      move(0.1f, 0.0f), // déplacement, ou retour de recul mis à 0 par la lecture du planificateur
                      move(0.1f, 50.0f), // 500 mm/s : chargement ou saut
                      move(0.1f, std::numeric_limits<float>::quiet_NaN()),
                      move(0.1f, 0.5f), // 5 mm/s
                  });
        CHECK_THAT(r.e_speed_ahead(), WithinAbs(5.0f * 0.1f / 0.5f, 1e-4f));
    }

    SECTION("blocs de synchronisation et durées inconnues : ignorés") {
        Ring r(0, { sync_block(), move(0, 3.0f), move(0.25f, 1.0f), sync_block(), move(-1, 1.0f), move(0.25f, 2.0f) });
        CHECK_THAT(r.e_speed_ahead(), WithinAbs((4 * 0.25f + 8 * 0.25f) / 0.5f, 1e-4f));
    }

    SECTION("file plus courte que la fenêtre : moyenne sur ce qu'elle contient") {
        Ring r(0, { move(0.1f, 0.4f), move(0.1f, 0.8f) });
        CHECK_THAT(r.e_speed_ahead(), WithinAbs(6.0f, 1e-4f));
    }

    SECTION("fenêtre nulle : rien") {
        Ring r(0, { move(0.1f, 0.4f) });
        CHECK(r.e_speed_ahead(0) == 0);
    }
}

TEST_CASE("lookahead : parcours de la file circulaire") {
    SECTION("ordre d'exécution, en faisant le tour du tampon") {
        Ring r(30, { move(0.01f, 0), move(0.01f, 0), move(0.01f, 0), move(0.01f, 0), move(0.01f, 0) });
        CHECK(r.head == 3);
        r.e_speed_ahead();
        CHECK(r.visited == std::vector<uint8_t> { 30, 31, 0, 1, 2 });
    }

    SECTION("file pleine (31 blocs) : toutes les cases, une fois") {
        Ring r(7, {});
        for (int i = 0; i < size - 1; ++i) {
            r.cells[r.head] = move(0.001f, 0);
            r.head = uint8_t((r.head + 1) % size);
        }
        r.e_speed_ahead();
        CHECK(r.visited.size() == size - 1);
        CHECK(r.visited.front() == 7);
        CHECK(r.visited.back() == 5);
    }

    SECTION("index incohérents : au plus 32 cases lues, toujours dans le tampon") {
        std::mt19937 rng { 3 };
        std::uniform_int_distribution<int> idx(0, 255);
        for (int i = 0; i < 2000; ++i) {
            Ring r(0, {});
            for (auto &c : r.cells) {
                c = move(0.001f, 0.001f);
            }
            r.tail = uint8_t(idx(rng));
            r.head = uint8_t(idx(rng));
            r.e_speed_ahead();
            REQUIRE(r.visited.size() <= size);
            for (uint8_t v : r.visited) {
                REQUIRE(v < size);
            }
        }
    }

    SECTION("débit d'une file simulée réaliste") {
        // Périmètre lent (25 mm à 50 mm/s, 0,9 mm de filament), puis remplissage rapide (40 mm à 200 mm/s,
        // 1,6 mm de filament : 8 mm/s, 19 mm³/s)
        Ring r(28, { move(25.0f / 50, 0.9f), move(40.0f / 200, 1.6f), move(40.0f / 200, 1.6f) });
        // 0,5 s : tout le périmètre (0,5 s), rien du remplissage
        CHECK_THAT(r.e_speed_ahead(), WithinAbs(1.8f, 1e-4f));
        // Un bloc plus tard, le remplissage remplit la fenêtre (0,4 s) et la file s'arrête
        Ring r2(31, { move(40.0f / 200, 1.6f), move(40.0f / 200, 1.6f) });
        CHECK_THAT(r2.e_speed_ahead(), WithinAbs(8.0f, 1e-4f));
    }
}
