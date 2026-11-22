// OneDrop3D (F8, F26) : lecture des mouvements à venir sur une file du planificateur simulée (tampon circulaire de
// 32 cases, comme BLOCK_BUFFER_SIZE de la CORE One).
#include <feature/od3d/lookahead_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
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

BlockInfo fan_move(float duration_s, uint8_t fan) {
    return BlockInfo { .duration_s = duration_s, .e_mm = 0, .fan = fan, .is_move = true };
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

    std::optional<uint8_t> fan_ahead(float lead_s) {
        FanScan scan(lead_s);
        scan_ring(tail, head, size, scan, [this](uint8_t i) {
            visited.push_back(i);
            return cells[i];
        });
        return scan.fan();
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

TEST_CASE("lookahead F26 : ventilateur appliqué en avance") {
    SECTION("avance 0 : le ventilateur du premier bloc de mouvement, comme Prusa") {
        Ring r(0, { sync_block(), fan_move(0.2f, 50), fan_move(0.01f, 255) });
        CHECK(r.fan_ahead(0) == 50);
        CHECK(r.visited.size() == 2); // rien n'est lu après le premier bloc
    }

    SECTION("une hausse qui commence dans l'avance est prise") {
        // Débuts : 0 ; 0,2 ; 0,4 s
        Ring r(0, { fan_move(0.2f, 0), fan_move(0.2f, 0), fan_move(0.2f, 255) });
        CHECK(r.fan_ahead(0.5f) == 255);
        Ring r2(0, { fan_move(0.2f, 0), fan_move(0.2f, 0), fan_move(0.2f, 255) });
        CHECK(r2.fan_ahead(0.3f) == 0); // commence à 0,4 s : trop tard
        CHECK(r2.visited.size() == 2); // le 2e bloc finit après l'avance : le 3e n'est pas lu
    }

    SECTION("le plus fort l'emporte ; une baisse n'est pas avancée") {
        Ring r(0, { fan_move(0.1f, 255), fan_move(0.1f, 0), fan_move(0.1f, 128) });
        CHECK(r.fan_ahead(1.0f) == 255);
        Ring r2(0, { fan_move(0.1f, 64), fan_move(0.1f, 200), fan_move(0.1f, 128) });
        CHECK(r2.fan_ahead(1.0f) == 200);
    }

    SECTION("blocs de synchronisation ignorés (ventilateur à 0) ; aucun mouvement : rien") {
        Ring r(0, { sync_block(), fan_move(0.1f, 100), sync_block(), fan_move(0.1f, 90) });
        CHECK(r.fan_ahead(1.0f) == 100);
        Ring empty(0, { sync_block(), sync_block() });
        CHECK_FALSE(empty.fan_ahead(1.0f).has_value());
        Ring none(4, {});
        CHECK_FALSE(none.fan_ahead(1.0f).has_value());
    }

    SECTION("durées inconnues : comptent pour 0 s") {
        Ring r(0, { fan_move(0, 10), fan_move(0, 20), fan_move(0.5f, 30), fan_move(0.1f, 40) });
        CHECK(r.fan_ahead(0.4f) == 30); // le 4e commence à 0,5 s
    }

    SECTION("pont dans une file qui fait le tour du tampon") {
        // Remplissage (fan 35 %), puis un pont à 100 % qui commence 0,3 s plus tard
        Ring r(30, { fan_move(0.15f, 90), fan_move(0.15f, 90), fan_move(0.6f, 255), fan_move(0.2f, 90) });
        CHECK(r.fan_ahead(0.2f) == 90);
        Ring r2(30, { fan_move(0.15f, 90), fan_move(0.15f, 90), fan_move(0.6f, 255), fan_move(0.2f, 90) });
        CHECK(r2.fan_ahead(0.4f) == 255);
        CHECK(r2.visited == std::vector<uint8_t> { 30, 31, 0 });
    }

    SECTION("index incohérents : au plus 32 cases, résultat parmi les valeurs de la file") {
        std::mt19937 rng { 9 };
        std::uniform_int_distribution<int> idx(0, 255);
        for (int i = 0; i < 2000; ++i) {
            Ring r(0, {});
            for (auto &c : r.cells) {
                c = fan_move(0.01f, 77);
            }
            r.tail = uint8_t(idx(rng));
            r.head = uint8_t(idx(rng));
            const auto f = r.fan_ahead(1.0f);
            REQUIRE(r.visited.size() <= size);
            REQUIRE((!f.has_value() || *f == 77));
        }
    }
}

namespace {

/// Fronts du tachymètre par créneau de 50 ms d'un ventilateur simulé, commandé de 0 à 100 % à command_s : retard
/// dead_s, puis montée du premier ordre (constante tau_s) vers rpm. Compteur entier, comme dans l'interruption.
std::vector<uint8_t> simulated_bins(size_t bins, float command_s, float dead_s, float tau_s, float rpm) {
    const auto revolutions_edges = [&](float t) { // fronts cumulés depuis 0 (4 par tour)
        const float on = t - command_s - dead_s;
        if (on <= 0) {
            return 0.f;
        }
        const float turns = rpm / 60.f * (on - tau_s * (1.f - std::exp(-on / tau_s)));
        return 4.f * turns;
    };
    std::vector<uint8_t> out(bins);
    for (size_t i = 0; i < bins; ++i) {
        out[i] = uint8_t(std::floor(revolutions_edges((i + 1) * fan_bin_s)) - std::floor(revolutions_edges(i * fan_bin_s)));
    }
    return out;
}

} // namespace

TEST_CASE("lookahead N3 : temps de montée du ventilateur pièce") {
    CHECK_THAT(fan_rpm(23), WithinAbs(6900, 1e-3)); // 23 fronts en 50 ms = 6 900 tr/min

    // Commande à 0,3 s (créneau 6), retard 80 ms, constante 0,25 s, 6 600 tr/min : 90 % à 80 ms + 0,25 × ln 10
    const float expected = 0.08f + 0.25f * std::log(10.f);
    const auto bins = simulated_bins(60, 0.3f, 0.08f, 0.25f, 6600.f);
    const FanRise r = fan_rise(bins, 0.3f, 1000.f);
    REQUIRE(r.found);
    CHECK_THAT(r.rise_s, WithinAbs(expected, 0.04));
    CHECK_THAT(r.rpm, WithinAbs(6600, 6600 * 0.03));
    CHECK(fan_lead_for(r.rise_s) == 7); // 0,66 s → avance 0,7 s

    // Ventilateur plus lent : plus d'avance, bornée à 1 s (max de F26)
    const FanRise slow = fan_rise(simulated_bins(60, 0.3f, 0.1f, 0.6f, 5000.f), 0.3f, 1000.f);
    REQUIRE(slow.found);
    CHECK(slow.rise_s > 1.f);
    CHECK(fan_lead_for(slow.rise_s) == 10);

    // Un front de bruit avant la montée n'est pas confirmé par le créneau suivant
    auto noisy = bins;
    noisy[7] = 30;
    const FanRise n = fan_rise(noisy, 0.3f, 1000.f);
    REQUIRE(n.found);
    CHECK_THAT(n.rise_s, WithinAbs(expected, 0.04));
}

TEST_CASE("lookahead N3 : pas de mesure exploitable") {
    const std::vector<uint8_t> silent(60, 0); // tachymètre muet : ventilateur débranché
    CHECK_FALSE(fan_rise(silent, 0.3f, 1000.f).found);
    const std::vector<uint8_t> weak(60, 2); // 600 tr/min à fond : sous le minimum
    CHECK_FALSE(fan_rise(weak, 0.3f, 1000.f).found);
    const auto bins = simulated_bins(60, 0.3f, 0.08f, 0.25f, 6600.f);
    CHECK_FALSE(fan_rise(std::span(bins).first(12), 0.3f, 1000.f).found); // trop peu de créneaux après la commande
    CHECK_FALSE(fan_rise(bins, 0.3f, 1000.f, 0).found);

    CHECK(fan_lead_for(0.01f) == 1); // une mesure donne toujours une avance
    CHECK(fan_lead_for(0.34f) == 3);
    CHECK(fan_lead_for(NAN) == 0);
}
