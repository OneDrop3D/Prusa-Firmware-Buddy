// OneDrop3D (F22) : carte des vitesses critiques — énergie vibratoire, passages, sommets.
#include <feature/od3d/speed_map_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

using namespace od3d::speed_map;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

constexpr float period = 1.f / 1344.f; // LIS2DH12 de Prusa
constexpr float pi = std::numbers::pi_v<float>;

std::string speeds(std::vector<uint8_t> idx, float start = 20, float step = 10, size_t size = 64) {
    std::vector<char> buf(size);
    format_speeds(idx, start, step, buf.data(), buf.size());
    return buf.data();
}

} // namespace

TEST_CASE("vitesses : énergie d'une vibration sur la pesanteur") {
    VibrationMeter meter(period, 0.01f);
    // Pesanteur + accélération constante + sinus de 100 Hz d'amplitude 2 m/s² sur X
    for (int i = 0; i < 5000; ++i) {
        const float t = float(i) * period;
        meter.add(0.5f + 2.f * std::sin(2 * pi * 100 * t), 1.f, 9.81f, i > 1000);
    }
    CHECK(meter.count() > 3500);
    // Moyenne des carrés d'un sinus d'amplitude 2 : 2 ; le passe-haut à 16 Hz en garde ~97 %
    CHECK_THAT(meter.mean_square(), WithinRel(2.0, 0.08));
}

TEST_CASE("vitesses : pas de vibration, pas d'énergie") {
    VibrationMeter meter(period, 0.01f);
    for (int i = 0; i < 3000; ++i) {
        meter.add(0.f, 0.f, 9.81f, true);
    }
    CHECK(meter.mean_square() < 1e-6f);
    VibrationMeter empty(period, 0.01f);
    CHECK(empty.mean_square() == 0.f);
    CHECK(empty.count() == 0);
}

TEST_CASE("vitesses : le filtre s'établit avant de compter") {
    VibrationMeter meter(period, 0.01f);
    // 5 τ = 67 échantillons ignorés, même s'ils sont « comptés »
    for (int i = 0; i < 60; ++i) {
        meter.add(0, 0, 0, true);
    }
    CHECK(meter.count() == 0);
    for (int i = 0; i < 20; ++i) {
        meter.add(0, 0, 0, true);
    }
    CHECK(meter.count() > 0);
}

TEST_CASE("vitesses : longueur d'un passage") {
    // 300 mm/s à 4 000 mm/s² : 11,25 mm d'accélération, marge 5 + 9 mm, 0,4 s de mesure
    const Pass p = pass_for(300, 4000, 0.4f, 5, 0.03f, 200);
    CHECK_THAT(p.ramp, WithinAbs(11.25, 1e-3));
    CHECK_THAT(p.margin, WithinAbs(14, 1e-3));
    CHECK_THAT(p.length, WithinAbs(2 * (11.25 + 14) + 120, 1e-3));
    CHECK(in_cruise(p, p.length / 2));
    CHECK_FALSE(in_cruise(p, 10));
    CHECK_FALSE(in_cruise(p, p.length - 10));

    // Bornée : la mesure raccourcit
    const Pass q = pass_for(300, 4000, 2.f, 5, 0.03f, 200);
    CHECK(q.length == 200.f);
    CHECK(in_cruise(q, 100));

    // Trop court pour mesurer : jamais « en mesure »
    const Pass r = pass_for(300, 4000, 0.4f, 5, 0.03f, 40);
    CHECK_FALSE(in_cruise(r, 20));

    // Vitesse ou accélération nulle
    CHECK(pass_for(0, 4000, 0.4f, 5, 0.03f, 200).length == 0.f);
    CHECK(pass_for(100, 0, 0.4f, 5, 0.03f, 200).length == 0.f);
}

TEST_CASE("vitesses : passages d'un axe sans recentrage") {
    // Comme measure_axis : 4 000 mm/s², 0,4 s de mesure, course de 100 mm de part et d'autre du centre, 300 mm/s au plus
    constexpr float centre = 125, bound = 100;
    const auto pass_at = [&](float v) { return pass_for(v, 4000, 0.4f, 5, 0.03f, 2 * bound); };
    const float from = sweep_from(centre, pass_at(300).length);
    CHECK_THAT(centre - from, WithinAbs(pass_at(300).length / 2, 1e-3)); // le plus long passage est centré

    float previous = 0;
    for (float v = 20; v <= 300; v += 10) {
        const Pass p = pass_at(v);
        // Aller de from à from + length, retour à from : la vitesse suivante repart de from, aucun autre déplacement
        CHECK(p.length >= previous);
        CHECK(from >= centre - bound);
        CHECK(from + p.length <= centre + bound);
        // Aller et retour de même longueur : même fenêtre de mesure, dans la vitesse constante
        const float window_from = p.ramp + p.margin, window_to = p.length - p.ramp - p.margin;
        CHECK(window_to > window_from);
        CHECK(in_cruise(p, (window_from + window_to) / 2));
        CHECK_FALSE(in_cruise(p, p.ramp / 2));
        CHECK_FALSE(in_cruise(p, p.length - p.ramp / 2));
        previous = p.length;
    }
}

TEST_CASE("vitesses : médiane") {
    CHECK(median(std::vector<float> {}) == 0.f);
    CHECK(median(std::vector<float> { 3, 1, 2 }) == 2.f);
    CHECK(median(std::vector<float> { 4, 1, 3, 2 }) == 2.5f);
    CHECK(median(std::vector<float> { NAN, 5, 1 }) == 3.f);
}

TEST_CASE("vitesses : sommets critiques") {
    //                         20  30  40  50  60  70  80  90  100 110
    const std::vector<float> v { 1, 1.1f, 4, 1.2f, 1, 1, 2.5f, 1, 1, 1.05f };
    uint8_t out[5];
    const size_t n = find_critical(v, 1.8f, 0.f, out);
    REQUIRE(n == 2);
    CHECK(out[0] == 2); // 40 mm/s, le plus fort
    CHECK(out[1] == 6); // 80 mm/s
    CHECK(speeds({ out[0], out[1] }) == "40, 80");

    // Seuil absolu : rien sous min_value
    CHECK(find_critical(v, 1.8f, 5.f, out) == 0);

    // Au plus out.size(), les plus forts
    uint8_t one[1];
    CHECK(find_critical(v, 1.8f, 0.f, one) == 1);
    CHECK(one[0] == 2);

    // Un bord ne compte pas (V-07) ; un plateau une seule fois
    const std::vector<float> edge { 5, 1, 1, 1, 3, 3, 1 };
    const size_t m = find_critical(edge, 1.5f, 0.f, out);
    REQUIRE(m == 1);
    CHECK(out[0] == 4);

    // Plat : rien ; vide ou trop court : rien
    CHECK(find_critical(std::vector<float>(10, 2.f), 1.5f, 0.f, out) == 0);
    CHECK(find_critical(std::vector<float> {}, 1.5f, 0.f, out) == 0);
    CHECK(find_critical(std::vector<float> { 1, 9 }, 1.5f, 0.f, out) == 0);
}

TEST_CASE("vitesses (V-07) : base locale, une énergie qui croît n'a pas de vitesse critique") {
    uint8_t out[5];
    // L'énergie croît avec la vitesse (de 20 à 80 mm/s par pas de 5) : avec la médiane de tout l'axe, la plus haute
    // vitesse ressortait « critique » ; aucune ne l'est
    std::vector<float> rising;
    for (int i = 0; i < 13; ++i) {
        rising.push_back(1.f + 0.25f * float(i));
    }
    CHECK(find_critical(rising, 1.5f, 0.f, out) == 0);
    std::vector<float> steep;
    for (int i = 0; i < 13; ++i) {
        steep.push_back(std::exp(0.3f * float(i)));
    }
    CHECK(find_critical(steep, 1.5f, 0.f, out) == 0);

    // Une résonance sur cette pente : trouvée par rapport à ses voisines (base 2,5), et rien d'autre
    std::vector<float> bump = rising;
    bump[6] += 3.f; // 50 mm/s : 5,5 contre 2 ; 2,25 ; 2,75 ; 3 autour
    REQUIRE(find_critical(bump, 1.5f, 0.f, out) == 1);
    CHECK(out[0] == 6);
    CHECK(speeds({ out[0] }, 20, 5) == "50");

    // Trop peu au-dessus de ses voisines : pas critique
    std::vector<float> small = rising;
    small[6] += 0.5f;
    CHECK(find_critical(small, 1.5f, 0.f, out) == 0);

    // Voisines non mesurées (NAN) : la base prend celles qui le sont ; sans aucune, pas de verdict
    const std::vector<float> holes { 1, NAN, 4, 1, 1 };
    REQUIRE(find_critical(holes, 1.5f, 0.f, out) == 1);
    CHECK(out[0] == 2);
    CHECK(find_critical(std::vector<float> { NAN, 4, NAN }, 1.5f, 0.f, out) == 0);
}

TEST_CASE("vitesses : liste lisible") {
    CHECK(speeds({}) == "");
    CHECK(speeds({ 4 }) == "60");
    CHECK(speeds({ 10, 0, 4 }) == "20, 60, 120");
    CHECK(speeds({ 10, 0, 4 }, 20, 10, 8) == "20, 60,"); // coupé, terminé
}

TEST_CASE("vitesses : hauteur des barres") {
    CHECK(bar_height(5, 10, 40) == 20);
    CHECK(bar_height(20, 10, 40) == 40);
    CHECK(bar_height(0, 10, 40) == 0);
    CHECK(bar_height(5, 0, 40) == 0);
    CHECK(bar_height(NAN, 10, 40) == 0);
    CHECK(bar_height(5, 10, 0) == 0);
}
