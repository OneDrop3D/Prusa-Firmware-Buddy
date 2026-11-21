// OneDrop3D (F14) : tension des courroies — pic du spectre, conseil de réglage (logique de Prusa), jauges et courbe.
#include <feature/od3d/belt_tension_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <string>
#include <vector>

using namespace od3d::belts;
using Catch::Matchers::WithinAbs;

namespace {

/// Cibles de la CORE One (sans INDX) calculées comme manual_belt_tuning_config.hpp de Prusa :
/// tension optimale 19 N, courroie de 0,267 m ∓ 5 mm, 7,569 g/m, arrondi au demi-hertz inférieur
Targets coreone_targets() {
    const auto freq = [](float tension, float length) {
        return std::sqrt(tension / (4.f * 0.007569f * length * length));
    };
    const auto floor_to_half = [](float v) { return float(int(v * 2)) / 2.f; };
    const float higher = floor_to_half(freq(19, 0.267f - 0.005f));
    const float lower = floor_to_half(freq(19, 0.267f + 0.005f));
    return {
        .higher_optimal = higher,
        .lower_optimal = lower,
        .result_min = float(uint16_t(lower - 3)),
        .result_max = float(uint16_t(higher + 3)),
        .diff_limit = 8,
        .hz_per_rev = 15,
        .hz_per_rev2 = 12,
    };
}

/// Spectre de 70 à 120 Hz par pas de 1 Hz : pente décroissante (résonance de la tête vers 50 Hz) + pic gaussien
std::vector<float> spectrum(float peak_hz, float peak_height, float width_hz = 1.5f) {
    std::vector<float> v;
    for (int f = 70; f <= 120; ++f) {
        const float slope = 4.f * 50.f / float(f);
        const float d = (float(f) - peak_hz) / width_hz;
        v.push_back(slope + peak_height * std::exp(-0.5f * d * d));
    }
    return v;
}

std::string turns(int eighths) {
    char buf[16];
    format_turns(eighths, buf, sizeof(buf));
    return buf;
}

} // namespace

TEST_CASE("courroies : cibles de Prusa pour la CORE One") {
    const Targets t = coreone_targets();
    CHECK(t.higher_optimal == 95.5f);
    CHECK(t.lower_optimal == 92.0f);
    CHECK(t.result_min == 89.f);
    CHECK(t.result_max == 98.f);
}

TEST_CASE("courroies : interpolation parabolique") {
    CHECK(parabolic_offset(1, 2, 1) == 0.f);
    CHECK_THAT(parabolic_offset(1, 3, 2), WithinAbs(1.f / 6.f, 1e-5));
    CHECK_THAT(parabolic_offset(2, 3, 1), WithinAbs(-1.f / 6.f, 1e-5));
    CHECK(parabolic_offset(1, 1, 1) == 0.f); // plat
    CHECK(parabolic_offset(2, 1, 2) == 0.f); // creux
    // Parabole exacte y = -(x - 0.3)² : sommet retrouvé
    const auto y = [](float x) { return -(x - 0.3f) * (x - 0.3f); };
    CHECK_THAT(parabolic_offset(y(-1), y(0), y(1)), WithinAbs(0.3f, 1e-5));
}

TEST_CASE("courroies : pic retrouvé sur une pente") {
    for (const float peak : { 85.f, 92.4f, 95.7f, 101.2f }) {
        const auto v = spectrum(peak, 1.f);
        const Peak p = find_peak(v, 70, 1, 79, 108);
        CHECK(p.found);
        CHECK_THAT(p.frequency, WithinAbs(peak, 0.25));
        CHECK(p.relative_prominence > 0.1f);
    }
}

TEST_CASE("courroies : pas de pic net") {
    // Pente seule : aucun sommet
    const auto flat = spectrum(95, 0);
    CHECK_FALSE(find_peak(flat, 70, 1, 79, 108).found);

    // Petite bosse (5 % du niveau) sur un fond plat : trouvée mais pas assez nette
    std::vector<float> weak;
    for (int f = 70; f <= 120; ++f) {
        const float d = (float(f) - 95.f) / 1.5f;
        weak.push_back(2.f + 0.1f * std::exp(-0.5f * d * d));
    }
    const Peak p = find_peak(weak, 70, 1, 79, 108);
    CHECK_FALSE(p.found);
    CHECK_THAT(p.frequency, WithinAbs(95, 0.5));

    // Pic hors de la fenêtre : ignoré
    const auto outside = spectrum(75, 2.f);
    CHECK_FALSE(find_peak(outside, 70, 1, 79, 108).found);

    // Trop peu de points, pas nul, valeurs nulles
    const std::vector<float> two { 1, 2 };
    CHECK_FALSE(find_peak(two, 70, 1, 0, 200).found);
    CHECK_FALSE(find_peak(spectrum(95, 1), 70, 0, 0, 200).found);
    const std::vector<float> zeros(51, 0.f);
    CHECK_FALSE(find_peak(zeros, 70, 1, 0, 200).found);
}

TEST_CASE("courroies : le pic le plus proéminent l'emporte") {
    auto v = spectrum(90, 0.4f);
    const auto big = spectrum(100, 2.f);
    for (size_t i = 0; i < v.size(); ++i) {
        v[i] += big[i] - spectrum(100, 0.f)[i];
    }
    const Peak p = find_peak(v, 70, 1, 79, 108);
    CHECK(p.found);
    CHECK_THAT(p.frequency, WithinAbs(100, 0.25));
}

TEST_CASE("courroies : zone de la plage de Prusa") {
    const Targets t = coreone_targets();
    CHECK(zone(88.9f, t) == Zone::low);
    CHECK(zone(89.f, t) == Zone::ok);
    CHECK(zone(98.f, t) == Zone::ok);
    CHECK(zone(98.1f, t) == Zone::high);
}

TEST_CASE("courroies : conseil identique à l'assistant de Prusa") {
    const Targets t = coreone_targets();

    SECTION("dans la plage : rien à tourner") {
        const Advice a = advise(95, 92, t);
        CHECK(a.verdict == Verdict::ok);
        CHECK(a.turn_eighths == 0);
    }
    SECTION("écart de plus de 8 Hz : portique de travers") {
        CHECK(advise(100, 91, t).verdict == Verdict::misaligned);
        CHECK(advise(91, 100, t).verdict == Verdict::misaligned);
    }
    SECTION("trop tendues : desserrer") {
        // Calcul à la main avec calc_revs_from_freq de Prusa : dfh = 95,5 - 100, dfl = 92 - 95
        const float dfh = -4.5f, dfl = -3.f;
        const float drh = ((dfl * 12) - (dfh * 15)) / (144.f - 225.f);
        const float drl = ((dfh * 12) - (dfl * 15)) / (144.f - 225.f);
        const int expected = int(std::lround((drh + drl) / 2 * 8));
        const Advice a = advise(100, 95, t);
        CHECK(a.turn_eighths == expected);
        CHECK(expected == -1);
        CHECK(a.verdict == Verdict::adjust);
        // L'ordre A/B ne change rien
        CHECK(advise(95, 100, t).turn_eighths == expected);
    }
    SECTION("trop lâches : serrer") {
        const Advice a = advise(84, 86, t);
        CHECK(a.verdict == Verdict::adjust);
        CHECK(a.turn_eighths > 0);
    }
    SECTION("borné à 8 tours") {
        Targets wide = t;
        wide.diff_limit = 1000;
        CHECK(advise(-500, -500, wide).turn_eighths == 64);
        CHECK(advise(400, 400, wide).turn_eighths == -64);
    }
}

TEST_CASE("courroies : formule des tours de Prusa") {
    CHECK_THAT(revs_from_freq(15, 12, 15, 12), WithinAbs((12.f * 12 - 15.f * 15) / (144.f - 225.f), 1e-6));
    CHECK_THAT(revs_from_freq(0, 0, 15, 12), WithinAbs(0, 1e-6));
}

TEST_CASE("courroies : tours lisibles") {
    CHECK(turns(0) == "0");
    CHECK(turns(1) == "1/8");
    CHECK(turns(-2) == "1/4");
    CHECK(turns(4) == "1/2");
    CHECK(turns(6) == "3/4");
    CHECK(turns(8) == "1");
    CHECK(turns(11) == "1 3/8");
    CHECK(turns(-20) == "2 1/2");
    char tiny[3];
    format_turns(11, tiny, sizeof(tiny));
    CHECK(std::string(tiny) == "1 ");
}

TEST_CASE("courroies : jauge") {
    CHECK(gauge_x(80, 80, 108, 200) == 0);
    CHECK(gauge_x(108, 80, 108, 200) == 199);
    CHECK(gauge_x(94, 80, 108, 200) == 100);
    CHECK(gauge_x(50, 80, 108, 200) == 0);
    CHECK(gauge_x(500, 80, 108, 200) == 199);
    CHECK(gauge_x(NAN, 80, 108, 200) == 0);
    CHECK(gauge_x(90, 80, 80, 200) == 0);
    CHECK(plot_x(95, 70, 120, 101) == 50);
}

TEST_CASE("courroies : courbe du spectre") {
    const std::vector<float> v { 0, 1, 2, 3, 4 };
    // 5 points sur 9 colonnes : un point toutes les 2 colonnes, interpolation entre eux
    CHECK(curve_height(v, 5, 0, 9, 11, 4) == 0);
    CHECK(curve_height(v, 5, 8, 9, 11, 4) == 10);
    CHECK(curve_height(v, 5, 4, 9, 11, 4) == 5);
    CHECK(curve_height(v, 5, 3, 9, 11, 4) == 4); // 1,5 / 4 × 10 = 3,75
    // Points pas encore mesurés : rien
    CHECK(curve_height(v, 2, 2, 9, 11, 4) == 3); // point 1 mesuré
    CHECK(curve_height(v, 2, 3, 9, 11, 4) == -1);
    CHECK(curve_height(v, 0, 0, 9, 11, 4) == -1);
    // Bornes et échelle
    CHECK(curve_height(v, 5, -1, 9, 11, 4) == -1);
    CHECK(curve_height(v, 5, 9, 9, 11, 4) == -1);
    CHECK(curve_height(v, 5, 8, 9, 11, 2) == 10); // au-dessus de l'échelle : en haut
    CHECK(curve_height(v, 5, 8, 9, 11, 0) == 0); // échelle nulle : en bas
}
