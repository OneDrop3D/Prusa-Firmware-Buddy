/// OneDrop3D (F5, F6) : coude de la courbe force/débit et température minimale, sur des courbes synthétiques.
#include <feature/od3d/spool_calib_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>
#include <vector>

using namespace od3d::calib;
using Catch::Matchers::WithinAbs;

namespace {

/// Bruit déterministe dans [−1, 1]
float pseudo_noise(int i) {
    const uint32_t h = uint32_t(i + 1) * 2654435761u;
    return float(h >> 8) / float(1u << 24) * 2.f - 1.f;
}

/// Buse simulée : force = 40·Q^0,5 (fluidification), puis surcroît quadratique au-delà de q_melt (la buse ne fond plus
/// assez). Le dépassement de 25 % est atteint à q_melt + sqrt(0,25 / c).
std::vector<StepPoint> flow_ramp(float q_melt, float c, float sign = 1.f, float noise = 0.f, float lo = 2.f, float hi = 30.f, float step = 2.f) {
    std::vector<StepPoint> pts;
    int i = 0;
    for (float q = lo; q <= hi + 1e-3f; q += step, ++i) {
        const float over = q > q_melt ? c * (q - q_melt) * (q - q_melt) : 0.f;
        const float f = 40.f * std::sqrt(q) * (1.f + over) * (1.f + noise * pseudo_noise(i));
        pts.push_back({ q, sign * f, 0.02f * f });
    }
    return pts;
}

} // namespace

TEST_CASE("od3d F5: knee of a synthetic force/flow curve", "[od3d][calib]") {
    // Dépassement de 25 % à 18 + 5 = 23 mm³/s ; interpolé entre 22 et 24
    const auto pts = flow_ramp(18.f, 0.01f);
    const Knee k = find_knee(pts, true);
    REQUIRE(k.fit == KneeFit::found);
    CHECK_FALSE(k.skipping);
    CHECK_THAT(k.n, WithinAbs(0.5, 0.01));
    CHECK_THAT(k.a, WithinAbs(40.0, 0.5));
    CHECK_THAT(k.flow, WithinAbs(23.0, 0.6));
    CHECK(k.sign == 1.f);
    CHECK(pts[k.index].x > k.flow);
}

TEST_CASE("od3d F5: knee with the load cell mounted the other way", "[od3d][calib]") {
    const Knee k = find_knee(flow_ramp(18.f, 0.01f, -1.f), true);
    REQUIRE(k.fit == KneeFit::found);
    CHECK(k.sign == -1.f);
    CHECK_THAT(k.flow, WithinAbs(23.0, 0.6));
}

TEST_CASE("od3d F5: knee with 3 % noise", "[od3d][calib]") {
    const Knee k = find_knee(flow_ramp(14.f, 0.01f, 1.f, 0.03f), true);
    REQUIRE(k.fit == KneeFit::found);
    CHECK_THAT(k.flow, WithinAbs(19.0, 1.5));
}

TEST_CASE("od3d F5: no knee within the ramp", "[od3d][calib]") {
    const auto pts = flow_ramp(100.f, 0.01f);
    const Knee k = find_knee(pts, true);
    CHECK(k.fit == KneeFit::not_reached);
    CHECK_THAT(k.flow, WithinAbs(30.0, 1e-4));
    CHECK(k.index == pts.size());
    CHECK_FALSE(k.pending);
}

TEST_CASE("od3d F5: a linear curve (n = 1) has no knee", "[od3d][calib]") {
    std::vector<StepPoint> pts;
    for (float q = 2; q <= 30; q += 2) {
        pts.push_back({ q, 12.f * q, 0.5f });
    }
    const Knee k = find_knee(pts, true);
    CHECK(k.fit == KneeFit::not_reached);
    CHECK_THAT(k.n, WithinAbs(1.0, 1e-3));
}

TEST_CASE("od3d F5: an isolated outlier is not a knee", "[od3d][calib]") {
    auto pts = flow_ramp(100.f, 0.01f);
    pts[6].force_g *= 1.4f; // 14 mm³/s
    const Knee k = find_knee(pts, true);
    CHECK(k.fit == KneeFit::not_reached);
}

TEST_CASE("od3d F5: extruder skipping (force drop) ends at the previous step", "[od3d][calib]") {
    auto pts = flow_ramp(100.f, 0.01f);
    pts[9].force_g = pts[8].force_g * 0.8f; // 20 mm³/s : la force chute
    const Knee k = find_knee(pts, true);
    REQUIRE(k.fit == KneeFit::found);
    CHECK(k.skipping);
    CHECK_THAT(k.flow, WithinAbs(18.0, 1e-4));
    CHECK(k.index == 9);
}

TEST_CASE("od3d F5: extruder skipping (noisy force) ends at the previous step", "[od3d][calib]") {
    auto pts = flow_ramp(100.f, 0.01f);
    pts[10].noise_g = 0.5f * pts[10].force_g; // 22 mm³/s
    const Knee k = find_knee(pts, true);
    REQUIRE(k.fit == KneeFit::found);
    CHECK(k.skipping);
    CHECK_THAT(k.flow, WithinAbs(20.0, 1e-4));
}

TEST_CASE("od3d F5: an excess on the last step waits for its confirmation", "[od3d][calib]") {
    // Coude à 23 : la rampe s'arrête à 24, premier palier au-delà
    const auto all = flow_ramp(18.f, 0.01f);
    const std::span<const StepPoint> upto24(all.data(), 12);
    REQUIRE(upto24.back().x == 24.f);
    const Knee running = find_knee(upto24, false);
    CHECK(running.fit == KneeFit::not_reached);
    CHECK(running.pending);
    const Knee last = find_knee(upto24, true);
    CHECK(last.fit == KneeFit::found);
    CHECK_THAT(last.flow, WithinAbs(23.0, 0.6));
    // Le palier suivant confirme pendant la rampe
    const Knee confirmed = find_knee(std::span<const StepPoint>(all.data(), 13), false);
    CHECK(confirmed.fit == KneeFit::found);
}

TEST_CASE("od3d F5: weak or missing signal", "[od3d][calib]") {
    std::vector<StepPoint> weak;
    for (float q = 2; q <= 30; q += 2) {
        weak.push_back({ q, 0.1f * q, 0.05f });
    }
    CHECK(find_knee(weak, true).fit == KneeFit::weak_signal);

    auto zero_ref = flow_ramp(18.f, 0.01f);
    zero_ref[1].force_g = 0.f;
    CHECK(find_knee(zero_ref, true).fit == KneeFit::weak_signal);

    const auto few = flow_ramp(18.f, 0.01f, 1.f, 0.f, 2.f, 8.f);
    CHECK(few.size() == 4);
    CHECK(find_knee(few, true).fit == KneeFit::too_few_points);
}

TEST_CASE("od3d F5: exponent bounded", "[od3d][calib]") {
    // Force presque constante (n ≈ 0,05) : n borné à 0,2, la loi reste au-dessus des premiers points
    std::vector<StepPoint> pts;
    for (float q = 2; q <= 30; q += 2) {
        pts.push_back({ q, 100.f * std::pow(q, 0.05f), 1.f });
    }
    const Knee k = find_knee(pts, true);
    CHECK_THAT(k.n, WithinAbs(0.2, 1e-5));
    CHECK(k.fit == KneeFit::not_reached);
}

namespace {

/// Force qui croît quand la température baisse : f = 50·exp(k·(t0 − t))
std::vector<StepPoint> temperature_sweep(float k, float sign = 1.f, int16_t t0 = 215, int16_t floor = 180, int16_t step = 5) {
    std::array<int16_t, 24> temps;
    const size_t n = temperature_steps(t0, floor, step, temps);
    std::vector<StepPoint> pts;
    for (size_t i = 0; i < n; ++i) {
        const float f = 50.f * std::exp(k * float(t0 - temps[i]));
        pts.push_back({ float(temps[i]), sign * f, 0.03f * f });
    }
    return pts;
}

} // namespace

TEST_CASE("od3d F6: lowest temperature under the force threshold", "[od3d][calib]") {
    // Rapport 1,5 atteint 20,3 °C sous 215 : 195 °C passe (1,49), 190 °C non (1,65)
    const auto pts = temperature_sweep(0.02f);
    const MinTemp m = find_min_temperature(pts, 180.f);
    REQUIRE(m.fit == MinTempFit::found);
    CHECK(m.temperature == 195.f);
    CHECK(m.index == 5);
    CHECK_THAT(m.reference_g, WithinAbs(50.0, 1e-3));
}

TEST_CASE("od3d F6: other sign and other threshold", "[od3d][calib]") {
    const auto pts = temperature_sweep(0.02f, -1.f);
    const MinTemp m = find_min_temperature(pts, 180.f, { .max_ratio = 2.f });
    REQUIRE(m.fit == MinTempFit::found);
    CHECK(m.sign == -1.f);
    CHECK(m.temperature == 185.f); // exp(0,6) = 1,82 ; exp(0,7) = 2,01
}

TEST_CASE("od3d F6: floor reached", "[od3d][calib]") {
    const auto pts = temperature_sweep(0.005f);
    const MinTemp m = find_min_temperature(pts, 180.f);
    CHECK(m.fit == MinTempFit::floor_reached);
    CHECK(m.temperature == 180.f);
    // Série interrompue avant le plancher
    const MinTemp partial = find_min_temperature(std::span<const StepPoint>(pts.data(), 4), 180.f);
    CHECK(partial.fit == MinTempFit::not_reached);
    CHECK(partial.temperature == 200.f);
}

TEST_CASE("od3d F6: a noisy step is not acceptable", "[od3d][calib]") {
    auto pts = temperature_sweep(0.005f);
    pts[3].noise_g = 0.4f * pts[3].force_g; // 200 °C : filament qui accroche
    const MinTemp m = find_min_temperature(pts, 180.f);
    REQUIRE(m.fit == MinTempFit::found);
    CHECK(m.temperature == 205.f);
}

TEST_CASE("od3d F6: weak signal and too few points", "[od3d][calib]") {
    std::vector<StepPoint> pts { { 215, 1.f, 0.1f }, { 210, 1.2f, 0.1f } };
    CHECK(find_min_temperature(pts, 180.f).fit == MinTempFit::weak_signal);
    CHECK(find_min_temperature(std::span<const StepPoint>(pts.data(), 1), 180.f).fit == MinTempFit::too_few_points);
}

TEST_CASE("od3d F6: temperature steps", "[od3d][calib]") {
    std::array<int16_t, 24> t;
    REQUIRE(temperature_steps(215, 180, 5, t) == 8);
    CHECK(t[0] == 215);
    CHECK(t[7] == 180);
    REQUIRE(temperature_steps(215, 182, 5, t) == 8);
    CHECK(t[6] == 185);
    CHECK(t[7] == 182);
    CHECK(temperature_steps(180, 180, 5, t) == 0);
    CHECK(temperature_steps(215, 180, 0, t) == 0);
    std::array<int16_t, 3> small;
    CHECK(temperature_steps(215, 180, 5, small) == 3);
}

TEST_CASE("od3d F5: memory and planner limit", "[od3d][calib]") {
    CHECK(flow_to_e1(18.46f) == 185);
    CHECK_THAT(flow_from_e1(185), WithinAbs(18.5, 1e-5));
    CHECK(flow_to_e1(NAN) == flow_none);
    CHECK(flow_to_e1(-1.f) == flow_none);
    CHECK(std::isnan(flow_from_e1(flow_none)));

    constexpr float area = 2.4053f;
    CHECK_THAT(e_speed_limit(24.f, 5.f, 40.f, area), WithinAbs(24.0 / area, 1e-4));
    CHECK_THAT(e_speed_limit(1.f, 5.f, 40.f, area), WithinAbs(5.0 / area, 1e-4)); // borne basse sûre
    CHECK_THAT(e_speed_limit(90.f, 5.f, 40.f, area), WithinAbs(40.0 / area, 1e-4));
    CHECK(e_speed_limit(NAN, 5.f, 40.f, area) == 0.f);
    CHECK(e_speed_limit(0.f, 5.f, 40.f, area) == 0.f);

    Record r;
    CHECK_FALSE(r.has_flow());
    CHECK_FALSE(r.has_min_temperature());
}
