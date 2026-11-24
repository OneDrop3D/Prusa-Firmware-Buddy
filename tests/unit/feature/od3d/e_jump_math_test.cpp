// OneDrop3D : accélération du filament (M2000). Paliers, classement, F = saut / K borné, estimation de la ligne.
#include <feature/od3d/e_jump_math_od3d.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <vector>

using namespace od3d::e_jump;
using Catch::Approx;

namespace {

/// Paliers 4, 5, 6.25... avec leurs détections
std::vector<Step> run(std::initializer_list<uint8_t> stalls) {
    std::array<float, max_steps> jumps;
    jump_steps(jumps);
    std::vector<Step> out;
    size_t i = 0;
    for (const uint8_t s : stalls) {
        out.push_back({ jumps[i++], s });
    }
    return out;
}

} // namespace

TEST_CASE("e_jump: steps") {
    std::array<float, 20> j;
    const size_t n = jump_steps(j);
    REQUIRE(n == max_steps);
    CHECK(j[0] == Approx(first_jump));
    CHECK(j[n - 1] <= max_jump);
    CHECK(j[n - 1] * step_ratio > max_jump);
    for (size_t i = 1; i < n; ++i) {
        CHECK(j[i] == Approx(j[i - 1] * step_ratio));
    }
    std::array<float, 3> few;
    CHECK(jump_steps(few) == 3);
}

TEST_CASE("e_jump: pulse shape") {
    const Pulse p = pulse(20.f, 0.04f);
    CHECK(p.peak_speed == Approx(base_speed + 20.f));
    CHECK(p.accel == Approx(500.f)); // 20 mm/s en 0,04 s
    // Montée et descente (0,04 × (2 + 10) chacune) et palier (0,04 × 22) tiennent dans le segment du saut
    const float ramp = (p.peak_speed * p.peak_speed - p.base_speed * p.base_speed) / (2.f * p.accel);
    CHECK(p.jump_mm == Approx(2.f * ramp + 0.04f * p.peak_speed));
    // Tous les paliers : au plus ~60 mm de filament
    std::array<float, max_steps> j;
    float total = 0;
    for (size_t i = 0; i < jump_steps(j); ++i) {
        total += pulses * pulse(j[i], 0.04f).total_mm();
    }
    CHECK(total < 65.f);
}

TEST_CASE("e_jump: step states and stop") {
    CHECK(state({ 4, 0 }) == StepState::clean);
    CHECK(state({ 4, 1 }) == StepState::doubtful);
    CHECK(state({ 4, 2 }) == StepState::stalled);
    CHECK_FALSE(must_stop({}));
    CHECK_FALSE(must_stop(run({ 0, 0, 1 })));
    CHECK(must_stop(run({ 0, 0, 2 })));
    CHECK(must_stop(run({ 0, 1, 1 })));
    CHECK_FALSE(must_stop(run({ 1, 0, 1 })));
}

TEST_CASE("e_jump: classify") {
    std::array<float, max_steps> j;
    jump_steps(j);

    // Calage au 5e palier : le 4e est le dernier sûr
    Verdict v = classify(run({ 0, 0, 0, 0, 2 }), false);
    CHECK(v.fit == Fit::found);
    CHECK(v.safe_index == 3);
    CHECK(v.stop_index == 4);
    CHECK(v.safe_jump == Approx(j[3]));
    CHECK(v.kept_jump() == Approx(margin * j[3]));

    // Un douteux isolé suivi d'un palier propre : parasite, la rampe continue
    v = classify(run({ 0, 1, 0, 0, 2 }), false);
    CHECK(v.fit == Fit::found);
    CHECK(v.safe_index == 3);

    // Deux douteux de suite : le calage commence au premier, le palier propre d'avant est le dernier sûr
    v = classify(run({ 0, 0, 1, 1 }), false);
    CHECK(v.fit == Fit::found);
    CHECK(v.stop_index == 2);
    CHECK(v.safe_index == 1);

    // Douteux juste avant le calage : jamais compté comme sûr
    v = classify(run({ 0, 0, 1, 2 }), false);
    CHECK(v.safe_index == 1);

    // Calage dès le premier palier : rien d'enregistrable
    v = classify(run({ 2 }), false);
    CHECK(v.fit == Fit::too_low);
    CHECK(v.safe_index == 0xff);
    v = classify(run({ 1, 1 }), false);
    CHECK(v.fit == Fit::too_low);

    // Tous les paliers sans calage : borne basse au dernier propre
    v = classify(run({ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }), true);
    CHECK(v.fit == Fit::not_reached);
    CHECK(v.safe_index == max_steps - 1);
    v = classify(run({ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }), true);
    CHECK(v.fit == Fit::not_reached);
    CHECK(v.safe_index == max_steps - 2);

    // Arrêtée avant la fin sans calage : rien
    v = classify(run({ 0, 0, 0 }), false);
    CHECK(v.fit == Fit::none);
}

TEST_CASE("e_jump: F = jump / K, bounded") {
    // PLA K 0,04 : saut gardé 12 mm/s → 300 mm/s²
    CHECK(filament_accel(12.f, 0.04f) == Approx(300.f));
    // Jamais sous Prusa, jamais au-dessus de 300 (N-04)
    CHECK(filament_accel(2.f, 0.04f) == Approx(prusa_filament_accel));
    CHECK(filament_accel(40.f, 0.005f) == Approx(max_filament_accel));
    CHECK(max_filament_accel == 300.f); // N-04 : 2,5 fois Prusa au plus
    CHECK(filament_accel(50.f, 0.04f) == Approx(300.f)); // 1 250 demandés : bornés
    // Sans PA ou sans saut : rien à limiter
    CHECK(std::isnan(filament_accel(12.f, 0.f)));
    CHECK(std::isnan(filament_accel(0.f, 0.04f)));
    CHECK(std::isnan(filament_accel(NAN, 0.04f)));
    CHECK(std::isnan(filament_accel(12.f, NAN)));
}

TEST_CASE("e_jump: line acceleration (estimated)") {
    // Prusa (120) : ~3500 mm/s² en 0,45 × 0,2 (section du trancheur 0,0814 mm²), ~1560 en buse HF 0,68 × 0,3
    CHECK(line_accel(prusa_filament_accel) == Approx(3545.f).margin(5.f));
    CHECK(line_accel(prusa_filament_accel, 0.68f, 0.3f) == Approx(1563.f).margin(5.f));
    CHECK(line_accel(300.f) == Approx(2.5f * line_accel(prusa_filament_accel)));
    CHECK(std::isnan(line_accel(120.f, 0.05f, 0.4f))); // cordon impossible (plus haut que large)
    CHECK(line_accel(1500.f) == Approx(xy_accel_cap)); // borné à l'XY de la machine
    CHECK(std::isnan(line_accel(NAN)));
}

TEST_CASE("e_jump: record") {
    Record r;
    CHECK_FALSE(r.has_jump());
    r.jump_e1 = jump_to_e1(8.44f);
    CHECK(r.jump_e1 == 84);
    CHECK(r.jump() == Approx(8.4f));
    CHECK(jump_to_e1(NAN) == 0);
    CHECK(jump_to_e1(-1.f) == 0);
}
