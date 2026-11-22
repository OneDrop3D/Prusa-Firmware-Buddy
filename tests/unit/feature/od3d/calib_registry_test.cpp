// OneDrop3D : registre de l'assistant de calibration. Quels tests proposer (accéléromètre oui / non, bobines, versions
// changées par une mise à jour), ordre des étapes et leurs G-codes, buse froide avant l'accéléromètre.
#include <feature/od3d/calib_registry_od3d.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <string_view>

using namespace od3d::calib_registry;

namespace {

using Records = std::array<Record, max_tests>;

/// Tout fait dans la version actuelle
Records all_done() {
    Records r {};
    for (const Entry &e : entries) {
        r[size_t(e.test)] = Record { .version = e.version, .result = Result::ok };
    }
    return r;
}

Context full(Accelerometer accel = Accelerometer::yes) {
    return Context { .accelerometer = accel, .mmu = true, .flow_limit = true, .spools = 0b00101 };
}

const uint32_t accel = needs_mask(Need::accelerometer);

} // namespace

TEST_CASE("calib_registry: accelerometer yes / no / unknown") {
    const Records none {};
    const uint32_t yes = select(full(Accelerometer::yes), none, true);
    const uint32_t unknown = select(full(Accelerometer::unknown), none, true);
    const uint32_t no = select(full(Accelerometer::no), none, true);

    CHECK((yes & accel) == accel);
    CHECK(unknown == yes); // jamais répondu : proposés, l'assistant demande
    CHECK((no & accel) == 0);
    CHECK((no | accel) == yes);
    CHECK((accel & test_bit(Test::belts)));
    CHECK((accel & test_bit(Test::input_shaper)));
    CHECK((accel & test_bit(Test::phase_stepping)));
    CHECK_FALSE((accel & test_bit(Test::mpc)));
    CHECK_FALSE((accel & test_bit(Test::pa)));
}

TEST_CASE("calib_registry: spools and flow limit") {
    const Records none {};
    Context ctx = full();
    ctx.spools = 0;
    uint32_t m = select(ctx, none, true);
    CHECK_FALSE((m & test_bit(Test::pa)));
    CHECK_FALSE((m & test_bit(Test::min_temp)));
    CHECK_FALSE((m & test_bit(Test::e_jump))); // une mesure par bobine
    CHECK((m & test_bit(Test::mpc)));

    ctx = full();
    ctx.flow_limit = false;
    m = select(ctx, none, true);
    CHECK_FALSE((m & test_bit(Test::max_flow))); // son résultat ne sert qu'à « limiter au débit mesuré »
    CHECK((m & test_bit(Test::pa)));
    CHECK((m & test_bit(Test::min_temp)));
}

TEST_CASE("calib_registry: version bump proposes only that test") {
    Records r = all_done();
    CHECK(select(full(), r, true) == 0);
    CHECK_FALSE(should_propose(full(), r, 0));

    // Une mise à jour a augmenté la version des courroies et du PA : seuls eux sont à refaire
    r[size_t(Test::belts)].version = uint8_t(entry(Test::belts).version - 1);
    r[size_t(Test::pa)].version = uint8_t(entry(Test::pa).version - 1);
    CHECK(select(full(), r, true) == (test_bit(Test::belts) | test_bit(Test::pa)));
    CHECK(should_propose(full(), r, 0));

    // Sans accéléromètre, seul le PA
    CHECK(select(full(Accelerometer::no), r, true) == test_bit(Test::pa));

    // Toutes les calibrations applicables, à la demande (menu)
    CHECK((select(full(), r, false) & test_bit(Test::damping)));
}

TEST_CASE("calib_registry: first install proposes everything automatic") {
    const Records none {};
    const uint32_t m = select(full(), none, true);
    CHECK((m & automatic_mask()) == automatic_mask());
    CHECK(should_propose(full(), none, 0));
}

TEST_CASE("calib_registry: don't ask again until a version changes") {
    const Records none {};
    CHECK_FALSE(should_propose(full(), none, signature()));
    CHECK(should_propose(full(), none, signature() ^ 1)); // signature d'une autre version du registre
}

TEST_CASE("calib_registry: manual tests never trigger the prompt alone") {
    Records r = all_done();
    r[size_t(Test::skew)] = {};
    CHECK((select(full(), r, true) == test_bit(Test::skew)));
    CHECK_FALSE(should_propose(full(), r, 0));
    CHECK_FALSE((automatic_mask() & test_bit(Test::skew)));
}

TEST_CASE("calib_registry: steps order") {
    std::array<Step, 32> steps;
    const size_t n = build_steps(automatic_mask(), full(), steps);
    // 5 à l'accéléromètre, MPC, tube PTFE, dilatation de la buse, ventilateur, vitesse de la poulie, puis 4 mesures pour
    // chacune des 2 bobines
    REQUIRE(n == 5 + 1 + 1 + 1 + 1 + 1 + 4 * 2);

    // Accéléromètre d'abord (une seule pose), courroies en premier
    CHECK(steps[0].test == Test::belts);
    for (size_t i = 0; i < 5; ++i) {
        CHECK((accel & test_bit(steps[i].test)));
    }
    CHECK(steps[5].test == Test::mpc);
    // Tube PTFE avant les chargements, une fois, avec la première bobine
    CHECK(steps[6].test == Test::mmu_ptfe);
    CHECK(steps[6].slot == 0);
    // Dilatation de la buse (buse vide) avant les chargements, une fois ; puis la réponse du ventilateur (sans besoin)
    CHECK(steps[7].test == Test::nozzle_expansion);
    CHECK(steps[8].test == Test::fan_response);
    // Vitesse de la poulie : après le tube PTFE (sa longueur fait l'avance rapide), avant les chargements
    CHECK(steps[9].test == Test::mmu_speed);
    CHECK(steps[9].slot == 0);

    // Puis par bobine : PA, débit, température minimale, accélération du filament (un seul chargement par bobine)
    const Test per_slot[] = { Test::pa, Test::max_flow, Test::min_temp, Test::e_jump };
    for (size_t k = 0; k < 8; ++k) {
        CHECK(steps[10 + k].test == per_slot[k % 4]);
        CHECK(steps[10 + k].slot == (k < 4 ? 0 : 2));
    }

    // Jamais le test manuel
    for (size_t i = 0; i < n; ++i) {
        CHECK(steps[i].test != Test::skew);
    }
}

TEST_CASE("calib_registry: front PTFE needs the MMU and a spool") {
    const Records none {};
    Context ctx = full();
    ctx.spools = 0b11000;
    CHECK((select(ctx, none, true) & test_bit(Test::mmu_ptfe)));
    std::array<Step, 4> steps;
    REQUIRE(build_steps(test_bit(Test::mmu_ptfe), ctx, steps) == 1);
    CHECK(steps[0].slot == 3); // la première bobine

    ctx.mmu = false;
    CHECK_FALSE((select(ctx, none, true) & test_bit(Test::mmu_ptfe)));
    ctx = full();
    ctx.spools = 0;
    CHECK_FALSE((select(ctx, none, true) & test_bit(Test::mmu_ptfe)));
    CHECK(build_steps(test_bit(Test::mmu_ptfe), ctx, steps) == 0);

    // Vitesse de la poulie : mêmes besoins
    ctx = full();
    ctx.spools = 0b10000;
    REQUIRE(build_steps(test_bit(Test::mmu_speed), ctx, steps) == 1);
    CHECK(steps[0].slot == 4);
    ctx.mmu = false;
    CHECK_FALSE((select(ctx, none, true) & test_bit(Test::mmu_speed)));
}

TEST_CASE("calib_registry: nozzle expansion needs neither filament nor MMU") {
    const Records none {};
    Context ctx = full(Accelerometer::no);
    ctx.mmu = false;
    ctx.spools = 0;
    CHECK((select(ctx, none, true) & test_bit(Test::nozzle_expansion)));
    CHECK((needs_mask(Need::empty_bed) & test_bit(Test::nozzle_expansion))); // palpe le plateau
    std::array<Step, 4> steps;
    REQUIRE(build_steps(test_bit(Test::nozzle_expansion), ctx, steps) == 1);
    CHECK(steps[0].test == Test::nozzle_expansion);
}

TEST_CASE("calib_registry: steps of a subset and a short buffer") {
    std::array<Step, 32> steps;
    CHECK(build_steps(test_bit(Test::damping) | test_bit(Test::skew), full(), steps) == 1);
    CHECK(steps[0].test == Test::damping);

    std::array<Step, 3> few;
    CHECK(build_steps(automatic_mask(), full(), few) == 3);
    CHECK(build_steps(0, full(), steps) == 0);
}

TEST_CASE("calib_registry: G-codes") {
    std::array<char, 24> out;
    const auto gcode = [&](Step s) {
        return step_gcode(s, out) ? std::string_view(out.data()) : std::string_view("-");
    };
    CHECK(gcode({ Test::pa, 2 }) == "M1994 P2");
    CHECK(gcode({ Test::max_flow, 4 }) == "M1995 P4");
    CHECK(gcode({ Test::min_temp, 0 }) == "M1996 P0");
    CHECK(gcode({ Test::input_shaper }) == "M1959 A");
    CHECK(gcode({ Test::phase_stepping }) == "M1977 U A");
    CHECK(gcode({ Test::belts }) == "M1992");
    CHECK(gcode({ Test::speed_map }) == "M1993");
    CHECK(gcode({ Test::damping }) == "M1997");
    CHECK(gcode({ Test::mpc }) == "M306 T");
    CHECK(gcode({ Test::mmu_ptfe, 1 }) == "M1999 B1 P1");
    CHECK(gcode({ Test::fan_response }) == "M2001");
    CHECK(gcode({ Test::nozzle_expansion }) == "M2003");
    CHECK(gcode({ Test::e_jump, 3 }) == "M2000 S3");
    CHECK(gcode({ Test::mmu_speed, 3 }) == "M2002 P3");
    CHECK(gcode({ Test::skew }) == "-");

    std::array<char, 4> tiny;
    CHECK_FALSE(step_gcode({ Test::pa, 1 }, tiny));
}

TEST_CASE("calib_registry: duration") {
    const Context ctx = full();
    const uint16_t one = minutes(test_bit(Test::pa), ctx);
    CHECK(one == 2 * entry(Test::pa).minutes); // par bobine
    CHECK(minutes(test_bit(Test::skew), ctx) == 0);
    CHECK(minutes(test_bit(Test::belts) | test_bit(Test::pa), ctx) == entry(Test::belts).minutes + one);
}

TEST_CASE("calib_registry: results and records") {
    CHECK(fold(Result::none, Result::ok) == Result::ok);
    CHECK(fold(Result::ok, Result::ok) == Result::ok);
    CHECK(fold(Result::ok, Result::failed) == Result::failed);
    CHECK(fold(Result::failed, Result::ok) == Result::failed);
    CHECK(fold(Result::ok, Result::skipped) == Result::skipped);
    CHECK(fold(Result::skipped, Result::failed) == Result::failed);

    const Entry &e = entry(Test::pa);
    const Record old { .version = 0 };
    const Record failed = after_run(old, e, Result::failed, 1234);
    CHECK(failed.version == 0); // reste à refaire
    CHECK(failed.result == Result::failed);
    CHECK(failed.date == 1234);
    CHECK(outdated(e, failed));
    const Record ok = after_run(failed, e, Result::ok, 5678);
    CHECK(ok.version == e.version);
    CHECK_FALSE(outdated(e, ok));
    const Record skipped = after_run(ok, e, Result::skipped, 9999);
    CHECK(skipped.version == e.version); // une série annulée n'efface pas la calibration faite
}

TEST_CASE("calib_registry: cold nozzle gate") {
    CHECK(nozzle_cold(25.f, 0.f));
    CHECK(nozzle_cold(39.9f, 0.f));
    CHECK_FALSE(nozzle_cold(40.f, 0.f));
    CHECK_FALSE(nozzle_cold(87.f, 0.f));
    CHECK_FALSE(nozzle_cold(25.f, 215.f)); // chauffe allumée : elle va monter
    CHECK_FALSE(nozzle_cold(NAN, 0.f));
}

TEST_CASE("calib_registry: summary numbers") {
    // Baisse des vibrations restantes : 0,02 au lieu de 0,08 = -75 %
    CHECK(drop_percent(0.02f, 0.08f) == std::optional<int16_t>(75));
    CHECK(drop_percent(0.10f, 0.08f) == std::optional<int16_t>(-25)); // plus de vibrations qu'avant : affiché tel quel
    CHECK(drop_percent(0.f, 0.08f) == std::optional<int16_t>(100));
    CHECK_FALSE(drop_percent(0.02f, 0.f).has_value()); // pas de référence : pas de chiffre
    CHECK_FALSE(drop_percent(NAN, 0.08f).has_value());

    // Amortissement : ζ 0,1 (Prusa) → 4,8 périodes pour tomber à 5 % ; ζ plus faible → plus long
    CHECK(std::fabs(ringing_cycles(0.1f) - 4.77f) < 0.01f);
    CHECK(ringing_cycles(0.05f) > ringing_cycles(0.1f));
    CHECK(std::isnan(ringing_cycles(0.f)));

    // Valeurs gardées pour le « avant » : vides par défaut
    const Values v;
    for (const int16_t x : v.v) {
        CHECK(x == no_value);
    }
}
