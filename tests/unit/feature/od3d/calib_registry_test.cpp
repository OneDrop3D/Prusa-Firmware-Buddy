// OneDrop3D : registre de l'assistant de calibration. Quels tests proposer (accéléromètre oui / non, bobines, versions
// changées par une mise à jour), ordre des étapes et leurs G-codes, buse froide avant l'accéléromètre.
#include <feature/od3d/calib_registry_od3d.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
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
    return Context { .accelerometer = accel, .mmu = true, .flow_limit = true, .mpc = true, .spools = 0b00101 };
}

/// Comme full(), chaque bobine déjà mesurée (K, débit…)
Context measured(Accelerometer accel = Accelerometer::yes) {
    Context ctx = full(accel);
    ctx.measured.fill(0xff);
    return ctx;
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

TEST_CASE("calib_registry: MPC only while the MPC regulates (M-16)") {
    const Records none {};
    Context ctx = full();
    ctx.mpc = false; // PID de Prusa : 12 min pour rien
    CHECK_FALSE((select(ctx, none, true) & test_bit(Test::mpc)));
    CHECK_FALSE((proposed(ctx, none) & test_bit(Test::mpc)));
    CHECK((select(ctx, none, true) & test_bit(Test::nozzle_expansion))); // le reste de la série « Chauffe » reste
    ctx.mpc = true;
    CHECK((proposed(ctx, none) & test_bit(Test::mpc)));
}

TEST_CASE("calib_registry: version bump proposes only that test") {
    Records r = all_done();
    CHECK(select(full(), r, true) == 0);
    CHECK(proposed(measured(), r) == 0);

    // Une mise à jour a augmenté la version des courroies et du PA : seuls eux sont à refaire
    r[size_t(Test::belts)].version = uint8_t(entry(Test::belts).version - 1);
    r[size_t(Test::pa)].version = uint8_t(entry(Test::pa).version - 1);
    CHECK(select(full(), r, true) == (test_bit(Test::belts) | test_bit(Test::pa)));
    CHECK(proposed(measured(), r) != 0);

    // Sans accéléromètre, seul le PA
    CHECK(select(full(Accelerometer::no), r, true) == test_bit(Test::pa));

    // Toutes les calibrations applicables, à la demande (menu)
    CHECK((select(full(), r, false) & test_bit(Test::damping)));
}

TEST_CASE("calib_registry: first install proposes everything automatic") {
    const Records none {};
    const uint32_t m = select(full(), none, true);
    CHECK((m & automatic_mask()) == automatic_mask());
    CHECK(proposed(full(), none) != 0);
}

TEST_CASE("calib_registry: the boot prompt signature changes with any version, never 0") {
    // Une seule proposition au démarrage par signature (demande du 24/11) : une mise à jour qui monte une version ou
    // ajoute un test doit la changer
    CHECK(signature() != 0);
    CHECK(signature() == signature(entries));
    auto bumped = entries;
    bumped[3].version = uint8_t(bumped[3].version + 1);
    CHECK(signature(bumped) != signature());
    CHECK(signature(std::span(entries).first(entries.size() - 1)) != signature()); // un test de plus
}

TEST_CASE("calib_registry: manual tests are never checked by default") {
    Records r = all_done();
    r[size_t(Test::skew)] = {};
    CHECK((select(full(), r, true) == test_bit(Test::skew)));
    CHECK(proposed(measured(), r) == 0);
    CHECK_FALSE((automatic_mask() & test_bit(Test::skew)));
}

TEST_CASE("calib_registry: steps order") {
    std::array<Step, 32> steps;
    const size_t n = build_steps(automatic_mask(), full(), steps);
    // 8 à l'accéléromètre, MPC, tube PTFE, vitesse de la poulie, dilatation de la buse, ventilateurs, puis 4 mesures
    // pour chacune des 2 bobines
    constexpr size_t accel_steps = 8;
    REQUIRE(n == accel_steps + 1 + 1 + 1 + 1 + 1 + 4 * 2);

    // Accéléromètre d'abord (une seule pose) : sa pose vérifiée en premier, puis les courroies ; l'accélération max utile
    // après l'input shaper (elle mesure avec son filtre), la vérification du filtre ensuite
    CHECK(steps[0].test == Test::accel_mount);
    CHECK(steps[1].test == Test::belts);
    for (size_t i = 0; i < accel_steps; ++i) {
        CHECK((accel & test_bit(steps[i].test)));
    }
    const auto at = [&](Test t) {
        return std::find_if(steps.begin(), steps.begin() + n, [t](const Step &s) { return s.test == t; }) - steps.begin();
    };
    CHECK(at(Test::input_shaper) < at(Test::accel_limit));
    CHECK(at(Test::accel_limit) < at(Test::shaper_check));
    CHECK(steps[accel_steps].test == Test::mpc);
    // Tests du MMU à froid ensemble, avant la dilatation (le plateau chauffe pour elle pendant eux), avec la première
    // bobine ; la vitesse de la poulie après le tube PTFE (sa longueur fait l'avance rapide)
    CHECK(steps[accel_steps + 1].test == Test::mmu_ptfe);
    CHECK(steps[accel_steps + 1].slot == 0);
    CHECK(steps[accel_steps + 2].test == Test::mmu_speed);
    CHECK(steps[accel_steps + 2].slot == 0);
    // Dilatation de la buse (buse vide) avant les chargements, une fois ; puis les ventilateurs (sans besoin)
    CHECK(steps[accel_steps + 3].test == Test::nozzle_expansion);
    CHECK(steps[accel_steps + 4].test == Test::fan_response);

    // Puis par bobine : PA, débit, accélération du filament, température minimale en dernier (elle finit sous la
    // température de la matière : rien ne remonte après elle) ; un seul chargement par bobine
    const Test per_slot[] = { Test::pa, Test::max_flow, Test::e_jump, Test::min_temp };
    for (size_t k = 0; k < 8; ++k) {
        CHECK(steps[accel_steps + 5 + k].test == per_slot[k % 4]);
        CHECK(steps[accel_steps + 5 + k].slot == (k < 4 ? 0 : 2));
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
    CHECK(gcode({ Test::pa, 2 }) == "M1994 P2 V1"); // motif compact, même avec « Grand motif PA »
    CHECK(gcode({ Test::max_flow, 4 }) == "M1995 P4");
    CHECK(gcode({ Test::min_temp, 0 }) == "M1996 P0");
    CHECK(gcode({ Test::input_shaper }) == "M1959 A");
    CHECK(gcode({ Test::phase_stepping }) == "M1977 U A");
    CHECK(gcode({ Test::belts }) == "M1992");
    CHECK(gcode({ Test::speed_map }) == "M1993");
    CHECK(gcode({ Test::damping }) == "M1997");
    CHECK(gcode({ Test::mpc }) == "M306 T");
    CHECK(gcode({ Test::mmu_ptfe, 1 }) == "M1999 B1 P1");
    CHECK(gcode({ Test::fan_response }) == "M2001 V");
    CHECK(gcode({ Test::accel_limit }) == "M2004");
    CHECK(gcode({ Test::accel_mount }) == "M2005");
    CHECK(gcode({ Test::shaper_check }) == "M2006");
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

    // Dilatation après les tests du MMU à froid : son plateau a chauffé et s'est reposé pendant eux
    const uint32_t expansion = test_bit(Test::nozzle_expansion);
    CHECK(minutes(expansion, ctx) == entry(Test::nozzle_expansion).minutes);
    CHECK(minutes(expansion | test_bit(Test::mmu_ptfe), ctx) == entry(Test::nozzle_expansion).minutes + entry(Test::mmu_ptfe).minutes - 2);
}

TEST_CASE("calib_registry: the bed heats for the next test during what does not measure it") {
    std::array<Step, 32> steps;
    const size_t n = build_steps(series_tests(true), full(), steps); // bobines 1 et 3
    const std::span<const Step> s(steps.data(), n);
    const auto at = [&](Test t, uint8_t slot = 0) {
        return size_t(std::find_if(steps.begin(), steps.begin() + n, [&](const Step &x) { return x.test == t && x.slot == slot; }) - steps.begin());
    };
    // MPC : rien, il mesure les pertes de la buse dans l'air de la chambre
    CHECK_FALSE(bed_step(s, at(Test::mpc), false).has_value());
    // Tube PTFE, vitesse de la poulie (déchargement d'abord ou non) : pour la dilatation
    CHECK(bed_step(s, at(Test::mmu_ptfe), false) == at(Test::nozzle_expansion));
    CHECK(bed_step(s, at(Test::mmu_speed), true) == at(Test::nozzle_expansion));
    // La dilatation chauffe son plateau elle-même ; un déchargement avant elle : pendant lui
    CHECK_FALSE(bed_step(s, at(Test::nozzle_expansion), false).has_value());
    CHECK(bed_step(s, at(Test::nozzle_expansion), true) == at(Test::nozzle_expansion));
    // Bobine suivante : pendant son chargement, pour son PA ; jamais pendant une mesure (motif, accélération du filament)
    CHECK(bed_step(s, at(Test::pa, 2), true) == at(Test::pa, 2));
    CHECK_FALSE(bed_step(s, at(Test::max_flow), false).has_value());
    CHECK_FALSE(bed_step(s, at(Test::e_jump), false).has_value());
    CHECK_FALSE(bed_step(s, at(Test::e_jump), true).has_value()); // chargée pour elle seule : rien n'attend le plateau
    CHECK_FALSE(bed_step(s, n, false).has_value());
    // Seuls les tests du MMU laissent le plateau libre
    for (const Entry &e : entries) {
        CHECK(bed_free(e.test) == (e.test == Test::mmu_ptfe || e.test == Test::mmu_speed));
    }

    // Sans dilatation : pendant le tube PTFE, pour le PA de la première bobine
    const size_t m = build_steps(test_bit(Test::mmu_ptfe) | test_bit(Test::pa), full(), steps);
    REQUIRE(m == 3);
    CHECK(bed_step(std::span<const Step>(steps.data(), m), 0, false) == size_t(1));
    // Tube PTFE remis à la fin (MMU indisponible) : rien après lui
    CHECK_FALSE(bed_step(std::span<const Step>(steps.data(), 1), 0, false).has_value());
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

TEST_CASE("calib_registry: what is already done is not checked, and not redone") {
    // Retour du 23/11 : le PA de 2 bobines déjà mesuré à la main (registre jamais passé par l'assistant), l'assistant
    // refaisait le PA de toutes les bobines
    const Records none {};
    Context ctx = full(); // bobines 1 et 3 (bits 0 et 2)
    ctx.spools = 0b00111;
    ctx.measured[size_t(Test::pa)] = 0b00101;
    std::array<Step, 32> steps;
    size_t n = build_steps(test_bit(Test::pa), ctx, steps);
    REQUIRE(n == 1);
    CHECK(steps[0].slot == 1); // seulement la bobine sans K
    CHECK(minutes(test_bit(Test::pa), ctx) == entry(Test::pa).minutes);
    CHECK((proposed(ctx, none) & test_bit(Test::pa))); // coché : une bobine n'a pas de K

    // Toutes mesurées : pas coché ; coché à la main (redo) : toutes refaites
    ctx.measured[size_t(Test::pa)] = 0b00111;
    CHECK_FALSE((proposed(ctx, none) & test_bit(Test::pa)));
    CHECK(build_steps(test_bit(Test::pa), ctx, steps) == 0);
    CHECK(build_steps(test_bit(Test::pa), ctx, steps, test_bit(Test::pa)) == 3);

    // Même règle pour les autres mesures par bobine
    ctx.measured[size_t(Test::e_jump)] = 0b00011;
    n = build_steps(test_bit(Test::e_jump), ctx, steps);
    REQUIRE(n == 1);
    CHECK(steps[0].slot == 2);

    // Tests machine déjà réussis dans cette version : pas cochés
    Records r = all_done();
    ctx.measured.fill(0b00111);
    CHECK(proposed(ctx, r) == 0);
    r[size_t(Test::belts)] = {};
    CHECK(proposed(ctx, r) == test_bit(Test::belts));

    // Une mise à jour change l'algorithme du PA (version augmentée après une série) : toutes les bobines à refaire
    r = all_done();
    r[size_t(Test::pa)].version = uint8_t(entry(Test::pa).version - 1);
    if (r[size_t(Test::pa)].version > 0) {
        CHECK(redo_mask(r) == test_bit(Test::pa));
        CHECK((proposed(ctx, r) & test_bit(Test::pa)));
    }
    // Mesure faite à la main (version 0 ici) : gardée, rien à refaire
    r[size_t(Test::pa)] = {};
    CHECK(redo_mask(r) == 0);
    CHECK_FALSE((proposed(ctx, r) & test_bit(Test::pa)));
}

TEST_CASE("calib_registry: summary keeps the numbers of the spools that worked") {
    // Le PA d'une bobine échoue (confiance trop faible) : le test est « en échec », mais le K des autres reste montré
    Numbers after = no_numbers;
    after[0] = 400;
    CHECK(numbers_shown(entry(Test::pa), Result::failed, after));
    CHECK(numbers_shown(entry(Test::pa), Result::skipped, after));
    CHECK(numbers_shown(entry(Test::min_temp), Result::ok, after));
    CHECK_FALSE(numbers_shown(entry(Test::pa), Result::failed, no_numbers));
    // Test machine en échec : pas de chiffres
    CHECK_FALSE(numbers_shown(entry(Test::belts), Result::failed, after));
    CHECK(numbers_shown(entry(Test::belts), Result::ok, after));
}

TEST_CASE("calib_registry: spools in order of bed then nozzle temperature, loaded one first") {
    // 1 PETG (85 / 230), 2 PLA (60 / 215), 3 PETG, 4 PLA dans la buse, 5 PLA
    const std::array<int16_t, 5> bed { 85, 60, 85, 60, 60 };
    const std::array<int16_t, 5> nozzle { 230, 215, 230, 215, 215 };
    const auto order = spool_order(bed, nozzle, 3);
    CHECK(order[0] == 3); // dans la buse : pas de changement de bobine
    CHECK(order[1] == 1);
    CHECK(order[2] == 4);
    CHECK(order[3] == 0);
    CHECK(order[4] == 2);
    // Les étapes suivent cet ordre : plus de refroidissement du plateau entre deux bobines
    Context ctx = full();
    ctx.spools = 0b00111;
    ctx.order = spool_order(bed, nozzle, std::nullopt);
    std::array<Step, 8> steps;
    REQUIRE(build_steps(test_bit(Test::pa), ctx, steps) == 3);
    CHECK(steps[0].slot == 1);
    CHECK(steps[1].slot == 0);
    CHECK(steps[2].slot == 2);
}

TEST_CASE("calib_registry: MMU tests wait for the MMU, the others run while it is flashed") {
    const Context ctx = full();
    CHECK(uses_mmu(entry(Test::mmu_ptfe), ctx));
    CHECK(uses_mmu(entry(Test::mmu_speed), ctx));
    CHECK(uses_mmu(entry(Test::pa), ctx)); // charge ses bobines par le MMU
    CHECK(uses_mmu(entry(Test::e_jump), ctx));
    CHECK_FALSE(uses_mmu(entry(Test::mpc), ctx));
    CHECK_FALSE(uses_mmu(entry(Test::belts), ctx));
    CHECK_FALSE(uses_mmu(entry(Test::nozzle_expansion), ctx)); // sauf buse à vider : vu par l'assistant
    CHECK_FALSE(uses_mmu(entry(Test::fan_response), ctx));
    Context single = ctx;
    single.mmu = false;
    CHECK_FALSE(uses_mmu(entry(Test::pa), single)); // sans MMU : la bobine unique

    // Remise à la fin une seule fois, les autres dans leur ordre
    std::array<Step, 4> s { Step { Test::mpc }, Step { Test::mmu_ptfe, 0 }, Step { Test::nozzle_expansion }, Step { Test::fan_response } };
    defer(s, 1);
    CHECK(s[0].test == Test::mpc);
    CHECK(s[1].test == Test::nozzle_expansion);
    CHECK(s[2].test == Test::fan_response);
    CHECK(s[3].test == Test::mmu_ptfe);
    CHECK(s[3].deferred);
    CHECK_FALSE(s[1].deferred);
    defer(s, 3); // déjà remise : reste à sa place, l'assistant la reporte
    CHECK(s[3].test == Test::mmu_ptfe);
    defer(s, 9); // hors de la série : rien
    CHECK(s[0].test == Test::mpc);
}

TEST_CASE("calib_registry: changing spools never cools the nozzle in between") {
    // PLA (215) dans la buse, PETG (240) à charger : déchargement et chargement à 240 ; et dans l'autre sens aussi
    CHECK(load_temperature(215, 240) == 240);
    CHECK(load_temperature(240, 215) == 240);
    CHECK(load_temperature(std::nullopt, 215) == 215); // buse vide : la nouvelle matière
}

TEST_CASE("calib_registry: loading G-code, above the reserved purge line for a printing test") {
    std::array<char, 128> out;
    // Test qui imprime : palpage de la série d'abord, tête au-dessus du départ de la purge (Z absolu, après l'origine)
    REQUIRE(load_gcode(2, "#3", 240, Spot { 11.f, 13.f, 15.f }, out));
    CHECK(std::string_view(out.data()) == "M1989 L2\nG28 O I\nM104 S240\nG27 X11.0 Y13.0 Z15.0 P1\nM701 S\"#3\" W2 P2");
    // Sinon (accélération du filament) : position de chargement de Prusa
    REQUIRE(load_gcode(0, "PLA", 215, std::nullopt, out));
    CHECK(std::string_view(out.data()) == "G28 O I\nM104 S215\nG27 W2\nM701 S\"PLA\" W2 P0");
    // Origine avant toute descente : G28 avant G27
    REQUIRE(load_gcode(4, "PETG", 240, Spot { 200.f, 150.f, 15.f }, out));
    const std::string_view g(out.data());
    CHECK(g.find("G28") < g.find("G27"));
    std::array<char, 16> tiny;
    CHECK_FALSE(load_gcode(0, "PLA", 215, std::nullopt, tiny));
}

TEST_CASE("calib_registry: two series, cold with the accelerometer and heating") {
    const uint32_t cold = series_tests(false), heat = series_tests(true);
    // Chaque test automatique dans une seule série
    CHECK((cold & heat) == 0);
    CHECK((cold | heat) == automatic_mask());
    // À froid : tous les tests à l'accéléromètre (buse froide exigée), et la réponse du ventilateur (rien n'y chauffe)
    CHECK(cold == (needs_mask(Need::accelerometer) | test_bit(Test::fan_response)));
    for (const Entry &e : entries) {
        INFO("test " << int(e.test));
        if (cold & test_bit(e.test)) {
            CHECK_FALSE((e.needs & (Need::filament | Need::empty_bed | Need::mmu))); // ni bobine, ni plateau, ni MMU
        }
        if (e.needs & Need::accelerometer) {
            CHECK((e.needs & Need::cold_nozzle));
            CHECK((cold & test_bit(e.test)));
        }
    }
    // Chauffe : MPC, tube PTFE et poulie (extrudeur vidé à chaud, comme pour la dilatation), dilatation, par bobine
    for (const Test t : { Test::mpc, Test::mmu_ptfe, Test::nozzle_expansion, Test::mmu_speed, Test::pa, Test::max_flow, Test::min_temp, Test::e_jump }) {
        CHECK((heat & test_bit(t)));
    }
    // Les étapes d'une série ne contiennent que ses tests ; ordre gardé (MPC part de la buse froide)
    std::array<Step, 32> steps;
    const size_t n = build_steps(heat, full(), steps);
    REQUIRE(n > 0);
    CHECK(steps[0].test == Test::mpc);
    for (size_t i = 0; i < n; ++i) {
        CHECK((heat & test_bit(steps[i].test)));
    }
}
