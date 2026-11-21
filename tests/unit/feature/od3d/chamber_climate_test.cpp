// OneDrop3D (F29) : climat de la chambre. Recettes en bits, plafond de chambre, descente douce du plateau
// (paliers, jamais au-dessus de la mesure, arrêt dès qu'une autre consigne apparaît, durée bornée).
#include <feature/od3d/chamber_climate_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <cmath>

using namespace od3d::chamber_climate;

namespace {
constexpr uint32_t minute = 60 * 1000;
}

TEST_CASE("climat : recettes rangées par matière", "[od3d][climate]") {
    uint32_t presets = 0;
    presets = recipe_set<uint32_t>(presets, 0, Recipe::ceiling); // PLA
    presets = recipe_set<uint32_t>(presets, 5, Recipe::soft_cooldown); // ABS
    presets = recipe_set<uint32_t>(presets, 13, Recipe::both); // BVOH
    CHECK(recipe_get<uint32_t>(presets, 0) == Recipe::ceiling);
    CHECK(recipe_get<uint32_t>(presets, 1) == Recipe::none);
    CHECK(recipe_get<uint32_t>(presets, 5) == Recipe::soft_cooldown);
    CHECK(recipe_get<uint32_t>(presets, 13) == Recipe::both);
    presets = recipe_set<uint32_t>(presets, 13, Recipe::none);
    CHECK(recipe_get<uint32_t>(presets, 13) == Recipe::none);
    CHECK(recipe_get<uint32_t>(presets, 0) == Recipe::ceiling); // les autres ne bougent pas

    // Hors du mot : rien n'est écrit ni lu
    CHECK(recipe_set<uint32_t>(presets, 16, Recipe::both) == presets);
    CHECK(recipe_get<uint32_t>(0xffffffff, 16) == Recipe::none);

    uint16_t users = recipe_set<uint16_t>(0, 7, Recipe::both);
    CHECK(recipe_get<uint16_t>(users, 7) == Recipe::both);
    CHECK(recipe_get<uint16_t>(users, 6) == Recipe::none);
    CHECK(recipe_get<uint16_t>(0, 3) == Recipe::none); // par défaut : tout coupé
}

TEST_CASE("climat : plafond de la chambre", "[od3d][climate]") {
    // PLA avec plafond, maximum 38 °C, aucune cible, ventilateurs en automatique
    CHECK(ceiling_target(Recipe::ceiling, 38, false, true) == std::optional<uint8_t>(38));
    CHECK(ceiling_target(Recipe::both, 45, false, true) == std::optional<uint8_t>(45));
    // Une cible donnée (trancheur, M141, utilisateur) prime
    CHECK_FALSE(ceiling_target(Recipe::ceiling, 38, true, true).has_value());
    // Ventilateurs en manuel : la chambre ne peut rien
    CHECK_FALSE(ceiling_target(Recipe::ceiling, 38, false, false).has_value());
    // Pas de maximum, ou recette sans plafond
    CHECK_FALSE(ceiling_target(Recipe::ceiling, std::nullopt, false, true).has_value());
    CHECK_FALSE(ceiling_target(Recipe::soft_cooldown, 38, false, true).has_value());
    CHECK_FALSE(ceiling_target(Recipe::none, 38, false, true).has_value());
}

TEST_CASE("climat : réglages ramenés dans leurs plages", "[od3d][climate]") {
    const CooldownParams p = CooldownParams { .step_c = 0, .interval_min = 99, .floor_c = 200 }.clamped();
    CHECK(p.step_c == CooldownParams::step_min);
    CHECK(p.interval_min == CooldownParams::interval_min_max);
    CHECK(p.floor_c == CooldownParams::floor_max);
    CHECK(CooldownParams::interval_min_max < 30); // toujours un palier avant les 30 min du minuteur de sécurité
}

TEST_CASE("climat : descente douce complète", "[od3d][climate]") {
    Cooldown c;
    const CooldownParams p { .step_c = 5, .interval_min = 5, .floor_c = 60 };
    uint32_t now = 1000;

    // Impression à 100 °C, plateau à 97,6 °C au moment de la fin : premier palier 97 (jamais au-dessus de la mesure)
    const auto first = c.start(p, 100, 97.6f, now);
    REQUIRE(first == std::optional<int16_t>(97));
    REQUIRE(c.active());
    CHECK(c.steps_left() == 8); // 92 87 82 77 72 67 62 puis fin

    int16_t bed_target = *first;
    float bed = 97.0f;
    int16_t previous = bed_target;
    int steps = 0;
    for (; steps < 50; ++steps) {
        // Rien avant la fin du palier
        CHECK(c.step({ .now_ms = now + 4 * minute, .bed_current = bed, .bed_target = bed_target, .nozzle_heating = false }).action == Cooldown::Action::none);
        now += 5 * minute;
        const auto s = c.step({ .now_ms = now, .bed_current = bed, .bed_target = bed_target, .nozzle_heating = false });
        if (s.action == Cooldown::Action::finish) {
            CHECK(s.target == 0);
            break;
        }
        REQUIRE(s.action == Cooldown::Action::set_target);
        CHECK(s.target < previous); // ne fait que baisser
        CHECK(s.target <= previous - 5);
        CHECK(s.target > p.floor_c);
        previous = s.target;
        bed_target = s.target;
        c.confirm(bed_target);
        bed = bed_target; // le plateau suit
    }
    CHECK(steps == 7);
    CHECK_FALSE(c.active());
    CHECK(previous == 62);
}

TEST_CASE("climat : jamais de réchauffe", "[od3d][climate]") {
    Cooldown c;
    const CooldownParams p { .step_c = 2, .interval_min = 1, .floor_c = 40 };
    uint32_t now = 0;
    REQUIRE(c.start(p, 110, 80.2f, now) == std::optional<int16_t>(80)); // plateau déjà plus froid que l'impression
    // Le plateau refroidit plus vite que les paliers : le palier suit la mesure
    now += minute;
    const auto s = c.step({ .now_ms = now, .bed_current = 70.4f, .bed_target = 80, .nozzle_heating = false });
    REQUIRE(s.action == Cooldown::Action::set_target);
    CHECK(s.target == 70);
}

TEST_CASE("climat : rien à faire sous le seuil ou sans mesure", "[od3d][climate]") {
    Cooldown c;
    const CooldownParams p { .step_c = 5, .interval_min = 5, .floor_c = 60 };
    CHECK_FALSE(c.start(p, 60, 59.0f, 0).has_value());
    CHECK_FALSE(c.start(p, 100, 60.9f, 0).has_value()); // 60 = seuil
    CHECK_FALSE(c.start(p, 0, 90.0f, 0).has_value()); // plateau non chauffé pendant l'impression
    CHECK_FALSE(c.start(p, 100, NAN, 0).has_value());
    CHECK_FALSE(c.active());
    CHECK(c.step({ .now_ms = 10 * minute, .bed_current = 90, .bed_target = 0, .nozzle_heating = false }).action == Cooldown::Action::none);
}

TEST_CASE("climat : une autre consigne arrête la descente sans rien toucher", "[od3d][climate]") {
    const CooldownParams p { .step_c = 5, .interval_min = 5, .floor_c = 50 };

    SECTION("M140, Refroidir, préchauffage, minuteur de sécurité") {
        Cooldown c;
        REQUIRE(c.start(p, 100, 100, 0).has_value());
        const auto s = c.step({ .now_ms = 1000, .bed_current = 99, .bed_target = 0, .nozzle_heating = false });
        CHECK(s.action == Cooldown::Action::abort);
        CHECK_FALSE(c.active());
        CHECK(c.step({ .now_ms = 6 * minute, .bed_current = 99, .bed_target = 0, .nozzle_heating = false }).action == Cooldown::Action::none);
    }

    SECTION("une buse chauffe au moment du palier") {
        Cooldown c;
        REQUIRE(c.start(p, 100, 100, 0).has_value());
        CHECK(c.step({ .now_ms = minute, .bed_current = 100, .bed_target = 100, .nozzle_heating = true }).action == Cooldown::Action::none);
        CHECK(c.step({ .now_ms = 5 * minute, .bed_current = 100, .bed_target = 100, .nozzle_heating = true }).action == Cooldown::Action::abort);
        CHECK_FALSE(c.active());
    }

    SECTION("arrêt demandé") {
        Cooldown c;
        REQUIRE(c.start(p, 100, 100, 0).has_value());
        c.stop();
        CHECK(c.step({ .now_ms = 5 * minute, .bed_current = 100, .bed_target = 100, .nozzle_heating = false }).action == Cooldown::Action::none);
    }
}

TEST_CASE("climat : durée bornée", "[od3d][climate]") {
    Cooldown c;
    // Paliers de 1 °C toutes les 20 min depuis 110 °C : bien plus de 3 h ; la borne coupe le plateau
    const CooldownParams p { .step_c = 1, .interval_min = 20, .floor_c = 35 };
    uint32_t now = 0xfffff000u; // passage par zéro du compteur de millisecondes
    REQUIRE(c.start(p, 110, 110, now).has_value());
    int16_t target = 110;
    Cooldown::Step s;
    for (int i = 0; i < 20; ++i) {
        now += 20 * minute;
        s = c.step({ .now_ms = now, .bed_current = float(target), .bed_target = target, .nozzle_heating = false });
        if (s.action != Cooldown::Action::set_target) {
            break;
        }
        target = s.target;
        c.confirm(target);
    }
    CHECK(s.action == Cooldown::Action::finish);
    CHECK(s.target == 0);
    CHECK(target >= 110 - 9); // 9 paliers au plus en 3 h
}

TEST_CASE("climat : temps avant le palier suivant", "[od3d][climate]") {
    Cooldown c;
    const CooldownParams p { .step_c = 5, .interval_min = 5, .floor_c = 50 };
    REQUIRE(c.start(p, 90, 90, 1000).has_value());
    CHECK(c.ms_to_next_step(1000) == 5 * minute);
    CHECK(c.ms_to_next_step(1000 + 2 * minute) == 3 * minute);
    CHECK(c.ms_to_next_step(1000 + 6 * minute) == 0);
}
