// OneDrop3D (F43) : filament chargé retenu après un redémarrage, ni rechargement ni purge s'il est déjà dans la buse.
#include <feature/od3d/filament_memory_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace od3d::filament_memory;

namespace {

Record pla_in(uint8_t slot) {
    return make_record(Record {}, slot, "PLA", 1'700'000'000);
}

/// Tout d'accord, registres relus : le cas où l'on restaure
BootView agreeing(uint8_t slot) {
    return BootView {
        .stored = pla_in(slot),
        .material_matches = true,
        .finda = true,
        .extruder = true,
        .selector = selector_unknown, // filament dans le sélecteur : le MMU n'a pas refait son origine
        .mmu_busy = false,
        .active_ms = settle_ms,
    };
}

} // namespace

TEST_CASE("mémoire du filament : restauré quand capteurs, MMU et matière sont d'accord") {
    CHECK(boot_decision(agreeing(2)) == Boot::restore);
    BootView v = agreeing(2);
    v.selector = 2; // MMU qui a gardé son origine : même emplacement
    CHECK(boot_decision(v) == Boot::restore);
}

TEST_CASE("mémoire du filament : jamais « chargé » si un capteur ne voit rien") {
    BootView v = agreeing(1);
    v.finda = false;
    CHECK(boot_decision(v) == Boot::forget);
    v = agreeing(1);
    v.extruder = false; // capteur de l'extrudeur vide, inconnu ou coupé
    CHECK(boot_decision(v) == Boot::forget);
}

TEST_CASE("mémoire du filament : oubliée si le MMU contredit l'emplacement ou si la matière a changé") {
    BootView v = agreeing(3);
    v.selector = 0;
    CHECK(boot_decision(v) == Boot::forget);
    v.selector = 5; // parqué : pas de filament dans le sélecteur
    CHECK(boot_decision(v) == Boot::forget);
    v = agreeing(3);
    v.material_matches = false;
    CHECK(boot_decision(v) == Boot::forget);
}

TEST_CASE("mémoire du filament : attend les registres du MMU et la fin de ses commandes ou erreurs") {
    BootView v = agreeing(0);
    v.active_ms = settle_ms - 1; // FINDA vaut 0 avant la 1re lecture
    CHECK(boot_decision(v) == Boot::wait);
    v.finda = false; // même un « rien » trop tôt ne fait rien oublier
    CHECK(boot_decision(v) == Boot::wait);
    v = agreeing(0);
    v.mmu_busy = true;
    CHECK(boot_decision(v) == Boot::wait);
}

TEST_CASE("mémoire du filament : rien d'enregistré, rien à restaurer") {
    BootView v = agreeing(0);
    v.stored = Record {};
    CHECK(boot_decision(v) == Boot::forget);
    v.active_ms = 0;
    CHECK(boot_decision(v) == Boot::forget);
}

TEST_CASE("déjà dans la buse : l'emplacement de Prusa seulement si les capteurs sont d'accord") {
    CHECK(confirmed_slot(2, true, true) == uint8_t(2));
    CHECK_FALSE(confirmed_slot(2, false, true));
    CHECK_FALSE(confirmed_slot(2, true, false));
    CHECK_FALSE(confirmed_slot(0xff, true, true)); // Prusa ne sait rien
}

TEST_CASE("buse : jamais de filament supposé (PA imprimé dans le vide, retour du 23/11)") {
    // MMU actif : Prusa, FINDA et la tête d'accord
    NozzleView v { .mmu_configured = true, .mmu_active = true, .prusa_slot = 2, .finda = true, .extruder = Sensor::has };
    CHECK(nozzle_slot(v) == uint8_t(2));
    v.extruder = Sensor::none;
    CHECK_FALSE(nozzle_slot(v));
    v.extruder = Sensor::unknown; // tête sans réponse : pas confirmé
    CHECK_FALSE(nozzle_slot(v));

    // La cause : MMU activé mais muet (à flasher, M709 S2), Prusa rend l'outil 0 ; la tête même pleine ne dit pas lequel
    v = { .mmu_configured = true, .mmu_active = false, .prusa_slot = 0, .finda = false, .extruder = Sensor::none };
    CHECK_FALSE(nozzle_slot(v));
    v.extruder = Sensor::has;
    CHECK_FALSE(nozzle_slot(v));

    // Sans MMU : l'outil unique, sauf si la tête voit sûrement qu'il est vide ; capteur coupé : comme Prusa
    v = { .mmu_configured = false, .mmu_active = false, .prusa_slot = 0, .finda = false, .extruder = Sensor::none };
    CHECK_FALSE(nozzle_slot(v));
    v.extruder = Sensor::has;
    CHECK(nozzle_slot(v) == uint8_t(0));
    v.extruder = Sensor::unknown;
    CHECK(nozzle_slot(v) == uint8_t(0));
    v.prusa_slot = no_slot;
    CHECK_FALSE(nozzle_slot(v));
}

TEST_CASE("ni rechargement ni purge : même emplacement, même matière") {
    CHECK(skip_reload(2, uint8_t(2), true));
    CHECK_FALSE(skip_reload(2, uint8_t(2), false)); // autre matière demandée : Prusa recharge
    CHECK_FALSE(skip_reload(2, uint8_t(1), true)); // autre emplacement dans la buse
    CHECK_FALSE(skip_reload(2, std::nullopt, true)); // rien de confirmé
}

TEST_CASE("mémoire du filament : effacée pendant une commande, pas d'écriture pendant l'impression") {
    CHECK(slot_to_keep(true, false, 2) == no_slot);
    CHECK(slot_to_keep(true, true, 2) == no_slot); // 1er changement d'outil de l'impression
    CHECK_FALSE(slot_to_keep(false, true, 2));
    CHECK(slot_to_keep(false, false, 2) == uint8_t(2));
    CHECK(slot_to_keep(false, false, 0xff) == no_slot);
}

TEST_CASE("emplacement sûr : jamais l'ancien d'un changement d'outil interrompu") {
    CHECK(certain_slot(2, no_slot) == 2); // aucun changement commencé (démarrage, reprise)
    CHECK(certain_slot(2, 2) == 2); // dernier changement fini
    CHECK(certain_slot(3, 0) == no_slot); // arrêt pendant le changement 3 → 0 : Prusa garde 3, le MMU charge 0
    CHECK(certain_slot(no_slot, 0) == no_slot);
    CHECK(certain_slot(no_slot, no_slot) == no_slot);
}

TEST_CASE("buse : rien de « chargé » pendant ou après un changement d'outil inachevé") {
    NozzleView v { .mmu_configured = true, .mmu_active = true, .prusa_slot = 3, .changing_slot = 0, .finda = true, .extruder = Sensor::has };
    CHECK_FALSE(nozzle_slot(v)); // capteurs pleins, mais de quel filament ?
    v.changing_slot = 3;
    CHECK(nozzle_slot(v) == uint8_t(3));
    v.changing_slot = no_slot;
    CHECK(nozzle_slot(v) == uint8_t(3));
    // Sans MMU, un ancien changement ne compte pas
    v = { .mmu_configured = false, .mmu_active = false, .prusa_slot = 0, .changing_slot = 3, .finda = false, .extruder = Sensor::has };
    CHECK(nozzle_slot(v) == uint8_t(0));
}

TEST_CASE("mémoire du filament : écrite seulement quand l'emplacement change, matière du chargement gardée") {
    const Record pla2 = pla_in(2);
    CHECK_FALSE(record_update(pla2, 2, "PLA", 1'800'000'000)); // rien de neuf
    CHECK_FALSE(record_update(pla2, 2, "PETG", 1'800'000'000)); // matière changée sur l'emplacement chargé : pas suivie
    CHECK(record_update(pla2, 4, "PETG", 1'800'000'000) == make_record(pla2, 4, "PETG", 1'800'000'000));
    CHECK(record_update(pla2, no_slot, "", 1'800'000'000) == Record {});
    CHECK(record_update(pla2, 0xfe, "", 1'800'000'000) == Record {}); // hors bornes : aucun
    CHECK_FALSE(record_update(Record {}, no_slot, "", 1'800'000'000));
}

TEST_CASE("retour du 25/11 : arrêt pendant le changement PETG → PLA, jamais « PETG chargé » ensuite") {
    // PETG (emplacement 3) gardé dans la buse, enregistré
    Record stored = make_record(Record {}, 3, "PETG", 1'700'000'000);
    // Changement vers le PLA (emplacement 0) : la commande efface l'enregistrement
    stored = *record_update(stored, *slot_to_keep(true, true, certain_slot(3, 0)), "", 1'700'000'100);
    CHECK(stored.slot == no_slot);
    // Arrêt de l'impression au milieu : Prusa garde 3, le MMU finit de charger 0. Au repos, rien n'est réécrit
    const auto keep = slot_to_keep(false, false, certain_slot(3, 0));
    REQUIRE(keep);
    CHECK_FALSE(record_update(stored, *keep, "PETG", 1'700'000'200));
    // Au démarrage suivant (mise à jour, flash du MMU) : rien à restaurer, comportement de Prusa
    BootView v = agreeing(3);
    v.stored = stored;
    CHECK(boot_decision(v) == Boot::forget);
    // Premier T de l'impression suivante vers le PETG : déchargé puis rechargé, jamais repris tel quel
    CHECK(start_load(3, 3, 0, true, true) == StartLoad::unload_first);
}

TEST_CASE("retour du 25/11 : matière de l'emplacement chargé changée, filament oublié au démarrage") {
    Record stored = pla_in(2);
    // L'emplacement 2, toujours chargé du PLA, est renommé PETG : l'enregistrement garde le PLA
    CHECK_FALSE(record_update(stored, *slot_to_keep(false, false, certain_slot(2, 2)), "PETG", 1'800'000'000));
    BootView v = agreeing(2);
    v.stored = stored;
    v.material_matches = make_record(stored, stored.slot, "PETG", stored.time) == stored; // comme step()
    CHECK_FALSE(v.material_matches);
    CHECK(boot_decision(v) == Boot::forget);
}

TEST_CASE("premier T d'une impression : repris seulement s'il est sûr, sinon déchargé") {
    CHECK(start_load(2, 2, no_slot, true, true) == StartLoad::keep); // restauré au démarrage
    CHECK(start_load(2, 2, 2, true, true) == StartLoad::keep); // gardé depuis l'impression d'avant
    CHECK(start_load(2, 1, 1, true, true) == StartLoad::unload_first); // un autre
    CHECK(start_load(2, 2, 2, true, false) == StartLoad::unload_first); // capteur de l'extrudeur vide : pas sûr
    CHECK(start_load(2, no_slot, no_slot, true, false) == StartLoad::unload_first); // inconnu vu par FINDA
    CHECK(start_load(2, no_slot, no_slot, false, true) == StartLoad::unload_first); // inconnu vu par la tête
    CHECK(start_load(2, no_slot, no_slot, false, false) == StartLoad::load); // buse vide : Prusa
    CHECK(start_load(2, no_slot, 2, false, false) == StartLoad::load); // chargement interrompu, rien vu
}

TEST_CASE("mémoire du filament : enregistrement, date gardée tant que rien ne change") {
    const Record r = pla_in(4);
    CHECK(r.slot == 4);
    CHECK(r.material[0] == 'P');
    CHECK(r.material[3] == '\0');
    CHECK(r.time == 1'700'000'000);
    CHECK(make_record(r, 4, "PLA", 1'800'000'000) == r);
    CHECK(make_record(r, 4, "PETG", 1'800'000'000).time == 1'800'000'000);
    CHECK(make_record(r, 1, "PLA", 1'800'000'000).time == 1'800'000'000);
    CHECK(make_record(r, no_slot, "PLA", 1'800'000'000) == Record {});
    // Nom trop long : coupé, toujours terminé
    const Record l = make_record(Record {}, 0, "ABCDEFGHIJKLMNOP", 1);
    CHECK(l.material.back() == '\0');
}

TEST_CASE("bobine qui convient au fichier : comme le contrôle de matière de Prusa") {
    CHECK(material_fits("PETG", "PETG"));
    CHECK_FALSE(material_fits("PETG", "PLA"));
    CHECK_FALSE(material_fits("PETG", "---")); // emplacement sans matière
    CHECK(material_fits("", "PLA")); // matière absente du fichier : pas de contrôle
    CHECK(material_fits("---", "PLA"));
}

TEST_CASE("retour du 25/11 au soir : G-code mono-outil, emplacement 3 voulu, pas l'emplacement 1 de Prusa") {
    // Emplacements 1 à 5 : PLA, PLA, PETG, ASA, PETG ; fichier PETG
    const std::array<const char *, slots> spools { "PLA", "PLA", "PETG", "ASA", "PETG" };
    const auto petg = [&](uint8_t s) { return material_fits("PETG", spools[s]); };
    CHECK(single_tool_choice(uint8_t(2), petg) == 2); // l'emplacement 3 dans la buse : gardé, pas déchargé
    CHECK(single_tool_choice(std::nullopt, petg) == 2); // buse vide : la 1re bobine de PETG, pas le PLA de l'emplacement 1
    CHECK(single_tool_choice(uint8_t(4), petg) == 4); // l'autre PETG dans la buse : gardé aussi
    CHECK(single_tool_choice(uint8_t(0), petg) == 2); // PLA dans la buse : déchargé, le PETG chargé
    CHECK(single_tool_choice(uint8_t(no_slot), petg) == 2);
    // Aucune bobine de la matière : l'emplacement 1, où Prusa et « Avant l'impression » signalent la différence
    CHECK(single_tool_choice(uint8_t(2), [&](uint8_t s) { return material_fits("PC", spools[s]); }) == 0);
    // Matière absente du fichier : le filament dans la buse, sinon l'emplacement 1 comme Prusa
    const auto any = [&](uint8_t s) { return material_fits("", spools[s]); };
    CHECK(single_tool_choice(uint8_t(3), any) == 3);
    CHECK(single_tool_choice(std::nullopt, any) == 0);
    // PLA dans le fichier et sur l'emplacement 1 : rien ne change par rapport à Prusa
    CHECK(single_tool_choice(std::nullopt, [&](uint8_t s) { return material_fits("PLA", spools[s]); }) == 0);
}
