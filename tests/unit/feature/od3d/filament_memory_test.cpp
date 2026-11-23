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
