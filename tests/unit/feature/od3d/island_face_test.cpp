// OneDrop3D (UI-10) : ce que montre la pastille « Dynamic Island » de l'en-tête, et l'hystérésis du préchauffage.
#include <gui/od3d/island_face_od3d.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace od3d::island;

TEST_CASE("island: rien à montrer") {
    CHECK(choose({}).kind == Kind::none);
}

TEST_CASE("island: le chargement réduit garde la priorité") {
    Inputs in;
    in.filament_minimized = true;
    in.filament_percent = 45;
    in.print_fsm = true;
    in.print_percent = 60;
    in.nozzle_heating = true;
    const Face f = choose(in);
    CHECK(f.kind == Kind::filament);
    CHECK(f.percent == 45);
}

TEST_CASE("island: impression en cours hors de l'écran Impression") {
    Inputs in;
    in.print_fsm = true;
    in.print_percent = 47;
    in.print_tone = Tone::paused;
    Face f = choose(in);
    CHECK(f.kind == Kind::print);
    CHECK(f.percent == 47);
    CHECK(f.tone == Tone::paused);

    in.print_percent = 140; // borné
    CHECK(choose(in).percent == 100);

    SECTION("cachée sur l'écran Impression") {
        in.on_print_screen = true;
        CHECK(choose(in).kind == Kind::none);
    }
    SECTION("cachée à la fin") {
        in.print_ended = true;
        CHECK(choose(in).kind == Kind::none);
    }
    SECTION("cachée sous l'écran d'une autre machine à états") {
        in.print_hidden_by_fsm = true;
        CHECK(choose(in).kind == Kind::none);
    }
    SECTION("pas de préchauffage pendant une impression, même finie") {
        in.print_ended = true;
        in.nozzle_heating = true;
        CHECK(choose(in).kind == Kind::none);
    }
}

TEST_CASE("island: préchauffage hors impression") {
    Inputs in;
    in.nozzle_heating = true;
    in.nozzle = 185;
    in.nozzle_target = 215;
    Face f = choose(in);
    CHECK(f.kind == Kind::preheat);
    CHECK(f.current == 185);
    CHECK(f.target == 215);

    SECTION("cachée sur l'aperçu des températures") {
        in.on_temperatures_screen = true;
        CHECK(choose(in).kind == Kind::none);
    }
    SECTION("cachée pendant une autre machine à états (chargement, calibration)") {
        in.other_fsm = true;
        CHECK(choose(in).kind == Kind::none);
    }
}

TEST_CASE("island: hystérésis du préchauffage") {
    PreheatLatch latch;
    CHECK_FALSE(latch.update(25, 0));
    CHECK(latch.update(25, 215)); // nouvelle cible loin au-dessus
    CHECK(latch.update(200, 215));
    CHECK(latch.update(212, 215));
    CHECK_FALSE(latch.update(213, 215)); // atteinte (2 °C)
    CHECK_FALSE(latch.update(205, 215)); // refroidit un peu : pas de retour
    CHECK_FALSE(latch.update(216, 215));

    CHECK_FALSE(latch.update(215, 218)); // cible à peine plus haute : pas de pastille
    CHECK(latch.update(215, 250)); // nouvelle cible loin au-dessus
    CHECK_FALSE(latch.update(240, 170)); // cible baissée : rien à chauffer
    CHECK_FALSE(latch.update(240, 0));
    CHECK(latch.update(100, 170)); // à nouveau une cible
}
