// OneDrop3D (UI-10) : ce que montre la pastille « Dynamic Island » de l'en-tête, et l'hystérésis du préchauffage.
#include <gui/od3d/island_face_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <string>

using namespace od3d::island;

TEST_CASE("island: rien à montrer") {
    CHECK(choose({}).kind == Kind::none);
}

TEST_CASE("island: priorité processus > impression > chargement > attente > préchauffage") {
    Inputs in;
    in.process = Face { .kind = Kind::process, .slot = 2, .process = Process::pa, .stage = Stage::heating };
    in.print_fsm = true;
    in.print_percent = 60;
    in.filament_minimized = true;
    in.filament_percent = 45;
    in.wait_minimized = true;
    in.nozzle_heating = true;

    Face f = choose(in);
    CHECK(f.kind == Kind::process); // sa chauffe n'est pas un préchauffage, son chargement est le sien
    CHECK(f.process == Process::pa);
    CHECK(f.stage == Stage::heating);
    CHECK(f.slot == 2);

    in.process = {};
    CHECK(choose(in).kind == Kind::print);

    in.print_fsm = false;
    f = choose(in);
    CHECK(f.kind == Kind::filament);
    CHECK(f.percent == 45);

    in.filament_minimized = false;
    CHECK(choose(in).kind == Kind::wait);

    in.wait_minimized = false;
    CHECK(choose(in).kind == Kind::preheat);
}

TEST_CASE("island: processus") {
    Inputs in;
    in.process = Face { .kind = Kind::process, .percent = 45, .process = Process::belts, .stage = Stage::percent };
    in.nozzle_heating = true;
    in.other_fsm = true; // son « Patientez » ou son chargement, réduits

    SECTION("hors de son écran") {
        CHECK(choose(in) == in.process);
    }
    SECTION("sur son écran : rien, pas même le préchauffage") {
        in.on_process_screen = true;
        CHECK(choose(in).kind == Kind::none);
    }
    SECTION("pendant une impression (PA automatique) : le processus d'abord") {
        in.print_fsm = true;
        in.print_percent = 3;
        CHECK(choose(in).kind == Kind::process);
        in.on_process_screen = true;
        CHECK(choose(in).kind == Kind::none);
    }
}

TEST_CASE("island: changement d'outil d'une impression, l'impression cachée sous le chargement") {
    Inputs in;
    in.print_fsm = true;
    in.print_hidden_by_fsm = true; // Load_unload au-dessus
    in.filament_minimized = true;
    in.filament_percent = 45;
    CHECK(choose(in).kind == Kind::filament);
    in.filament_minimized = false;
    in.wait_minimized = true;
    CHECK(choose(in).kind == Kind::wait);
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
    SECTION("grise à la fin, jusqu'à « Accueil » : elle ramène à l'écran de fin") {
        in.print_tone = Tone::ended;
        CHECK(choose(in).kind == Kind::print);
        CHECK(choose(in).tone == Tone::ended);
    }
    SECTION("cachée avant le départ (aperçu, questions)") {
        in.print_not_started = true;
        CHECK(choose(in).kind == Kind::none);
    }
    SECTION("cachée sous l'écran d'une autre machine à états") {
        in.print_hidden_by_fsm = true;
        CHECK(choose(in).kind == Kind::none);
    }
    SECTION("pas de préchauffage pendant une impression, même pas commencée") {
        in.print_not_started = true;
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

namespace {
std::string text_of(const Face &f, const char *name = "", const char *word = "") {
    char buf[32];
    format_text(f, buf, sizeof(buf), name, word);
    return buf;
}
} // namespace

TEST_CASE("island: texte de la pastille") {
    CHECK(text_of({}).empty());
    CHECK(text_of({ .kind = Kind::filament, .percent = 45, .slot = 1 }) == "2\xC2\xB7"
                                                                           "45%");
    CHECK(text_of({ .kind = Kind::filament, .percent = 45 }) == "45%");
    CHECK(text_of({ .kind = Kind::preheat, .current = 185, .target = 215 }) == "185/215\xC2\xB0");

    SECTION("impression : avancement et temps restant") {
        CHECK(text_of({ .kind = Kind::print, .percent = 47, .minutes = 83 }) == "47% 1h23");
        CHECK(text_of({ .kind = Kind::print, .percent = 47, .minutes = 25 }) == "47% 25min");
        CHECK(text_of({ .kind = Kind::print, .percent = 3, .minutes = 725 }) == "3% 12h05");
        CHECK(text_of({ .kind = Kind::print, .percent = 3, .minutes = 6100 }) == "3% 101h");
        CHECK(text_of({ .kind = Kind::print, .percent = 47 }) == "47%"); // temps inconnu
        CHECK(text_of({ .kind = Kind::print, .percent = 100, .minutes = 59 }) == "100% 59min");
    }
    SECTION("impression : pause, attente, erreur sans le temps") {
        CHECK(text_of({ .kind = Kind::print, .tone = Tone::paused, .percent = 47, .minutes = 83 }) == "II 47%");
        CHECK(text_of({ .kind = Kind::print, .tone = Tone::waiting, .percent = 47, .minutes = 83 }) == "47%");
        CHECK(text_of({ .kind = Kind::print, .tone = Tone::error, .percent = 47, .minutes = 83 }) == "! 47%");
        CHECK(text_of({ .kind = Kind::print, .tone = Tone::ended, .percent = 100, .minutes = 0 }) == "100%");
    }
    SECTION("12 caractères au plus (la place de la pastille, l'heure sur 12 h)") {
        for (uint16_t m : { 0, 1, 59, 60, 61, 599, 600, 5999, 6000, 65000 }) {
            for (uint8_t p : { 0, 9, 10, 99, 100 }) {
                CHECK(text_of({ .kind = Kind::print, .percent = p, .minutes = m }).size() <= 12);
            }
        }
    }
    SECTION("processus : nom, puis étape") {
        CHECK(text_of({ .kind = Kind::process, .stage = Stage::step, .step = 2, .steps = 5 }, "PA") == "PA 2/5");
        CHECK(text_of({ .kind = Kind::process, .stage = Stage::heating }, "PA", "chauffe") == "PA chauffe");
        CHECK(text_of({ .kind = Kind::process, .stage = Stage::probing }, "PA", "palpage") == "PA palpage");
        CHECK(text_of({ .kind = Kind::process, .percent = 45, .stage = Stage::percent }, "Courroies") == "Courroies 45%");
        CHECK(text_of({ .kind = Kind::process, .stage = Stage::busy }, "Musique") == "Musique");
        CHECK(text_of({ .kind = Kind::process, .stage = Stage::busy }, "PA", "") == "PA");
    }
    SECTION("« Patientez » réduit") {
        CHECK(text_of({ .kind = Kind::wait }, "Patientez") == "Patientez");
    }
}

TEST_CASE("island: l'anneau tourne sans avancement lisible") {
    CHECK(spins(Kind::process));
    CHECK(spins(Kind::filament));
    CHECK(spins(Kind::wait));
    CHECK_FALSE(spins(Kind::print));
    CHECK_FALSE(spins(Kind::preheat));
    CHECK_FALSE(spins(Kind::none));
}

TEST_CASE("island: temps restant et emplacement passés à la pastille") {
    Inputs in;
    in.print_fsm = true;
    in.print_percent = 47;
    in.print_minutes = 83;
    CHECK(choose(in).minutes == 83);
    in.on_print_screen = true; // l'impression ne se montre pas : le chargement réduit, avec son emplacement
    in.filament_minimized = true;
    in.filament_slot = 2;
    CHECK(choose(in).slot == 2);
}

TEST_CASE("island: opération du filament ouverte en arrière-plan") {
    CHECK_FALSE(open_in_background({})); // hors impression : à l'écran
    CHECK(open_in_background({ .chained = true })); // suite d'une opération réduite
    CHECK(open_in_background({ .printing = true })); // changement d'outil pendant qu'on est ailleurs
    CHECK_FALSE(open_in_background({ .printing = true, .on_print_screen = true })); // on regarde l'impression
    CHECK(open_in_background({ .printing = true, .kept_for_print = true, .on_print_screen = true })); // refusé une fois
    CHECK_FALSE(open_in_background({ .kept_for_print = true })); // l'impression est finie
    CHECK(open_in_background({ .process = true })); // chargement d'une mesure du PA : la pastille montre la mesure
}
