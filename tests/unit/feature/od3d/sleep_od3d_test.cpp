// OneDrop3D (F39) : mode veille. Quand l'imprimante peut s'endormir (jamais occupée : impression, pause, G-code,
// machine à états), ce qui la réveille (toucher, molette, clé USB, travail reçu, erreur, MMU), l'entrée qui réveille
// n'agit pas, les coupures dues une fois par veille.
#include <feature/od3d/sleep_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace od3d::sleep;

namespace {

constexpr uint32_t min_ms = 60'000;

Settings settings_with(uint8_t delay_min) {
    Settings s;
    s.delay_min = delay_min;
    return s;
}

/// Imprimante à l'arrêt depuis t = 0, endormie à delay_min
struct Fixture {
    Idle idle;
    Settings s = settings_with(10);
    Watch w;

    Fixture() {
        idle.step(0, s, w);
    }

    void fall_asleep() {
        idle.step(10 * min_ms, s, w);
        REQUIRE(idle.asleep());
    }
};

} // namespace

TEST_CASE("sleep: falls asleep after the delay without activity") {
    Fixture f;
    CHECK_FALSE(f.idle.step(10 * min_ms - 1, f.s, f.w));
    CHECK_FALSE(f.idle.asleep());
    CHECK(f.idle.step(10 * min_ms, f.s, f.w)); // vient de s'endormir
    CHECK(f.idle.asleep());
    CHECK_FALSE(f.idle.step(11 * min_ms, f.s, f.w)); // pas de nouveau changement
}

TEST_CASE("sleep: disabled never sleeps by itself, sleep now still works") {
    Fixture f;
    f.s.enabled = false;
    f.idle.step(120 * min_ms, f.s, f.w);
    CHECK_FALSE(f.idle.asleep());
    CHECK(f.idle.sleep_now(120 * min_ms));
    CHECK(f.idle.asleep());
}

TEST_CASE("sleep: a corrupted zero delay counts as one minute") {
    Fixture f;
    f.s.delay_min = 0;
    f.idle.step(min_ms - 1, f.s, f.w);
    CHECK_FALSE(f.idle.asleep());
    f.idle.step(min_ms, f.s, f.w);
    CHECK(f.idle.asleep());
}

TEST_CASE("sleep: never while busy, the delay restarts when the work ends") {
    Fixture f;
    f.w.busy = true; // impression, pause, G-code ou machine à états
    f.idle.step(30 * min_ms, f.s, f.w);
    CHECK_FALSE(f.idle.asleep());
    CHECK_FALSE(f.idle.sleep_now(30 * min_ms)); // refusé aussi à la main
    CHECK_FALSE(f.idle.asleep());

    f.w.busy = false; // fin de l'impression à 30 min
    f.idle.step(30 * min_ms + 100, f.s, f.w);
    f.idle.step(39 * min_ms, f.s, f.w);
    CHECK_FALSE(f.idle.asleep());
    f.idle.step(40 * min_ms + 100, f.s, f.w);
    CHECK(f.idle.asleep());
}

TEST_CASE("sleep: printer events wake it up") {
    SECTION("busy (print, pause, G-code, state machine)") {
        Fixture f;
        f.fall_asleep();
        f.w.busy = true;
        CHECK(f.idle.step(11 * min_ms, f.s, f.w));
        CHECK_FALSE(f.idle.asleep());
    }
    SECTION("print state changes (job from PrusaLink or Connect)") {
        Fixture f;
        f.fall_asleep();
        f.w.print_state = 7;
        f.idle.step(11 * min_ms, f.s, f.w);
        CHECK_FALSE(f.idle.asleep());
    }
    SECTION("new state machine (error, warning)") {
        Fixture f;
        f.fall_asleep();
        f.w.fsm = 3;
        f.idle.step(11 * min_ms, f.s, f.w);
        CHECK_FALSE(f.idle.asleep());
    }
    SECTION("USB inserted") {
        Fixture f;
        f.fall_asleep();
        f.w.usb = true;
        f.idle.step(11 * min_ms, f.s, f.w);
        CHECK_FALSE(f.idle.asleep());
    }
    SECTION("MMU event") {
        Fixture f;
        f.fall_asleep();
        f.w.mmu = 0x0101;
        f.idle.step(11 * min_ms, f.s, f.w);
        CHECK_FALSE(f.idle.asleep());
    }
}

TEST_CASE("sleep: USB removal does not wake, a finished print screen does not block") {
    Fixture f;
    f.w.usb = true;
    f.w.fsm = 5; // écran de fin d'impression : l'interface ne le compte pas comme occupé
    f.idle.step(1, f.s, f.w);
    f.idle.step(11 * min_ms, f.s, f.w);
    REQUIRE(f.idle.asleep());
    f.w.usb = false;
    f.idle.step(12 * min_ms, f.s, f.w);
    CHECK(f.idle.asleep());
}

TEST_CASE("sleep: the touch that wakes a dark screen is swallowed, the next one acts") {
    Fixture f;
    f.fall_asleep();
    CHECK(f.idle.input(11 * min_ms, Input::touch, true));
    CHECK_FALSE(f.idle.asleep());
    CHECK(f.idle.input(11 * min_ms + 200, Input::touch, true)); // même geste
    CHECK_FALSE(f.idle.input(11 * min_ms + Idle::grace_ms, Input::touch, false));
}

TEST_CASE("sleep: the knob turn that wakes is swallowed for the rest of the gesture") {
    Fixture f;
    f.fall_asleep();
    const uint32_t t = 11 * min_ms;
    CHECK(f.idle.input(t, Input::encoder, true));
    CHECK(f.idle.input(t + 100, Input::encoder, false));
    CHECK_FALSE(f.idle.input(t + 600, Input::encoder, false));
}

TEST_CASE("sleep: the knob press that wakes is swallowed until its release") {
    Fixture f;
    f.fall_asleep();
    const uint32_t t = 11 * min_ms;
    CHECK(f.idle.input(t, Input::button, true));
    CHECK(f.idle.input(t + 900, Input::button, false)); // maintenu
    CHECK(f.idle.input(t + 2000, Input::button_release, false)); // relâché bien après la grâce : pas de clic
    CHECK_FALSE(f.idle.input(t + 3000, Input::button, false)); // appui suivant : normal
    CHECK_FALSE(f.idle.input(t + 3100, Input::button_release, false));
}

TEST_CASE("sleep: after a touch wake, a knob press acts normally") {
    Fixture f;
    f.fall_asleep();
    const uint32_t t = 11 * min_ms;
    CHECK(f.idle.input(t, Input::touch, true));
    CHECK_FALSE(f.idle.input(t + 100, Input::button, false));
    CHECK_FALSE(f.idle.input(t + 200, Input::button_release, false));
}

TEST_CASE("sleep: with the screen left on, the waking input acts") {
    Fixture f;
    f.fall_asleep();
    CHECK_FALSE(f.idle.input(11 * min_ms, Input::touch, false));
    CHECK_FALSE(f.idle.asleep());
}

TEST_CASE("sleep: inputs are activity") {
    Fixture f;
    CHECK_FALSE(f.idle.input(9 * min_ms, Input::touch, false));
    f.idle.step(10 * min_ms, f.s, f.w);
    CHECK_FALSE(f.idle.asleep());
    f.idle.step(19 * min_ms, f.s, f.w);
    CHECK(f.idle.asleep());
}

TEST_CASE("sleep: motors at once, heaters after their delay, once per sleep") {
    Fixture f;
    f.s.heaters_delay_min = 15;
    CHECK_FALSE(f.idle.take_due(0, f.s).motors); // éveillée : rien
    f.fall_asleep(); // à 10 min

    Due d = f.idle.take_due(10 * min_ms, f.s);
    CHECK(d.motors);
    CHECK_FALSE(d.heaters);
    CHECK_FALSE(f.idle.take_due(10 * min_ms + 1, f.s).motors); // une fois

    CHECK_FALSE(f.idle.take_due(25 * min_ms - 1, f.s).heaters);
    CHECK(f.idle.take_due(25 * min_ms, f.s).heaters);
    CHECK_FALSE(f.idle.take_due(26 * min_ms, f.s).heaters);

    // Réveil puis nouvelle veille : de nouveau dues
    f.idle.input(30 * min_ms, Input::touch, true);
    f.idle.step(40 * min_ms, f.s, f.w);
    REQUIRE(f.idle.asleep());
    CHECK(f.idle.take_due(40 * min_ms, f.s).motors);
}

TEST_CASE("sleep: heaters immediately, options off, woken before the delay") {
    Fixture f;
    f.s.heaters_delay_min = 0;
    f.fall_asleep();
    CHECK(f.idle.take_due(10 * min_ms, f.s).heaters);

    Fixture g;
    g.s.motors = false;
    g.s.heaters = false;
    g.fall_asleep();
    const Due none = g.idle.take_due(100 * min_ms, g.s);
    CHECK_FALSE(none.motors);
    CHECK_FALSE(none.heaters);

    Fixture h; // réveillée avant le délai des chauffes : pas de coupure
    h.fall_asleep();
    h.idle.input(12 * min_ms, Input::touch, true);
    CHECK_FALSE(h.idle.take_due(30 * min_ms, h.s).heaters);
}

TEST_CASE("sleep: tick wrap-around") {
    Idle idle;
    Settings s = settings_with(1);
    Watch w;
    const uint32_t start = UINT32_MAX - 1000;
    idle.step(start, s, w);
    idle.step(start + 30'000, s, w); // passe par 0
    CHECK_FALSE(idle.asleep());
    idle.step(start + min_ms, s, w);
    CHECK(idle.asleep());
}

TEST_CASE("od3d sleep: Z hold current reduced, never zero unless cut") {
    CHECK(reduced_ihold(19, 100) == 19); // comme Prusa
    CHECK(reduced_ihold(19, 50) == 10); // arrondi
    CHECK(reduced_ihold(19, 25) == 5);
    CHECK(reduced_ihold(2, 25) == 1); // au moins 1 pas : 0 serait une coupure
    CHECK(reduced_ihold(19, 0) == 0); // coupé (géré à part : disable_Z)
    CHECK(reduced_ihold(31, 120) == 31);
}

TEST_CASE("od3d sleep: custom delays up to 240 min") {
    Idle idle;
    Settings s = settings_with(240);
    Watch w;
    idle.step(0, s, w);
    idle.step(239 * min_ms, s, w);
    CHECK_FALSE(idle.asleep());
    idle.step(240 * min_ms, s, w);
    CHECK(idle.asleep());
}
