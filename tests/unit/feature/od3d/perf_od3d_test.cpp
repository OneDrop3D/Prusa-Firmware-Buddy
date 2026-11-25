/// OneDrop3D (lot UI-0) : mesure de l'écran (perf_od3d.hpp) : les animations se cumulent, une entrée garde sa ligne
#include <gui/od3d/perf_od3d.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace od3d::perf;

namespace {
Row anim(uint32_t t_ms, uint32_t screen = 0x08123458) {
    return Row { .t_ms = t_ms, .screen = screen, .draw_us = 400, .bytes = 768, .dma_wait_us = 100, .windows = 2, .span_ms = 0, .input_10us = 0, .draws = 1, .cause = Cause::none };
}
} // namespace

TEST_CASE("od3d perf: animation draws of one screen add up, for a minute at most") {
    Row last = anim(1000);
    REQUIRE(merge(last, anim(1100), 1101));
    CHECK(last.t_ms == 1000);
    CHECK(last.draws == 2);
    CHECK(last.draw_us == 800);
    CHECK(last.bytes == 1536);
    CHECK(last.dma_wait_us == 200);
    CHECK(last.windows == 4);
    CHECK(last.span_ms == 101);

    CHECK_FALSE(merge(last, anim(1000 + merge_max_ms), 1001 + merge_max_ms)); // au-delà d'une minute
    CHECK_FALSE(merge(last, anim(1200, 0x08200000), 1201)); // autre écran
    last.draws = UINT8_MAX;
    CHECK_FALSE(merge(last, anim(1200), 1201));
}

TEST_CASE("od3d perf: a draw asked by an input keeps its own row") {
    Row last = anim(1000);
    Row knob = anim(1100);
    knob.cause = Cause::knob;
    knob.input_10us = 350;
    CHECK_FALSE(merge(last, knob, 1105));
    CHECK_FALSE(merge(knob, anim(1150), 1151)); // l'animation qui suit part sur une nouvelle ligne
    CHECK(knob.draws == 1);
}

TEST_CASE("od3d perf: windows saturate instead of wrapping") {
    Row last = anim(1000);
    last.windows = UINT16_MAX - 1;
    REQUIRE(merge(last, anim(1100), 1101));
    CHECK(last.windows == UINT16_MAX);
}

namespace {
Row usb_piece(uint32_t t_ms, Step step, uint32_t us, uint32_t sectors, uint16_t commands, uint32_t split, uint16_t span_ms) {
    return Row { .t_ms = t_ms, .screen = uint32_t(step) | split << 8, .draw_us = us, .bytes = sectors * 512, .dma_wait_us = us / 2, .windows = commands, .span_ms = span_ms, .input_10us = uint16_t(us / 10), .draws = 1, .cause = Cause::usb };
}
} // namespace

TEST_CASE("od3d perf: the slices of one card add up on its row (Imprimer tab)") {
    Row card = usb_piece(5000, Step::card, 40000, 20, 20, 7, 40); // ouverture
    REQUIRE(merge_step(card, usb_piece(5000, Step::card, 3000, 1, 1, 0, 90))); // métadonnées
    REQUIRE(merge_step(card, usb_piece(5000, Step::card, 12000, 9, 9, 8, 300))); // recherche de la vignette
    CHECK(card.draws == 3);
    CHECK(card.draw_us == 55000);
    CHECK(card.bytes == 30 * 512);
    CHECK(card.windows == 30);
    CHECK((card.screen & 0xff) == uint32_t(Step::card));
    CHECK(card.screen >> 8 == 15); // demandes d'un secteur dues à un tampon décalé
    CHECK(card.span_ms == 300); // de l'ouverture à la fin de la dernière tranche
    CHECK(card.input_10us == 4000); // la plus longue tranche : 40 ms sans réponse de l'écran

    CHECK_FALSE(merge_step(card, usb_piece(5100, Step::card, 1000, 1, 1, 0, 1))); // carte suivante
    CHECK_FALSE(merge_step(card, usb_piece(5000, Step::folder, 1000, 1, 1, 0, 1))); // autre étape
    Row draw = anim(5000);
    CHECK_FALSE(merge_step(draw, usb_piece(5000, Step::card, 1000, 1, 1, 0, 1))); // un dessin garde sa ligne
    CHECK_FALSE(merge(card, anim(5050), 5051)); // et une ligne « u » n'est pas une animation
}

TEST_CASE("od3d perf: USB counters subtract") {
    const Usb before { .commands = 10, .sectors = 40, .us = 1000, .split = 3 };
    const Usb after { .commands = 25, .sectors = 72, .us = 9000, .split = 10 };
    const Usb delta = after - before;
    CHECK(delta.commands == 15);
    CHECK(delta.sectors == 32);
    CHECK(delta.us == 8000);
    CHECK(delta.split == 7);
}
