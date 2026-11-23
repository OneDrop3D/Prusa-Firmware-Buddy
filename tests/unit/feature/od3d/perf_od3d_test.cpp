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
