/// OneDrop3D (interface G) : anneau d'attente (spinner_od3d.hpp) : ce qui est repeint d'une image à l'autre suffit
#include <gui/od3d/spinner_od3d.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace od3d::spinner;

TEST_CASE("od3d spinner: the arc turns once in 1.6 s, 40 positions") {
    CHECK(step_at(0) == 0);
    CHECK(step_at(40) == 1);
    CHECK(step_at(1599) == steps - 1);
    CHECK(step_at(1600) == 0);
}

TEST_CASE("od3d spinner: every pixel that changes is in a repainted rect") {
    constexpr Color bg = od3d::style::surface;
    struct Ring {
        uint8_t d, w;
    };
    for (const Ring ring : { Ring { 16, 3 }, Ring { 40, 4 }, Ring { 44, 4 } }) {
        for (uint8_t from = 0; from < steps; ++from) {
            for (const uint8_t jump : { 1, 2, 5, 11, 39 }) {
                const uint8_t to = (from + jump) % steps;
                const auto rects = changed(ring.d, ring.w, from, to);
                int repainted = 0;
                for (const Rect16 &r : rects) {
                    repainted += r.Width() * r.Height();
                }
                if (jump == 1) {
                    CHECK(repainted < ring.d * ring.d / 2); // une image : bien moins que tout le carré
                }
                for (int y = 0; y < ring.d; ++y) {
                    for (int x = 0; x < ring.d; ++x) {
                        if (pixel(x, y, ring.d, ring.w, from, bg) == pixel(x, y, ring.d, ring.w, to, bg)) {
                            continue;
                        }
                        const point_i16_t p { int16_t(x), int16_t(y) };
                        INFO("d " << int(ring.d) << " from " << int(from) << " to " << int(to) << " pixel " << x << "," << y);
                        CHECK((rects[0].Contain(p) || rects[1].Contain(p)));
                    }
                }
            }
        }
    }
}

TEST_CASE("od3d spinner: a quarter of the ring is orange, the rest is the track") {
    constexpr Color bg = od3d::style::surface;
    // Arc en haut à droite à la position 0 ; piste à gauche ; fond au centre et dans les coins
    CHECK(pixel(30, 4, 40, 4, 0, bg) == od3d::style::accent);
    CHECK(pixel(1, 20, 40, 4, 0, bg) == od3d::style::line);
    CHECK(pixel(20, 20, 40, 4, 0, bg) == bg);
    CHECK(pixel(0, 0, 40, 4, 0, bg) == bg);
}
