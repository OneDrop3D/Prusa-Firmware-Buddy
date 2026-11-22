/// OneDrop3D (interface G) : fond givré des dialogues (frost_od3d.hpp) : flou 3 × 3 et voile, par tranches, sur un
/// afficheur simulé, comparés au calcul direct
#include <gui/od3d/frost_od3d.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstring>
#include <random>
#include <vector>

using namespace od3d::frost;

namespace {

constexpr int screen_w = 480, screen_h = 320;

// Afficheur simulé : 3 octets par pixel, 6 bits en haut de chaque octet comme l'ILI9488 les rend
std::vector<uint8_t> lcd;
std::vector<uint8_t> buffer;
size_t reads = 0;

uint8_t *read(Area a) {
    ++reads;
    REQUIRE(size_t(a.w) * a.h * 3 <= buffer.size());
    REQUIRE(a.x >= 0);
    REQUIRE(a.y >= 0);
    REQUIRE(a.x + a.w <= screen_w);
    REQUIRE(a.y + a.h <= screen_h);
    for (int y = 0; y < a.h; ++y) {
        std::memcpy(&buffer[size_t(y) * a.w * 3], &lcd[(size_t(a.y + y) * screen_w + a.x) * 3], size_t(a.w) * 3);
    }
    return buffer.data();
}

void write(Area a, const uint8_t *buf) {
    for (int y = 0; y < a.h; ++y) {
        std::memcpy(&lcd[(size_t(a.y + y) * screen_w + a.x) * 3], buf + size_t(y) * a.w * 3, size_t(a.w) * 3);
    }
}

void fill_random(unsigned seed) {
    std::mt19937 gen(seed);
    lcd.resize(size_t(screen_w) * screen_h * 3);
    for (auto &b : lcd) {
        b = uint8_t(gen() & 0xFC);
    }
}

/// Calcul direct : chaque pixel de la bande, somme des 9 voisins (bords de la bande répétés), voile
std::vector<uint8_t> expected(const std::vector<uint8_t> &before, const std::array<Area, 4> &bands) {
    std::vector<uint8_t> out = before;
    for (const Area &b : bands) {
        for (int y = b.y; y < b.y + b.h; ++y) {
            for (int x = b.x; x < b.x + b.w; ++x) {
                for (int c = 0; c < 3; ++c) {
                    unsigned sum = 0;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            const int sx = std::clamp(x + dx, int(b.x), b.x + b.w - 1);
                            const int sy = std::clamp(y + dy, int(b.y), b.y + b.h - 1);
                            sum += before[(size_t(sy) * screen_w + sx) * 3 + c] >> 2;
                        }
                    }
                    out[(size_t(y) * screen_w + x) * 3 + c] = shade(sum);
                }
            }
        }
    }
    return out;
}

void run(Area area, Area hole, size_t capacity, unsigned seed) {
    fill_random(seed);
    buffer.assign(capacity * 3, 0);
    const std::vector<uint8_t> before = lcd;
    const auto want = expected(before, margins(area, hole));
    Job job;
    job.start(area, hole);
    int steps = 0;
    while (!job.done()) {
        job.step(capacity, read, write);
        REQUIRE(++steps < 10000);
    }
    CHECK(lcd == want); // la bande givrée, rien d'autre (ni le trou, ni hors de area)
}

} // namespace

TEST_CASE("od3d frost: the plain G background becomes the veil colour") {
    // #1B1C1E (octets 0x1E 0x1C 0x1B) : 6 bits 7, 7, 6, neuf fois → #0C0C0C, la couleur de style::veil sur l'afficheur
    CHECK(shade(9 * (0x1E >> 2)) == 0x0C);
    CHECK(shade(9 * (0x1C >> 2)) == 0x0C);
    CHECK(shade(9 * (0x1B >> 2)) == 0x0C);
    CHECK(shade(0) == 0);
    CHECK(shade(9 * 63) == 0x70); // blanc → 44 %
    for (unsigned s = 0; s < 9 * 63; ++s) {
        CHECK(shade(s + 1) >= shade(s));
        CHECK((shade(s) & 3) == 0);
    }
}

TEST_CASE("od3d frost: margins around the dialog") {
    const Area area { 0, 41, 480, 279 };
    const auto m = margins(area, { 28, 46, 424, 226 }); // écran de chargement
    CHECK((m[0].x == 0 && m[0].y == 41 && m[0].w == 480 && m[0].h == 5));
    CHECK((m[1].x == 0 && m[1].y == 46 && m[1].w == 28 && m[1].h == 226));
    CHECK((m[2].x == 452 && m[2].y == 46 && m[2].w == 28 && m[2].h == 226));
    CHECK((m[3].x == 0 && m[3].y == 272 && m[3].w == 480 && m[3].h == 48));
    const auto sheet = margins(area, { 0, 114, 480, 206 }); // fiche de bobine : seulement le haut
    CHECK((sheet[0].y == 41 && sheet[0].h == 73 && sheet[0].w == 480));
    CHECK((sheet[1].empty() && sheet[2].empty() && sheet[3].empty()));
    const auto none = margins(area, { 0, 0, 10, 10 }); // trou hors de la zone : toute la zone
    CHECK((none[0].y == 41 && none[0].h == 279));
    CHECK(none[3].empty());
}

TEST_CASE("od3d frost: slices give the direct 3x3 blur") {
    const Area area { 0, 41, 480, 279 };
    // Tampon de l'ILI9488 (4 lignes de 480), un tampon de 2 lignes à peine, et un tampon impair
    for (size_t capacity : { size_t(1920), size_t(960), size_t(1500) }) {
        run(area, { 28, 46, 424, 226 }, capacity, 1);
        run(area, { 0, 114, 480, 206 }, capacity, 2);
        run(area, { 0, 120, 480, 200 }, capacity, 3);
    }
    // Bandes d'une ligne, d'une colonne, collées aux bords de l'écran
    run({ 0, 0, 480, 320 }, { 1, 1, 478, 318 }, 1920, 4);
    run({ 10, 300, 7, 20 }, {}, 1920, 5);
}

TEST_CASE("od3d frost: around the load screen, about 175 ms of reading in ~20 slices") {
    fill_random(6);
    buffer.assign(1920 * 3, 0);
    reads = 0;
    Job job;
    job.start({ 0, 41, 480, 279 }, { 28, 46, 424, 226 });
    while (!job.done()) {
        job.step(1920, read, write);
    }
    // 38 096 pixels relus une fois chacun à 5,25 MHz (3 octets) : 174 ms ; chaque tranche ≤ 1 920 pixels (8,8 ms)
    CHECK(reads <= 24);
}

TEST_CASE("od3d frost: a stopped job writes nothing more") {
    fill_random(7);
    buffer.assign(1920 * 3, 0);
    const std::vector<uint8_t> before = lcd;
    Job job;
    job.start({ 0, 41, 480, 279 }, { 28, 46, 424, 226 });
    job.stop();
    CHECK(job.done());
    job.step(1920, read, write);
    CHECK(lcd == before);
}
