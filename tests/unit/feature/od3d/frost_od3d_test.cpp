/// OneDrop3D (interface G) : fond givré des dialogues (frost_od3d.hpp) : nœuds lus sur 2 lignes sur 8, interpolés, sur
/// un afficheur simulé, comparés au calcul direct ; tout est lu avant d'écrire ; durée estimée de chaque dialogue
#include <gui/od3d/frost_od3d.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

using namespace od3d::frost;

namespace {

constexpr int screen_w = 480, screen_h = 320;

// Afficheur simulé : 3 octets par pixel, 6 bits en haut de chaque octet comme l'ILI9488 les rend
std::vector<uint8_t> lcd;
std::vector<uint8_t> buffer;
size_t reads = 0, read_px = 0, writes = 0, written_px = 0;
bool read_after_write = false;

uint8_t *read(Area a) {
    ++reads;
    read_px += size_t(a.w) * a.h;
    read_after_write = read_after_write || writes > 0;
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
    ++writes;
    written_px += size_t(a.w) * a.h;
    REQUIRE(buf == buffer.data());
    REQUIRE(size_t(a.w) * a.h * 3 <= buffer.size());
    for (int y = 0; y < a.h; ++y) {
        std::memcpy(&lcd[(size_t(a.y + y) * screen_w + a.x) * 3], buf + size_t(y) * a.w * 3, size_t(a.w) * 3);
    }
}

void reset_counters(size_t capacity) {
    buffer.assign(capacity * 3, 0);
    reads = read_px = writes = written_px = 0;
    read_after_write = false;
}

void fill_random(unsigned seed) {
    std::mt19937 gen(seed);
    lcd.resize(size_t(screen_w) * screen_h * 3);
    for (auto &b : lcd) {
        b = uint8_t(gen() & 0xFC);
    }
}

// ---- Calcul direct, pixel par pixel ----

int count(int len, int step) {
    return len <= 1 ? 1 : (len - 2) / step + 2;
}
int at(int i, int len, int step) {
    return std::min(i * step, len - 1);
}
int32_t slope(int a, int b, int len) {
    return len ? (b - a) * 65536 / len : 0;
}
/// Pas des nœuds : 8, ou plus large si les nœuds de toutes les bandes ne tiennent pas dans 960
int step_for(const std::array<Area, 4> &bands) {
    for (int step = 8;; step *= 2) {
        size_t n = 0;
        for (const Area &b : bands) {
            n += b.empty() ? 0 : size_t(count(b.w, step)) * count(b.h, step);
        }
        if (n <= 960) {
            return step;
        }
    }
}

/// Nœud (i, j) d'une bande, composante c : moyenne des 2 lignes lues, 8 colonnes autour
unsigned node(const std::vector<uint8_t> &before, const Area &b, int step, int i, int j, int c) {
    const int k = std::min<int>(b.h, 2);
    const int r0 = std::min(at(j, b.h, step), b.h - k);
    const int cx = at(i, b.w, step);
    const int x0 = std::max(cx - step / 2, 0), x1 = std::min(cx + step / 2, int(b.w));
    unsigned sum = 0, n = 0;
    for (int r = r0; r < r0 + k; ++r) {
        for (int x = x0; x < x1; ++x, ++n) {
            sum += before[(size_t(b.y + r) * screen_w + b.x + x) * 3 + c] >> 2;
        }
    }
    return (sum * 4 + n / 2) / n;
}

std::vector<uint8_t> expected(const std::vector<uint8_t> &before, const std::array<Area, 4> &bands) {
    std::vector<uint8_t> out = before;
    const int step = step_for(bands);
    for (const Area &b : bands) {
        if (b.empty()) {
            continue;
        }
        const int nc = count(b.w, step), nr = count(b.h, step);
        for (int y = 0; y < b.h; ++y) {
            int j = 0;
            while (j + 1 < nr && at(j + 1, b.h, step) <= y) {
                ++j;
            }
            const int j1 = std::min(j + 1, nr - 1), y0 = at(j, b.h, step), y1 = at(j1, b.h, step);
            for (int x = 0; x < b.w; ++x) {
                int i = 0;
                while (i + 1 < nc && at(i + 1, b.w, step) <= x) {
                    ++i;
                }
                const int i1 = std::min(i + 1, nc - 1), x0 = at(i, b.w, step), x1 = i1 == i ? b.w : at(i1, b.w, step);
                for (int c = 0; c < 3; ++c) {
                    const auto line = [&](int col) {
                        const int a = node(before, b, step, col, j, c), z = node(before, b, step, col, j1, c);
                        return a + ((slope(a, z, y1 - y0) * (y - y0) + 0x8000) >> 16);
                    };
                    const int a = line(i), z = line(i1);
                    const int32_t v = (int32_t(a) << 16) + 0x8000 + (x - x0) * slope(a, z, x1 - x0);
                    out[(size_t(b.y + y) * screen_w + b.x + x) * 3 + c] = shade(unsigned(v >> 16));
                }
            }
        }
    }
    return out;
}

void check(Area area, Area hole, size_t capacity, unsigned seed) {
    fill_random(seed);
    reset_counters(capacity);
    const std::vector<uint8_t> before = lcd;
    const auto want = expected(before, margins(area, hole));
    run(area, hole, capacity, read, write);
    CHECK(lcd == want); // les bandes givrées, rien d'autre (ni le trou, ni hors de area)
    CHECK_FALSE(read_after_write); // tout est lu avant la première écriture : rien ne progresse à l'œil
}

/// Durée estimée sur l'imprimante : lecture SPI à 5,25 MHz (3 octets : 4,57 µs par pixel) et ~1 ms de pauses par
/// lecture (ili9488_rd : deux osDelay(1)), écriture à 42 MHz (0,57 µs par pixel), calcul compté à part
double estimate_ms() {
    return read_px * 0.00457 + reads * 1.0 + written_px * 0.00057;
}

/// Écran uni : chaque pixel du givrage prend la couleur du voile
void fill(uint8_t r, uint8_t g, uint8_t b) {
    lcd.resize(size_t(screen_w) * screen_h * 3);
    for (size_t i = 0; i < lcd.size(); i += 3) {
        lcd[i] = r;
        lcd[i + 1] = g;
        lcd[i + 2] = b;
    }
}

} // namespace

TEST_CASE("od3d frost: the plain G background becomes the veil colour") {
    // #1B1C1E (octets 0x1E 0x1C 0x1B, 6 bits 7, 7, 6) → #0C0C0C, la couleur de style::veil sur l'afficheur
    CHECK(shade(0x1E & 0xFC) == 0x0C);
    CHECK(shade(0x1C) == 0x0C);
    CHECK(shade(0x1B & 0xFC) == 0x0C);
    CHECK(shade(0) == 0);
    CHECK(shade(252) == 0x70); // blanc → 44 %
    for (unsigned v = 0; v < 252; ++v) {
        CHECK(shade(v + 1) >= shade(v));
        CHECK((shade(v) & 3) == 0);
    }

    fill(0x1C, 0x1C, 0x18);
    reset_counters(1920);
    run({ 0, 41, 480, 279 }, { 0, 114, 480, 206 }, 1920, read, write);
    for (int y = 41; y < 114; ++y) {
        for (int x = 0; x < screen_w; ++x) {
            const uint8_t *p = &lcd[(size_t(y) * screen_w + x) * 3];
            REQUIRE((p[0] == 0x0C && p[1] == 0x0C && p[2] == 0x0C));
        }
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

TEST_CASE("od3d frost: nodes and interpolation give the direct calculation") {
    const Area area { 0, 41, 480, 279 };
    // Tampon de l'ILI9488 (4 lignes de 480), un tampon de 2 lignes à peine, et un tampon impair
    for (size_t capacity : { size_t(1920), size_t(960), size_t(1500) }) {
        check(area, { 28, 46, 424, 226 }, capacity, 1); // écran de chargement
        check(area, { 0, 114, 480, 206 }, capacity, 2); // fiche de bobine
        check(area, { 0, 120, 480, 200 }, capacity, 3); // éditeur de valeur
    }
    // Bandes d'une ligne, d'une colonne, collées aux bords de l'écran ; tout l'écran (nœuds plus espacés)
    check({ 0, 0, 480, 320 }, { 1, 1, 478, 318 }, 1920, 4);
    check({ 10, 300, 7, 20 }, {}, 1920, 5);
    check({ 0, 0, 480, 320 }, {}, 1920, 6);
    check({ 3, 50, 9, 17 }, {}, 1920, 7); // nœuds du bord à moins de 8 px
}

TEST_CASE("od3d frost: a smooth background stays smooth") {
    // Dégradé horizontal : chaque pixel givré reste à un pas de l'afficheur du même pixel simplement assombri
    lcd.resize(size_t(screen_w) * screen_h * 3);
    for (int y = 0; y < screen_h; ++y) {
        for (int x = 0; x < screen_w; ++x) {
            for (int c = 0; c < 3; ++c) {
                lcd[(size_t(y) * screen_w + x) * 3 + c] = uint8_t((x * 252 / (screen_w - 1)) & 0xFC);
            }
        }
    }
    const std::vector<uint8_t> before = lcd;
    reset_counters(1920);
    run({ 0, 41, 480, 279 }, { 0, 114, 480, 206 }, 1920, read, write);
    for (int y = 41; y < 114; ++y) {
        for (int x = 0; x < screen_w; ++x) {
            const size_t i = (size_t(y) * screen_w + x) * 3;
            REQUIRE(std::abs(int(lcd[i]) - int(shade(before[i]))) <= 4);
        }
    }
}

TEST_CASE("od3d frost: each dialog frosts within budget, before it draws") {
    fill_random(8);
    // Fiche de bobine : 480 × 73 au-dessus d'elle
    reset_counters(1920);
    run({ 0, 41, 480, 279 }, { 0, 114, 480, 206 }, 1920, read, write);
    CHECK(reads == 10);
    CHECK(read_px == 9600); // 2 lignes sur 8 (35 040 pixels relus avant)
    CHECK(written_px == 480 * 73);
    CHECK(estimate_ms() < 80);

    // Éditeur de valeur : 480 × 79
    reset_counters(1920);
    run({ 0, 41, 480, 279 }, { 0, 120, 480, 200 }, 1920, read, write);
    CHECK(reads == 11);
    CHECK(estimate_ms() < 85);

    // Écran de chargement : 38 096 pixels autour du cadre (230 ms en tranches avant) ; les bandes de côté (28 px)
    // coûtent surtout leurs pauses
    reset_counters(1920);
    run({ 0, 41, 480, 279 }, { 28, 46, 424, 226 }, 1920, read, write);
    CHECK(reads == 69);
    CHECK(written_px == 38096);
    CHECK(estimate_ms() < 150);
}
