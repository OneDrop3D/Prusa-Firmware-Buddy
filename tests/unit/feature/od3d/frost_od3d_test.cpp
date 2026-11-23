/// OneDrop3D (interface G) : ouverture des fenêtres sur le voile (frost_od3d.hpp) : chaque pixel de la place de la
/// fenêtre et du voile est peint, chaque élément une seule fois et jamais recouvert, le corps une fois hors des coins
/// de chaque image ; état final juste ; octets envoyés à l'écran pour chaque fenêtre
#include <gui/od3d/frost_od3d.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <vector>

using namespace od3d::frost;

namespace {

constexpr int screen_w = 480, screen_h = 320;

enum class Op : uint8_t {
    none,
    part,
    body,
    corner,
    veil,
};

struct Pixel {
    Op last = Op::none;
    uint8_t count = 0;
    uint8_t part = 0; ///< élément du dernier part()
    Rect16 square; ///< carré du dernier corner()
};

/// Afficheur simulé : qui a peint chaque pixel en dernier, et combien de fois
struct Recorder final : Painter {
    std::span<const Rect16> parts;
    std::vector<Pixel> px = std::vector<Pixel>(screen_w * screen_h);
    std::vector<size_t> part_calls;
    size_t pixels = 0;
    int stop_after = -1; ///< stop() vrai à partir de cet appel
    int stops = 0;
    bool part_overwritten = false; ///< un pixel d'élément repeint après lui

    void mark(Rect16 r, Op op, uint8_t part = 0, Rect16 sq = {}) {
        REQUIRE(r.Left() >= 0);
        REQUIRE(r.Top() >= 0);
        REQUIRE(r.Left() + r.Width() <= screen_w);
        REQUIRE(r.Top() + r.Height() <= screen_h);
        for (int y = r.Top(); y < r.Top() + r.Height(); ++y) {
            for (int x = r.Left(); x < r.Left() + r.Width(); ++x) {
                Pixel &p = px[y * screen_w + x];
                part_overwritten = part_overwritten || (p.last == Op::part && op != Op::part); // un élément peint sur un autre : voulu
                p = { op, uint8_t(p.count + 1), part, sq };
                ++pixels;
            }
        }
    }
    void part(size_t i) override {
        part_calls.push_back(i);
        mark(parts[i], Op::part, uint8_t(i));
    }
    void body(Rect16 r) override {
        mark(r, Op::body);
    }
    void corner(Rect16 sq, uint8_t, Rect16 piece) override {
        CHECK((piece.Left() >= sq.Left() && piece.Top() >= sq.Top() && piece.Left() + piece.Width() <= sq.Left() + sq.Width() && piece.Top() + piece.Height() <= sq.Top() + sq.Height()));
        mark(piece, Op::corner, 0, sq);
    }
    void veil(Rect16 r) override {
        mark(r, Op::veil);
    }
    bool stop() override {
        return stop_after >= 0 && stops++ >= stop_after;
    }
    const Pixel &at(int x, int y) const {
        return px[y * screen_w + x];
    }
};

bool contains(const Rect16 &r, int x, int y) {
    return x >= r.Left() && x < r.Left() + r.Width() && y >= r.Top() && y < r.Top() + r.Height();
}

bool inside(const Rect16 &outer, const Rect16 &r) {
    return r.Left() >= outer.Left() && r.Top() >= outer.Top() && r.Left() + r.Width() <= outer.Left() + outer.Width() && r.Top() + r.Height() <= outer.Top() + outer.Height();
}

/// Le coin arrondi bit de rect, s'il l'est
Rect16 final_square(const Rect16 &rect, uint8_t corners, int x, int y) {
    for (const uint8_t bit : { uint8_t(MIC_TOP_LEFT), uint8_t(MIC_TOP_RIGHT), uint8_t(MIC_BOT_LEFT), uint8_t(MIC_BOT_RIGHT) }) {
        const bool left = bit & (MIC_TOP_LEFT | MIC_BOT_LEFT), top = bit & (MIC_TOP_LEFT | MIC_TOP_RIGHT);
        const Rect16 sq(rect.Left() + (left ? 0 : rect.Width() - radius), rect.Top() + (top ? 0 : rect.Height() - radius), radius, radius);
        if ((corners & bit) && contains(sq, x, y)) {
            return sq;
        }
    }
    return {};
}

/// Ouvre rect depuis from et vérifie l'image finale et le nombre de passages ; rend les pixels envoyés
size_t check_open(Rect16 rect, uint8_t corners, Rect16 from, std::span<const Rect16> parts, int stop_after = -1) {
    Recorder rec;
    rec.parts = parts;
    rec.stop_after = stop_after;
    plan(rect, corners, from, parts, rec, true);

    CHECK_FALSE(rec.part_overwritten);
    CHECK(rec.part_calls.size() == parts.size());
    std::vector<int> calls(parts.size());
    for (const size_t i : rec.part_calls) {
        ++calls[i];
    }
    for (const int c : calls) {
        CHECK(c == 1); // chaque élément une seule fois
    }

    size_t twice = 0;
    const Rect16 zone = od3d::style::below_header;
    for (int y = 0; y < screen_h; ++y) {
        for (int x = 0; x < screen_w; ++x) {
            const Pixel &p = rec.at(x, y);
            if (!contains(zone, x, y)) {
                REQUIRE(p.count == 0); // l'en-tête n'est pas touché
                continue;
            }
            REQUIRE(p.count >= 1); // rien ne reste de l'écran d'avant
            int in_part = -1;
            uint8_t parts_here = 0;
            for (size_t i = 0; i < parts.size(); ++i) {
                if (contains(parts[i], x, y)) {
                    in_part = int(i);
                    ++parts_here;
                }
            }
            twice += p.count > std::max<int>(parts_here, 1);
            if (!contains(rect, x, y)) {
                REQUIRE(p.last == Op::veil);
                REQUIRE(p.count == 1);
            } else if (in_part >= 0) {
                REQUIRE(p.last == Op::part);
                REQUIRE(p.part == in_part); // le dernier de ceux qui se chevauchent, comme le cadre les dessine
                REQUIRE(p.count == parts_here);
            } else if (const Rect16 sq = final_square(rect, corners, x, y); !sq.IsEmpty()) {
                REQUIRE(p.last == Op::corner);
                REQUIRE(p.square == sq);
            } else {
                REQUIRE(p.last == Op::body);
            }
        }
    }
    // Peints deux fois : seulement les coins des images intermédiaires (4 coins r × r par image)
    CHECK(twice <= size_t(frames) * 4 * radius * radius);
    return rec.pixels;
}

// Fenêtres de nos écrans (value_editor_od3d.cpp, spool_sheet_od3d.cpp, screen_progress_od3d.cpp)
constexpr Rect16 sheet = { 0, 120, 480, 200 };
constexpr Rect16 editor_parts[] = { { 0, 134, 480, 16 }, { 22, 160, 52, 52 }, { 406, 160, 52, 52 }, { 80, 159, 320, 53 }, { 30, 226, 420, 22 }, { 40, 262, 194, 40 }, { 246, 262, 194, 40 } };
constexpr Rect16 band = { 0, 114, 480, 206 };
constexpr Rect16 sheet_parts[] = { { 89, 114, 391, 56 }, { 14, 172, 292, 46 }, { 310, 172, 156, 46 }, { 14, 222, 148, 42 }, { 166, 222, 148, 42 }, { 318, 222, 148, 42 }, { 14, 128, 71, 32 } };
constexpr Rect16 box = { 28, 46, 424, 226 };
constexpr Rect16 box_parts[] = { { 40, 54, 400, 166 }, { 46, 56, 48, 48 }, { 46, 222, 388, 6 }, { 44, 229, 392, 40 } };

constexpr uint8_t top_corners = MIC_TOP_LEFT | MIC_TOP_RIGHT;

} // namespace

TEST_CASE("od3d frost: the first image starts at the touched element, inside the window") {
    // Une carte des températures (au-dessus de la feuille de l'éditeur, qu'elle touche) : sa largeur, au bord de la feuille
    const Rect16 card { 170, 46, 150, 90 };
    const Rect16 s = start(card, sheet, 2 * radius);
    CHECK(s == Rect16(170, 120, 150, 24));
    // Une bobine de l'accueil, au-dessus de la fiche : sa largeur, en haut de la fiche
    CHECK(start({ 100, 46, 78, 60 }, band, 24) == Rect16(100, 114, 78, 24));
    // Un élément dans la fenêtre : lui-même
    CHECK(start({ 50, 200, 100, 40 }, sheet, 24) == Rect16(50, 200, 100, 40));
    // Un petit élément : agrandi autour de son milieu, sans sortir de la fenêtre
    CHECK(start({ 0, 300, 4, 4 }, sheet, 24) == Rect16(0, 290, 24, 24));
    // Redessin à sa place
    CHECK(start(sheet, sheet, 24) == sheet);
    // Sans élément, ou une liste entière : depuis le centre
    CHECK(start({}, sheet, 24) == Rect16(228, 208, 24, 24));
    CHECK(start({ 8, 46, 470, 200 }, sheet, 24) == Rect16(228, 208, 24, 24));
}

TEST_CASE("od3d frost: each image contains the one before, the last is the window") {
    const Rect16 s = start({ 100, 46, 78, 60 }, band, 24);
    CHECK(step(s, band, 0, frames) == s);
    CHECK(step(s, band, frames, frames) == band);
    CHECK(step(s, band, 0, 0) == band);
    Rect16 was = s;
    for (int k = 1; k <= frames; ++k) {
        const Rect16 now = step(s, band, k, frames);
        CHECK(inside(now, was));
        CHECK(inside(band, now));
        // ralentit : chaque image gagne moins de largeur que la précédente
        if (k > 1) {
            CHECK(now.Width() - was.Width() <= was.Width() - step(s, band, k - 2, frames).Width());
        }
        was = now;
    }
}

TEST_CASE("od3d frost: every window paints each pixel once, the screen around veiled") {
    SECTION("value editor, from a temperature card") {
        const size_t px = check_open(sheet, top_corners, { 170, 46, 150, 90 }, editor_parts);
        // Voile 480 × 79 et feuille 480 × 200, plus les coins des images : 3 octets par pixel à 42 MHz (0,571 µs)
        CHECK(px <= 480 * 279 + frames * 8 * radius * radius);
        INFO("editor: " << px * 3 << " bytes, ~" << px * 0.571 / 1000 << " ms");
        CHECK(px * 0.571 / 1000 < 82);
    }
    SECTION("value editor from a list (centre), stopped by the knob after the first image") {
        check_open(sheet, top_corners, { 8, 46, 470, 200 }, editor_parts, 0);
    }
    SECTION("spool sheet, from a home spool") {
        const size_t px = check_open(band, top_corners, { 100, 46, 78, 60 }, sheet_parts);
        CHECK(px <= 480 * 279 + frames * 8 * radius * radius);
    }
    SECTION("loading box, all corners, from the centre (overlapping body and spool mark)") {
        const size_t px = check_open(box, MIC_ALL_CORNERS, {}, box_parts);
        CHECK(px <= 480 * 279 + frames * 8 * radius * radius);
    }
    SECTION("redrawn in place: a single image, no corner twice") {
        const size_t px = check_open(sheet, top_corners, sheet, editor_parts);
        CHECK(px == 480 * 279);
    }
}

TEST_CASE("od3d frost: the element under the finger is painted first") {
    Recorder rec;
    rec.parts = sheet_parts;
    plan(band, top_corners, { 14, 172, 292, 46 }, sheet_parts, rec, false); // « Charger », touché
    REQUIRE_FALSE(rec.part_calls.empty());
    CHECK(rec.part_calls.front() == 1);
}
