/// @file
/// Givrage (frost_od3d.hpp) : l'afficheur ILI9488
#include "frost_od3d.hpp"
#include "style_od3d.hpp"

#include <display.hpp>
#include <guitypes.hpp>
#include <ili9488.hpp>
#include <screen.hpp>
#include <ScreenHandler.hpp>

namespace od3d::frost {

namespace {

    Area area_of(const Rect16 &r) {
        return Area { r.Left(), r.Top(), r.Width(), r.Height() };
    }

    uint8_t *read(Area a) {
        return ili9488_read_rect(a.x, a.y, a.w, a.h);
    }

    void write(Area a, const uint8_t *) {
        display::draw_from_buffer(point_ui16(a.x, a.y), a.w, a.h); // depuis le tampon de l'afficheur, que read a rendu
    }

    /// Ce que montre l'écran autour du dialogue depuis le dernier givrage
    struct State {
        bool frosted = false;
        bool corners = false; ///< les coins de hole ont été faits par le givrage (draw_body ne les repeint pas)
        Rect16 hole;
        uint8_t r = 0;
        uint8_t mask = 0; ///< MIC_*
        Color body;
    };
    State state;

    /// Carré du coin bit (MIC_*) de rect, de côté r
    Rect16 corner_square(const Rect16 &rect, uint8_t bit, uint8_t r) {
        const bool left = bit & (MIC_TOP_LEFT | MIC_BOT_LEFT), top = bit & (MIC_TOP_LEFT | MIC_TOP_RIGHT);
        const int16_t x = rect.Left() + (left ? 0 : rect.Width() - r), y = rect.Top() + (top ? 0 : rect.Height() - r);
        return Rect16(x, y, r, r);
    }

    /// Le coin bit de rect est un de ceux que le givrage a faits (même place, même rayon, même corps)
    bool frosted_corner(const Rect16 &rect, uint8_t bit, uint8_t r, Color body) {
        if (!state.corners || r != state.r || body != state.body) {
            return false;
        }
        for (const uint8_t b : { uint8_t(MIC_TOP_LEFT), uint8_t(MIC_TOP_RIGHT), uint8_t(MIC_BOT_LEFT), uint8_t(MIC_BOT_RIGHT) }) {
            if ((state.mask & b) && corner_square(state.hole, b, r) == corner_square(rect, bit, r)) {
                return true;
            }
        }
        return false;
    }

} // namespace

// Le voile uni (coins sans givrage, fiche tirée) est la couleur que prend le fond uni givré (composantes lues dans raw :
// une constante ne lit pas r, g, b de son union)
constexpr bool veil_is_frosted_screen() {
    for (const int shift : { 0, 8, 16 }) {
        if (shade((style::screen.raw >> shift) & 0xFC) != ((style::veil.raw >> shift) & 0xFC)) {
            return false;
        }
    }
    return true;
}
static_assert(veil_is_frosted_screen());

void run(const Rect16 &area, const Rect16 &hole, uint8_t r, uint8_t corners, Color body) {
    Corners c { .r = r, .mask = 0, .body = { uint8_t(body.b & 0xFC), uint8_t(body.g & 0xFC), uint8_t(body.r & 0xFC) } }; // ordre de l'afficheur
    c.mask = uint8_t((corners & MIC_TOP_LEFT ? Corners::top_left : 0) | (corners & MIC_TOP_RIGHT ? Corners::top_right : 0)
        | (corners & MIC_BOT_LEFT ? Corners::bottom_left : 0) | (corners & MIC_BOT_RIGHT ? Corners::bottom_right : 0));
    run(area_of(area), area_of(hole), ili9488_buffer_size() / 3, read, write, r ? c : Corners {});
    state = { .frosted = true, .corners = r && corners, .hole = hole, .r = r, .mask = corners, .body = body };
}

bool frosted() {
    return state.frosted;
}

void draw_body(const Rect16 &rect, Color body, uint8_t r, uint8_t corners) {
    // Coins faits par le givrage : le corps sans eux (bande du haut entre les coins, le milieu, bande du bas)
    const auto skip = [&](uint8_t bit) { return (corners & bit) && frosted_corner(rect, bit, r, body); };
    const bool tl = skip(MIC_TOP_LEFT), tr = skip(MIC_TOP_RIGHT), bl = skip(MIC_BOT_LEFT), br = skip(MIC_BOT_RIGHT);
    const uint8_t rest = corners & ~((tl ? MIC_TOP_LEFT : 0) | (tr ? MIC_TOP_RIGHT : 0) | (bl ? MIC_BOT_LEFT : 0) | (br ? MIC_BOT_RIGHT : 0));
    if (!(tl || tr || bl || br) || rest) {
        // Rien de givré (ou pas tous ses coins) : les coins du voile uni, la couleur du fond givré sur un fond uni
        display::draw_rounded_rect(rect, style::veil, body, r, corners);
        return;
    }
    const int16_t top_h = (tl || tr) ? r : 0, bottom_h = (bl || br) ? r : 0;
    const auto band = [&](int16_t y, int16_t h, bool left, bool right) {
        if (h > 0) {
            display::fill_rect(Rect16(rect.Left() + (left ? r : 0), y, rect.Width() - (left ? r : 0) - (right ? r : 0), h), body);
        }
    };
    band(rect.Top(), top_h, tl, tr);
    band(rect.Top() + top_h, rect.Height() - top_h - bottom_h, false, false);
    band(rect.Top() + rect.Height() - bottom_h, bottom_h, bl, br);
}

void forget_corners() {
    state.corners = false;
}

void forget() {
    state = {};
}

void close() {
    forget();
    if (screen_t *screen = Screens::Access()->Get()) {
        screen->Invalidate();
    }
}

} // namespace od3d::frost
