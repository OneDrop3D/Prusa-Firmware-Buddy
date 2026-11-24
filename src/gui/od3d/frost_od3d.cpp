/// @file
/// Voile et ouverture des fenêtres (frost_od3d.hpp) : l'afficheur
#include "frost_od3d.hpp"
#include "spool_draw.hpp"
#include "thumbnail_od3d.hpp"

#include <array>
#include <display.hpp>
#include <Jogwheel.hpp>
#include <screen.hpp>
#include <ScreenHandler.hpp>
#include <timing.h>

namespace od3d::frost {

namespace {

    class Display final : public Painter {
    public:
        Display(PaintPart paint, void *ctx)
            : paint(paint)
            , ctx(ctx) {}

        void part(size_t i) override {
            paint(ctx, i);
        }
        void body(Rect16 r) override {
            display::fill_rect(r, style::surface);
        }
        void corner(Rect16 sq, uint8_t bit, Rect16 piece) override {
            const bool left = bit & (MIC_TOP_LEFT | MIC_BOT_LEFT), top = bit & (MIC_TOP_LEFT | MIC_TOP_RIGHT);
            draw_pixels(piece, [&](int x, int y) {
                const int cx = piece.Left() + x - sq.Left(), cy = piece.Top() + y - sq.Top(); // (0, 0) : coin extérieur
                return spool::mix_quarters(style::surface, style::veil, spool::corner_beyond_quarters(left ? cx : radius - 1 - cx, top ? cy : radius - 1 - cy, radius));
            });
        }
        void veil(Rect16 r) override {
            display::fill_rect(r, style::veil);
        }
        bool stop() override {
            return jogwheel.GetEncoder() != knob;
        }

    private:
        PaintPart paint;
        void *ctx;
        int32_t knob = jogwheel.GetEncoder();
    };

    Rect16 closed_origin;
    uint32_t closed_ms = 0;

    /// Membres protégés d'un écran : ses fenêtres, et ce qu'il a déjà à repeindre (comme chrome::hand_over)
    struct FrameAccess : window_frame_t {
        static window_t *first(const window_frame_t &f) {
            return (f.*(&FrameAccess::getFirstNormal))();
        }
        static Rect16 pending(const window_frame_t &f) {
            return (f.*(&FrameAccess::getInvalidationRect))();
        }
    };

} // namespace

Rect16 origin() {
    const window_t *w = window_t::GetFocusedWindow();
    return w ? w->GetRect() : Rect16();
}

void open(Rect16 rect, uint8_t corners, Rect16 from, std::span<const Rect16> parts, PaintPart paint, void *ctx, bool veil) {
    Display painter(paint, ctx);
    // Ouverture instantanée (retour de l'utilisateur, 24/11 : « des animations de changement de page qui ne sont pas
    // instantanées ») : la fenêtre se peint d'une seule image, à sa place, au lieu de grandir depuis l'élément touché
    (void)from;
    plan(rect, corners, rect, parts, painter, veil);
}

void draw(window_t *first, Rect16 rect, uint8_t corners, Rect16 from, bool veil) {
    struct Parts {
        std::array<window_t *, max_parts> wins;
        std::array<Rect16, max_parts> rects;
        size_t n = 0;
    } parts;
    for (window_t *w = first; w && parts.n < max_parts; w = w->GetNext()) {
        if (w->IsVisible() && !w->GetRect().IsEmpty()) {
            parts.wins[parts.n] = w;
            parts.rects[parts.n++] = w->GetRect();
        }
    }
    const PaintPart paint = [](void *ctx, size_t i) {
        window_t *w = static_cast<Parts *>(ctx)->wins[i];
        w->Invalidate();
        w->Draw();
    };
    open(rect, corners, from, std::span<const Rect16>(parts.rects.data(), parts.n), paint, &parts, veil);
    for (window_t *w = first; w; w = w->GetNext()) {
        w->Draw(); // au-delà de max_parts (jamais avec nos fenêtres) ; les autres sont déjà peints
    }
}

void close() {
    screen_t *screen = Screens::Access()->Get();
    if (!screen) {
        return;
    }
    closed_origin = origin(); // l'élément ciblé de nouveau (IDialog rend son état à l'écran)
    closed_ms = ticks_ms();

    // Comme draw_whole() (widgets_od3d.hpp), sur la place de la fenêtre seulement : les éléments de l'écran qui y sont, de
    // haut en bas, chacun avec la bande du fond où il commence ; pas d'aplat sur toute la place avant eux
    constexpr Rect16 zone = style::below_header;
    std::array<window_t *, style::max_gap_rects> wins;
    std::array<Rect16, style::max_gap_rects> rects;
    size_t n = 0;
    bool whole = screen->IsInvalid() || Screens::Access()->is_change_pending();
    for (window_t *w = FrameAccess::first(*screen); w && !whole; w = w->GetNext()) {
        if (w->IsDialog() || (n == wins.size() && w->GetRect().HasIntersection(zone))) {
            whole = true; // un autre dialogue est encore ouvert, ou trop d'éléments
        } else if (w->IsVisible() && w->GetRect().HasIntersection(zone)) {
            wins[n] = w;
            rects[n++] = w->GetRect();
        }
    }
    if (const Rect16 pending = FrameAccess::pending(*screen); whole || !(pending.IsEmpty() || style::inside(zone, pending))) {
        screen->Invalidate(); // tout l'écran, d'une passe (draw_whole), au prochain dessin
        return;
    }
    const auto draw_one = [&](size_t i) {
        wins[i]->Invalidate();
        wins[i]->Draw();
    };
    style::paint_top_down(zone, std::span<const Rect16>(rects.data(), n), draw_one, [&](Rect16 r) { display::fill_rect(r, screen->GetBackColor()); });
    screen->Validate(zone); // la place de la fenêtre est repeinte ; l'en-tête garde ce qu'il a à repeindre
}

Rect16 take_closed_origin() {
    const Rect16 r = ticks_diff(ticks_ms(), closed_ms) < 3000 ? closed_origin : Rect16();
    closed_origin = Rect16();
    return r;
}

} // namespace od3d::frost
