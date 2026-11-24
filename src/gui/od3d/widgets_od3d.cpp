/// @file
#include "widgets_od3d.hpp"
#include "screen_calib_assistant_od3d.hpp" // calibration_follow
#include "screen_files_od3d.hpp"
#include "screen_mmu_od3d.hpp"
#include "screen_printing_od3d.hpp"
#include "screen_temperatures_od3d.hpp"
#include "island_od3d.hpp"
#include "layout_od3d.hpp"
#include "frost_od3d.hpp"
#include "perf_od3d.hpp"
#include "spinner_od3d.hpp"
#include "text_od3d.hpp"
#include "thumbnail_od3d.hpp"

#include <algorithm>
#include <utility>
#include <cmath>
#include <numbers>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <display_helper.h>
#include <feature/od3d/collision_od3d.hpp>
#include <feature/od3d/spool_usage_od3d.hpp>
#include <feature/prusa/MMU2/mmu2_state.h>
#include <filament.hpp>
#include <i18n.h>
#include <i_window_menu_item.hpp>
#include <ili9488.hpp>
#include <img_resources.hpp>
#include <common/marlin_vars.hpp>
#include <netdev.h>
#include <option/has_chamber_api.h>
#include <option/has_esp.h>
#include <screen_filebrowser.hpp>
#include <screen_menu_filament.hpp>
#include <screen_menu_filament_mmu.hpp>
#include <screen_menu_settings.hpp>
#include <sound.hpp>
#include <time_tools.hpp>
#include <timing.h>
#include <tool_index.hpp>
#include <window_msgbox.hpp>

#if HAS_CHAMBER_API()
    #include <feature/chamber/chamber.hpp>
#endif
#if HAS_ESP()
    #include <espif.h>
#endif

namespace od3d {

Slot Slot::read(uint8_t index) {
    static_assert(sizeof(Slot::material) == filament_name_buffer_size);
    const VirtualToolIndex tool = VirtualToolIndex::from_raw(index);
    Slot slot;
    slot.enabled = tool.is_enabled();
    const FilamentType type = FilamentType::for_tool(tool);
    // Sans MMU, l'outil 0 est toujours « choisi » : chargé seulement s'il a une matière
    slot.loaded = VirtualToolIndex::currently_selected_opt() == tool && type;
    if (type && slot.enabled) {
        const FilamentTypeParameters params = type.parameters();
        strlcpy(slot.material.data(), params.name.data(), slot.material.size());
        // Seuls les filaments ad hoc ont une couleur : sinon une couleur par matière
        slot.look.filament = params.color.has_value() ? *params.color : spool::default_color(params.name.data());
        // Remplissage réel (F13, ou étiquette OpenPrintTag) ; bobine pleine dessinée si le poids n'est pas suivi
        if (const auto usage = spool_usage::report(index)) {
            slot.look.fill_percent = usage->fill_percent();
            slot.low = usage->low();
        }
    } else {
        strlcpy(slot.material.data(), "-", slot.material.size()); // « — » n'est pas dans la police
    }
    return slot;
}

// ---- Dessin ----

namespace {

    Rect16 inner(Rect16 outer) {
        return Rect16(outer.Left() + style::ring, outer.Top() + style::ring, outer.Width() - 2 * style::ring, outer.Height() - 2 * style::ring);
    }

    Parts *recording = nullptr; ///< Record en cours

} // namespace

Record::Record(Parts &parts)
    : was(std::exchange(recording, &parts)) {}

Record::~Record() {
    recording = was;
}

bool noted(Rect16 rect) {
    if (!recording) {
        return false;
    }
    if (!rect.IsEmpty()) {
        if (recording->n == recording->rects.size()) {
            recording->full = true;
        } else {
            recording->rects[recording->n++] = rect;
        }
    }
    return true;
}

void draw_margin(Rect16 outer, Color back, std::span<const Rect16> parts) {
    constexpr int16_t m = style::ring;
    for (const Rect16 strip : { Rect16(outer.Left(), outer.Top(), outer.Width(), m), Rect16(outer.Left(), outer.Bottom() + 1 - m, outer.Width(), m),
             Rect16(outer.Left(), outer.Top() + m, m, outer.Height() - 2 * m), Rect16(outer.Right() + 1 - m, outer.Top() + m, m, outer.Height() - 2 * m) }) {
        style::for_each_gap(strip, parts, [&](Rect16 r) { display::fill_rect(r, back); });
    }
}

void draw_rounded_frame(Rect16 rect, Color back, Color border, Color fill, uint8_t radius, uint8_t border_w, std::span<const Rect16> parts) {
    if (noted(rect)) {
        return; // une tuile ou une pastille dans le contenu d'une carte : un morceau du contenu
    }
    if (!border_w && parts.empty()) {
        display::draw_rounded_rect(rect, back, fill, radius, MIC_ALL_CORNERS);
        return;
    }
    const int w = rect.Width(), h = rect.Height(), r = std::min<int>({ radius, w / 2, h / 2 }), bw = std::min<int>(border_w, r);
    const auto corner = [&](int x, int y) { // relatif à rect, dans un carré de coin
        const int cx = x < r ? x : w - 1 - x; // position dans le carré de coin le plus proche
        const int cy = y < r ? y : h - 1 - y;
        // Coin anticrénelé (retour du 22/11 : « Annuler » en escalier) : contour entre deux arcs concentriques,
        // mélangé en quarts au fond à l'intérieur et à back à l'extérieur
        const float d = spool::radius_in_square(cx, cy, 2 * r);
        const Color inside = bw ? spool::mix_quarters(fill, border, spool::beyond_quarters(d, r - bw)) : fill;
        return spool::mix_quarters(inside, back, spool::beyond_quarters(d, r));
    };
    style::for_each_frame_piece(
        rect, r, bw, parts, [&](Rect16 p, bool inside) { display::fill_rect(p, inside ? fill : border); },
        [&](Rect16 p) { draw_pixels(p, [&](int x, int y) { return corner(p.Left() - rect.Left() + x, p.Top() - rect.Top() + y); }); });
}

void draw_frame(Rect16 outer, Color back, Color border, Color fill, uint8_t radius, uint8_t border_w, std::span<const Rect16> parts) {
    draw_margin(outer, back, parts);
    draw_rounded_frame(inner(outer), back, border, fill, radius, border_w, parts);
}

void draw_text(Rect16 rect, const string_view_utf8 &text, Font font, Color fg, Color bg, Align_t align) {
    if (noted(rect)) {
        return;
    }
    render_text_align(rect, text, font, bg, fg, { 0, 0, 0, 0 }, text_flags(align, is_multiline::no, check_overflow::no));
}

void draw_icon(point_i16_t at, const img::Resource &icon, Color bg, ropfn rop) {
    if (noted(Rect16(at, icon.w, icon.h))) {
        return;
    }
    display::draw_img(point_ui16(at.x, at.y), icon, bg, rop);
}

void draw_computed(Rect16 rect, PixelFn pixel, const void *ctx) {
    const Rect16 clip = rect.Intersection(style::screen_rect);
    if (clip.IsEmpty()) {
        return;
    }
    // Lot UI-0 : la bande suivante est calculée pendant que la précédente part par DMA, en une seule fenêtre
    struct Job {
        Rect16 rect;
        PixelFn pixel;
        const void *ctx;
    } const job { rect, pixel, ctx };
    ili9488_draw_bands(clip, [](const void *p, uint8_t *out, Rect16 band) {
        const Job &j = *static_cast<const Job *>(p);
        store_pixels(out, j.rect, band, j.pixel, j.ctx);
    }, &job);
}

void draw_spool(point_i16_t at, const spool::Look &look, uint8_t d, Color back, std::optional<Color> ring) {
    if (noted(ring ? Rect16(at.x - 2, at.y - 2, d + 4, d + 4) : Rect16(at, d, d))) {
        return;
    }
    const spool::Disc disc(look, d);
    if (!ring) {
        draw_pixels(Rect16(at.x, at.y, d, d), [&](int x, int y) { return disc.pixel(x, y, back); });
        return;
    }
    const int outer = d + 4; // anneau de 2 px, même centre que la bobine
    draw_pixels(Rect16(at.x - 2, at.y - 2, outer, outer), [&](int x, int y) {
        const float r = spool::radius_in_square(x, y, outer);
        return disc.at_radius(r, spool::edge_color(r, outer / 2, *ring, back));
    });
}

void fill_background(const window_t &frame, window_t *first, window_t *last) {
    std::array<Rect16, 40> rects;
    size_t n = 0;
    for (window_t *w = first; w; w = w == last ? nullptr : w->GetNext()) {
        if (!w->IsVisible() || w->GetRect().IsEmpty()) {
            continue;
        }
        if (n == rects.size()) {
            chrome::forget_all(); // effacés avant d'être repeints
            display::fill_rect(frame.GetRect(), frame.GetBackColor()); // trop d'éléments : tout le fond, comme Prusa
            return;
        }
        rects[n++] = w->GetRect();
    }
    style::for_each_gap(frame.GetRect(), std::span<const Rect16>(rects.data(), n), [&](Rect16 r) {
        display::fill_rect(r, frame.GetBackColor());
    });
}

bool draw_whole(window_frame_t &frame, window_t *first, window_t *last, void (*on_background)(window_frame_t &)) {
    if (!frame.IsVisible() || !frame.IsInvalid()) {
        return false;
    }
    // Comme window_frame_t::draw() : tout est à peindre, sauf une fenêtre cachée dont la place est couverte. Une fenêtre
    // neuve est déjà toute à peindre : l'invalider encore ferait oublier à l'en-tête et aux onglets qu'ils sont déjà à
    // l'écran (chrome::forget), et ils se repeignaient à chaque page
    for (window_t *w = first; w; w = w->GetNext()) {
        if (w->IsVisible() || !frame.GetFirstEnabledSubWin(w->GetRect())) {
            if (!w->IsInvalid() || w->HasValidBackground()) {
                w->Invalidate();
            }
        } else {
            w->Validate();
        }
    }
    std::array<window_t *, style::max_gap_rects> wins;
    std::array<Rect16, style::max_gap_rects> rects;
    size_t n = 0;
    for (window_t *w = first; w && n < wins.size(); w = w == last ? nullptr : w->GetNext()) {
        if (w->IsVisible() && !w->GetRect().IsEmpty()) {
            wins[n] = w;
            rects[n++] = w->GetRect();
        }
    }
    const auto draw = [&](size_t i) { wins[i]->Draw(); };
    const auto fill = [&](Rect16 r) { display::fill_rect(r, frame.GetBackColor()); };
    const std::span<const Rect16> elements(rects.data(), n);
    // De haut en bas, sans onde depuis l'élément touché : la page arrive d'un coup (retour de l'utilisateur, 24/11)
    (void)chrome::take_origin();
    if (n == wins.size()) {
        fill_background(frame, first, last); // trop d'éléments : comme avant, le fond puis les éléments
    } else {
        style::paint_top_down(frame.GetRect(), elements, draw, fill);
    }
    for (window_t *w = first; w; w = w == last ? nullptr : w->GetNext()) {
        w->Draw(); // le reste (cachés, ou au-delà de 40) ; ceux déjà peints sont valides et passent
    }
    if (on_background) {
        on_background(frame);
    }
    for (window_t *w = last ? last->GetNext() : nullptr; w; w = w->GetNext()) {
        w->Draw(); // dialogues, par-dessus
    }
    return true;
}

// ---- Navigation ----

void open_filament_menu() {
    switch (MMU2::xState(marlin_vars().mmu2_state.get())) {
    case MMU2::xState::Active:
        open_screen<ScreenMenuFilamentMMU>();
        break;
    case MMU2::xState::Stopped:
        open_screen<ScreenMenuFilament>();
        break;
    case MMU2::xState::Connecting:
    case MMU2::xState::Bootloader:
        break; // le MMU démarre : comme l'accueil Prusa, rien tant qu'il ne répond pas
    }
}

void open_files() {
    open_screen<ScreenFiles>(); // sans clé, la grille le dit
}

void open_temperatures() {
    if (!Screens::Access()->IsScreenOpened<ScreenTemperatures>()) {
        open_screen<ScreenTemperatures>();
    }
}

// ---- Changement d'écran ----

namespace chrome {
    namespace {
        std::array<uint64_t, size_t(Slot::count)> on_display {}; ///< clé de ce que montre l'écran, 0 : inconnu
        uint8_t drawn_slots = 0; ///< dessinés par l'écran courant
        bool handed = false;
        const window_t *gap_parent = nullptr;
        Rect16 origin; ///< élément cliqué ou touché au dernier changement d'écran
        uint32_t origin_ms = 0;
        static_assert(size_t(Slot::count) <= 8);
    } // namespace

    uint64_t shown(Slot slot) {
        return on_display[size_t(slot)];
    }

    Rect16 take_origin() {
        const Rect16 r = std::exchange(origin, Rect16());
        return ticks_diff(ticks_ms(), origin_ms) < 1000 ? r : Rect16();
    }

    void gap_screen(const window_t *screen) {
        gap_parent = screen;
    }

    bool kept(Slot slot, uint64_t key, bool first, const window_t *parent) {
        uint64_t &shown = on_display[size_t(slot)];
        if (parent != gap_parent) {
            shown = 0; // un dialogue ou un écran de Prusa peint par-dessus
            return false;
        }
        const bool same = first && shown == key;
        shown = key;
        drawn_slots |= 1 << size_t(slot);
        return same;
    }

    void forget(Slot slot) {
        on_display[size_t(slot)] = 0;
    }

    void forget_all() {
        on_display = {};
    }

    void hand_over(window_frame_t &screen) {
        // Un élément (pas une liste entière) cliqué ou touché : le nouvel écran se peint en onde depuis lui ; sinon (appui
        // long, balayage, fin d'une opération) de haut en bas
        const window_t *focused = window_t::GetFocusedWindow();
        const bool clicked = last_gui_input_event == GUI_event_t::CLICK || last_gui_input_event == GUI_event_t::TOUCH_CLICK;
        origin = clicked && focused && 4 * uint32_t(focused->GetRect().Width()) * focused->GetRect().Height() < uint32_t(style::screen_w) * style::screen_h ? focused->GetRect() : Rect16();
        origin_ms = ticks_ms();
        if (&screen != gap_parent) {
            return;
        }
        // Zone à repeindre en attente (dialogue refermé, élément caché) : ce qu'elle couvre n'est plus sûr
        struct FrameAccess : window_frame_t {
            static Rect16 pending(const window_frame_t &f) {
                return (f.*(&FrameAccess::getInvalidationRect))();
            }
        };
        const bool on_screen = !screen.HasDialog() && !screen.IsInvalid() && FrameAccess::pending(screen).IsEmpty();
        // Ce que l'écran n'a pas dessiné lui-même (onglets d'un écran sans onglets, retour caché) a été recouvert
        for (size_t i = 0; i < on_display.size(); ++i) {
            if (!on_screen || !(drawn_slots & (1 << i))) {
                on_display[i] = 0;
            }
        }
        handed = true;
    }

    void next_screen() {
        if (!handed) {
            forget_all();
        }
        handed = false;
        drawn_slots = 0;
        gap_parent = nullptr;
    }
} // namespace chrome

// ---- Écran ----

Screen::Screen(bool has_back)
    : has_back(has_back) {
    SetBackColor(style::screen);
    chrome::gap_screen(this);
}



screen_init_variant Screen::GetCurrentState() const {
    screen_init_variant state;
    size_t i = 0;
    for (window_t *w = GetFirstEnabledSubWin(); w; w = GetNextEnabledSubWin(w), ++i) {
        if (w->IsFocused()) {
            // Pas un onglet : on quitte l'écran par lui, et au retour son focus restait sur « Réglages » alors qu'on était
            // dans l'Accueil (retour du 22/11) ; l'écran revient alors avec son focus de départ
            if (!style::inside(style::tab_bar_rect, w->GetRect())) {
                state.SetPosition(i);
            }
            break;
        }
    }
    return state;
}

void Screen::InitState(screen_init_variant var) {
    if (!var.GetPosition()) {
        return;
    }
    size_t i = *var.GetPosition();
    window_t *w = GetFirstEnabledSubWin();
    while (w && i--) {
        w = GetNextEnabledSubWin(w);
    }
    if (w) {
        w->SetFocus();
    }
}

void Screen::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP && !HasDialog()) {
        calibration_follow(); // série de l'assistant : l'écran de la mesure en cours, avec ses courbes
        // F13 : une bobine vient de passer sous son seuil d'alerte (une fois par passage), sur nos écrans seulement
        // Sans boîte de message : elle bloquait les dialogues de l'impression (chargement, erreurs du MMU) tant qu'elle
        // restait ouverte (RET-22)
        if (const auto slot = spool_usage::take_low_alert()) {
            const auto usage = spool_usage::report(*slot);
            StringViewUtf8Parameters<16> params;
            show_notice(_("Spool %d: %d g\nrunning low").formatted(params, *slot + 1, usage ? int(usage->remaining_g) : 0));
        }
        // F38 : chocs répétés au même endroit (la pause, si demandée, est déjà faite par le fil de Marlin)
        if (const auto hit = collision::take_alert()) {
            StringViewUtf8Parameters<24> params;
            if (hit->test) {
                MsgBoxWarning(_("Collision test: %d hits at the same spot. Detection works.").formatted(params, int(hit->hits)), Responses_Ok);
            } else {
                const string_view_utf8 text = hit->paused ? _("Collision: %d hits near X %d, Y %d mm. Print paused: check the part, then resume or stop.")
                                                          : _("Collision: %d hits near X %d, Y %d mm. The print goes on.");
                MsgBoxWarning(text.formatted(params, int(hit->hits), int(hit->x_mm), int(hit->y_mm)), Responses_Ok);
            }
        }
    }
    if (event == GUI_event_t::HELD_RELEASED && has_back) {
        if (!GetFirstDialog()) { // sous un éditeur de valeur ou une question, l'écran ne se ferme pas derrière
            close_screen();
        }
        return;
    }
    // Sans capture, molette, clic et toucher passent par la fenêtre qui a le focus : un dialogue ou un élément détruit
    // qui l'avait le laisse à personne, et plus rien ne répondrait. Il revient au premier élément actif.
    if ((event == GUI_event_t::KNOB || event == GUI_event_t::CLICK || event == GUI_event_t::TOUCH_CLICK) && !get_focused_child_window()) {
        if (window_t *first = GetFirstEnabledSubWin()) {
            first->SetFocus();
        }
    }
    screen_t::windowEvent(sender, event, param);
}

void Screen::draw() {
    if (!draw_whole(*this, getFirstNormal(), getLastNormal(), [](window_frame_t &screen) { static_cast<Screen &>(screen).unconditionalDraw(); })) {
        screen_t::draw();
    }
}

void Screen::unconditionalDraw() {
    // Le fond est peint par draw_whole(), avec chaque élément
}

void GapFrame::draw() {
    if (!gaps_only || !draw_whole(*this, getFirstNormal(), getLastNormal())) {
        window_frame_t::draw();
    }
}

void draw_spinner(Rect16 box, uint8_t w, uint8_t step, Color bg, std::optional<uint8_t> from) {
    const uint8_t d = box.Width();
    const auto paint = [&](Rect16 part) { // part : relatif au carré
        if (part.IsEmpty()) {
            return;
        }
        draw_pixels(Rect16(box.Left() + part.Left(), box.Top() + part.Top(), part.Width(), part.Height()), [&](int x, int y) {
            return spinner::pixel(part.Left() + x, part.Top() + y, d, w, step, bg);
        });
    };
    if (!from) {
        paint(Rect16(0, 0, d, d));
        return;
    }
    for (const Rect16 &part : spinner::changed(d, w, *from, step)) {
        paint(part);
    }
}

// ---- Éléments ----

Spinner::Spinner(window_t *parent, point_i16_t at)
    : window_t(parent, Rect16(at.x, at.y, size, size)) {}

void Spinner::unconditionalDraw() {
    // Fond invalidé (ouverture, dialogue refermé) : tout ; sinon, ce qui a bougé depuis l'image d'avant
    const uint8_t step = spinner::step_at(ticks_ms());
    draw_spinner(GetRect(), 4, step, GetParent() ? GetParent()->GetBackColor() : GetBackColor(), HasValidBackground() ? drawn : std::nullopt);
    drawn = step;
}

void Spinner::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::TEXT_ROLL && drawn != spinner::step_at(ticks_ms())) {
        const bool background_valid = HasValidBackground(); // comme Card::invalidate_content
        Invalidate();
        if (background_valid) {
            ValidateBackground();
        }
    }
    window_t::windowEvent(sender, event, param);
}

Card::Card(window_t *parent, Rect16 rect, Action action, Color body_color, Color back, uint8_t radius)
    : window_t(parent, rect)
    , action(action)
    , body_color(body_color)
    , back(back)
    , radius(radius) {
    Enable();
}

Rect16 Card::body() const {
    return inner(GetRect());
}

Color Card::fill() const {
    return IsFocused() ? style::focused(body_color) : body_color;
}

void Card::invalidate_content() {
    // Comme IWindowMenu : le fond reste valide si rien d'autre ne l'a invalidé (focus, dialogue refermé)
    const bool background_valid = HasValidBackground();
    Invalidate();
    if (background_valid) {
        ValidateBackground();
    }
}

void Card::draw_card(Color border, uint8_t border_w) {
    Parts parts;
    {
        Record record(parts);
        draw_content(body());
    }
    draw_frame(GetRect(), back, border, fill(), radius, border_w, parts.span());
    draw_content(body());
}

void Card::unconditionalDraw() {
    if (HasValidBackground()) {
        draw_content(body()); // invalidate_content : le contenu seul
    } else {
        draw_card(fill(), 0); // focus, ou tout à repeindre : le corps autour du contenu, puis le contenu
    }
}

void Card::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::CLICK:
    case GUI_event_t::TOUCH_CLICK:
        // Toujours consommé : window_t le renverrait au parent, qui le renverrait à l'élément ciblé
        if (action && IsEnabled()) {
            action();
        }
        break;
    default:
        window_t::windowEvent(sender, event, param);
        break;
    }
}

IconButton::IconButton(window_t *parent, Rect16 rect, Action action, const img::Resource &icon, const string_view_utf8 &label, Color body_color, uint8_t radius)
    : Card(parent, rect, action, body_color, style::screen, radius)
    , icon(icon)
    , label(label) {}

void IconButton::draw_content(Rect16 body) {
    const int16_t x = body.Left() + (body.Width() - icon.w) / 2;
    if (label.isNULLSTR()) {
        draw_icon({ x, int16_t(body.Top() + (body.Height() - icon.h) / 2) }, icon, fill());
        return;
    }
    draw_icon({ x, int16_t(body.Top() + 12) }, icon, fill());
    // Le fond du libellé ne repeint pas les coins arrondis du bas (« MMU > » de l'accueil : 56 px de haut)
    draw_text(layout::clear_of_corners(body, radius, Rect16(body.Left(), body.Top() + 33, body.Width(), height(Font::small))), label, Font::small, style::text_strong, fill(), Align_t::Center());
}

IconCard::IconCard(window_t *parent, Rect16 body, Action action, const img::Resource &icon, Color tile, const string_view_utf8 &title, const string_view_utf8 &subtitle)
    : Card(parent, style::with_ring(body), action)
    , icon(icon)
    , tile(tile)
    , title(title)
    , subtitle(subtitle) {}

void IconCard::set_subtitle(const string_view_utf8 &set) {
    if (!subtitle.is_same_ref(set)) {
        subtitle = set;
        invalidate_content();
    }
}

void IconCard::draw_content(Rect16 body) {
    // Tuile de 22 px : le titre en gras (8 caractères, « Fichiers », « Contrôle ») garde une marge dans 128 px
    const Rect16 t(body.Left() + 7, body.Top() + (body.Height() - 22) / 2, 22, 22);
    const Rect16 glyph(t.Left() + 3, t.Top() + 3, 16, 16);
    draw_rounded_frame(t, fill(), tile, tile, style::radius_small, 0, std::span<const Rect16>(&glyph, 1)); // la tuile autour de l'icône
    draw_icon({ glyph.Left(), glyph.Top() }, icon, tile);
    const int16_t x = t.Left() + t.Width() + 5;
    const uint16_t w = body.Left() + body.Width() - x;
    // Textes jusqu'au bord droit : leur fond s'arrête avant les coins arrondis
    draw_text(layout::clear_of_corners(body, radius, Rect16(x, body.Top() + 7, w, height(Font::normal))), title, Font::normal, style::text_strong, fill());
    draw_text(layout::clear_of_corners(body, radius, Rect16(x, body.Top() + 27, w, height(Font::small))), subtitle, Font::small, style::text_muted, fill());
}

Temps Temps::read() {
    static Temps held; // ce qui est à l'écran : en-tête de chaque écran, carte de l'accueil, écran Températures
    held.nozzle = style::held_degrees(marlin_vars().active_hotend().temp_nozzle.get(), held.nozzle);
    held.bed = style::held_degrees(marlin_vars().temp_bed.get(), held.bed);
#if HAS_CHAMBER_API()
    const auto chamber = buddy::chamber().current_temperature();
    held.chamber = chamber ? style::held_degrees(static_cast<float>(*chamber), held.chamber) : int16_t(-1);
#endif
    return held;
}

void draw_temps(Rect16 rect, const Temps &temps, Color fg, Color bg, bool centered) {
    if (noted(rect)) {
        return;
    }
    const img::Resource *icons[] = { &img::nozzle_16x16, &img::heatbed_16x16, &img::enclosure_16x16 };
    const int16_t values[] = { temps.nozzle, temps.bed, temps.chamber };
    constexpr int16_t icon_gap = 2, item_gap = 6;
    constexpr uint16_t char_w = width(Font::small);

    char texts[3][8];
    uint16_t total = 2 * item_gap;
    for (int i = 0; i < 3; ++i) {
        // Négatif : pas encore mesuré (Marlin démarre à -1) ou chambre sans capteur
        if (values[i] < 0) {
            strlcpy(texts[i], "--\xC2\xB0", sizeof(texts[i]));
        } else {
            snprintf(texts[i], sizeof(texts[i]), "%d\xC2\xB0", std::min<int>(values[i], 999));
        }
        total += 16 + icon_gap + (strlen(texts[i]) - 1) * char_w; // « ° » : 2 octets, 1 caractère
    }

    // Chaque morceau peint son fond : pas d'effacement global, donc pas de clignotement
    int16_t pen = rect.Left();
    const auto fill_to = [&](int16_t x) {
        if (x > pen) {
            display::fill_rect(Rect16(pen, rect.Top(), x - pen, rect.Height()), bg);
            pen = x;
        }
    };
    fill_to(centered && rect.Width() > total ? pen + (rect.Width() - total) / 2 : pen);
    for (int i = 0; i < 3; ++i) {
        if (i) {
            fill_to(pen + item_gap);
        }
        draw_icon({ pen, rect.Top() }, *icons[i], bg);
        pen += 16;
        fill_to(pen + icon_gap);
        const uint16_t w = (strlen(texts[i]) - 1) * char_w;
        draw_text(Rect16(pen, rect.Top(), w, height(Font::small)), string_view_utf8::MakeRAM(texts[i]), Font::small, fg, bg);
        pen += w;
    }
    fill_to(rect.Left() + rect.Width());
}

TempsButton::TempsButton(window_t *parent, Rect16 body)
    : Card(parent, style::with_ring(body), open_temperatures, style::surface, style::screen, style::radius_small)
    , temps(Temps::read()) {}

void TempsButton::unconditionalDraw() {
    const Rect16 r = GetRect();
    const uint64_t key = uint64_t(uint16_t(temps.nozzle)) | uint64_t(uint16_t(temps.bed)) << 16 | uint64_t(uint16_t(temps.chamber)) << 32
        | uint64_t(IsFocused()) << 48 | uint64_t(r.Left() & 0x1ff) << 49 | uint64_t(r.Top() & 0x3f) << 58; // même hauteur dans les deux en-têtes
    if (!chrome::kept(chrome::Slot::temps, key, std::exchange(first_draw, false), GetParent())) {
        Card::unconditionalDraw();
    }
}

void TempsButton::invalidate(Rect16 rect) {
    chrome::forget(chrome::Slot::temps);
    Card::invalidate(rect);
}

void TempsButton::draw_content(Rect16 body) {
    draw_temps(Rect16(body.Left() + 4, body.Top() + (body.Height() - 16) / 2, body.Width() - 8, 16), temps, style::text, fill(), true);
}

void TempsButton::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP) {
        if (const Temps t = Temps::read(); t != temps) {
            temps = t;
            invalidate_content(); // les chiffres seuls : le bouton n'est pas repeint, il ne clignote plus
        }
    }
    Card::windowEvent(sender, event, param);
}

TempsCard::TempsCard(window_t *parent, Rect16 body)
    : Card(parent, style::with_ring(body), open_temperatures)
    , temps(Temps::read()) {}

void TempsCard::draw_content(Rect16 body) {
    draw_text(Rect16(body.Left() + 10, body.Top() + 7, body.Width() - 20, height(Font::normal)), _("Temperatures"), Font::normal, style::text_strong, fill());
    draw_temps(Rect16(body.Left() + 10, body.Top() + 29, body.Width() - 20, 16), temps, style::text_soft, fill(), false);
}

void TempsCard::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP) {
        if (const Temps t = Temps::read(); t != temps) {
            temps = t;
            invalidate_content();
        }
    }
    Card::windowEvent(sender, event, param);
}

NetworkIcon::NetworkIcon(window_t *parent, point_i16_t at)
    : window_icon_t(parent, Rect16(at.x, at.y, 16, 16), &img::wifi_16x16) {
    SetBackColor(style::screen);
    update();
}

void NetworkIcon::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP) {
        update();
    }
    window_icon_t::windowEvent(sender, event, param);
}

void NetworkIcon::update() {
    // Même règle que l'en-tête Prusa (window_header_t::updateNetwork)
    const uint32_t id = netdev_get_active_id();
    const auto status = netdev_get_status(id);
    const bool up = status == NETDEV_NETIF_UP;
    const img::Resource *icon = &img::lan_16x16;
#if HAS_ESP()
    if (id == NETDEV_ESP_ID) {
        if (const uint32_t now = ticks_ms(); ticks_diff(now, last_signal_ms) > 1000) {
            last_signal_ms = now;
            signal = esp_signal_strength().value_or(0);
        }
        icon = (!up || signal >= -70) ? &img::wifi_16x16 : (signal >= -80 ? &img::wifi_mid_16x16 : &img::wifi_low_16x16);
    }
#endif
    SetRes(icon);
    set_visible(status != NETDEV_UNLINKED);
    set_shadow(!up);
}

Clock::Clock(window_t *parent, Rect16 rect)
    : window_text_t(parent, rect, is_multiline::no) {
    set_font(Font::small);
    SetAlignment(Align_t::RightCenter());
    SetTextColor(style::text_muted);
    SetBackColor(style::screen);
    SetPadding({ 0, 0, 0, 0 });
    time_tools::update_time();
    SetText(string_view_utf8::MakeRAM(time_tools::get_time()));
}

void Clock::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP && time_tools::update_time()) {
        Invalidate(); // même texte (même tampon), autre heure
    }
    window_text_t::windowEvent(sender, event, param);
}

namespace {

    // En-tête de 40 px (style::header), de droite à gauche : heure, réseau, températures, puis le titre prend le reste
    constexpr int16_t header_gap = 6;
    constexpr uint16_t temps_w = 173; ///< 3 × (icône 16 + 2) + « 230° » « 110° » « 45° » + 2 × 6, plus 4 de chaque côté
    /// Une seule hauteur pour le retour, les températures et la pastille, dans les deux en-têtes : la tuile de la ligne
    /// « ‹ Retour » des listes (26 px, menu_od3d.cpp)
    constexpr uint8_t item_h = 26;

    /// Hauteurs : l'en-tête normal (40 px, trait en 40) ou compact (31 px, trait en 31) pour les écrans de Prusa dont
    /// le contenu commence à 32 (GuiDefaults::HeaderHeight), sans retour
    struct Geometry {
        uint16_t h; ///< hauteur sans le trait
    };
    constexpr Geometry normal_geometry { style::header.Height() };
    constexpr Geometry compact_geometry { 31 };
    static_assert(compact_geometry.h + 1 <= GuiDefaults::HeaderHeight, "le contenu des écrans de Prusa commence sous le trait");
    static_assert((compact_geometry.h - item_h) / 2 >= style::ring, "la marge de 2 px des éléments tient dans l'en-tête compact");
    const Geometry &geometry(bool compact) {
        return compact ? compact_geometry : normal_geometry;
    }
    constexpr Rect16 back_body = { style::margin, (style::header.Height() - item_h) / 2, item_h, item_h };
    static_assert(style::inside(style::back_touch, style::with_ring(back_body)));

    uint16_t clock_w() {
        return (time_tools::get_time_format() == time_tools::TimeFormat::_24h ? 5 : 8) * width(Font::small);
    }
    int16_t clock_x() {
        return style::screen_w - style::margin - clock_w();
    }
    int16_t network_x() {
        return clock_x() - header_gap - 16;
    }
    Rect16 temps_body(const Geometry &g = normal_geometry) {
        return Rect16(network_x() - header_gap - temps_w, (g.h - item_h) / 2, temps_w, item_h);
    }
    Rect16 title_rect(bool has_back, const Geometry &g = normal_geometry) {
        // Le titre n'est pas actif : il peut passer sur la zone tactile du retour, 6 px après son disque
        const int16_t x = has_back ? back_body.Right() + 1 + header_gap : style::margin;
        return Rect16(x, 0, temps_body(g).Left() - style::ring - header_gap - x, g.h);
    }

    // Pastille de l'opération en arrière-plan, collée au bouton des températures : anneau, « 47% 1h23 », « PA chauffe ».
    // Elle prend la place du titre, qu'elle cache, jusqu'au disque du retour.
    constexpr uint16_t island_w = 150; ///< 13 caractères ; 12 avec l'heure sur 12 h, plus large
    constexpr uint8_t island_spinner = 16;
    constexpr int16_t island_pad = 7; ///< bord gauche → anneau
    constexpr int16_t island_text_gap = 5; ///< anneau → texte
    constexpr int16_t island_text_end = 4; ///< texte → bord droit
    constexpr int16_t island_min_left = back_body.Right() + 1 + 2 * style::ring + header_gap; ///< après le disque du retour
    constexpr uint32_t island_spin_ms = 1200; ///< un tour
    /// Gris du disque du retour : le noir de la maquette (Apple) ne se voyait pas sur le fond #1B1C1E de l'en-tête
    constexpr Color island_fill = style::secondary;
    Rect16 island_body(const Geometry &g) {
        const int16_t right = temps_body(g).Left() - 2 * style::ring - header_gap;
        const int16_t left = std::max<int16_t>(right - island_w, island_min_left);
        return Rect16(left, (g.h - item_h) / 2, right - left, item_h);
    }
    // Heure sur 12 h (8 caractères) : la place la plus étroite
    static_assert(style::screen_w - style::margin - 8 * width(Font::small) - 2 * header_gap - 16 - temps_w - 2 * style::ring - header_gap - island_min_left - island_pad - island_spinner - island_text_gap - island_text_end >= 12 * width(Font::small), "12 caractères (island::format_text)");
    /// Anneau de 3 px dans un carré de island_spinner px : chaque pixel de l'anneau prend arc(angle depuis le haut, dans
    /// le sens des aiguilles d'une montre, de 0 à 2π) ; hors de l'anneau, bg, mélangé à ses bords (anticrénelage)
    template <class Arc>
    void draw_island_ring(Rect16 rect, Color bg, Arc arc) {
        if (noted(rect)) {
            return;
        }
        constexpr int d = island_spinner;
        draw_pixels(rect, [&](int x, int y) {
            const float r = spool::radius_in_square(x, y, d);
            const uint8_t cover = spool::beyond_quarters(r, d / 2.0f - 3) - spool::beyond_quarters(r, d / 2.0f);
            if (cover == 0) {
                return bg;
            }
            float a = std::atan2(float(2 * x + 1 - d), float(d - 2 * y - 1));
            if (a < 0) {
                a += 2 * std::numbers::pi_v<float>;
            }
            return spool::mix_quarters(bg, arc(a), cover);
        });
    }

    void go_back() {
        Screens::Access()->close_screen();
    }

} // namespace

void menu_back() {
    go_back();
}

BackButton::BackButton(window_t *parent, Action action)
    : IconButton(parent, style::with_ring(back_body), action ? action : go_back, img::arrow_left_16x16, {}, style::tile, style::radius_small) {}

Rect16 BackButton::get_rect_for_touch() const {
    return style::back_touch;
}

void BackButton::unconditionalDraw() {
    if (!chrome::kept(chrome::Slot::back, 1 | uint64_t(IsFocused()) << 1 | uint64_t(GetRect().Top() & 0xff) << 2, std::exchange(first_draw, false), GetParent())) {
        IconButton::unconditionalDraw();
    }
}

void BackButton::invalidate(Rect16 rect) {
    chrome::forget(chrome::Slot::back);
    IconButton::invalidate(rect);
}

void BackButton::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    // Les deux sens, comme les écrans de Prusa (un balayage vers la gauche y est aussi un retour) ; pas pendant le réglage
    // d'une valeur à la molette (ligne de liste en mode édition)
    if (event == GUI_event_t::TOUCH_SWIPE_RIGHT || event == GUI_event_t::TOUCH_SWIPE_LEFT) {
        // Dans un menu, sa ligne « Retour » le prend (WindowMenu) : un seul retour, un seul son
        if (IsVisible() && IsEnabled() && !IWindowMenuItem::edited_item() && action != menu_back) {
            sound::play(SoundType::button_echo);
            action();
        }
        return;
    }
    IconButton::windowEvent(sender, event, param);
}

IslandPill::IslandPill(window_t *parent, Rect16 body, window_t &title)
    : Card(parent, style::with_ring(body), island::activate, island_fill, style::screen, body.Height() / 2)
    , title(title) {
    set_visible(false);
    set_enabled(false);
    drawn = island::face();
    sync(drawn.kind != island::Kind::none);
}

void IslandPill::sync(bool show) {
    if (show == IsVisible()) {
        return;
    }
    set_visible(show);
    set_enabled(show);
    // Le cadre repeint le fond du titre caché ; cachée, sa place revient au titre, dont le rect la couvre
    title.set_visible(!show);
}

void IslandPill::draw_content(Rect16 body) {
    using island::Kind;
    using island::Tone;
    const island::Face &f = drawn;
    const Color bg = fill();
    const Rect16 mark(body.Left() + island_pad, body.Top() + (body.Height() - island_spinner) / 2, island_spinner, island_spinner);
    Color fg = style::text_strong;
    switch (f.kind) {
    case Kind::none:
        return; // cachée
    case Kind::process:
    case Kind::wait:
    case Kind::filament: {
        // Anneau dont le quart orange tourne, puis « PA 2/5 », « 2·45% » (emplacement de l'atelier) ou « 45% »
        const float start = 2 * std::numbers::pi_v<float> * (ticks_ms() % island_spin_ms) / island_spin_ms;
        draw_island_ring(mark, bg, [&](float a) {
            a -= start;
            if (a < 0) {
                a += 2 * std::numbers::pi_v<float>;
            }
            return a < std::numbers::pi_v<float> / 2 ? style::accent : style::line;
        });
        break;
    }
    case Kind::print: {
        // Anneau de progression : orange en cours, jaune en pause, gris en attente ou à la fin, rouge en erreur
        Color arc = style::accent;
        switch (f.tone) {
        case Tone::normal:
            break;
        case Tone::paused:
            arc = fg = style::warning;
            break;
        case Tone::waiting:
        case Tone::ended:
            arc = fg = style::text_muted;
            break;
        case Tone::error:
            arc = style::danger;
            fg = style::text_danger;
            break;
        }
        const float end = 2 * std::numbers::pi_v<float> * f.percent / 100;
        draw_island_ring(mark, bg, [&](float a) { return a < end ? arc : style::line; });
        break;
    }
    case Kind::preheat:
        // La buse qui chauffe : « 185/215° »
        draw_icon({ mark.Left(), mark.Top() }, img::nozzle_16x16, bg);
        break;
    }
    if (HasValidBackground() && f == text_drawn) {
        return; // l'anneau seul a tourné : le texte est déjà à l'écran
    }
    text_drawn = f;
    char text[32];
    island::text(f, text, sizeof(text));
    const int16_t x = mark.Right() + 1 + island_text_gap;
    draw_text(Rect16(x, body.Top() + (body.Height() - height(Font::small)) / 2, body.Left() + body.Width() - island_text_end - x, height(Font::small)), string_view_utf8::MakeRAM(text), Font::small, fg, bg, Align_t::Center());
}

void IslandPill::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP) {
        const island::Face f = island::face();
        sync(f.kind != island::Kind::none);
        if (IsVisible()) {
            const uint32_t now = ticks_ms();
            const bool spin = island::spins(f.kind) && ticks_diff(now, last_spin_ms) >= 100;
            if (f != drawn || spin) {
                drawn = f;
                last_spin_ms = now;
                invalidate_content(); // l'anneau et le texte seulement, sans repeindre la pastille ni l'en-tête
            }
        }
    }
    Card::windowEvent(sender, event, param);
}

// ---- Alerte de l'en-tête ----

namespace {
    constexpr uint32_t notice_ms = 8000;
    std::array<char, 48> notice_text {}; ///< deux lignes séparées par '\n' ; vide : pas d'alerte
    std::optional<uint32_t> notice_since; ///< premier affichage en entier

    /// L'alerte tient-elle en entier dans rect (deux lignes à la police small) ?
    bool notice_fits(Rect16 rect) {
        const char *second = strchr(notice_text.data(), '\n');
        const size_t first_len = second ? size_t(second - notice_text.data()) : strlen(notice_text.data());
        const size_t second_len = second ? strlen(second + 1) : 0;
        // Octets, pas caractères : un accent compte double, l'alerte attend alors un titre plus large (sans danger)
        return rect.Height() >= 2 * height(Font::small) && std::max(first_len, second_len) * width(Font::small) <= size_t(rect.Width());
    }
} // namespace

void show_notice(const string_view_utf8 &text) {
    text.copyToRAM(notice_text.data(), notice_text.size());
    notice_since.reset();
}

void HeaderTitle::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP) {
        if (notice_since && ticks_diff(ticks_ms(), *notice_since) >= int32_t(notice_ms)) {
            notice_text[0] = '\0'; // montrée assez longtemps : une seule fois
            notice_since.reset();
        }
        const bool show = notice_text[0] && IsVisible() && notice_fits(GetRect());
        if (show && !notice_since) {
            notice_since = ticks_ms();
        }
        if (show != showing_notice) {
            showing_notice = show;
            Invalidate();
        }
    }
    window_text_t::windowEvent(sender, event, param);
}

void HeaderTitle::unconditionalDraw() {
    if (!showing_notice) {
        // Police normale (retour du 23/11 : l'en-tête trop petit) ; un titre trop long finit par « ... » au lieu d'être coupé net
        std::array<char, 64> shown;
        GetText().copyToRAM(shown.data(), shown.size());
        text::ellipsize(shown.data(), GetRect().Width() / width(Font::normal));
        draw_text(GetRect(), string_view_utf8::MakeRAM(shown.data()), Font::normal, style::text, GetBackColor());
        return;
    }
    // Deux lignes centrées en hauteur, chaque pixel peint une fois : bande du haut, lignes (fond compris), bande du bas
    const Rect16 r = GetRect();
    const uint16_t h = height(Font::small);
    const int16_t top = r.Top() + (r.Height() - 2 * h) / 2;
    std::array<char, 48> lines = notice_text;
    char *second = strchr(lines.data(), '\n');
    if (second) {
        *second++ = '\0';
    }
    display::fill_rect(Rect16(r.Left(), r.Top(), r.Width(), top - r.Top()), GetBackColor());
    draw_text(Rect16(r.Left(), top, r.Width(), h), string_view_utf8::MakeRAM(lines.data()), Font::small, style::warning, GetBackColor());
    draw_text(Rect16(r.Left(), top + h, r.Width(), h), string_view_utf8::MakeRAM(second ? second : ""), Font::small, style::text_soft, GetBackColor());
    display::fill_rect(Rect16(r.Left(), top + 2 * h, r.Width(), r.Top() + r.Height() - top - 2 * h), GetBackColor());
}

Header::Header(window_t *parent, const string_view_utf8 &title_text, bool has_back, Card::Action back_action, bool compact)
    : back(parent, back_action)
    , title(parent, title_rect(has_back, geometry(compact)), is_multiline::no, is_closed_on_click_t::no, title_text)
    , island(parent, island_body(geometry(compact)), title)
    , temps(parent, temps_body(geometry(compact)))
    , network(parent, { network_x(), int16_t((geometry(compact).h - 16) / 2) })
    , clock(parent, Rect16(clock_x(), 0, clock_w(), geometry(compact).h))
    , line(parent, Rect16(0, geometry(compact).h, style::screen_w, 1)) {
    has_back = has_back && !compact; // le compact n'a pas la place du disque du retour
    back.set_visible(has_back);
    back.set_enabled(has_back);
    title.SetBackColor(style::screen);
    title.SetPadding({ 0, 0, 0, 0 });
    line.SetBackColor(style::line);
}

void Header::show_back(bool show) {
    back.set_visible(show);
    back.set_enabled(show);
}

void Header::set_title(const string_view_utf8 &text) {
    title.SetText(text);
    title.Invalidate();
}

// ---- Barre d'onglets ----

namespace {

    void tab_home() {
        Screens::Access()->CloseAll();
    }
    void tab_print() {
        open_print_tab(); // l'écran Impression pendant une impression, sinon les fichiers
    }
    void tab_workshop() {
        open_section<ScreenMMU>();
    }
    void tab_settings() {
        open_section<ScreenMenuSettings>();
    }

    struct TabDef {
        const img::Resource *icon;
        const char *label;
        Card::Action action;
    };

    // « Imprimer » : « Print » est déjà traduit « Impression » chez Prusa, d'où une autre clé
    const TabDef tab_defs[] = {
        { &img::home_full_16x16, N_("Home"), tab_home },
        { &img::print_16x16, N_("Print files"), tab_print },
        { &img::user_filament_16x16, N_("Workshop"), tab_workshop },
        { &img::settings_16x16, N_("Settings"), tab_settings },
    };

    // Repère de l'onglet actif (SPEC_G.md §1.4) : 44 × 3 px à x + 38 de l'onglet, en haut de la barre
    constexpr uint8_t marker_h = 3;
    constexpr int16_t marker_dx = 38;
    constexpr uint16_t marker_w = 44;
    constexpr Rect16 marker_band = { 0, style::tab_bar_rect.Top(), style::screen_w, marker_h };
    static_assert(style::tab_line.Bottom() + 1 == marker_band.Top());

    // Glissement du repère d'un onglet à l'autre : une image par TEXT_ROLL (40 ms), position selon le temps écoulé
    constexpr int glide_frames = 5;
    constexpr uint32_t glide_frame_ms = 40;
    struct {
        int16_t from = 0;
        uint32_t start_ms = 0;
        uint32_t inputs = 0; ///< entrées comptées au départ (perf::inputs) : une de plus termine le glissement
        bool on = false;
    } glide;
    std::optional<txtroll_t> glide_tick; ///< tant qu'il existe, l'interface envoie TEXT_ROLL toutes les 40 ms

    void stop_glide() {
        glide.on = false;
        glide_tick.reset();
    }

    /// La bande du repère d'une passe, repère à x : la barre autour, le repère arrondi en bas
    void draw_marker_band(int16_t x) {
        const Rect16 mark(x, marker_band.Top(), marker_w, marker_h);
        style::for_each_gap(marker_band, std::span<const Rect16>(&mark, 1), [](Rect16 r) { display::fill_rect(r, style::tab_bar); });
        display::draw_rounded_rect(mark, style::tab_bar, style::accent, 2, MIC_BOT_LEFT | MIC_BOT_RIGHT);
    }

} // namespace

TabBar::Button::Button(window_t *parent, Tab tab, Tab active)
    : Card(parent, Rect16(style::tab_w * int(tab), style::tab_bar_rect.Top(), style::tab_w, style::tab_bar_rect.Height()), tab_defs[int(tab)].action, style::tab_bar, style::tab_bar, 8)
    , icon(*tab_defs[int(tab)].icon)
    , label(_(tab_defs[int(tab)].label))
    , active(tab == active) {}

chrome::Slot TabBar::Button::slot() const {
    return chrome::Slot(size_t(chrome::Slot::tab0) + GetRect().Left() / style::tab_w);
}

void TabBar::Button::invalidate(Rect16 rect) {
    chrome::forget(slot());
    Card::invalidate(rect);
}

void TabBar::Button::unconditionalDraw() {
    if (chrome::kept(slot(), 1 | uint64_t(active) << 1 | uint64_t(IsFocused()) << 2, std::exchange(first_draw, false), GetParent())) {
        return;
    }
    const Rect16 r = GetRect();
    const Rect16 area(r.Left(), r.Top() + marker_h, r.Width(), r.Height() - marker_h); // la bande du repère est au Marker
    const Rect16 glyph(r.Left() + (r.Width() - icon.w) / 2, r.Top() + 6, icon.w, icon.h);
    const Rect16 text(r.Left() + 10, r.Top() + 24, r.Width() - 20, height(Font::small));
    const Rect16 inner[] = { glyph, text };
    // Ciblé à la molette : une pastille plus claire dans la case (la barre n'a pas de marge). D'une passe : la barre
    // autour de la pastille, la pastille autour de l'icône et du libellé (avant : tout l'onglet uni, puis le reste)
    const Color bg = IsFocused() ? style::surface : style::tab_bar;
    const auto fill_bar = [](Rect16 g) { display::fill_rect(g, style::tab_bar); };
    if (IsFocused()) {
        const Rect16 pill(r.Left() + 6, r.Top() + 4, r.Width() - 12, r.Height() - 5);
        style::for_each_gap(area, std::span<const Rect16>(&pill, 1), fill_bar);
        draw_rounded_frame(pill, style::tab_bar, bg, bg, radius, 0, inner);
    } else {
        style::for_each_gap(area, inner, fill_bar);
    }
    draw_icon({ glyph.Left(), glyph.Top() }, icon, bg, active ? ropfn {} : ropfn { .shadow = is_shadowed::yes });
    draw_text(text, label, Font::small, active ? style::accent : style::text_muted, bg, Align_t::Center());
}

TabBar::Marker::Marker(window_t *parent, Tab active)
    : window_t(parent, Rect16(0, style::tab_line.Top(), style::screen_w, style::tab_line.Height() + marker_h))
    , x(int16_t(style::tab_w * int(active) + marker_dx)) {
    if (glide.on) {
        stop_glide(); // écran changé pendant un glissement : le repère à l'écran n'est pas sûr, il se repeint
        chrome::forget(chrome::Slot::marker);
    }
}

void TabBar::Marker::invalidate(Rect16 rect) {
    chrome::forget(chrome::Slot::marker);
    window_t::invalidate(rect);
}

void TabBar::Marker::unconditionalDraw() {
    const uint64_t was = chrome::shown(chrome::Slot::marker); // x + 1 du repère à l'écran, 0 : inconnu
    const bool first = std::exchange(first_draw, false);
    if (chrome::kept(chrome::Slot::marker, uint64_t(x) + 1, first, GetParent())) {
        return; // trait et repère déjà là
    }
    // Le repère saute directement au nouvel onglet (le glissement en 5 images retardait le changement d'onglet, 24/11)
    (void)was;
    int16_t at = x;
    if (glide.on) {
        const int k = perf::inputs() != glide.inputs ? glide_frames : int(ticks_diff(ticks_ms(), glide.start_ms) / glide_frame_ms);
        at = frost::step(Rect16(glide.from, 0, 1, 1), Rect16(x, 0, 1, 1), k, glide_frames).Left();
        if (k >= glide_frames) {
            stop_glide();
        }
    }
    if (!HasValidBackground()) {
        display::fill_rect(style::tab_line, style::line);
    }
    draw_marker_band(at); // 480 × 3 px : 4,3 Ko
}

void TabBar::Marker::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::TEXT_ROLL && glide.on) {
        if (!IsVisible()) {
            stop_glide(); // sous un dialogue : il se repeindra en entier, à sa place
            chrome::forget(chrome::Slot::marker);
        } else {
            const bool background_valid = HasValidBackground(); // comme Card::invalidate_content : la bande seule
            Invalidate();
            if (background_valid) {
                ValidateBackground();
            }
        }
    }
    window_t::windowEvent(sender, event, param);
}

TabBar::TabBar(window_t *parent, Tab active)
    : line(parent, active)
    , buttons {
        { parent, Tab::home, active },
        { parent, Tab::print, active },
        { parent, Tab::workshop, active },
        { parent, Tab::settings, active },
    } {}

} // namespace od3d
