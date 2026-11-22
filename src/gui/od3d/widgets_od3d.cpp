/// @file
#include "widgets_od3d.hpp"
#include "print_info_od3d.hpp"
#include "screen_files_od3d.hpp"
#include "screen_mmu_od3d.hpp"
#include "screen_printing_od3d.hpp"
#include "screen_temperatures_od3d.hpp"
#include "island_od3d.hpp"
#include "thumbnail_od3d.hpp"

#include <algorithm>
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
#include <img_resources.hpp>
#include <common/marlin_vars.hpp>
#include <netdev.h>
#include <option/has_chamber_api.h>
#include <option/has_esp.h>
#include <screen_filebrowser.hpp>
#include <screen_menu_filament.hpp>
#include <screen_menu_filament_mmu.hpp>
#include <screen_menu_settings.hpp>
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
    slot.loaded = VirtualToolIndex::currently_selected_opt() == tool;
    const FilamentType type = FilamentType::for_tool(tool);
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

} // namespace

void draw_margin(Rect16 outer, Color back) {
    constexpr int16_t m = style::ring;
    display::fill_rect(Rect16(outer.Left(), outer.Top(), outer.Width(), m), back);
    display::fill_rect(Rect16(outer.Left(), outer.Bottom() + 1 - m, outer.Width(), m), back);
    display::fill_rect(Rect16(outer.Left(), outer.Top() + m, m, outer.Height() - 2 * m), back);
    display::fill_rect(Rect16(outer.Right() + 1 - m, outer.Top() + m, m, outer.Height() - 2 * m), back);
}

void draw_body(Rect16 outer, Color body, Color back, uint8_t radius) {
    draw_margin(outer, back);
    display::draw_rounded_rect(inner(outer), back, body, radius, MIC_ALL_CORNERS);
}

void draw_rounded_frame(Rect16 rect, Color back, Color border, Color fill, uint8_t radius, uint8_t border_w) {
    if (!border_w) {
        display::draw_rounded_rect(rect, back, fill, radius, MIC_ALL_CORNERS);
        return;
    }
    const int w = rect.Width(), h = rect.Height(), bw = border_w;
    int row = -1, out = 0, in = 0; // retraits du bord et du contour pour la ligne row, comme display::draw_rounded_rect
    draw_pixels(rect, [&](int x, int y) {
        if (y != row) {
            row = y;
            out = print_info::corner_inset(y, h, radius);
            in = (y < bw || y >= h - bw) ? w : bw + print_info::corner_inset(y - bw, h - 2 * bw, radius - bw);
        }
        if (x < out || x >= w - out) {
            return back;
        }
        return (x < in || x >= w - in) ? border : fill;
    });
}

void draw_frame(Rect16 outer, Color back, Color border, Color fill, uint8_t radius, uint8_t border_w) {
    draw_margin(outer, back);
    draw_rounded_frame(inner(outer), back, border, fill, radius, border_w);
}

void draw_text(Rect16 rect, const string_view_utf8 &text, Font font, Color fg, Color bg, Align_t align) {
    render_text_align(rect, text, font, bg, fg, { 0, 0, 0, 0 }, text_flags(align, is_multiline::no, check_overflow::no));
}

void draw_icon(point_i16_t at, const img::Resource &icon, Color bg, ropfn rop) {
    display::draw_img(point_ui16(at.x, at.y), icon, bg, rop);
}

void draw_qoi(Rect16 rect, AbstractByteReader &reader) {
    display::draw_img(point_ui16(rect.Left(), rect.Top()), reader);
}

void draw_spool(point_i16_t at, const spool::Look &look, uint8_t d, Color back, std::optional<Color> ring) {
    const spool::Disc disc(look, d);
    if (!ring) {
        draw_pixels(Rect16(at.x, at.y, d, d), [&](int x, int y) { return disc.pixel(x, y).value_or(back); });
        return;
    }
    const int outer = d + 4; // disque de l'anneau, même règle que spool::Disc (demi-pixels)
    draw_pixels(Rect16(at.x - 2, at.y - 2, outer, outer), [&](int x, int y) {
        if (const auto c = disc.pixel(x - 2, y - 2)) {
            return *c;
        }
        const int dx = 2 * x + 1 - outer, dy = 2 * y + 1 - outer;
        return dx * dx + dy * dy < outer * outer ? *ring : back;
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
            display::fill_rect(frame.GetRect(), frame.GetBackColor()); // trop d'éléments : tout le fond, comme Prusa
            return;
        }
        rects[n++] = w->GetRect();
    }
    style::for_each_gap(frame.GetRect(), std::span<const Rect16>(rects.data(), n), [&](Rect16 r) {
        display::fill_rect(r, frame.GetBackColor());
    });
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

// ---- Écran ----

Screen::Screen(bool has_back)
    : has_back(has_back) {
    SetBackColor(style::screen);
}

screen_init_variant Screen::GetCurrentState() const {
    screen_init_variant state;
    size_t i = 0;
    for (window_t *w = GetFirstEnabledSubWin(); w; w = GetNextEnabledSubWin(w), ++i) {
        if (w->IsFocused()) {
            state.SetPosition(i);
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
        // F13 : une bobine vient de passer sous son seuil d'alerte (une fois par passage), sur nos écrans seulement
        if (const auto slot = spool_usage::take_low_alert()) {
            const auto usage = spool_usage::report(*slot);
            StringViewUtf8Parameters<16> params;
            MsgBoxWarning(_("Spool %d is running low: about %d g left.").formatted(params, *slot + 1, usage ? int(usage->remaining_g) : 0), Responses_Ok);
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

void Screen::unconditionalDraw() {
    fill_background(*this, getFirstNormal(), getLastNormal());
}

// ---- Éléments ----

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

void Card::unconditionalDraw() {
    if (!HasValidBackground()) {
        draw_body(GetRect(), fill(), back, radius);
    }
    draw_content(body());
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
    draw_text(Rect16(body.Left(), body.Top() + 33, body.Width(), height(Font::small)), label, Font::small, style::text_strong, fill(), Align_t::Center());
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
    display::draw_rounded_rect(t, fill(), tile, style::radius_small, MIC_ALL_CORNERS);
    draw_icon({ int16_t(t.Left() + 3), int16_t(t.Top() + 3) }, icon, tile);
    const int16_t x = t.Left() + t.Width() + 5;
    const uint16_t w = body.Left() + body.Width() - x;
    draw_text(Rect16(x, body.Top() + 7, w, height(Font::normal)), title, Font::normal, style::text_strong, fill());
    draw_text(Rect16(x, body.Top() + 27, w, height(Font::small)), subtitle, Font::small, style::text_muted, fill());
}

Temps Temps::read() {
    Temps t;
    t.nozzle = static_cast<int16_t>(std::lround(marlin_vars().active_hotend().temp_nozzle.get()));
    t.bed = static_cast<int16_t>(std::lround(marlin_vars().temp_bed.get()));
#if HAS_CHAMBER_API()
    if (const auto chamber = buddy::chamber().current_temperature()) {
        t.chamber = static_cast<int16_t>(std::lround(*chamber));
    }
#endif
    return t;
}

void draw_temps(Rect16 rect, const Temps &temps, Color fg, Color bg, bool centered) {
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
    constexpr uint8_t back_d = 32; ///< disque du retour (22 dans la maquette)

    /// Hauteurs : l'en-tête normal (40 px, trait en 40) ou compact (31 px, trait en 31) pour les écrans de Prusa dont
    /// le contenu commence à 32 (GuiDefaults::HeaderHeight), sans retour
    struct Geometry {
        uint16_t h; ///< hauteur sans le trait
        uint16_t temps_h; ///< bouton des températures
        uint16_t island_h; ///< pastille de l'opération en arrière-plan
    };
    constexpr Geometry normal_geometry { style::header.Height(), 30, 28 };
    constexpr Geometry compact_geometry { 31, 26, 24 };
    static_assert(compact_geometry.h + 1 <= GuiDefaults::HeaderHeight, "le contenu des écrans de Prusa commence sous le trait");
    static_assert((compact_geometry.h - compact_geometry.temps_h) / 2 >= style::ring && (compact_geometry.h - compact_geometry.island_h) / 2 >= style::ring);
    const Geometry &geometry(bool compact) {
        return compact ? compact_geometry : normal_geometry;
    }
    constexpr Rect16 back_body = { 6, (style::header.Height() - back_d) / 2, back_d, back_d };
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
        return Rect16(network_x() - header_gap - temps_w, (g.h - g.temps_h) / 2, temps_w, g.temps_h);
    }
    Rect16 title_rect(bool has_back, const Geometry &g = normal_geometry) {
        // Le titre n'est pas actif : il peut passer sur la zone tactile du retour, 6 px après son disque
        const int16_t x = has_back ? back_body.Right() + 1 + header_gap : style::margin;
        return Rect16(x, 0, temps_body(g).Left() - style::ring - header_gap - x, g.h);
    }

    // Pastille de l'opération en arrière-plan, collée au bouton des températures : anneau, « 2 · 100 % »
    constexpr uint16_t island_w = 116;
    constexpr uint8_t island_spinner = 16;
    constexpr uint32_t island_spin_ms = 1200; ///< un tour
    constexpr Color island_black = COLOR_BLACK;
    Rect16 island_body(const Geometry &g) {
        return Rect16(temps_body(g).Left() - 2 * style::ring - header_gap - island_w, (g.h - g.island_h) / 2, island_w, g.island_h);
    }
    /// Ce que montre la pastille, résumé en 32 bits : elle se redessine quand il change
    uint32_t face_key(const island::Face &f) {
        if (f.kind == island::Kind::filament) {
            return uint32_t(f.kind) | uint32_t(island::revision()) << 8; // l'anneau qui tourne la redessine de toute façon
        }
        return uint32_t(f.kind) | uint32_t(f.tone) << 2 | uint32_t(f.percent) << 4 | (uint32_t(uint16_t(f.current)) & 0x3ff) << 11 | (uint32_t(uint16_t(f.target)) & 0x3ff) << 21;
    }

    /// Anneau de 3 px dans un carré de island_spinner px : chaque pixel de l'anneau prend arc(angle depuis le haut, dans
    /// le sens des aiguilles d'une montre, de 0 à 2π) ; hors de l'anneau, bg
    template <class Arc>
    void draw_island_ring(Rect16 rect, Color bg, Arc arc) {
        constexpr int d = island_spinner, outer = d * d, inner = (d - 6) * (d - 6);
        draw_pixels(rect, [&](int x, int y) {
            const int dx = 2 * x + 1 - d, dy = 2 * y + 1 - d, q = dx * dx + dy * dy;
            if (q >= outer || q < inner) {
                return bg;
            }
            float a = std::atan2(float(dx), float(-dy));
            if (a < 0) {
                a += 2 * std::numbers::pi_v<float>;
            }
            return arc(a);
        });
    }

    Rect16 title_short_rect(bool has_back, const Geometry &g) {
        const Rect16 full = title_rect(has_back, g);
        return Rect16(full.Left(), full.Top(), std::max<int>(island_body(g).Left() - style::ring - header_gap - full.Left(), 0), full.Height());
    }

    void go_back() {
        Screens::Access()->close_screen();
    }

} // namespace

BackButton::BackButton(window_t *parent, Action action)
    : IconButton(parent, style::with_ring(back_body), action ? action : go_back, img::arrow_left_16x16, {}, style::secondary, back_d / 2) {}

Rect16 BackButton::get_rect_for_touch() const {
    return style::back_touch;
}

IslandPill::IslandPill(window_t *parent, Rect16 body, window_text_t &title, Rect16 title_full, Rect16 title_short)
    : Card(parent, style::with_ring(body), island::activate, island_black, style::screen, body.Height() / 2)
    , title(title)
    , title_full(title_full)
    , title_short(title_short) {
    set_visible(false);
    set_enabled(false);
    const island::Face f = island::face();
    drawn_key = face_key(f);
    sync(f.kind != island::Kind::none);
}

void IslandPill::sync(bool show) {
    if (show == IsVisible()) {
        return;
    }
    set_visible(show);
    set_enabled(show);
    title.SetRect(show ? title_short : title_full);
    title.Invalidate();
    // Cachée, sa place revient au titre, dont le rect complet la couvre (il repeint tout son fond)
}

void IslandPill::draw_content(Rect16 body) {
    using island::Kind;
    using island::Tone;
    const island::Face f = island::face();
    const Color bg = fill();
    const Rect16 mark(body.Left() + 8, body.Top() + (body.Height() - island_spinner) / 2, island_spinner, island_spinner);
    char text[16] = "";
    Color fg = style::text_strong;
    switch (f.kind) {
    case Kind::none:
        return; // cachée
    case Kind::filament: {
        // Anneau dont le quart orange tourne, puis « 2 · 45 % » (emplacement de l'atelier) ou « 45 % »
        const float start = 2 * std::numbers::pi_v<float> * (ticks_ms() % island_spin_ms) / island_spin_ms;
        draw_island_ring(mark, bg, [&](float a) {
            a -= start;
            if (a < 0) {
                a += 2 * std::numbers::pi_v<float>;
            }
            return a < std::numbers::pi_v<float> / 2 ? style::accent : style::line;
        });
        if (const auto slot = island::slot()) {
            snprintf(text, sizeof(text), "%d\xC2\xB7%d %%", *slot + 1, f.percent);
        } else {
            snprintf(text, sizeof(text), "%d %%", f.percent);
        }
        break;
    }
    case Kind::print: {
        // Anneau de progression : orange en cours, jaune en pause, gris en attente, rouge en erreur
        Color arc = style::accent;
        const char *prefix = "";
        switch (f.tone) {
        case Tone::normal:
            break;
        case Tone::paused:
            arc = fg = style::warning;
            prefix = "II ";
            break;
        case Tone::waiting:
            arc = fg = style::text_muted;
            break;
        case Tone::error:
            arc = style::danger;
            fg = Color::from_raw(0xFF8A73); // texte danger (SPEC_G.md §1.1)
            prefix = "! ";
            break;
        }
        const float end = 2 * std::numbers::pi_v<float> * f.percent / 100;
        draw_island_ring(mark, bg, [&](float a) { return a < end ? arc : style::line; });
        snprintf(text, sizeof(text), "%s%d %%", prefix, f.percent);
        break;
    }
    case Kind::preheat:
        // La buse qui chauffe : « 185/215° »
        draw_icon({ mark.Left(), mark.Top() }, img::nozzle_16x16, bg);
        snprintf(text, sizeof(text), "%d/%d\xC2\xB0", f.current, f.target);
        break;
    }
    const int16_t x = mark.Right() + 1 + 6;
    draw_text(Rect16(x, body.Top() + (body.Height() - height(Font::small)) / 2, body.Left() + body.Width() - 8 - x, height(Font::small)), string_view_utf8::MakeRAM(text), Font::small, fg, bg, Align_t::Center());
}

void IslandPill::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP) {
        const island::Face f = island::face();
        sync(f.kind != island::Kind::none);
        if (IsVisible()) {
            const uint32_t now = ticks_ms();
            const uint32_t key = face_key(f);
            const bool spin = f.kind == island::Kind::filament && ticks_diff(now, last_spin_ms) >= 100;
            if (key != drawn_key || spin) {
                drawn_key = key;
                last_spin_ms = now;
                invalidate_content(); // l'anneau et le texte seulement, sans repeindre la pastille ni l'en-tête
            }
        }
    }
    Card::windowEvent(sender, event, param);
}

Header::Header(window_t *parent, const string_view_utf8 &title_text, bool has_back, Card::Action back_action, bool compact)
    : back(parent, back_action)
    , title(parent, title_rect(has_back, geometry(compact)), is_multiline::no, is_closed_on_click_t::no, title_text)
    , island(parent, island_body(geometry(compact)), title, title_rect(has_back, geometry(compact)), title_short_rect(has_back, geometry(compact)))
    , temps(parent, temps_body(geometry(compact)))
    , network(parent, { network_x(), int16_t((geometry(compact).h - 16) / 2) })
    , clock(parent, Rect16(clock_x(), 0, clock_w(), geometry(compact).h))
    , line(parent, Rect16(0, geometry(compact).h, style::screen_w, 1)) {
    has_back = has_back && !compact; // le compact n'a pas la place du disque du retour
    back.set_visible(has_back);
    back.set_enabled(has_back);
    title.set_font(Font::small);
    title.SetAlignment(Align_t::LeftCenter());
    title.SetTextColor(style::text);
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

uint8_t Header::title_chars() {
    return title_rect(true).Width() / width(Font::small);
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

} // namespace

TabBar::Button::Button(window_t *parent, Tab tab, Tab active)
    : Card(parent, Rect16(style::tab_w * int(tab), style::tab_bar_rect.Top(), style::tab_w, style::tab_bar_rect.Height()), tab_defs[int(tab)].action, style::tab_bar, style::tab_bar, 8)
    , icon(*tab_defs[int(tab)].icon)
    , label(_(tab_defs[int(tab)].label))
    , active(tab == active) {}

void TabBar::Button::unconditionalDraw() {
    const Rect16 r = GetRect();
    // Ciblé à la molette : une pastille plus claire dans la case (la barre n'a pas de marge)
    const Color bg = IsFocused() ? style::surface : style::tab_bar;
    display::fill_rect(r, style::tab_bar);
    if (IsFocused()) {
        display::draw_rounded_rect(Rect16(r.Left() + 6, r.Top() + 4, r.Width() - 12, r.Height() - 5), style::tab_bar, bg, radius, MIC_ALL_CORNERS);
    }
    if (active) {
        display::draw_rounded_rect(Rect16(r.Left() + 38, r.Top(), 44, 3), style::tab_bar, style::accent, 2, MIC_BOT_LEFT | MIC_BOT_RIGHT);
    }
    draw_icon({ int16_t(r.Left() + (r.Width() - icon.w) / 2), int16_t(r.Top() + 6) }, icon, bg, active ? ropfn {} : ropfn { .shadow = is_shadowed::yes });
    draw_text(Rect16(r.Left() + 10, r.Top() + 24, r.Width() - 20, height(Font::small)), label, Font::small, active ? style::accent : style::text_muted, bg, Align_t::Center());
}

TabBar::TabBar(window_t *parent, Tab active)
    : line(parent, style::tab_line)
    , buttons {
        { parent, Tab::home, active },
        { parent, Tab::print, active },
        { parent, Tab::workshop, active },
        { parent, Tab::settings, active },
    } {
    line.SetBackColor(style::line);
}

} // namespace od3d
