/// @file
#include "widgets_od3d.hpp"
#include "screen_files_od3d.hpp"
#include "screen_printing_od3d.hpp"
#include "screen_temperatures_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <display_helper.h>
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
    } else {
        strlcpy(slot.material.data(), "-", slot.material.size()); // « — » n'est pas dans la police
    }
    return slot;
}

// ---- Dessin ----

void draw_body(Rect16 outer, Color body, Color back, uint8_t radius, bool focused) {
    const Rect16 inner(outer.Left() + style::ring, outer.Top() + style::ring, outer.Width() - 2 * style::ring, outer.Height() - 2 * style::ring);
    const Color ring = focused ? style::focus_ring : back;
    display::draw_rounded_rect(outer, back, ring, radius + style::ring, MIC_ALL_CORNERS);
    display::draw_rounded_rect(inner, ring, body, radius, MIC_ALL_CORNERS);
}

void draw_text(Rect16 rect, const string_view_utf8 &text, Font font, Color fg, Color bg, Align_t align) {
    render_text_align(rect, text, font, bg, fg, { 0, 0, 0, 0 }, text_flags(align, is_multiline::no, check_overflow::no));
}

void draw_icon(point_i16_t at, const img::Resource &icon, Color bg, ropfn rop) {
    display::draw_img(point_ui16(at.x, at.y), icon, bg, rop);
}

void draw_spool(point_i16_t at, const spool::Look &look, uint8_t d) {
    const spool::Disc disc(look, d);
    for (int y = 0; y < d; ++y) {
        disc.for_each_run(y, [&](int x, int len, Color c) {
            display::fill_rect(Rect16(at.x + x, at.y + y, len, 1), c);
        });
    }
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
    if (event == GUI_event_t::HELD_RELEASED && has_back) {
        close_screen();
        return;
    }
    screen_t::windowEvent(sender, event, param);
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
    const Rect16 r = GetRect();
    return Rect16(r.Left() + style::ring, r.Top() + style::ring, r.Width() - 2 * style::ring, r.Height() - 2 * style::ring);
}

void Card::unconditionalDraw() {
    draw_body(GetRect(), body_color, back, radius, IsFocused());
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
        draw_icon({ x, int16_t(body.Top() + (body.Height() - icon.h) / 2) }, icon, body_color);
        return;
    }
    draw_icon({ x, int16_t(body.Top() + 12) }, icon, body_color);
    draw_text(Rect16(body.Left(), body.Top() + 33, body.Width(), height(Font::small)), label, Font::small, style::text_strong, body_color, Align_t::Center());
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
        Invalidate();
    }
}

void IconCard::draw_content(Rect16 body) {
    // Tuile de 22 px : le titre en gras (8 caractères, « Fichiers », « Contrôle ») garde une marge dans 128 px
    const Rect16 t(body.Left() + 7, body.Top() + (body.Height() - 22) / 2, 22, 22);
    display::draw_rounded_rect(t, body_color, tile, 6, MIC_ALL_CORNERS);
    draw_icon({ int16_t(t.Left() + 3), int16_t(t.Top() + 3) }, icon, tile);
    const int16_t x = t.Left() + t.Width() + 5;
    const uint16_t w = body.Left() + body.Width() - x;
    draw_text(Rect16(x, body.Top() + 7, w, height(Font::normal)), title, Font::normal, style::text_strong, body_color);
    draw_text(Rect16(x, body.Top() + 27, w, height(Font::small)), subtitle, Font::small, style::text_muted, body_color);
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
    : Card(parent, style::with_ring(body), open_temperatures, style::surface, style::screen, 6)
    , temps(Temps::read()) {}

void TempsButton::draw_content(Rect16 body) {
    draw_temps(Rect16(body.Left() + 4, body.Top() + (body.Height() - 16) / 2, body.Width() - 8, 16), temps, style::text, body_color, true);
}

void TempsButton::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP) {
        if (const Temps t = Temps::read(); t != temps) {
            temps = t;
            Invalidate();
        }
    }
    Card::windowEvent(sender, event, param);
}

TempsCard::TempsCard(window_t *parent, Rect16 body)
    : Card(parent, style::with_ring(body), open_temperatures)
    , temps(Temps::read()) {}

void TempsCard::draw_content(Rect16 body) {
    draw_text(Rect16(body.Left() + 10, body.Top() + 7, body.Width() - 20, height(Font::normal)), _("Temperatures"), Font::normal, style::text_strong, body_color);
    draw_temps(Rect16(body.Left() + 10, body.Top() + 29, body.Width() - 20, 16), temps, style::text_soft, body_color, false);
}

void TempsCard::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP) {
        if (const Temps t = Temps::read(); t != temps) {
            temps = t;
            Invalidate();
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

    // En-tête, de droite à gauche (SPEC_G.md §1.3) : heure, réseau, températures, puis le titre prend le reste
    constexpr int16_t header_gap = 6;
    constexpr uint16_t temps_w = 182; ///< 3 × (icône 16 + 2 + « 230° ») + 2 × 6, plus 4 de chaque côté

    uint16_t clock_w() {
        return (time_tools::get_time_format() == time_tools::TimeFormat::_24h ? 5 : 8) * width(Font::small);
    }
    int16_t clock_x() {
        return style::screen_w - style::margin - clock_w();
    }
    int16_t network_x() {
        return clock_x() - header_gap - 16;
    }
    Rect16 temps_body() {
        return Rect16(network_x() - header_gap - temps_w, 2, temps_w, 22);
    }
    Rect16 title_rect(bool has_back) {
        const int16_t x = has_back ? 36 : style::margin;
        return Rect16(x, 0, temps_body().Left() - style::ring - header_gap - x, style::header.Height());
    }

    void go_back() {
        Screens::Access()->close_screen();
    }

} // namespace

Header::Header(window_t *parent, const string_view_utf8 &title_text, bool has_back)
    : back(parent, style::with_ring(Rect16(style::margin, 2, 22, 22)), go_back, img::arrow_left_16x16, {}, style::secondary, 11)
    , title(parent, title_rect(has_back), is_multiline::no, is_closed_on_click_t::no, title_text)
    , temps(parent, temps_body())
    , network(parent, { network_x(), 5 })
    , clock(parent, Rect16(clock_x(), 0, clock_w(), style::header.Height()))
    , line(parent, style::header_line) {
    back.set_visible(has_back);
    back.set_enabled(has_back);
    title.set_font(Font::small);
    title.SetAlignment(Align_t::LeftCenter());
    title.SetTextColor(style::text);
    title.SetBackColor(style::screen);
    title.SetPadding({ 0, 0, 0, 0 });
    line.SetBackColor(style::line);
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
        Screens::Access()->CloseAll();
        open_filament_menu();
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
    display::fill_rect(r, style::tab_bar);
    // La barre n'a pas de marge : l'anneau de focus est dessiné à l'intérieur de la case
    draw_body(Rect16(r.Left() + 6, r.Top() + 3, r.Width() - 12, r.Height() - 4), style::tab_bar, style::tab_bar, radius, IsFocused());
    if (active) {
        display::draw_rounded_rect(Rect16(r.Left() + 38, r.Top(), 44, 3), style::tab_bar, style::accent, 2, MIC_BOT_LEFT | MIC_BOT_RIGHT);
    }
    draw_icon({ int16_t(r.Left() + (r.Width() - icon.w) / 2), int16_t(r.Top() + 7) }, icon, style::tab_bar, active ? ropfn {} : ropfn { .shadow = is_shadowed::yes });
    draw_text(Rect16(r.Left() + 8, r.Top() + 26, r.Width() - 16, height(Font::small)), label, Font::small, active ? style::accent : style::text_muted, style::tab_bar, Align_t::Center());
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
