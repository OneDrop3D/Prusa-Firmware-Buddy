/// @file
#include "screen_mmu_od3d.hpp"
#include "menu_items_od3d.hpp"

#include "spool_sheet_od3d.hpp"

#include <common/marlin_vars.hpp>
#include <cstdio>
#include <display.hpp>
#include <feature/prusa/MMU2/mmu2_mk4.h>
#include <feature/prusa/MMU2/mmu2_state.h>
#include <marlin_client.hpp>
#include <timing.h>
#include <i18n.h>
#include <img_resources.hpp>
#include <iterator>

namespace od3d {

namespace {

    using atelier::Op;

    // Géométrie de l'étagère (SPEC_G.md §3.2), sous l'en-tête de 40 px : cartes de 130 px (136 dans la maquette) ;
    // une carte chargée remonte de 3 px, d'où un rect de 133 px de haut
    constexpr uint8_t lift = 3;
    constexpr uint16_t shelf_card_h = 130;
    constexpr Rect16 shelf_rect(int i) {
        return Rect16(style::margin + 94 * i, style::top, 88, shelf_card_h + lift);
    }
    constexpr Rect16 shortcut_body(int i) {
        return Rect16(style::margin + 156 * (i % 3), 187 + 46 * (i / 3), 148, 40);
    }
    static_assert(style::with_ring(shelf_rect(0)).Bottom() < style::with_ring(shortcut_body(0)).Top());
    static_assert(style::inside(style::content, style::with_ring(shelf_rect(0))) && style::inside(style::content, style::with_ring(shelf_rect(4))));
    static_assert(style::inside(style::content, style::with_ring(shortcut_body(5))));
    static_assert(shelf_rect(4).Right() + 1 == style::screen_w - style::margin);

    enum ShelfTag : uint8_t {
        // 0 à 4 : les bobines
        tag_preload_all = 5,
        tag_assist,
        tag_motors,
        tag_pressure_advance,
        tag_settings,
        tag_info,
    };

    bool mmu_ready() {
        const auto state = MMU2::xState(marlin_vars().mmu2_state.get());
        return state != MMU2::xState::Connecting && state != MMU2::xState::Bootloader;
    }

    /// Un élément désactivé ne garde pas le focus : le premier élément actif le prend (le retour de l'en-tête)
    void keep_focus(window_frame_t &screen) {
        if (window_t *focused = screen.get_focused_child_window(); !focused || !focused->IsEnabled()) {
            if (window_t *first = screen.GetFirstEnabledSubWin()) {
                first->SetFocus();
            }
        }
    }

} // namespace

// ---- Étagère ----

ShelfSpool::ShelfSpool(window_t *parent, uint8_t index)
    : Button(parent, shelf_rect(index), index, Look::card, {}, nullptr, style::screen, style::radius_big)
    , slot(Slot::read(index)) {}

void ShelfSpool::update(bool mmu_ready) {
    const Slot s = Slot::read(tag);
    const bool enabled = s.enabled && mmu_ready;
    if (!(s == slot) || enabled != IsEnabled()) {
        slot = s;
        set_enabled(enabled);
        Invalidate();
    }
}

void ShelfSpool::unconditionalDraw() {
    // Le corps bouge selon l'état : il est repeint avec sa marge, puis la bande de 3 px qu'il laisse, au fond
    const Rect16 r = GetRect();
    const Rect16 full = body();
    const Rect16 b(full.Left(), full.Top() + (slot.loaded ? 0 : lift), full.Width(), shelf_card_h);
    const Rect16 outer = style::with_ring(b);
    display::fill_rect(slot.loaded ? Rect16(r.Left(), outer.Bottom() + 1, r.Width(), lift) : Rect16(r.Left(), r.Top(), r.Width(), lift), style::screen);
    const Color base = slot.loaded ? style::secondary : style::surface;
    const Color fill = IsFocused() ? style::focused(base) : base;
    draw_body(outer, fill, style::screen, radius);

    const bool enabled = IsEnabled();
    constexpr uint8_t d = 64;
    const point_i16_t at { int16_t(b.Left() + (b.Width() - d) / 2), int16_t(b.Top() + 8) };
    draw_spool(at, slot.look, d, fill, std::optional<Color>(slot.loaded ? style::accent : fill));
    // Le numéro après l'anneau orange, dont le carré repeint les coins
    const char number[2] = { char('1' + tag), '\0' };
    draw_text(Rect16(b.Left() + 8, b.Top() + 5, 9, height(Font::small)), string_view_utf8::MakeRAM(number), Font::small, style::text_muted, fill);

    draw_text(Rect16(b.Left(), b.Top() + 74, b.Width(), height(Font::normal)), string_view_utf8::MakeRAM(slot.material.data()), Font::normal,
        enabled ? style::text_strong : style::text_faint, fill, Align_t::Center());

    // 2e ligne : « Chargé » en orange, « Vide », ou le nom de la couleur choisie dans l'assistant
    char color_name[16];
    string_view_utf8 line;
    Color line_color = style::text_muted;
    if (slot.loaded) {
        line = _("Loaded");
        line_color = style::accent;
    } else if (!slot.look.filament) {
        line = _("Empty");
    } else {
        atelier::paint_name(color_name, atelier::paint_index(FilamentType::for_tool(VirtualToolIndex::from_raw(tag))));
        line = string_view_utf8::MakeRAM(color_name);
    }
    draw_text(Rect16(b.Left(), b.Top() + 94, b.Width(), height(Font::small)), line, Font::small, enabled ? line_color : style::text_faint, fill, Align_t::Center());

    if (slot.loaded) {
        display::draw_rounded_rect(Rect16(b.Left() + 26, b.Top() + shelf_card_h - 4, 36, 3), fill, style::accent, 2, MIC_ALL_CORNERS);
    }
}

Shortcut::Shortcut(window_t *parent, Rect16 body, uint8_t tag, const img::Resource &icon, Color tile, const string_view_utf8 &title, const string_view_utf8 &subtitle)
    : Button(parent, body, tag, Look::card, title, &icon, style::screen)
    , tile(tile)
    , subtitle(subtitle) {}

void Shortcut::unconditionalDraw() {
    const Colors c = colors();
    const Rect16 b = body();
    draw_frame(b, c);
    const bool enabled = IsEnabled();
    const Rect16 t(b.Left() + 8, b.Top() + 8, 24, 24);
    display::draw_rounded_rect(t, c.fill, tile, style::radius_small, MIC_ALL_CORNERS);
    draw_icon({ int16_t(t.Left() + 4), int16_t(t.Top() + 4) }, *icon, tile, enabled ? ropfn {} : ropfn { .shadow = is_shadowed::yes });
    const int16_t x = b.Left() + 40;
    const uint16_t w = b.Left() + b.Width() - 4 - x;
    draw_text(Rect16(x, b.Top() + 4, w, height(Font::small)), label, Font::small, enabled ? style::text : style::text_faint, c.fill);
    draw_text(Rect16(x, b.Top() + 20, w, height(Font::small)), subtitle, Font::small, enabled ? style::text_muted : style::text_faint, c.fill);
}

ScreenMMU::ScreenMMU()
    : Screen(true)
    , header(this, string_view_utf8::MakeCPUFLASH("MMU"), true)
    , spools { { this, 0 }, { this, 1 }, { this, 2 }, { this, 3 }, { this, 4 } }
    , preload_all(this, shortcut_body(0), tag_preload_all, img::cycle_16x16, style::tile_motion, _("Preload"), _("All slots"))
    , assist(this, shortcut_body(1), tag_assist, img::speed_16x16, style::tile_motion, _("MMU assist"), _("soon"))
    , motors(this, shortcut_body(2), tag_motors, img::move_16x16, style::tile_motion, _("Motors"), _("Open >"))
    , pressure_advance(this, shortcut_body(3), tag_pressure_advance, img::user_filament_16x16, style::tile_filament, _("Press. adv."), _("soon"))
    , settings(this, shortcut_body(4), tag_settings, img::settings_16x16, style::tile, _("Settings"), _("Open >"))
    , info(this, shortcut_body(5), tag_info, img::info_16x16, style::tile, _("MMU info"), _("Open >"))
    , tabs(this, TabBar::Tab::workshop) {
    assist.set_enabled(false); // F11 (accompagnement) n'existe pas encore
    pressure_advance.set_enabled(false); // F4 (mesure du PA) non plus
    update();

    // Focus de départ : l'emplacement chargé, sinon la première bobine utilisable
    for (ShelfSpool &spool : spools) {
        if (spool.loaded() && spool.IsEnabled()) {
            spool.SetFocus();
            return;
        }
    }
    for (ShelfSpool &spool : spools) {
        if (spool.IsEnabled()) {
            spool.SetFocus();
            return;
        }
    }
    keep_focus(*this);
}

void ScreenMMU::update() {
    if (GetFirstDialog()) {
        return; // la fiche est ouverte : ne pas lui prendre le focus
    }
    const bool ready = mmu_ready();
    for (ShelfSpool &spool : spools) {
        spool.update(ready);
    }
    preload_all.enable(atelier::mmu_active() && atelier::can_run() && !atelier::filament_in_nozzle());
    keep_focus(*this);
}

void ScreenMMU::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::HELD_RELEASED:
        if (GetFirstDialog()) {
            return; // l'appui long ferme la fiche, pas l'étagère
        }
        break;

    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag < std::size(spools)) {
            open_sheet(tag);
            return;
        }
        switch (tag) {
        case tag_preload_all:
            atelier::run(Op::preload_all, 0);
            break;
        case tag_motors:
            open_screen<ScreenMMUMotors>();
            break;
        case tag_settings:
            open_screen<ScreenMenuMMUSettings>();
            break;
        case tag_info:
            open_screen<ScreenMenuMMUInfo>();
            break;
        default:
            break;
        }
        return;
    }

    default:
        break;
    }
    Screen::windowEvent(sender, event, param);
}

// ---- Moteurs ----

namespace {

    constexpr Rect16 panel_rect(int i) {
        return Rect16(style::margin, style::top + 78 * i, 464, 71);
    }
    constexpr Rect16 note_rect(int i) {
        return Rect16(200, panel_rect(i).Top() + 7, 260, height(Font::small));
    }
    constexpr Rect16 pulley_body(int i) {
        return Rect16(18 + 150 * i, panel_rect(0).Top() + 28, 142, 36);
    }
    constexpr Rect16 position_body(int row, int i) {
        return Rect16(18 + 75 * i, panel_rect(row).Top() + 28, 68, 36);
    }
    static_assert(style::inside(panel_rect(0), style::with_ring(pulley_body(2))) && style::inside(panel_rect(2), style::with_ring(position_body(2, 5))));
    static_assert(style::inside(style::content, panel_rect(2)));

    // Registres du MMU (Prusa-Firmware-MMU, registers.cpp) : 0 à 4 = emplacement, 5 = parking / galet libre
    constexpr uint8_t reg_selector = 0x1b;
    constexpr uint8_t reg_idler = 0x1c;
    constexpr uint8_t park = 5;

    /// BUG-1 : un galet serré garde son moteur à 530 mA (Idler::FinishMove du firmware MMU). Chez Prusa, il n'est
    /// serré que le temps d'un chargement : ici, on le relâche seul après ce délai sans nouvelle commande.
    constexpr uint32_t idler_release_ms = 120'000;

    /// Position rapportée par le MMU (registres lus à chaque interrogation) ; -1 si inconnue (MMU absent, prise
    /// d'origine en cours = 0xff)
    int8_t reported(uint8_t value) {
        return value <= park ? int8_t(value) : int8_t(-1);
    }

    enum MotorTag : uint8_t {
        tag_pulley = 0, // 0 à 2, inactifs
        tag_selector = 10, // 10 à 15 : emplacements 1 à 5 puis parking
        tag_idler = 20, // 20 : libre, 21 à 25 : serré sur 1 à 5
    };

    void setup_note(window_text_t &note) {
        note.set_font(Font::small);
        note.SetAlignment(Align_t::RightCenter());
        note.SetTextColor(style::text_muted);
        note.SetBackColor(style::surface);
        note.SetPadding({ 0, 0, 0, 0 });
    }

    constexpr const char *slot_numbers[] = { "1", "2", "3", "4", "5" };

} // namespace

Panel::Panel(window_t *parent, Rect16 rect, const string_view_utf8 &title)
    : window_t(parent, rect)
    , title(title) {}

void Panel::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    draw_text(Rect16(r.Left() + 12, r.Top() + 7, 170, height(Font::small)), title, Font::small, style::text_strong, style::surface);
}

PositionButton::PositionButton(window_t *parent, Rect16 body, uint8_t tag, int8_t spool, const string_view_utf8 &label)
    : Button(parent, body, tag, Look::secondary, label, nullptr, style::surface, 10)
    , spool(spool) {}

void PositionButton::unconditionalDraw() {
    if (spool < 0) {
        Button::unconditionalDraw();
        return;
    }
    const Colors c = colors();
    const Rect16 inner = draw_frame(body(), c);
    constexpr uint8_t d = 16;
    const int16_t y = inner.Top() + (inner.Height() - d) / 2;
    draw_spool({ int16_t(inner.Left() + 16), y }, Slot::read(spool).look, d, c.fill);
    draw_text(Rect16(inner.Left() + 38, inner.Top() + (inner.Height() - height(Font::small)) / 2, width(Font::small), height(Font::small)), label, Font::small,
        c.fg, c.fill);
}

ScreenMMUMotors::ScreenMMUMotors()
    : Screen(true)
    , header(this, _("MMU motors"), true)
    , pulley_panel(this, panel_rect(0), _("Pulley"))
    , pulley_note(this, note_rect(0), is_multiline::no, is_closed_on_click_t::no, _("soon: MMU firmware"))
    , pulley_back(this, pulley_body(0), tag_pulley, Button::Look::secondary, string_view_utf8::MakeCPUFLASH("<< 10 mm"), nullptr, style::surface, 10)
    , pulley_forward(this, pulley_body(1), tag_pulley + 1, Button::Look::secondary, string_view_utf8::MakeCPUFLASH("10 mm >>"), nullptr, style::surface, 10)
    , pulley_clean(this, pulley_body(2), tag_pulley + 2, Button::Look::selected, _("Clean"), nullptr, style::surface, 10)
    , selector_panel(this, panel_rect(1), _("Selector"))
    , selector_note(this, note_rect(1), is_multiline::no)
    , selector {
        { this, position_body(1, 0), tag_selector + 0, 0, string_view_utf8::MakeCPUFLASH(slot_numbers[0]) },
        { this, position_body(1, 1), tag_selector + 1, 1, string_view_utf8::MakeCPUFLASH(slot_numbers[1]) },
        { this, position_body(1, 2), tag_selector + 2, 2, string_view_utf8::MakeCPUFLASH(slot_numbers[2]) },
        { this, position_body(1, 3), tag_selector + 3, 3, string_view_utf8::MakeCPUFLASH(slot_numbers[3]) },
        { this, position_body(1, 4), tag_selector + 4, 4, string_view_utf8::MakeCPUFLASH(slot_numbers[4]) },
        { this, position_body(1, 5), tag_selector + 5, -1, string_view_utf8::MakeCPUFLASH("Park") },
    }
    , idler_panel(this, panel_rect(2), _("Idler"))
    , idler_note(this, note_rect(2), is_multiline::no)
    , idler {
        { this, position_body(2, 0), tag_idler + 0, -1, _("Free") },
        { this, position_body(2, 1), tag_idler + 1, -1, string_view_utf8::MakeCPUFLASH(slot_numbers[0]) },
        { this, position_body(2, 2), tag_idler + 2, -1, string_view_utf8::MakeCPUFLASH(slot_numbers[1]) },
        { this, position_body(2, 3), tag_idler + 3, -1, string_view_utf8::MakeCPUFLASH(slot_numbers[2]) },
        { this, position_body(2, 4), tag_idler + 4, -1, string_view_utf8::MakeCPUFLASH(slot_numbers[3]) },
        { this, position_body(2, 5), tag_idler + 5, -1, string_view_utf8::MakeCPUFLASH(slot_numbers[4]) },
    }
    , tabs(this, TabBar::Tab::workshop) {
    for (window_text_t *note : { &pulley_note, &selector_note, &idler_note }) {
        setup_note(*note);
    }
    // Poulie : rotation libre et pas de 10 mm demandent une commande que le firmware du MMU n'a pas (F12)
    pulley_back.set_enabled(false);
    pulley_forward.set_enabled(false);
    pulley_clean.set_enabled(false);
    update();
    show();
    if (selector[0].IsEnabled()) {
        selector[0].SetFocus();
    } else {
        keep_focus(*this);
    }
}

ScreenMMUMotors::~ScreenMMUMotors() {
    // Sans dialogue (destruction de l'écran) : simple mise en file du G-code
    if (engaged_here && atelier::mmu_active() && MMU2::mmu2.IdlerSlot() < park) {
        marlin_client::gcode("M708 A0x1c X5");
    }
}

void ScreenMMUMotors::update() {
    const bool active = atelier::mmu_active();
    const bool ok = active && atelier::can_run();

    // Vraies positions (et non la dernière demande) : on voit un galet resté serré
    const int8_t sel = active ? reported(MMU2::mmu2.SelectorSlot()) : int8_t(-1);
    const int8_t idl = active ? reported(MMU2::mmu2.IdlerSlot()) : int8_t(-1);
    if (idl == park && ticks_ms() - engaged_ms > 5'000) {
        engaged_here = false; // relâché (par nous, par Prusa ou par le MMU) ; 5 s : le temps que le MMU serre et le rapporte
    }
    int16_t left = -1;
    if (engaged_here) {
        const int32_t remaining = int32_t(idler_release_ms) - int32_t(ticks_ms() - engaged_ms);
        if (remaining <= 0) {
            if (marlin_client::gcode_try("M708 A0x1c X5") == marlin_client::GcodeTryResult::Submitted) {
                engaged_here = false;
            }
        } else {
            left = int16_t((remaining + 999) / 1000);
        }
    }
    if (sel != selector_at || idl != idler_at || left != seconds_left) {
        selector_at = sel;
        idler_at = idl;
        seconds_left = left;
        show();
    }

    // Le MMU refuse de bouger le sélecteur quand le FINDA voit un filament (selector.cpp) : bouton inactif
    const bool selector_ok = ok && !marlin_vars().mmu2_finda.get();
    for (PositionButton &b : selector) {
        b.enable(selector_ok);
    }
    for (PositionButton &b : idler) {
        b.enable(ok);
    }
    keep_focus(*this);
}

void ScreenMMUMotors::show() {
    for (int i = 0; i < 6; ++i) {
        selector[i].set_look(i == selector_at ? Button::Look::selected : Button::Look::secondary);
        // Bouton 0 = galet libre (valeur 5), boutons 1 à 5 = serré sur l'emplacement 0 à 4
        const int idler_value = i == 0 ? park : i - 1;
        idler[i].set_look(idler_value == idler_at ? Button::Look::selected : Button::Look::secondary);
    }
    if (selector_at < 0) {
        selector_note.SetText(string_view_utf8::MakeNULLSTR());
    } else if (selector_at == park) {
        selector_note.SetText(_("parking"));
    } else {
        selector_note.SetText(_("position: %d").formatted(selector_params, selector_at + 1));
    }
    if (idler_at < 0) {
        idler_note.SetText(string_view_utf8::MakeNULLSTR());
    } else if (idler_at == park) {
        idler_note.SetText(_("idler free"));
    } else if (seconds_left >= 0) {
        idler_note.SetText(_("pressed on %d (free %d s)").formatted(idler_params, idler_at + 1, seconds_left));
    } else {
        idler_note.SetText(_("pressed on %d").formatted(idler_params, idler_at + 1));
    }
}

void ScreenMMUMotors::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag >= tag_selector && tag <= tag_selector + 5) {
            const uint8_t value = tag - tag_selector; // 0 à 4, 5 = parking
            atelier::write_register(reg_selector, value); // la position affichée suit le MMU (update)
        } else if (tag >= tag_idler && tag <= tag_idler + 5) {
            const uint8_t value = tag == tag_idler ? park : tag - tag_idler - 1;
            if (atelier::write_register(reg_idler, value) && value != park) {
                engaged_here = true;
                engaged_ms = ticks_ms();
            }
        }
        return;
    }

    default:
        break;
    }
    Screen::windowEvent(sender, event, param);
}

// ---- Menus Prusa ----

ScreenMenuMMUSettings::ScreenMenuMMUSettings()
    : ScreenMenuMMUSettingsBase(_("MMU settings"), &img::settings_16x16) {}

ScreenMenuMMUInfo::ScreenMenuMMUInfo()
    : ScreenMenuMMUInfoBase(_("MMU info"), &img::info_16x16) {}

// ---- Contrôle › Axes du MMU ----

MI_MMU_AXES::MI_MMU_AXES()
    : IWindowMenuItem(_("MMU axes"), &img::spool_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_MMU_AXES::click(IWindowMenu &) {
    open_screen<ScreenMMUMotors>();
}

} // namespace od3d
