/// @file
#include "screen_mmu_od3d.hpp"
#include "layout_od3d.hpp"
#include "menu_items_od3d.hpp"
#include "menu_od3d.hpp"

#include "screen_pa_od3d.hpp"
#include "spool_sheet_od3d.hpp"

#include <common/marlin_vars.hpp>
#include <cstdio>
#include <display.hpp>
#include <feature/od3d/spool_usage_od3d.hpp>
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
    constexpr uint8_t shelf_gap = 6;
    constexpr Rect16 shelf_area = { style::margin, style::top, 464, shelf_card_h + lift };
    constexpr Rect16 shelf_rect(int i) {
        return layout::spool_cell(shelf_area, 5, shelf_gap, i);
    }
    /// Rect de la bobine index parmi tools (une ou deux rangées) ; celles au-delà, cachées, prennent la place de la 1re
    constexpr Rect16 shelf_rect(uint8_t index, uint8_t tools, int16_t top = style::top) {
        const Rect16 area(shelf_area.Left(), top, shelf_area.Width(), shelf_area.Height());
        return layout::spool_cell(area, tools, shelf_gap, index < tools ? index : 0);
    }
    constexpr Rect16 shortcut_body(int i) {
        return Rect16(style::margin + 156 * (i % 3), 187 + 46 * (i / 3), 148, 40);
    }
    static_assert(style::with_ring(shelf_rect(0)).Bottom() < style::with_ring(shortcut_body(0)).Top());
    static_assert(style::inside(style::content, style::with_ring(shelf_rect(0))) && style::inside(style::content, style::with_ring(shelf_rect(4))));
    static_assert(style::inside(style::content, style::with_ring(shortcut_body(5))));
    static_assert(shelf_rect(4).Right() + 1 == style::screen_w - style::margin);
    static_assert(shelf_rect(4) == Rect16(384, style::top, 88, 133), "les 5 bobines du MMU gardent leur place (SPEC_G.md §3.2)");

    /// Corps d'une carte de l'étagère dans sa fenêtre (son rect et la marge de l'anneau) : 3 px plus haut si la bobine
    /// est chargée. Avant, il prenait la hauteur de la fenêtre et non celle de son corps : 4 px plus bas que la fenêtre,
    /// repeints par le fond de l'écran (bas des cartes coupé).
    constexpr Rect16 shelf_card(Rect16 window, bool loaded) {
        const Rect16 full(window.Left() + style::ring, window.Top() + style::ring, window.Width() - 2 * style::ring, window.Height() - 2 * style::ring);
        return Rect16(full.Left(), full.Top() + (loaded ? 0 : lift), full.Width(), full.Height() - lift);
    }
    /// Cartes de l'étagère (1 à 8 outils), chargées ou non : dans leur fenêtre, sans chevauchement, au-dessus des raccourcis
    constexpr bool shelf_fits(uint8_t tools) {
        for (uint8_t i = 0; i < tools; ++i) {
            const Rect16 window = style::with_ring(shelf_rect(i, tools));
            if (!style::inside(window, style::with_ring(shelf_card(window, false))) || !style::inside(window, style::with_ring(shelf_card(window, true)))
                || window.Bottom() >= style::with_ring(shortcut_body(0)).Top()) {
                return false;
            }
            for (uint8_t j = 0; j < i; ++j) {
                if (layout::overlap(window, style::with_ring(shelf_rect(j, tools)))) {
                    return false;
                }
            }
        }
        return true;
    }
    static_assert(shelf_fits(1) && shelf_fits(2) && shelf_fits(3) && shelf_fits(4) && shelf_fits(5) && shelf_fits(6) && shelf_fits(7) && shelf_fits(8));
    static_assert(shelf_card(style::with_ring(shelf_rect(0)), true).Height() == shelf_card_h, "une rangée : cartes de 130 px");

    // Choix d'une bobine : l'aide sur une ligne sous l'en-tête, puis l'étagère de l'Atelier (mêmes cartes, 1 à 8 outils)
    constexpr Rect16 pick_help_rect = { style::margin, style::top, 464, 20 };
    constexpr int16_t pick_top = pick_help_rect.Bottom() + 1 + 8;
    static_assert(style::inside(style::content, style::with_ring(Rect16(shelf_area.Left(), pick_top, shelf_area.Width(), shelf_area.Height()))));
    static_assert(shelf_rect(0, 8, pick_top).Top() == pick_top && shelf_rect(7, 8, pick_top).Bottom() == pick_top + shelf_area.Height() - 1);

    /// Demande du choix d'une bobine (open_slot_pick), lue par l'écran à sa création
    struct PickRequest {
        string_view_utf8 title;
        string_view_utf8 help;
        uint8_t focus = 0;
        SlotPick pick = nullptr;
    };
    PickRequest pick_request;

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

    /// Emplacement de la tuile « Press. adv. » (F4) : celui de la buse, sinon le premier qui a une matière
    std::optional<uint8_t> pa_slot() {
        if (const auto tool = VirtualToolIndex::currently_selected_opt()) {
            return tool->to_raw();
        }
        for (uint8_t i = 0; i < VirtualToolIndex::count; ++i) {
            if (FilamentType::for_tool(VirtualToolIndex::from_raw(i))) {
                return i;
            }
        }
        return std::nullopt;
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

ShelfSpool::ShelfSpool(window_t *parent, uint8_t index, uint8_t tools, int16_t top)
    : Button(parent, shelf_rect(index, tools, top), index, Look::card, {}, nullptr, style::screen, style::radius_big)
    , slot(Slot::read(index)) {
    if (index >= tools) {
        set_visible(false); // outil absent (MMU désactivé, têtes de l'INDX non montées)
        set_enabled(false);
    }
}

void ShelfSpool::update(bool mmu_ready) {
    if (!HasVisibleFlag()) {
        return; // outil absent
    }
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
    const Rect16 b = shelf_card(r, slot.loaded);
    const uint16_t card_h = b.Height(); // 130 sur une rangée, ~60 sur deux
    const Rect16 outer = style::with_ring(b);
    display::fill_rect(slot.loaded ? Rect16(r.Left(), outer.Bottom() + 1, r.Width(), lift) : Rect16(r.Left(), r.Top(), r.Width(), lift), style::screen);
    const Color base = slot.loaded ? style::secondary : style::surface;
    const Color fill = IsFocused() ? style::focused(base) : base;
    draw_body(outer, fill, style::screen, radius);

    const bool enabled = IsEnabled();
    // 2e ligne : « Chargé » en orange, « Vide », le reste, ou le nom de la couleur choisie dans l'assistant
    char state_text[16];
    Color state_color;
    const string_view_utf8 state = atelier::slot_state(slot, tag, state_text, state_color);

    if (card_h < 100) {
        // Deux rangées (INDX 8 têtes) : bobine à gauche, « n MATIÈRE » et l'état à droite (6 caractères)
        constexpr uint8_t d = 32;
        const point_i16_t at { int16_t(b.Left() + 8), int16_t(b.Top() + (card_h - d) / 2) };
        draw_spool(at, slot.look, d, fill, std::optional<Color>(slot.loaded ? style::accent : fill));
        const int16_t x = at.x + d + 8;
        const uint16_t w = b.Left() + b.Width() - 4 - x;
        char label[12];
        snprintf(label, sizeof(label), "%d %s", tag + 1, slot.material.data());
        draw_text(Rect16(x, b.Top() + card_h / 2 - height(Font::small), w, height(Font::small)), string_view_utf8::MakeRAM(label), Font::small,
            enabled ? style::text_strong : style::text_faint, fill);
        draw_text(Rect16(x, b.Top() + card_h / 2, w, height(Font::small)), state, Font::small, enabled ? state_color : style::text_faint, fill);
        return;
    }

    constexpr uint8_t d = 64;
    const point_i16_t at { int16_t(b.Left() + (b.Width() - d) / 2), int16_t(b.Top() + 8) };
    draw_spool(at, slot.look, d, fill, std::optional<Color>(slot.loaded ? style::accent : fill));
    // Le numéro après l'anneau orange, dont le carré repeint les coins
    const char number[2] = { char('1' + tag), '\0' };
    draw_text(Rect16(b.Left() + 8, b.Top() + 5, 9, height(Font::small)), string_view_utf8::MakeRAM(number), Font::small, style::text_muted, fill);

    draw_text(Rect16(b.Left(), b.Top() + 74, b.Width(), height(Font::normal)), string_view_utf8::MakeRAM(slot.material.data()), Font::normal,
        enabled ? style::text_strong : style::text_faint, fill, Align_t::Center());
    draw_text(Rect16(b.Left(), b.Top() + 94, b.Width(), height(Font::small)), state, Font::small, enabled ? state_color : style::text_faint, fill, Align_t::Center());

    if (slot.loaded) {
        display::draw_rounded_rect(Rect16(b.Left() + (b.Width() - 36) / 2, b.Top() + shelf_card_h - 4, 36, 3), fill, style::accent, 2, MIC_ALL_CORNERS);
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
    : ScreenMMU(std::make_index_sequence<VirtualToolIndex::count>()) {}

template <size_t... I>
ScreenMMU::ScreenMMU(std::index_sequence<I...>)
    : Screen(true)
    // Sans MMU, l'étagère de l'onglet Atelier n'a que la bobine de la buse
    , header(this, atelier::mmu_enabled() ? string_view_utf8::MakeCPUFLASH("MMU") : _("Workshop"), true)
    , spools { ShelfSpool(this, I, atelier::visible_tools())... }
    , preload_all(this, shortcut_body(0), tag_preload_all, img::cycle_16x16, style::tile_motion, _("Preload"), _("All slots"))
    , assist(this, shortcut_body(1), tag_assist, img::speed_16x16, style::tile_motion, _("MMU assist"), _("soon"))
    , motors(this, shortcut_body(2), tag_motors, img::move_16x16, style::tile_motion, _("Motors"), _("Open >"))
    , pressure_advance(this, shortcut_body(3), tag_pressure_advance, img::user_filament_16x16, style::tile_filament, _("Press. adv."), _("Open >"))
    , settings(this, shortcut_body(4), tag_settings, img::settings_16x16, style::tile, _("Settings"), _("Open >"))
    , info(this, shortcut_body(5), tag_info, img::info_16x16, style::tile, _("MMU info"), _("Open >"))
    , tabs(this, TabBar::Tab::workshop) {
    assist.set_enabled(false); // F11 (accompagnement) n'existe pas encore
    pressure_advance.set_enabled(pa_slot().has_value()); // F4 : l'écran du PA (« Mesurer » charge la bobine)
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
    pressure_advance.enable(pa_slot().has_value());
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
        case tag_pressure_advance:
            if (const auto slot = pa_slot()) {
                open_pa(*slot);
            }
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

// ---- Choix d'une bobine ----

void open_slot_pick(const string_view_utf8 &title, const string_view_utf8 &help, uint8_t focus, SlotPick pick) {
    pick_request = { title, help, focus, pick };
    open_screen<ScreenSlotPick>();
}

ScreenSlotPick::ScreenSlotPick()
    : ScreenSlotPick(std::make_index_sequence<VirtualToolIndex::count>()) {}

template <size_t... I>
ScreenSlotPick::ScreenSlotPick(std::index_sequence<I...>)
    : Screen(true)
    , header(this, pick_request.title, true)
    , help(this, pick_help_rect, is_multiline::no, is_closed_on_click_t::no, pick_request.help)
    , spools { ShelfSpool(this, I, atelier::visible_tools(), pick_top)... }
    , tabs(this, section_tab()) {
    help.set_font(Font::small);
    help.SetTextColor(style::text_soft);
    help.SetBackColor(style::screen);
    help.SetPadding({ 0, 0, 0, 0 });
    const bool ready = mmu_ready();
    for (ShelfSpool &spool : spools) {
        spool.update(ready);
    }
    // La carte demandée (le dernier choix), sinon la première utilisable
    if (pick_request.focus < std::size(spools) && spools[pick_request.focus].IsEnabled()) {
        spools[pick_request.focus].SetFocus();
        return;
    }
    for (ShelfSpool &spool : spools) {
        if (spool.IsEnabled()) {
            spool.SetFocus();
            return;
        }
    }
    keep_focus(*this);
}

void ScreenSlotPick::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::CHILD_CLICK) {
        const uintptr_t tag = reinterpret_cast<uintptr_t>(param);
        if (tag < std::size(spools) && pick_request.pick && pick_request.pick(uint8_t(tag))) {
            Screens::Access()->close_screen();
        }
        return;
    }
    if (event == GUI_event_t::LOOP && !GetFirstDialog()) { // sous la question, le focus reste à la question
        const bool ready = mmu_ready();
        for (ShelfSpool &spool : spools) {
            spool.update(ready);
        }
        keep_focus(*this);
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

    /// F12 : registre de la poulie, ajouté par notre firmware du MMU (Prusa-Firmware-MMU OneDrop3D, branche
    /// od3d/f12-poulie, logic/pulley_service.h). Valeur = emplacement * 256 + mm (octet signé) ; 0 = arrêt ;
    /// emplacement 0 à 4 = galet serré, poulie de ±mm puis galet relâché ; 5 = galet libre, rotation à |mm| mm/s
    /// arrêtée par le MMU lui-même après 60 s.
    constexpr uint8_t reg_pulley = 0x24;
    constexpr int8_t pulley_step_mm = 10;
    constexpr int8_t clean_speed_mm_s = 10; ///< rotation lente
    constexpr uint32_t clean_ms = 60'000; ///< durée maximale fixée par le firmware du MMU
    constexpr uint16_t pulley_request(uint8_t slot, int8_t mm) {
        return uint16_t(slot << 8 | uint8_t(mm));
    }

    /// Le MMU doit rapporter 3.0.4 (sinon l'imprimante le refuse) : notre firmware se reconnaît à son numéro de build,
    /// le nombre de commits de sa branche. Officiel Prusa 3.0.4 : 935 ; OneDrop3D F0 : 936 ; le registre 0x24 existe
    /// depuis le build 939 (commit F12 de od3d/f12-poulie).
    constexpr uint16_t mmu_build_with_pulley = 939;

    /// BUG-1 : un galet serré garde son moteur à 530 mA (Idler::FinishMove du firmware MMU). Chez Prusa, il n'est
    /// serré que le temps d'un chargement : ici, on le relâche seul après ce délai sans nouvelle commande.
    constexpr uint32_t idler_release_ms = 120'000;

    /// Position rapportée par le MMU (registres lus à chaque interrogation) ; -1 si inconnue (MMU absent, prise
    /// d'origine en cours = 0xff)
    int8_t reported(uint8_t value) {
        return value <= park ? int8_t(value) : int8_t(-1);
    }

    enum MotorTag : uint8_t {
        tag_pulley = 0, // 0 : -10 mm, 1 : +10 mm, 2 : nettoyer / arrêter
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
    // Poulie : rotation libre et pas de 10 mm demandent le registre 0x24 de notre firmware du MMU (F12) ; sans lui,
    // les boutons restent grisés avec la note « bientôt : firmware MMU » (update)
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
    if (cleaning && atelier::mmu_active()) {
        marlin_client::gcode("M708 A0x24 X0"); // le MMU l'arrêterait seul au bout de 60 s
    }
    if (engaged_here && atelier::mmu_active() && MMU2::mmu2.IdlerSlot() < park) {
        marlin_client::gcode("M708 A0x1c X5");
    }
}

int8_t ScreenMMUMotors::pulley_slot() const {
    if (idler_at >= 0 && idler_at < park) {
        return idler_at;
    }
    return selector_at >= 0 && selector_at < park ? selector_at : int8_t(-1);
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
    // F12 : le MMU arrête seul le nettoyage après 60 s
    const bool supported = active && MMU2::mmu2.GetMMUFWVersionBuild() >= mmu_build_with_pulley;
    int16_t clean = -1;
    if (cleaning) {
        const int32_t remaining = int32_t(clean_ms) - int32_t(ticks_ms() - cleaning_ms);
        if (remaining <= 0 || !active) {
            cleaning = false;
        } else {
            clean = int16_t((remaining + 999) / 1000);
        }
    }
    if (sel != selector_at || idl != idler_at || left != seconds_left || supported != pulley_supported || clean != clean_left) {
        selector_at = sel;
        idler_at = idl;
        seconds_left = left;
        pulley_supported = supported;
        clean_left = clean;
        show();
    }

    // Le MMU refuse de bouger le sélecteur quand le FINDA voit un filament (selector.cpp) : bouton inactif.
    // Il refuse aussi la poulie (registre 0x24) dès qu'un filament est dans le sélecteur.
    const bool finda = marlin_vars().mmu2_finda.get();
    const bool selector_ok = ok && !finda && !cleaning;
    for (PositionButton &b : selector) {
        b.enable(selector_ok);
    }
    // Pendant le nettoyage, le galet ne doit pas serrer un filament contre la poulie qui tourne
    for (PositionButton &b : idler) {
        b.enable(ok && !cleaning);
    }
    const bool pulley_ok = ok && supported && !finda;
    pulley_back.enable(pulley_ok && !cleaning && pulley_slot() >= 0);
    pulley_forward.enable(pulley_ok && !cleaning && pulley_slot() >= 0);
    pulley_clean.enable(pulley_ok || cleaning);
    keep_focus(*this);
}

void ScreenMMUMotors::show() {
    for (int i = 0; i < 6; ++i) {
        selector[i].set_look(i == selector_at ? Button::Look::selected : Button::Look::secondary);
        // Bouton 0 = galet libre (valeur 5), boutons 1 à 5 = serré sur l'emplacement 0 à 4
        const int idler_value = i == 0 ? park : i - 1;
        idler[i].set_look(idler_value == idler_at ? Button::Look::selected : Button::Look::secondary);
    }
    const int8_t slot = pulley_slot();
    if (!pulley_supported) {
        pulley_note.SetText(_("soon: MMU firmware"));
    } else if (clean_left >= 0) {
        pulley_note.SetText(_("cleaning (stops in %d s)").formatted(pulley_params, clean_left));
    } else if (slot < 0) {
        pulley_note.SetText(_("choose a slot below"));
    } else {
        pulley_note.SetText(_("pulley on slot %d").formatted(pulley_params, slot + 1));
    }
    pulley_clean.set(clean_left >= 0 ? Button::Look::alert : Button::Look::selected, clean_left >= 0 ? _("Stop cleaning") : _("Clean"));

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
        if (tag == tag_pulley || tag == tag_pulley + 1) {
            // Pas de 10 mm : le MMU serre le galet, tourne, puis le relâche lui-même
            if (const int8_t slot = pulley_slot(); slot >= 0) {
                atelier::write_register(reg_pulley, pulley_request(slot, int8_t(tag == tag_pulley ? -pulley_step_mm : pulley_step_mm)));
            }
        } else if (tag == tag_pulley + 2) {
            if (cleaning) {
                if (atelier::write_register(reg_pulley, 0)) {
                    cleaning = false;
                }
            } else if (atelier::write_register(reg_pulley, pulley_request(park, clean_speed_mm_s))) {
                // Le galet est libéré par le MMU : plus rien à relâcher ici
                engaged_here = false;
                cleaning = true;
                cleaning_ms = ticks_ms();
            }
            update();
        } else if (tag >= tag_selector && tag <= tag_selector + 5) {
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
