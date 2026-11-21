/// @file
#include "screen_spool_od3d.hpp"

#include <algorithm>
#include <config_store/store_instance.hpp>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <filament_to_load.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <iterator>
#include <option/has_anfc.h>

#if HAS_ANFC()
    #include <feature/openprinttag/filament_usage_tracker/filament_usage_tracker.hpp>
#endif

namespace od3d {

namespace {

    using atelier::Op;

    /// État de l'assistant : il survit à l'écran, que l'écran de progression remplace pendant M705 et M704
    struct Wizard {
        uint8_t slot = 0;
        SpoolChange mode = SpoolChange::change;
        uint8_t step = 1; ///< 1 retirer, 2 insérer, 3 choisir
        uint8_t material = 0; ///< index dans materials
        int8_t paint = -1; ///< index dans atelier::paints, -1 = couleur du préréglage
    };
    Wizard wizard;

    // Matières de l'étape 3 (SPEC_G.md §3.6) ; le TPU s'appelle FLEX chez Prusa
    constexpr PresetFilamentType material_types[] = {
        PresetFilamentType::PLA, PresetFilamentType::PETG, PresetFilamentType::ASA,
        PresetFilamentType::PC, PresetFilamentType::FLEX, PresetFilamentType::PVB
    };
    constexpr const char *material_names[] = { "PLA", "PETG", "ASA", "PC", "FLEX", "PVB" };
    static_assert(std::size(material_types) == std::size(material_names));

    // Géométrie (SPEC_G.md §3.5, §3.6), boutons du bas communs aux 3 étapes
    constexpr Rect16 band_rect = { style::margin, 32, 464, 24 };
    constexpr Rect16 picture_rect = { style::margin, 62, 464, 96 };
    constexpr Rect16 title_rect = { 12, 164, 456, 19 };
    constexpr Rect16 text_rect = { 12, 186, 456, 34 };
    constexpr Rect16 material_body(int i) {
        return Rect16(style::margin + 156 * (i % 3), 60 + 34 * (i / 3), 148, 30);
    }
    constexpr Rect16 paint_body(int i) {
        return Rect16(13 + 58 * i, 128, 46, 46);
    }
    constexpr Rect16 weight_rect = { 12, 182, 456, 16 };
    constexpr Rect16 cancel_body = { style::margin, 226, 120, 36 };
    constexpr Rect16 primary_body = { 136, 226, 336, 36 };
    static_assert(style::inside(style::content, style::with_ring(paint_body(7))) && style::inside(style::content, style::with_ring(primary_body)));
    static_assert(style::with_ring(material_body(3)).Bottom() < style::with_ring(paint_body(0)).Top());

    enum Tag : uint8_t {
        tag_material = 0, // 0 à 5
        tag_paint = 10, // 10 à 17
        tag_cancel = 20,
        tag_primary,
    };

    uint8_t material_index(FilamentType type) {
        if (type) {
            const auto name = type.parameters().name;
            for (size_t i = 0; i < std::size(material_names); ++i) {
                if (strcmp(name.data(), material_names[i]) == 0) {
                    return i;
                }
            }
        }
        return 0; // PLA
    }

    void start_insert() {
        wizard.step = 2;
        atelier::run(Op::preload, wizard.slot); // le MMU attend le filament et l'arrête au FINDA
    }

} // namespace

void open_change_spool(uint8_t slot, SpoolChange mode) {
    const FilamentType type = FilamentType::for_tool(VirtualToolIndex::from_raw(slot));
    wizard = { .slot = slot, .mode = mode, .step = 3, .material = material_index(type), .paint = int8_t(atelier::paint_index(type)) };
    open_screen<ScreenChangeSpool>();
    switch (mode) {
    case SpoolChange::change:
        wizard.step = 1;
        atelier::run(Op::eject, slot); // SPEC_G.md §3.5 : M705 avant l'étape 1
        break;
    case SpoolChange::insert:
        start_insert();
        break;
    case SpoolChange::material:
        break;
    }
}

void open_spool_usage(uint8_t slot) {
    Screens::Access()->Open(ScreenFactory::ScreenWithArg<ScreenSpoolUsage>(slot));
}

// ---- Assistant ----

StepBand::StepBand(window_t *parent)
    : window_t(parent, band_rect) {}

void StepBand::unconditionalDraw() {
    static constexpr const char *labels[] = { N_("1 · Remove"), N_("2 · Insert"), N_("3 · Choose") };
    const Rect16 r = GetRect();
    display::fill_rect(r, style::screen);
    for (int i = 0; i < 3; ++i) {
        const int step = i + 1;
        const Rect16 pill(r.Left() + 157 * i, r.Top(), 150, r.Height());
        // En cours : orange ; faite : #303337 ; à venir : #26282B ; étapes sautées éteintes (« Matière et couleur » seule :
        // 1 et 2 ; emplacement vide : rien à retirer)
        const bool current = step == wizard.step;
        const bool skipped = (wizard.mode == SpoolChange::material && step < 3) || (wizard.mode == SpoolChange::insert && step == 1);
        const Color fill = current ? style::accent : (step < wizard.step && !skipped ? style::secondary : style::surface);
        const Color fg = current ? style::on_accent : (skipped ? style::text_faint : style::text_muted);
        display::draw_rounded_rect(pill, style::screen, fill, 12, MIC_ALL_CORNERS);
        draw_text(Rect16(pill.Left() + 4, pill.Top() + 4, pill.Width() - 8, height(Font::small)), _(labels[i]), Font::small, fg, fill, Align_t::Center());
    }
}

SpoolPicture::SpoolPicture(window_t *parent)
    : window_t(parent, picture_rect) {}

void SpoolPicture::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);

    // Étape 1 : la bobine de l'emplacement sort ; étape 2 : une bobine neuve entre
    constexpr uint8_t d = 70;
    const spool::Look look = wizard.step == 1 ? Slot::read(wizard.slot).look : spool::Look { spool::default_color("PLA"), 100 };
    draw_spool({ 110, int16_t(r.Top() + 13) }, look, d);
    draw_text(Rect16(202, r.Top() + 37, 3 * width(Font::big), height(Font::big)), string_view_utf8::MakeCPUFLASH(wizard.step == 1 ? "<<<" : ">>>"), Font::big, style::accent, style::surface);

    // Case « MMU n » (contour 2 px)
    const Rect16 box(274, r.Top() + 20, 96, 56);
    static constexpr Color box_line = Color::from_raw(0x5B5B5B);
    display::draw_rounded_rect(box, style::surface, box_line, 10, MIC_ALL_CORNERS);
    display::draw_rounded_rect(Rect16(box.Left() + 2, box.Top() + 2, box.Width() - 4, box.Height() - 4), box_line, style::surface, 8, MIC_ALL_CORNERS);
    draw_text(Rect16(box.Left() + 4, box.Top() + 6, box.Width() - 8, height(Font::small)), string_view_utf8::MakeCPUFLASH("MMU"), Font::small, style::text_muted, style::surface, Align_t::Center());
    const char number[2] = { char('1' + wizard.slot), '\0' };
    draw_text(Rect16(box.Left() + 4, box.Top() + 24, box.Width() - 8, height(Font::big)), string_view_utf8::MakeRAM(number), Font::big, style::text, style::surface, Align_t::Center());
}

PaintButton::PaintButton(window_t *parent, uint8_t index)
    : Button(parent, paint_body(index), tag_paint + index, Look::card, {}, nullptr, style::screen, 23) {}

void PaintButton::unconditionalDraw() {
    const Rect16 r = GetRect(); // 50 × 50 : anneau de focus, anneau du choix, bobine Ø40
    const uint8_t index = tag - tag_paint;
    display::fill_rect(r, style::screen);
    if (IsFocused()) {
        display::draw_rounded_rect(r, style::screen, style::focus_ring, r.Width() / 2, MIC_ALL_CORNERS);
    }
    const Rect16 inner(r.Left() + 2, r.Top() + 2, r.Width() - 4, r.Height() - 4);
    const Color ring = index == wizard.paint ? style::accent : style::screen;
    display::draw_rounded_rect(inner, IsFocused() ? style::focus_ring : style::screen, ring, inner.Width() / 2, MIC_ALL_CORNERS);
    draw_spool({ int16_t(r.Left() + 5), int16_t(r.Top() + 5) }, spool::Look { atelier::paints[index].color, 100 }, 40);
}

void DoneButton::unconditionalDraw() {
    if (wizard.step != 3) {
        Button::unconditionalDraw();
        return;
    }
    const Colors c = colors();
    const Rect16 b = body();
    draw_frame(b, c);
    // La bobine de la couleur choisie (ou de la matière), puis le texte, centrés ensemble (SPEC_G.md §3.6)
    constexpr uint8_t d = 24;
    constexpr int16_t gap = 10;
    const uint16_t text_w = label.computeNumUtf8Chars() * width(Font::small);
    const uint16_t total = std::min<uint16_t>(d + gap + text_w, b.Width() - 12);
    const int16_t x = b.Left() + (b.Width() - total) / 2;
    const Color color = wizard.paint >= 0 ? atelier::paints[wizard.paint].color : spool::default_color(material_names[wizard.material]);
    draw_spool({ x, int16_t(b.Top() + (b.Height() - d) / 2) }, spool::Look { color, 100 }, d);
    draw_text(Rect16(x + d + gap, b.Top() + (b.Height() - height(Font::small)) / 2, total - d - gap, height(Font::small)), label, Font::small, c.fg, c.fill);
}

ScreenChangeSpool::ScreenChangeSpool()
    : Screen(true)
    , header(this,
          wizard.mode == SpoolChange::material ? _("Slot %i").formatted(header_params, wizard.slot + 1) : _("New spool %d").formatted(header_params, wizard.slot + 1),
          true)
    , band(this)
    , picture(this)
    , title(this, title_rect, is_multiline::no)
    , text(this, text_rect, is_multiline::yes)
    , materials {
        { this, material_body(0), tag_material + 0, Button::Look::card, string_view_utf8::MakeCPUFLASH(material_names[0]), nullptr, style::screen, 10 },
        { this, material_body(1), tag_material + 1, Button::Look::card, string_view_utf8::MakeCPUFLASH(material_names[1]), nullptr, style::screen, 10 },
        { this, material_body(2), tag_material + 2, Button::Look::card, string_view_utf8::MakeCPUFLASH(material_names[2]), nullptr, style::screen, 10 },
        { this, material_body(3), tag_material + 3, Button::Look::card, string_view_utf8::MakeCPUFLASH(material_names[3]), nullptr, style::screen, 10 },
        { this, material_body(4), tag_material + 4, Button::Look::card, string_view_utf8::MakeCPUFLASH(material_names[4]), nullptr, style::screen, 10 },
        { this, material_body(5), tag_material + 5, Button::Look::card, string_view_utf8::MakeCPUFLASH(material_names[5]), nullptr, style::screen, 10 },
    }
    , paints { { this, 0 }, { this, 1 }, { this, 2 }, { this, 3 }, { this, 4 }, { this, 5 }, { this, 6 }, { this, 7 } }
    , weight_note(this, weight_rect, is_multiline::no, is_closed_on_click_t::no, _("Spool weight tracking: soon"))
    , cancel(this, cancel_body, tag_cancel, Button::Look::outline, _("Cancel"), nullptr, style::screen)
    , primary(this, primary_body, tag_primary, Button::Look::primary, {}, nullptr, style::screen, style::radius_big)
    , tabs(this, TabBar::Tab::workshop) {
    ClrMenuTimeoutClose(); // l'assistant attend l'utilisateur, pas un délai
    title.set_font(Font::normal);
    title.SetTextColor(style::text_strong);
    text.set_font(Font::small);
    text.SetTextColor(style::text_muted);
    weight_note.set_font(Font::small);
    weight_note.SetTextColor(style::text_muted);
    for (window_text_t *t : { &title, &text, &weight_note }) {
        t->SetBackColor(style::screen);
        t->SetPadding({ 0, 0, 0, 0 });
    }
    for (Button &m : materials) {
        m.set_font(Font::normal);
    }
    // L'écran de progression a montré le préchargement pendant que l'assistant était fermé : étape 3
    if (wizard.step == 2 && atelier::request_served(Op::preload, wizard.slot)) {
        wizard.step = 3;
    }
    layout();
}

void ScreenChangeSpool::layout() {
    const bool choose = wizard.step == 3;
    picture.set_visible(!choose);
    title.set_visible(!choose);
    text.set_visible(!choose);
    for (Button &m : materials) {
        m.set_visible(choose);
        m.set_enabled(choose);
        m.set_look(choose && m.tag - tag_material == wizard.material ? Button::Look::selected : Button::Look::card);
    }
    for (PaintButton &p : paints) {
        p.set_visible(choose);
        p.set_enabled(choose);
        p.Invalidate();
    }
    weight_note.set_visible(choose);
    band.Invalidate();
    picture.Invalidate();

    switch (wizard.step) {
    case 1:
        title.SetText(_("Remove spool %d").formatted(title_params, wizard.slot + 1));
        text.SetText(_("The MMU pulled the filament out of input %d. Remove the spool, then press.").formatted(text_params, wizard.slot + 1));
        primary.set(Button::Look::primary, _("It's done"));
        break;
    case 2:
        title.SetText(_("Insert the new spool"));
        text.SetText(_("Push the filament into input %d: the MMU grabs it by itself.").formatted(text_params, wizard.slot + 1));
        primary.set(Button::Look::primary, _("Preload"));
        primary.enable(atelier::mmu_active() && atelier::can_run());
        break;
    default: {
        char name[24];
        const FilamentType chosen(material_types[wizard.material]);
        if (wizard.paint >= 0) {
            StringViewUtf8Parameters<filament_name_buffer_size + 2> args;
            _(atelier::paints[wizard.paint].name).formatted(args, material_names[wizard.material]).copyToRAM(name, sizeof(name));
        } else {
            strlcpy(name, chosen.parameters().name.data(), sizeof(name));
        }
        strlcpy(done_name.data(), name, done_name.size());
        primary.set(Button::Look::primary, _("Done · %s").formatted(done_params, done_name.data()));
        primary.enable(true);
        break;
    }
    }

    // Focus de départ (SPEC_G.md) : le bouton principal, ou la matière choisie à l'étape 3
    if (choose) {
        materials[wizard.material].SetFocus();
    } else if (primary.IsEnabled()) {
        primary.SetFocus();
    } else {
        cancel.SetFocus();
    }
}

void ScreenChangeSpool::finish() {
    // Comme le menu Prusa « Changer tous les filaments » : préréglage + couleur = filament ad hoc de l'emplacement
    const VirtualToolIndex tool = VirtualToolIndex::from_raw(wizard.slot);
    const OptionalColor color = wizard.paint >= 0 ? OptionalColor(atelier::paints[wizard.paint].color) : OptionalColor(std::nullopt);
    config_store().set_filament_type(tool, filament::apply_load_color(FilamentType(material_types[wizard.material]), tool, color));
    close_screen();
}

void ScreenChangeSpool::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        if (wizard.step == 2 && atelier::request_served(Op::preload, wizard.slot)) {
            wizard.step = 3;
            layout();
        } else if (wizard.step == 2) {
            primary.enable(atelier::mmu_active() && atelier::can_run()); // le MMU peut redémarrer entre-temps
        }
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag < tag_material + std::size(material_types)) {
            wizard.material = tag - tag_material;
            layout();
        } else if (tag >= tag_paint && tag < tag_paint + atelier::paints.size()) {
            const int8_t paint = tag - tag_paint;
            wizard.paint = wizard.paint == paint ? -1 : paint; // un 2e clic revient à la couleur du préréglage
            layout();
            paints[paint].SetFocus();
        } else if (tag == tag_cancel) {
            close_screen();
        } else if (tag == tag_primary) {
            switch (wizard.step) {
            case 1:
                start_insert();
                layout();
                break;
            case 2:
                atelier::run(Op::preload, wizard.slot);
                break;
            default:
                finish();
                break;
            }
        }
        return;
    }

    default:
        break;
    }
    Screen::windowEvent(sender, event, param);
}

// ---- Utilisation ----

namespace {
    constexpr Rect16 usage_card_rect = { style::margin, 32, 168, 232 };
    constexpr Rect16 row_rect(int i) {
        return Rect16(184, 32 + 42 * i, 288, 38);
    }
    constexpr Rect16 note_rect = { 184, 160, 288, 48 };
    constexpr Rect16 edit_body = { 184, 226, 288, 36 };
    static_assert(style::inside(style::content, usage_card_rect) && style::inside(style::content, style::with_ring(edit_body)));
} // namespace

InfoRow::InfoRow(window_t *parent, Rect16 rect, const img::Resource &icon, const string_view_utf8 &label)
    : window_t(parent, rect)
    , icon(icon)
    , label(label) {}

void InfoRow::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::draw_rounded_rect(r, style::screen, style::surface, style::radius, MIC_ALL_CORNERS);
    const int16_t y = r.Top() + (r.Height() - 16) / 2;
    draw_icon({ int16_t(r.Left() + 12), y }, icon, style::surface);
    const uint16_t value_w = strlen(value.data()) * width(Font::small);
    const int16_t value_x = r.Left() + r.Width() - 12 - value_w;
    draw_text(Rect16(r.Left() + 36, y, value_x - 6 - (r.Left() + 36), height(Font::small)), label, Font::small, style::text, style::surface);
    draw_text(Rect16(value_x, y, value_w, height(Font::small)), string_view_utf8::MakeRAM(value.data()), Font::small, style::text_strong, style::surface);
}

UsageCard::UsageCard(window_t *parent, uint8_t slot)
    : window_t(parent, usage_card_rect)
    , slot(slot) {}

void UsageCard::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    const VirtualToolIndex tool = VirtualToolIndex::from_raw(slot);
    spool::Look look = Slot::read(slot).look;

    // Le reste n'est connu qu'avec une bobine OpenPrintTag suivie par Prusa (sinon F13, à venir)
    char weight[16] = "-";
    char detail[24] = "";
#if HAS_ANFC()
    if (const auto report = buddy::openprinttag::filament_usage_tracker().filament_report(tool); report && report->full_weight_g > 0) {
        const int percent = std::clamp(int(report->remaining_weight_g * 100 / report->full_weight_g), 0, 100);
        look.fill_percent = percent;
        snprintf(weight, sizeof(weight), "%d g", int(report->remaining_weight_g));
        if (report->g_per_mm > 0) {
            snprintf(detail, sizeof(detail), "~ %d m · %d %%", int(report->remaining_weight_g / report->g_per_mm / 1000), percent);
        }
    }
#endif
    constexpr uint8_t d = 104;
    draw_spool({ int16_t(r.Left() + (r.Width() - d) / 2), int16_t(r.Top() + 12) }, look, d);
    draw_text(Rect16(r.Left(), r.Top() + 124, r.Width(), height(Font::big)), string_view_utf8::MakeRAM(weight), Font::big, style::text, style::surface, Align_t::Center());
    draw_text(Rect16(r.Left(), r.Top() + 160, r.Width(), height(Font::small)), string_view_utf8::MakeRAM(detail), Font::small, style::text_muted, style::surface, Align_t::Center());
    char name[24];
    atelier::spool_name(name, FilamentType::for_tool(tool));
    draw_text(Rect16(r.Left(), r.Top() + 184, r.Width(), height(Font::small)), string_view_utf8::MakeRAM(name), Font::small, style::text_soft, style::surface, Align_t::Center());
}

ScreenSpoolUsage::ScreenSpoolUsage(uint8_t slot)
    : Screen(true)
    , slot(slot)
    , header(this, _("Slot %i").formatted(title_params, slot + 1), true)
    , card(this, slot)
    , material(this, row_rect(0), img::spool_16x16, _("Material"))
    , temperatures(this, row_rect(1), img::nozzle_16x16, _("Nozzle / bed"))
    , remaining(this, row_rect(2), img::warning_16x16, _("Remaining"))
    , note(this, note_rect, is_multiline::yes, is_closed_on_click_t::no, _("Remaining weight and print history per spool: soon."))
    , edit(this, edit_body, 0, Button::Look::outline, _("Material and colour"), &img::user_filament_16x16, style::screen)
    , tabs(this, TabBar::Tab::workshop) {
    note.set_font(Font::small);
    note.SetTextColor(style::text_muted);
    note.SetBackColor(style::screen);
    note.SetPadding({ 0, 0, 0, 0 });

    const FilamentType type = FilamentType::for_tool(VirtualToolIndex::from_raw(slot));
    if (type) {
        const FilamentTypeParameters p = type.parameters();
        strlcpy(material.value.data(), p.name.data(), material.value.size());
        snprintf(temperatures.value.data(), temperatures.value.size(), "%d/%d\xC2\xB0", p.nozzle_temperature, p.heatbed_temperature);
    } else {
        strlcpy(material.value.data(), "-", material.value.size());
        strlcpy(temperatures.value.data(), "-", temperatures.value.size());
    }
    _("soon").copyToRAM(remaining.value.data(), remaining.value.size());
#if HAS_ANFC()
    if (const auto report = buddy::openprinttag::filament_usage_tracker().filament_report(VirtualToolIndex::from_raw(slot))) {
        snprintf(remaining.value.data(), remaining.value.size(), "%d g", int(report->remaining_weight_g));
    }
#endif
    edit.SetFocus();
}

void ScreenSpoolUsage::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::CHILD_CLICK) {
        open_change_spool(slot, SpoolChange::material);
        return;
    }
    Screen::windowEvent(sender, event, param);
}

} // namespace od3d
