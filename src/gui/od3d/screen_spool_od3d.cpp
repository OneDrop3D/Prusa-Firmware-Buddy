/// @file
#include "screen_spool_od3d.hpp"

#include "thumbnail_od3d.hpp"
#include "value_editor_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <config_store/store_instance.hpp>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <feature/od3d/e_jump_od3d.hpp>
#include <feature/od3d/pa_od3d.hpp>
#include <feature/od3d/spool_calib_od3d.hpp>
#include <feature/od3d/spool_usage_od3d.hpp>
#include <filament_to_load.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <iterator>

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
        uint16_t net_g = 1000; ///< poids net de la bobine neuve (F13)
    };
    Wizard wizard;

    // Matières de l'étape 3 (SPEC_G.md §3.6) ; le TPU s'appelle FLEX chez Prusa
    constexpr PresetFilamentType material_types[] = {
        PresetFilamentType::PLA, PresetFilamentType::PETG, PresetFilamentType::ASA,
        PresetFilamentType::PC, PresetFilamentType::FLEX, PresetFilamentType::PVB
    };
    constexpr const char *material_names[] = { "PLA", "PETG", "ASA", "PC", "FLEX", "PVB" };
    static_assert(std::size(material_types) == std::size(material_names));

    // Géométrie (SPEC_G.md §3.5, §3.6) sous l'en-tête de 40 px, boutons du bas communs aux 3 étapes
    constexpr Rect16 band_rect = { style::margin, style::top, 464, 24 };
    constexpr Rect16 picture_rect = { style::margin, 75, 464, 92 };
    constexpr Rect16 title_rect = { 12, 172, 456, 19 };
    constexpr Rect16 text_rect = { 12, 194, 456, 34 };
    constexpr Rect16 material_body(int i) {
        return Rect16(style::margin + 156 * (i % 3), 75 + 34 * (i / 3), 148, 30);
    }
    constexpr Rect16 paint_body(int i) {
        return Rect16(13 + 58 * i, 143, 46, 46);
    }
    // Poids neuf (F13) : 250 g à 2 kg, puis « Autre » (valeur libre), sur toute la largeur
    constexpr uint8_t weight_count = spool_usage::net_presets.size() + 1;
    constexpr Rect16 weight_body(int i) {
        return Rect16(10 + 78 * i, 196, 70, 30);
    }
    constexpr const char *weight_labels[] = { "250 g", "500 g", "750 g", "1 kg", "2 kg" };
    static_assert(std::size(weight_labels) == spool_usage::net_presets.size());
    constexpr Rect16 cancel_body = { style::margin, 236, 120, 36 };
    constexpr Rect16 primary_body = { 136, 236, 336, 36 };
    static_assert(band_rect.Bottom() < picture_rect.Top() && picture_rect.Bottom() < title_rect.Top() && text_rect.Bottom() < style::with_ring(cancel_body).Top());
    static_assert(style::with_ring(paint_body(0)).Bottom() < style::with_ring(weight_body(0)).Top() && style::with_ring(weight_body(0)).Bottom() < style::with_ring(cancel_body).Top());
    static_assert(style::inside(style::content, style::with_ring(weight_body(weight_count - 1))));
    static_assert(style::inside(style::content, style::with_ring(paint_body(7))) && style::inside(style::content, style::with_ring(primary_body)));
    static_assert(style::with_ring(material_body(3)).Bottom() < style::with_ring(paint_body(0)).Top());

    enum Tag : uint8_t {
        tag_material = 0, // 0 à 5
        tag_paint = 10, // 10 à 17
        tag_cancel = 20,
        tag_primary,
        tag_weight = 30, // 30 à 35, le dernier : valeur libre
    };

    /// Le poids est-il l'un des choix fixes ? son rang, sinon le bouton « Autre »
    uint8_t weight_index(uint16_t net_g) {
        for (uint8_t i = 0; i < spool_usage::net_presets.size(); ++i) {
            if (spool_usage::net_presets[i] == net_g) {
                return i;
            }
        }
        return weight_count - 1;
    }

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
    // Poids proposé : celui de la bobine suivie, sinon 1 kg
    const spool_usage::Record usage = spool_usage::record(slot);
    const uint16_t net_g = spool_usage::is_tracked(usage) ? usage.net_g : 1000;
    wizard = { .slot = slot, .mode = mode, .step = 3, .material = material_index(type), .paint = int8_t(atelier::paint_index(type)), .net_g = net_g };
    open_screen<ScreenChangeSpool>();
    switch (mode) {
    case SpoolChange::change:
        wizard.step = 1;
        // Le filament sort par l'arrière, côté bobine (retour de l'utilisateur, 21/11 : l'éjection M705 le poussait
        // toujours vers l'avant). Chargé dans la buse : on le décharge jusqu'au MMU, galet libéré ; sinon il y est
        // déjà, et on le tire à la main par l'entrée de l'emplacement.
        if (Slot::read(slot).loaded) {
            atelier::run(Op::unload, slot);
        }
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
    for (int i = 0; i < 3; ++i) {
        const int step = i + 1;
        const Rect16 pill(r.Left() + 157 * i, r.Top(), 150, r.Height());
        if (i) {
            display::fill_rect(Rect16(pill.Left() - 7, r.Top(), 7, r.Height()), style::screen); // l'écart avant la pastille
        }
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
    draw_spool({ 110, int16_t(r.Top() + 11) }, look, d, style::surface);
    draw_text(Rect16(202, r.Top() + 35, 3 * width(Font::big), height(Font::big)), string_view_utf8::MakeCPUFLASH(wizard.step == 1 ? "<<<" : ">>>"), Font::big, style::accent, style::surface);

    // Case « MMU n » (contour 2 px, d'une passe : les coins suivent l'arrondi)
    const Rect16 box(274, r.Top() + 18, 96, 56);
    static constexpr Color box_line = style::handle;
    draw_frame(style::with_ring(box), style::surface, box_line, style::surface, 10, 2);
    draw_text(Rect16(box.Left() + 4, box.Top() + 6, box.Width() - 8, height(Font::small)), string_view_utf8::MakeCPUFLASH("MMU"), Font::small, style::text_muted, style::surface, Align_t::Center());
    const char number[2] = { char('1' + wizard.slot), '\0' };
    draw_text(Rect16(box.Left() + 4, box.Top() + 24, box.Width() - 8, height(Font::big)), string_view_utf8::MakeRAM(number), Font::big, style::text, style::surface, Align_t::Center());
}

PaintButton::PaintButton(window_t *parent, uint8_t index)
    : Button(parent, paint_body(index), tag_paint + index, Look::card, {}, nullptr, style::screen, 23) {}

void PaintButton::unconditionalDraw() {
    // 50 × 50 : bobine Ø40, anneau orange du choix jusqu'à Ø46, et au focus un disque plus clair derrière (plus d'anneau
    // blanc) ; une image d'une passe
    const Rect16 r = GetRect();
    if (noted(r)) {
        return; // une image d'une passe : un seul morceau (Button::draw)
    }
    const uint8_t index = tag - tag_paint;
    const Color halo = IsFocused() ? style::focused(style::surface) : style::screen;
    const Color ring = index == wizard.paint ? style::accent : halo;
    const spool::Disc disc(spool::Look { atelier::paints[index].color, 100 }, 40);
    const int outer = r.Width(); // bobine Ø40 au centre (décalée de 5 px), anneau jusqu'à Ø46, halo jusqu'au bord
    draw_pixels(r, [&](int x, int y) {
        const float rad = spool::radius_in_square(x, y, outer);
        const Color around = spool::edge_color(rad, outer / 2.0f - 2, ring, spool::edge_color(rad, outer / 2.0f, halo, style::screen));
        return disc.at_radius(rad, around);
    });
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
    draw_spool({ x, int16_t(b.Top() + (b.Height() - d) / 2) }, spool::Look { color, 100 }, d, c.fill);
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
    , weights {
        { this, weight_body(0), tag_weight + 0, Button::Look::card, string_view_utf8::MakeCPUFLASH(weight_labels[0]), nullptr, style::screen, 15 },
        { this, weight_body(1), tag_weight + 1, Button::Look::card, string_view_utf8::MakeCPUFLASH(weight_labels[1]), nullptr, style::screen, 15 },
        { this, weight_body(2), tag_weight + 2, Button::Look::card, string_view_utf8::MakeCPUFLASH(weight_labels[2]), nullptr, style::screen, 15 },
        { this, weight_body(3), tag_weight + 3, Button::Look::card, string_view_utf8::MakeCPUFLASH(weight_labels[3]), nullptr, style::screen, 15 },
        { this, weight_body(4), tag_weight + 4, Button::Look::card, string_view_utf8::MakeCPUFLASH(weight_labels[4]), nullptr, style::screen, 15 },
        { this, weight_body(5), tag_weight + 5, Button::Look::card, {}, nullptr, style::screen, 15 },
    }
    , cancel(this, cancel_body, tag_cancel, Button::Look::outline, _("Cancel"), nullptr, style::screen)
    , primary(this, primary_body, tag_primary, Button::Look::primary, {}, nullptr, style::screen, style::radius_big)
    , tabs(this, TabBar::Tab::workshop) {
    ClrMenuTimeoutClose(); // l'assistant attend l'utilisateur, pas un délai
    title.set_font(Font::normal);
    title.SetTextColor(style::text_strong);
    text.set_font(Font::small);
    text.SetTextColor(style::text_muted);
    for (window_text_t *t : { &title, &text }) {
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
    // Poids : le choix courant en brun et orange ; « Autre » montre la valeur libre choisie
    const uint8_t weight = weight_index(wizard.net_g);
    if (weight == weight_count - 1) {
        spool_usage::format_weight(other_weight.data(), other_weight.size(), wizard.net_g);
    } else {
        _("Other").copyToRAM(other_weight.data(), other_weight.size());
    }
    for (Button &w : weights) {
        const uint8_t i = w.tag - tag_weight;
        w.set_visible(choose);
        w.set_enabled(choose);
        const Button::Look look = choose && i == weight ? Button::Look::selected : Button::Look::card;
        if (i == weight_count - 1) {
            w.set(look, string_view_utf8::MakeRAM(other_weight.data()));
        } else {
            w.set_look(look);
        }
    }
    band.Invalidate();
    picture.Invalidate();

    switch (wizard.step) {
    case 1:
        title.SetText(_("Remove spool %d").formatted(title_params, wizard.slot + 1));
        text.SetText(_("Pull the filament out of input %d at the back of the MMU, remove the spool, then press.").formatted(text_params, wizard.slot + 1));
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
        char weight_text[12];
        spool_usage::format_weight(weight_text, sizeof(weight_text), wizard.net_g);
        snprintf(done_name.data(), done_name.size(), "%s · %s", name, weight_text);
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
    // F4, F5, F6, M2000 : PA, débit maximal, température minimale et saut de l'extrudeur appartiennent à la bobine.
    // Bobine neuve : effacés ; « Matière et couleur » seul : effacés si la matière change (la couleur ne les change pas)
    const FilamentType before = FilamentType::for_tool(tool);
    if (wizard.mode != SpoolChange::material || !before || strcmp(before.parameters().name.data(), material_names[wizard.material]) != 0) {
        pa::clear(wizard.slot);
        calib::clear(wizard.slot);
        e_jump::clear(wizard.slot);
    }
    const OptionalColor color = wizard.paint >= 0 ? OptionalColor(atelier::paints[wizard.paint].color) : OptionalColor(std::nullopt);
    config_store().set_filament_type(tool, filament::apply_load_color(FilamentType(material_types[wizard.material]), tool, color));
    // F13 : bobine neuve = compteur remis à zéro, reste = poids neuf ; « Matière et couleur » seul garde le reste
    // (au plus le nouveau poids net)
    if (wizard.mode == SpoolChange::material) {
        spool_usage::set_net(wizard.slot, wizard.net_g);
    } else {
        spool_usage::new_spool(wizard.slot, wizard.net_g);
    }
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
        } else if (tag >= tag_weight && tag < tag_weight + weight_count) {
            const uint8_t i = tag - tag_weight;
            if (i < spool_usage::net_presets.size()) {
                wizard.net_g = spool_usage::net_presets[i];
            } else {
                // Valeur libre, avec l'éditeur commun
                static constexpr NumericInputConfig config = {
                    .min_value = spool_usage::min_net_g,
                    .max_value = spool_usage::max_net_g,
                    .step = 10,
                };
                if (const auto v = ValueEditor::exec(_("New spool net weight"), wizard.net_g, config, nullptr, "g")) {
                    wizard.net_g = uint16_t(*v);
                }
            }
            layout();
            weights[i].SetFocus();
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
    constexpr Rect16 usage_card_rect = { style::margin, style::top, 168, 226 };
    constexpr Rect16 row_body(int i) {
        return Rect16(186, style::top + 42 * i, 284, 36);
    }
    constexpr Rect16 note_rect = { 186, 176, 284, 48 };
    constexpr Rect16 full_body = { 186, 236, 284, 36 };
    static_assert(style::with_ring(row_body(2)).Bottom() < note_rect.Top() && note_rect.Bottom() < style::with_ring(full_body).Top());
    static_assert(style::inside(style::content, usage_card_rect) && style::inside(style::content, style::with_ring(full_body)));
    static_assert(style::inside(style::content, style::with_ring(row_body(0))));

    enum UsageTag : uint8_t {
        tag_net,
        tag_remaining,
        tag_alert,
        tag_full,
    };

    /// Valeur d'une ligne ; la ligne n'est redessinée que si elle change
    void set_value(ValueRow &row, const char *text) {
        if (strcmp(row.value.data(), text) != 0) {
            strlcpy(row.value.data(), text, row.value.size());
            row.Invalidate();
        }
    }

    void set_grams(ValueRow &row, float grams) {
        char text[16];
        snprintf(text, sizeof(text), "%d g", int(std::lround(grams)));
        set_value(row, text);
    }
} // namespace

ValueRow::ValueRow(window_t *parent, Rect16 body, uint8_t tag, const img::Resource &icon, const string_view_utf8 &label)
    : Button(parent, body, tag, Look::card, label, &icon, style::screen) {}

void ValueRow::unconditionalDraw() {
    const Colors c = colors();
    const Rect16 b = body();
    draw_frame(b, c);
    const bool enabled = IsEnabled();
    const int16_t y = b.Top() + (b.Height() - height(Font::small)) / 2;
    if (icon) {
        draw_icon({ int16_t(b.Left() + 12), int16_t(b.Top() + (b.Height() - icon->h) / 2) }, *icon, c.fill, enabled ? ropfn {} : ropfn { .shadow = is_shadowed::yes });
    }
    const uint16_t value_w = string_view_utf8::MakeRAM(value.data()).computeNumUtf8Chars() * width(Font::small); // « Jiné » : 4 caractères, 5 octets
    const int16_t value_x = b.Left() + b.Width() - 12 - value_w;
    const int16_t label_x = b.Left() + 36;
    draw_text(Rect16(label_x, y, std::max(0, value_x - 6 - label_x), height(Font::small)), label, Font::small, c.fg, c.fill);
    if (value_w) {
        draw_text(Rect16(value_x, y, value_w, height(Font::small)), string_view_utf8::MakeRAM(value.data()), Font::small, enabled ? style::text_strong : style::text_faint, c.fill);
    }
}

UsageCard::UsageCard(window_t *parent, uint8_t slot)
    : window_t(parent, usage_card_rect)
    , slot(slot) {}

void UsageCard::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    const VirtualToolIndex tool = VirtualToolIndex::from_raw(slot);
    const spool::Look look = Slot::read(slot).look; // remplissage réel (F13 ou étiquette)

    char weight[16] = "-";
    char detail[24] = "";
    Color weight_color = style::text;
    const auto usage = spool_usage::report(slot);
    if (usage) {
        snprintf(weight, sizeof(weight), "%d g", int(std::lround(usage->remaining_g)));
        if (usage->g_per_mm > 0) {
            snprintf(detail, sizeof(detail), "~ %d m · %d %%", int(usage->remaining_g / usage->g_per_mm / 1000), usage->fill_percent());
        }
        if (usage->low()) {
            weight_color = style::warning;
        }
    }
    constexpr uint8_t d = 104;
    draw_spool({ int16_t(r.Left() + (r.Width() - d) / 2), int16_t(r.Top() + 12) }, look, d, style::surface);
    draw_text(Rect16(r.Left(), r.Top() + 124, r.Width(), height(Font::big)), string_view_utf8::MakeRAM(weight), Font::big, weight_color, style::surface, Align_t::Center());
    draw_text(Rect16(r.Left(), r.Top() + 160, r.Width(), height(Font::small)), string_view_utf8::MakeRAM(detail), Font::small, style::text_muted, style::surface, Align_t::Center());
    char name[24];
    atelier::spool_name(name, FilamentType::for_tool(tool));
    draw_text(Rect16(r.Left(), r.Top() + 184, r.Width(), height(Font::small)), string_view_utf8::MakeRAM(name), Font::small, style::text_soft, style::surface, Align_t::Center());
    if (usage && usage->low()) {
        draw_text(Rect16(r.Left(), r.Top() + 204, r.Width(), height(Font::small)), _("Running low"), Font::small, style::warning, style::surface, Align_t::Center());
    }
}

ScreenSpoolUsage::ScreenSpoolUsage(uint8_t slot)
    : Screen(true)
    , slot(slot)
    , header(this, _("Slot %i").formatted(title_params, slot + 1), true)
    , card(this, slot)
    , net(this, row_body(0), tag_net, img::spool_16x16, _("Net weight (new)"))
    , remaining(this, row_body(1), tag_remaining, img::spool_16x16, _("Remaining"))
    , alert(this, row_body(2), tag_alert, img::warning_16x16, _("Low-spool alert"))
    , note(this, note_rect, is_multiline::yes, is_closed_on_click_t::no)
    , full(this, full_body, tag_full, Button::Look::outline, _("Full spool (100%)"), nullptr, style::screen)
    , tabs(this, TabBar::Tab::workshop) {
    note.set_font(Font::small);
    note.SetTextColor(style::text_soft);
    note.SetBackColor(style::screen);
    note.SetPadding({ 0, 0, 0, 0 });
    refresh();
    // Focus de départ (SPEC_G.md §3.7) : le reste, à corriger après une pesée
    if (remaining.IsEnabled()) {
        remaining.SetFocus();
    } else if (net.IsEnabled()) {
        net.SetFocus();
    } else {
        full.SetFocus(); // inactif aussi (étiquette) : le cadre a quand même un élément ciblé
    }
}

void ScreenSpoolUsage::refresh() {
    const auto usage = spool_usage::report(slot);
    const spool_usage::Record rec = spool_usage::record(slot);
    const bool tag = usage && usage->from_tag;
    const bool tracked = usage && !tag;

    if (usage) {
        set_grams(net, usage->net_g);
        set_grams(remaining, usage->remaining_g);
    } else {
        set_value(net, "-");
        set_value(remaining, "-");
    }
    if (tag) {
        set_value(alert, "-");
    } else {
        set_grams(alert, rec.alert_g);
    }
    shown_g = usage ? int(std::lround(usage->remaining_g)) : -1;

    net.enable(!tag);
    remaining.enable(tracked);
    alert.enable(tracked);
    full.enable(tracked);
    card.Invalidate();

    // Consommations, ou pourquoi il n'y en a pas
    StringViewUtf8Parameters<8> params;
    if (tag) {
        _("Data from the NFC tag (OpenPrintTag).").copyToRAM(note_text.data(), note_text.size());
    } else if (tracked) {
        _("Used since loaded: %d g").formatted(params, int(std::lround(usage->used_g))).copyToRAM(note_text.data(), note_text.size());
        size_t len = strlen(note_text.data());
        if (len + 2 < note_text.size()) {
            note_text[len++] = '\n';
            StringViewUtf8Parameters<8> params2;
            _("Last print: %d g").formatted(params2, int(std::lround(usage->last_print_g))).copyToRAM(note_text.data() + len, note_text.size() - len);
        }
    } else {
        _("Set the net weight of the spool to start tracking it.").copyToRAM(note_text.data(), note_text.size());
    }
    note.SetText(string_view_utf8::MakeRAM(note_text.data()));
    note.Invalidate();
}

void ScreenSpoolUsage::edit(uint8_t tag) {
    const spool_usage::Record rec = spool_usage::record(slot);
    const auto usage = spool_usage::report(slot);
    switch (tag) {
    case tag_net: {
        static constexpr NumericInputConfig config = {
            .min_value = spool_usage::min_net_g,
            .max_value = spool_usage::max_net_g,
            .step = 10,
        };
        if (const auto v = ValueEditor::exec(_("Net weight (new)"), spool_usage::is_tracked(rec) ? rec.net_g : 1000, config, nullptr, "g")) {
            spool_usage::set_net(slot, uint16_t(*v));
        }
        break;
    }
    case tag_remaining: {
        if (!usage) {
            return;
        }
        const NumericInputConfig config = {
            .max_value = usage->net_g,
            .step = 10,
        };
        if (const auto v = ValueEditor::exec(_("Remaining"), usage->remaining_g, config, nullptr, "g")) {
            spool_usage::set_remaining(slot, *v);
        }
        break;
    }
    case tag_alert: {
        static constexpr NumericInputConfig config = {
            .max_value = spool_usage::max_alert_g,
            .step = 10,
        };
        if (const auto v = ValueEditor::exec(_("Low-spool alert"), rec.alert_g, config, nullptr, "g")) {
            spool_usage::set_alert(slot, uint16_t(*v));
        }
        break;
    }
    case tag_full:
        if (usage) {
            spool_usage::set_remaining(slot, usage->net_g);
        }
        break;
    default:
        return;
    }
    refresh();
}

void ScreenSpoolUsage::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::CHILD_CLICK:
        edit(static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param)));
        return;
    case GUI_event_t::LOOP:
        // Pendant une impression, le reste baisse : redessiné quand il change d'un gramme
        if (const auto usage = spool_usage::report(slot); usage && int(std::lround(usage->remaining_g)) != shown_g) {
            refresh();
        }
        break;
    default:
        break;
    }
    Screen::windowEvent(sender, event, param);
}

} // namespace od3d
