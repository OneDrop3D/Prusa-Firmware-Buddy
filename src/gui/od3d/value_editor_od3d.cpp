/// @file
#include "value_editor_od3d.hpp"

#include "widgets_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <event/knob_event.hpp>
#include <ScreenHandler.hpp>
#include <sound.hpp>

namespace od3d {

namespace {

    // Géométrie de la maquette (SPEC_G.md §1.8), en coordonnées de l'écran
    constexpr Rect16 sheet_rect = { 0, 120, 480, 200 };
    constexpr Rect16 label_rect = { 0, 134, 480, 16 };
    constexpr Rect16 minus_rect = { 22, 160, 52, 52 };
    constexpr Rect16 plus_rect = { 406, 160, 52, 52 };
    constexpr Rect16 value_rect = { 80, 159, 320, 53 };
    constexpr Rect16 bar_rect = { 40, 234, 400, 6 };
    constexpr Rect16 bar_zone = { 30, 226, 420, 22 }; ///< barre et son disque Ø18
    constexpr Rect16 ok_rect = { 140, 262, 200, 40 };
    constexpr Rect16 view_rect = { 30, 156, 420, 94 }; ///< valeur et barre ; les disques − et + n'y sont pas repeints
    constexpr uint8_t knob_d = 18;

    static_assert(style::inside(sheet_rect, view_rect) && style::inside(view_rect, value_rect) && style::inside(view_rect, bar_zone));

    void draw_round_button(Rect16 rect, const char *sign) {
        display::draw_rounded_rect(rect, style::surface, style::line, rect.Width() / 2, MIC_ALL_CORNERS);
        draw_text(rect, string_view_utf8::MakeCPUFLASH(sign), Font::big, style::text_strong, style::line, Align_t::Center());
    }

} // namespace

std::optional<float> ValueEditor::exec(const string_view_utf8 &label, float value, const NumericInputConfig &config) {
    ValueEditor editor(label, value, config);
    Screens::Access()->gui_loop_until_dialog_closed();
    return editor.accepted ? std::optional<float>(editor.value) : std::nullopt;
}

ValueEditor::ValueEditor(const string_view_utf8 &label, float value, const NumericInputConfig &config)
    : IDialog(sheet_rect)
    , label(label)
    , config(config)
    , value(config.clamp(value))
    , view(*this) {
    SetBackColor(style::surface);
}

void ValueEditor::unconditionalDraw() {
    // Coins du haut arrondis ; l'écran dessous n'est pas voilé (pas de transparence)
    display::draw_rounded_rect(GetRect(), style::screen, style::surface, 18, MIC_TOP_LEFT | MIC_TOP_RIGHT);
    draw_text(label_rect, label, Font::small, style::text_muted, style::surface, Align_t::Center());
    draw_round_button(minus_rect, "-");
    draw_round_button(plus_rect, "+");
    display::draw_rounded_rect(ok_rect, style::surface, style::accent, style::radius, MIC_ALL_CORNERS);
    draw_text(ok_rect, _("OK"), Font::small, style::on_accent, style::accent, Align_t::Center());
}

void ValueEditor::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::KNOB: {
        auto &ctx = *static_cast<GuiEventContext *>(param);
        step(ctx.event.value<gui_event::KnobEvent>().diff);
        ctx.accept();
        return;
    }
    case GUI_event_t::CLICK:
        accepted = true;
        close_dialog();
        return;
    case GUI_event_t::TOUCH_CLICK: {
        const point_ui16_t at = event_conversion_union { .pvoid = param }.point;
        if (minus_rect.Contain(at)) {
            step(-1);
        } else if (plus_rect.Contain(at)) {
            step(1);
        } else if (bar_zone.Contain(at)) {
            const float f = std::clamp((at.x - bar_rect.Left()) / float(bar_rect.Width()), 0.0f, 1.0f);
            set(config.min_value + f * (config.max_value - config.min_value));
        } else if (ok_rect.Contain(at)) {
            accepted = true;
            close_dialog();
        }
        return;
    }
    default:
        IDialog::windowEvent(sender, event, param);
        return;
    }
}

void ValueEditor::screenEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::TOUCH_SWIPE_LEFT || event == GUI_event_t::TOUCH_SWIPE_RIGHT) {
        close_dialog(); // annulé
        return;
    }
    IDialog::screenEvent(sender, event, param);
}

void ValueEditor::step(int steps) {
    // Comme WiSpin::change() : pas, puis limites de Prusa (et valeur spéciale « Arrêt »/« Auto » au-delà du minimum)
    set(config.clamp(std::round(value / config.step + float(steps)) * config.step, float(steps)));
}

void ValueEditor::set(float set) {
    const float previous = value;
    value = config.clamp(std::round(set / config.step) * config.step);
    if (value == previous) {
        sound::play(SoundType::blind_alert);
        return;
    }
    sound::play(SoundType::encoder_move);
    view.Invalidate();
}

float ValueEditor::fraction() const {
    if (config.special_value == value || config.max_value <= config.min_value) {
        return 0;
    }
    return std::clamp((value - config.min_value) / (config.max_value - config.min_value), 0.0f, 1.0f);
}

ValueEditor::ValueView::ValueView(ValueEditor &editor)
    : window_t(&editor, view_rect)
    , editor(editor) {}

void ValueEditor::ValueView::unconditionalDraw() {
    const Color bg = style::surface;

    // Valeur : chiffres en « large » (il n'a que des chiffres), unité ou texte spécial en « big » ; chaque morceau
    // peint son fond, sans effacement préalable (pas de clignotement)
    const bool special = editor.config.special_value == editor.value;
    char digits[12] = "";
    if (!special) {
        if (editor.config.max_decimal_places == 0) {
            snprintf(digits, sizeof(digits), "%ld", std::lround(editor.value));
        } else {
            snprintf(digits, sizeof(digits), "%.*f", editor.config.max_decimal_places, static_cast<double>(editor.value));
        }
    }
    const string_view_utf8 unit = special ? _(editor.config.special_value_str) : editor.config.unit_str();
    const uint16_t digits_w = strlen(digits) * width(Font::large);
    const uint16_t gap = digits_w && !unit.isNULLSTR() ? 8 : 0;
    const uint16_t unit_w = std::min<int>(unit.computeNumUtf8Chars() * width(Font::big), value_rect.Width() - digits_w - gap);
    const int16_t x = value_rect.Left() + (value_rect.Width() - digits_w - gap - unit_w) / 2;
    const int16_t top = value_rect.Top();
    const uint16_t h = value_rect.Height();

    display::fill_rect(Rect16(value_rect.Left(), top, x - value_rect.Left(), h), bg);
    if (digits_w) {
        draw_text(Rect16(x, top, digits_w, h), string_view_utf8::MakeRAM(digits), Font::large, style::text_strong, bg);
    }
    const int16_t unit_x = x + digits_w + gap;
    display::fill_rect(Rect16(x + digits_w, top, gap, h), bg);
    // L'unité s'aligne sur le haut des chiffres, le texte spécial est centré
    const int16_t unit_y = digits_w ? top + 8 : top + (h - height(Font::big)) / 2;
    display::fill_rect(Rect16(unit_x, top, unit_w, unit_y - top), bg);
    draw_text(Rect16(unit_x, unit_y, unit_w, height(Font::big)), unit, Font::big, style::text_strong, bg);
    display::fill_rect(Rect16(unit_x, unit_y + height(Font::big), unit_w, top + h - unit_y - height(Font::big)), bg);
    const int16_t end = unit_x + unit_w;
    display::fill_rect(Rect16(end, top, value_rect.Left() + value_rect.Width() - end, h), bg);

    // Barre : piste, remplissage orange, disque blanc
    const uint16_t filled = static_cast<uint16_t>(std::lround(editor.fraction() * bar_rect.Width()));
    display::fill_rect(bar_zone, bg);
    display::draw_rounded_rect(bar_rect, bg, style::line, 3, MIC_ALL_CORNERS);
    if (filled) {
        display::draw_rounded_rect(Rect16(bar_rect.Left(), bar_rect.Top(), filled, bar_rect.Height()), bg, style::accent, 3, MIC_ALL_CORNERS);
    }
    const int16_t knob_x = bar_rect.Left() + filled - knob_d / 2;
    display::draw_rounded_rect(Rect16(knob_x, bar_rect.Top() + bar_rect.Height() / 2 - knob_d / 2, knob_d, knob_d), bg, style::text_strong, knob_d / 2, MIC_ALL_CORNERS);
}

} // namespace od3d
