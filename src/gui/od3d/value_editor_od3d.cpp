/// @file
#include "value_editor_od3d.hpp"

#include "frost_od3d.hpp"
#include "print_info_od3d.hpp"
#include "screen_mpc_od3d.hpp" // MI_OD3D_NUMBER
#include "thumbnail_od3d.hpp"
#include "widgets_od3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <event/knob_event.hpp>
#include <i_window_menu.hpp>
#include <dialog_numeric_input.hpp>
#include <ScreenHandler.hpp>
#include <option/has_touch.h>
#include <sound.hpp>
#include <utility>
#if HAS_TOUCH()
    #include <hw/touchscreen/touchscreen.hpp>
#endif

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
    constexpr Rect16 drag_zone = { 0, 216, 480, 44 }; ///< où un glissement saisit la barre : plus haut que le dessin, pour le doigt
    // « Annuler » (bouton discret) et « OK » (principal) côte à côte, alignés sur la barre ; avant, seul un glissement
    // annulait : le clavier de Prusa refusait OK hors des limites et laissait l'utilisateur bloqué (BUG-6)
    constexpr Rect16 cancel_rect = { 40, 262, 194, 40 };
    constexpr Rect16 ok_rect = { 246, 262, 194, 40 };
    constexpr Rect16 view_rect = { 30, 156, 420, 94 }; ///< valeur et barre ; les disques − et + n'y sont pas repeints
    constexpr uint8_t knob_d = 18;

    /// Ce que la feuille peint elle-même, dans l'ordre de paint_part : le corps est peint autour, une fois
    constexpr Rect16 parts[] = { label_rect, minus_rect, plus_rect, value_rect, bar_zone, cancel_rect, ok_rect };
    constexpr uint8_t sheet_corners = MIC_TOP_LEFT | MIC_TOP_RIGHT;

    static_assert(style::inside(sheet_rect, view_rect) && style::inside(view_rect, value_rect) && style::inside(view_rect, bar_zone));
    static_assert(cancel_rect.Top() > drag_zone.Bottom() && ok_rect.Top() > drag_zone.Bottom(), "un toucher sur les boutons ne règle pas la barre");
    static_assert(style::inside(sheet_rect, cancel_rect) && style::inside(sheet_rect, ok_rect) && cancel_rect.Right() < ok_rect.Left());

    /// Texte centré dans rect, sur une case juste à sa taille (draw_text repeint toute sa case : pas les coins arrondis)
    void draw_centered(Rect16 rect, const string_view_utf8 &text, Font font, uint16_t text_w, Color fg, Color bg) {
        const uint16_t h = font == Font::big ? height(Font::big) : height(Font::small);
        draw_text(Rect16(rect.Left() + (rect.Width() - text_w) / 2, rect.Top() + (rect.Height() - h) / 2, text_w, h), text, font, fg, bg);
    }

    void draw_round_button(Rect16 rect, const char *sign) {
        display::draw_rounded_rect(rect, style::surface, style::line, rect.Width() / 2, MIC_ALL_CORNERS);
        draw_centered(rect, string_view_utf8::MakeCPUFLASH(sign), Font::big, width(Font::big), style::text_strong, style::line);
    }

} // namespace

// Ouvert au bout d'une longue chaîne d'événements (toucher d'une ligne WiSpin, pile de l'interface : 6 Ko) : il ne doit
// pas prendre plus de pile que le clavier numérique de Prusa qu'il remplace (UI-15)
static_assert(sizeof(ValueEditor) <= sizeof(DialogNumericInput));

Rect16 ValueEditor::row(const IWindowMenu &menu, const IWindowMenuItem &item) {
    return menu.slot_rect(menu.index_to_slot(menu.item_index(&item)).value_or(-1));
}

std::optional<float> ValueEditor::exec(const string_view_utf8 &label, float value, const NumericInputConfig &config, Apply apply, const char *unit, Rect16 from) {
    std::optional<float> chosen;
    {
        ValueEditor editor(label, value, config, apply, unit, from.IsEmpty() ? frost::origin() : from); // avant que la feuille ne prenne la molette
        loop_until_closed_or_fsm(); // I-01 : annulé si une machine à états de Prusa passe au-dessus
        if (editor.accepted) {
            chosen = editor.value;
        } else if (apply && editor.value != value) {
            apply(value); // annulé : on revient à la valeur de départ
        }
    } // la feuille est détruite : l'écran dessous peut repeindre sa place
    frost::close();
    return chosen;
}

ValueEditor::ValueEditor(const string_view_utf8 &label, float value, const NumericInputConfig &config, Apply apply, const char *unit, Rect16 from)
    : IDialog(style::below_header) // l'écran dessous reste visible, figé, au-dessus de la feuille ; l'en-tête reste vivant
    , label(label)
    , config(config)
    , value(config.clamp(value))
    , apply(apply)
    , unit(unit)
    , from(from)
    , view(*this) {
    SetBackColor(style::surface);
}

void ValueEditor::draw() {
    if (!IsVisible() || !IsInvalid()) {
        IDialog::draw(); // la valeur seule (ValueView)
        return;
    }
    // La feuille grandit depuis l'élément touché, le voile ensuite au-dessus (frost_od3d.hpp) ; à sa place ensuite
    view.forget();
    frost::open(sheet_rect, sheet_corners, std::exchange(from, sheet_rect), parts, [](void *editor, size_t i) { static_cast<ValueEditor *>(editor)->paint_part(i); }, this);
}

void ValueEditor::paint_part(size_t i) {
    switch (i) {
    case 0: {
        // Le nom seul : les libellés des listes G ont souvent une 2e ligne d'explication (« Pas\nBaisse à chaque palier »)
        std::array<char, label_rect.Width() / width(Font::small) + 1> name;
        label.copyToRAM(name.data(), name.size());
        if (char *nl = strchr(name.data(), '\n')) {
            *nl = '\0';
        }
        draw_text(label_rect, string_view_utf8::MakeRAM(name.data()), Font::small, style::text_muted, style::surface, Align_t::Center());
        break;
    }
    case 1:
        draw_round_button(minus_rect, "-");
        break;
    case 2:
        draw_round_button(plus_rect, "+");
        break;
    case 3:
        view.draw_value();
        break;
    case 4:
        view.draw_bar();
        break;
    case 5: {
        draw_rounded_frame(cancel_rect, style::surface, style::line, style::surface, style::radius, 1);
        const string_view_utf8 cancel = _("Cancel");
        draw_centered(cancel_rect, cancel, Font::small, cancel.computeNumUtf8Chars() * width(Font::small), style::text_soft, style::surface);
        break;
    }
    case 6: {
        display::draw_rounded_rect(ok_rect, style::surface, style::accent, style::radius, MIC_ALL_CORNERS);
        const string_view_utf8 ok = _("OK");
        draw_centered(ok_rect, ok, Font::small, ok.computeNumUtf8Chars() * width(Font::small), style::on_accent, style::accent);
        break;
    }
    }
}

void ValueEditor::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        follow_finger();
        IDialog::windowEvent(sender, event, param);
        return;
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
    case GUI_event_t::HELD_RELEASED: // appui long sur la molette = Annuler, comme le retour partout ailleurs
        close_dialog();
        return;
    case GUI_event_t::TOUCH_CLICK: {
        const point_ui16_t at = event_conversion_union { .pvoid = param }.point;
        if (minus_rect.Contain(at)) {
            step(-1);
        } else if (plus_rect.Contain(at)) {
            step(1);
        } else if (drag_zone.Contain(at)) {
            const float f = std::clamp((at.x - bar_rect.Left()) / float(bar_rect.Width()), 0.0f, 1.0f);
            set(config.min_value + f * (config.max_value - config.min_value));
        } else if (ok_rect.Contain(at)) {
            accepted = true;
            close_dialog();
        } else if (cancel_rect.Contain(at)) {
            close_dialog();
        }
        return;
    }
    default:
        IDialog::windowEvent(sender, event, param);
        return;
    }
}

void ValueEditor::follow_finger() {
#if HAS_TOUCH()
    const auto at = touchscreen.held_point();
    if (!at || !drag_zone.Contain(touchscreen.held_start())) {
        return;
    }
    const int16_t start_x = touchscreen.held_start().x;
    if (!dragged && std::abs(at->x - start_x) < 4) {
        return; // pas encore un glissement : un toucher reste un toucher
    }
    dragged = true;
    const float f = std::clamp((at->x - bar_rect.Left()) / float(bar_rect.Width()), 0.0f, 1.0f);
    set(config.min_value + f * (config.max_value - config.min_value), true);
#endif
}

void ValueEditor::screenEvent(window_t *sender, GUI_event_t event, void *param) {
    const bool swipe = event == GUI_event_t::TOUCH_SWIPE_LEFT || event == GUI_event_t::TOUCH_SWIPE_RIGHT
        || event == GUI_event_t::TOUCH_SWIPE_UP || event == GUI_event_t::TOUCH_SWIPE_DOWN;
    if (swipe && dragged) {
        dragged = false; // fin du glissement sur la barre : la valeur reste
        sound::play(SoundType::encoder_move);
        return;
    }
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

void ValueEditor::set(float set, bool quiet) {
    const float previous = value;
    value = config.clamp(std::round(set / config.step) * config.step);
    if (value == previous) {
        if (!quiet) {
            // Butée : audible aussi hors du mode « Assistance », le seul où blind_alert sonne
            sound::play(sound::get_mode() == SoundMode::assist ? SoundType::blind_alert : SoundType::button_echo);
        }
        return;
    }
    if (!quiet) {
        sound::play(SoundType::encoder_move);
    }
    view.Invalidate();
    if (apply) {
        apply(value);
    }
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
    draw_value();
    draw_bar();
}

void ValueEditor::ValueView::forget() {
    drawn = {};
    drawn_filled = -1;
}

void ValueEditor::ValueView::draw_value() {
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
    const string_view_utf8 unit = special ? _(editor.config.special_value_str) : (editor.unit ? string_view_utf8::MakeCPUFLASH(editor.unit) : editor.config.unit_str());
    // Chiffres en « large » ; plus de 10 caractères (valeurs brutes de capteurs) n'y tiennent pas : « big »
    const size_t len = strlen(digits);
    const Font digits_font = len * width(Font::large) + 3 * width(Font::big) <= value_rect.Width() ? Font::large : Font::big;
    const uint16_t char_w = digits_font == Font::large ? width(Font::large) : width(Font::big);
    const uint16_t digits_w = len * char_w;
    const uint16_t gap = digits_w && !unit.isNULLSTR() ? 8 : 0;
    const uint16_t unit_w = std::min<int>(unit.computeNumUtf8Chars() * width(Font::big), value_rect.Width() - digits_w - gap);
    const int16_t x = value_rect.Left() + (value_rect.Width() - digits_w - gap - unit_w) / 2;
    const int16_t top = value_rect.Top();
    const uint16_t h = value_rect.Height();

    // Autant de chiffres qu'avant : même police, même place, l'unité ne bouge pas. Seuls les chiffres changés, chacun
    // dans sa case (police à chasse fixe) : un cran de 1 °C repeint ~5 Ko au lieu de ~51 Ko
    if (len && strlen(drawn.data()) == len) {
        for (size_t i = 0; i < len; ++i) {
            if (digits[i] != drawn[i]) {
                const char one[2] = { digits[i], '\0' };
                draw_text(Rect16(int16_t(x + int(i) * char_w), top, char_w, h), string_view_utf8::MakeRAM(one), digits_font, style::text_strong, bg);
            }
        }
        strlcpy(drawn.data(), digits, drawn.size());
        return;
    }
    strlcpy(drawn.data(), digits, drawn.size());

    display::fill_rect(Rect16(value_rect.Left(), top, x - value_rect.Left(), h), bg);
    if (digits_w) {
        draw_text(Rect16(x, top, digits_w, h), string_view_utf8::MakeRAM(digits), digits_font, style::text_strong, bg);
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
}

void ValueEditor::ValueView::draw_bar() {
    const Color bg = style::surface;

    // Barre : piste, remplissage orange, disque blanc ; une image d'une passe (la zone s'effaçait à chaque pas)
    const int filled = static_cast<int>(std::lround(editor.fraction() * bar_rect.Width()));
    const int bar_x = bar_rect.Left() - bar_zone.Left(), bar_y = bar_rect.Top() - bar_zone.Top();
    const int bar_w = bar_rect.Width(), bar_h = bar_rect.Height();
    const int knob_x = bar_x + filled - knob_d / 2, knob_y = bar_y + bar_h / 2 - knob_d / 2;
    // À chaque pas, seulement les colonnes où le disque et le bout du remplissage ont bougé (~1,5 Ko au lieu de 27,7)
    int x0 = 0, x1 = bar_zone.Width();
    if (drawn_filled >= 0) {
        x0 = std::max(bar_x + std::min<int>(drawn_filled, filled) - knob_d / 2 - 1, 0);
        x1 = std::min(bar_x + std::max<int>(drawn_filled, filled) + knob_d / 2 + 1, int(bar_zone.Width()));
    }
    drawn_filled = filled;
    if (x1 <= x0) {
        return;
    }
    draw_pixels(Rect16(bar_zone.Left() + x0, bar_zone.Top(), x1 - x0, bar_zone.Height()), [&](int px, int py) {
        px += x0;
        const int kx = 2 * (px - knob_x) + 1 - knob_d, ky = 2 * (py - knob_y) + 1 - knob_d;
        if (kx * kx + ky * ky < knob_d * knob_d) {
            return style::text_strong;
        }
        const int by = py - bar_y, bx = px - bar_x;
        if (by < 0 || by >= bar_h) {
            return bg;
        }
        const int inset = print_info::corner_inset(by, bar_h, bar_h / 2);
        if (bx < inset || bx >= bar_w - inset) {
            return bg;
        }
        return bx < filled - inset ? style::accent : style::line;
    });
}

// ---- Réglage entier (MI_OD3D_NUMBER) : ici, compilé partout ; l'écran du MPC (pas sur l'INDX) et la veille s'en servent ----

MI_OD3D_NUMBER::MI_OD3D_NUMBER(const string_view_utf8 &label, const img::Resource *icon, const char *editor_label, const NumericInputConfig &config, const char *unit, Get get, Set set)
    // Sans icône au constructeur : WI_LAMBDA_LABEL_t réserverait alors à la valeur la place d'une icône
    : WI_LAMBDA_LABEL_t(label, nullptr, is_enabled_t::yes, is_hidden_t::no, [this](const std::span<char> &buffer) {
        snprintf(buffer.data(), buffer.size(), "%u %s", unsigned(shown), this->unit);
    })
    , editor_label(editor_label)
    , config(config)
    , unit(unit)
    , get(get)
    , set(set)
    , shown(get()) {
    SetIconId(icon);
}

void MI_OD3D_NUMBER::click(IWindowMenu &menu) {
    if (const auto v = ValueEditor::exec(_(editor_label), get(), config, nullptr, unit, ValueEditor::row(menu, *this))) {
        set(static_cast<uint8_t>(std::lround(config.clamp(*v))));
        Loop();
    }
}

void MI_OD3D_NUMBER::Loop() {
    if (const uint8_t now = get(); now != shown) {
        shown = now;
        InValidateExtension();
    }
}

} // namespace od3d
