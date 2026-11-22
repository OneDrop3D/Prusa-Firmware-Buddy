/// @file
#include "screen_temperatures_od3d.hpp"

#include "menu_od3d.hpp"
#include "thumbnail_od3d.hpp"
#include "value_editor_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <common/conversions.hpp>
#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <feature/chamber/chamber.hpp>
#include <feature/od3d/chamber_climate_od3d.hpp>
#include <feature/xbuddy_extension/xbuddy_extension.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <numeric_input_config_common.hpp>
#include <option/has_chamber_api.h>
#include <option/has_chamber_filtration_api.h>
#include <option/xbuddy_extension_variant.h>
#include <screen/filament/screen_filament_management_list.hpp>
#include <timing.h>
#include <tool_index.hpp>

static_assert(HAS_CHAMBER_API() && HAS_CHAMBER_FILTRATION_API() && XBUDDY_EXTENSION_VARIANT_IS_STANDARD(), "Températures OneDrop3D : CORE One");

namespace od3d {

namespace {

    // Géométrie (SPEC_G.md §3.15) : 3 colonnes de 149 px ; « Préréglages » ajouté entre les deux boutons du bas
    constexpr Rect16 heater_body(int i) {
        return Rect16(style::margin + i * 157, style::top, 149, 122);
    }
    constexpr Rect16 fan_body(int i) {
        return Rect16(style::margin + i * 157, 174, 149, 42);
    }
    constexpr Rect16 preheat_body = { 8, 222, 214, 50 };
    constexpr Rect16 presets_body = { 228, 222, 124, 50 };
    constexpr Rect16 cooldown_body = { 358, 222, 114, 50 };

    static_assert(style::inside(style::content, style::with_ring(heater_body(2))) && style::inside(style::content, style::with_ring(cooldown_body)));
    static_assert(heater_body(0).Bottom() + 2 * style::ring < fan_body(0).Top() && fan_body(0).Bottom() + 2 * style::ring < preheat_body.Top());
    static_assert(preheat_body.Right() + 1 + 6 == presets_body.Left() && presets_body.Right() + 1 + 6 == cooldown_body.Left() && cooldown_body.Right() + 1 == style::screen_w - style::margin);

    constexpr Color text_dim = Color::from_raw(0x7D8188); ///< « stable »
    constexpr Color target_line = Color::from_raw(0x5B5F66); ///< cible en pointillés sur les courbes
    constexpr uint32_t sample_ms = 2000;

    const img::Resource *const heater_icons[] = { &img::nozzle_16x16, &img::heatbed_16x16, &img::enclosure_16x16 };
    const char *const heater_labels[] = { N_("Nozzle"), N_("Bed"), N_("Chamber") };

    /// « 230° », « --° » sans mesure
    void degrees(char (&out)[8], int16_t value) {
        if (value < 0) {
            strlcpy(out, "--\xC2\xB0", sizeof(out));
        } else {
            snprintf(out, sizeof(out), "%d\xC2\xB0", std::min<int>(value, 999));
        }
    }

    // ---- Éditeurs des cibles (limites de Prusa : numeric_input_config_common) ----

    void edit_nozzle() {
        const auto tool = PhysicalToolIndex::currently_selected_opt();
        if (!tool) {
            return; // pas d'outil actif : comme le menu Prusa, rien à régler
        }
        NumericInputConfig config = numeric_input_config::nozzle_temperature(*tool);
        config.step = 5;
        if (const auto v = ValueEditor::exec(_("Nozzle"), marlin_vars().hotend(*tool).target_nozzle.get(), config)) {
            marlin_client::set_target_nozzle(static_cast<int16_t>(*v), *tool);
        }
    }

    void edit_bed() {
        NumericInputConfig config = numeric_input_config::bed_temperature;
        config.step = 5;
        if (const auto v = ValueEditor::exec(_("Bed"), marlin_vars().target_bed.get(), config)) {
            marlin_client::set_target_bed(static_cast<int16_t>(*v));
        }
    }

    void edit_chamber() {
        auto &chamber = buddy::chamber();
        if (!chamber.capabilities().temperature_control_at_this_moment()) {
            return; // ventilateurs de la chambre en manuel : la cible n'agit pas (le menu Prusa grise aussi la ligne)
        }
        const NumericInputConfig &config = numeric_input_config::chamber_temp_with_off();
        const float current = static_cast<float>(chamber.effective_target_temperature().value_or(0));
        if (const auto v = ValueEditor::exec(_("Chamber"), current, config)) {
            marlin_client::inject(GCodeLiteral("M141 S%f", *v)); // comme MI_CHAMBER_TARGET_TEMP
        }
    }

    void edit_print_fan() {
        static constexpr NumericInputConfig config = {
            .max_value = 100,
            .step = 5,
            .special_value = 0,
            .unit = Unit::percent,
        };
        if (const auto v = ValueEditor::exec(_("Part fan"), val_mapping(false, marlin_vars().print_fan_speed, 255, 100), config)) {
            marlin_client::set_fan_speed(val_mapping(true, static_cast<uint8_t>(*v), 100, 255)); // comme MI_PRINTFAN
        }
    }

    void edit_chamber_fans() {
        // Comme MI_XBUDDY_EXTENSION_COOLING_FANS : la valeur spéciale est le mode automatique
        auto &exb = buddy::xbuddy_extension();
        if (exb.status() == buddy::XBuddyExtension::Status::disabled) {
            return;
        }
        using XBE = buddy::XBuddyExtension;
        NumericInputConfig config = exb.using_filtration_fan_instead_of_cooling_fans() ? numeric_input_config::percent_with_disabled : numeric_input_config::percent_with_auto;
        config.step = 5;
        const float current = exb.fan_target_pwm(XBE::Fan::cooling_fan_1).transform(XBE::FanPWM::to_percent_static).value_or(static_cast<int8_t>(*config.special_value));
        if (const auto v = ValueEditor::exec(_("Chamber fans"), current, config)) {
            XBE::FanPWMOrAuto target = pwm_auto;
            if (*v != *config.special_value) {
                target = XBE::FanPWM::from_percent(static_cast<int8_t>(*v));
            }
            exb.set_fan_target_pwm(XBE::Fan::cooling_fan_1, target);
        }
    }

    /// Les cartes de la chambre sont grisées quand elles ne peuvent rien régler (leur toucher ne faisait rien)
    bool chamber_settable() {
        return buddy::chamber().capabilities().temperature_control_at_this_moment();
    }
    bool chamber_fans_settable() {
        return buddy::xbuddy_extension().status() != buddy::XBuddyExtension::Status::disabled;
    }

    void toggle_filtration() {
        // « Filtration pendant l'impression » de Réglages › Filtration de la chambre
        config_store().chamber_print_filtration_enable.set(!config_store().chamber_print_filtration_enable.get());
    }

    void preheat() {
        marlin_client::gcode("M1700 T-1"); // le préchauffage de Prusa (liste des filaments), comme son accueil
    }

    void open_presets() {
        open_screen<ScreenFilamentManagementList>(); // filaments de Prusa : buse, plateau, chambre, filtration...
    }

    void cooldown() {
        // Comme MI_TEMPERATURE_AND_FANS_COOLDOWN de Prusa
        for (const auto tool : PhysicalToolIndex::all()) {
            marlin_client::set_target_nozzle(0, tool);
        }
        marlin_client::set_target_bed(0);
        marlin_client::set_fan_speed(0);
    }

    constexpr Card::Action heater_actions[] = { edit_nozzle, edit_bed, edit_chamber };
    constexpr Card::Action fan_actions[] = { edit_print_fan, edit_chamber_fans, toggle_filtration };

    /// Courbe de la dernière minute : trait orange 2 px, cible en pointillés. Une image d'une passe : la courbe ne
    /// s'efface pas avant d'être redessinée (elle clignotait toutes les 2 s)
    void draw_graph(Rect16 r, const int16_t *values, size_t count, int16_t target, Color bg) {
        int16_t lo = *std::min_element(values, values + count);
        int16_t hi = *std::max_element(values, values + count);
        if (target > 0) {
            lo = std::min(lo, target);
            hi = std::max(hi, target);
        }
        if (lo < 0) {
            display::fill_rect(r, bg); // pas de mesure
            return;
        }
        if (hi - lo < 10) { // au moins 10 °C de haut : une courbe stable reste plate
            const int16_t mid = (hi + lo) / 2;
            lo = mid - 5;
            hi = mid + 5;
        }
        const int h = r.Height() - 2;
        const auto y_of = [&](int16_t v) { return h - (v - lo) * h / (hi - lo); };
        const auto x_of = [&](size_t i) { return int(i * (r.Width() - 1) / (count - 1)); };

        // Haut et bas de la courbe dans chaque colonne, segment par segment (comme draw_line)
        constexpr int max_w = 128;
        std::array<std::pair<int8_t, int8_t>, max_w> spans;
        spans.fill({ int8_t(h + 2), int8_t(-1) });
        for (size_t i = 1; i < count; ++i) {
            const int xa = x_of(i - 1), xb = x_of(i), ya = y_of(values[i - 1]), yb = y_of(values[i]);
            for (int x = xa; x <= xb && x < max_w; ++x) {
                const auto y_at = [&](int px) { return xb == xa ? yb : ya + (yb - ya) * (std::min(px, xb) - xa) / (xb - xa); };
                const int y0 = std::min(y_at(x), y_at(x + 1)), y1 = std::max(y_at(x), y_at(x + 1));
                spans[x] = { int8_t(std::min<int>(spans[x].first, y0)), int8_t(std::max<int>(spans[x].second, y1)) };
            }
        }
        const int target_y = target > 0 ? y_of(target) : -1;
        draw_pixels(r, [&](int x, int y) {
            if (x < max_w && y >= spans[x].first && y <= spans[x].second + 1) {
                return style::accent; // 2 px d'épaisseur
            }
            return (y == target_y && x % 4 < 2) ? target_line : bg;
        });
    }

} // namespace

// ---- Buse, plateau, chambre ----

HeaterCard::HeaterCard(window_t *parent, Heater heater)
    : Card(parent, style::with_ring(heater_body(int(heater))), heater_actions[int(heater)], style::surface, style::screen, style::radius_big)
    , heater(heater)
    , reading(read(heater))
    , last_sample_ms(ticks_ms()) {
    history.fill(reading.current);
    set_enabled(heater != Heater::chamber || chamber_settable());
}

HeaterCard::Reading HeaterCard::read(Heater heater) {
    Reading r;
    switch (heater) {
    case Heater::nozzle:
        r.current = static_cast<int16_t>(std::lround(marlin_vars().active_hotend().temp_nozzle.get()));
        r.target = marlin_vars().active_hotend().target_nozzle.get();
        break;
    case Heater::bed:
        r.current = static_cast<int16_t>(std::lround(marlin_vars().temp_bed.get()));
        r.target = static_cast<int16_t>(marlin_vars().target_bed.get());
        break;
    case Heater::chamber:
        if (const auto t = buddy::chamber().current_temperature()) {
            r.current = static_cast<int16_t>(std::lround(static_cast<float>(*t)));
        }
        r.target = static_cast<int16_t>(std::lround(static_cast<float>(buddy::chamber().effective_target_temperature().value_or(0))));
        break;
    }
    return r;
}

void HeaterCard::draw_content(Rect16 body) {
    const Color bg = fill();
    const int16_t x = body.Left() + 10;
    const uint16_t w = body.Width() - 20;

    const bool on = IsEnabled();
    draw_icon({ x, int16_t(body.Top() + 9) }, *heater_icons[int(heater)], bg, on ? ropfn {} : ropfn { .shadow = is_shadowed::yes });
    draw_text(Rect16(x + 22, body.Top() + 9, w - 22, height(Font::small)), _(heater_labels[int(heater)]), Font::small, on ? style::text_muted : style::text_faint, bg);

    char value[8];
    degrees(value, reading.current);
    draw_text(Rect16(x, body.Top() + 30, w, height(Font::big)), string_view_utf8::MakeRAM(value), Font::big, style::text_strong, bg);

    // « cible 230 °C » puis l'état : stable (écart ≤ 1 °C), chauffe, refroidit
    char line[32];
    if (reading.target > 0) {
        char word[16];
        _("target").copyToRAM(word, sizeof(word));
        snprintf(line, sizeof(line), "%s %d \xC2\xB0"
                                     "C",
            word, reading.target);
    } else {
        _("no target").copyToRAM(line, sizeof(line));
    }
    draw_text(Rect16(x, body.Top() + 61, w, height(Font::small)), string_view_utf8::MakeRAM(line), Font::small, style::text_muted, bg);

    string_view_utf8 state = string_view_utf8::MakeCPUFLASH("");
    Color state_color = style::accent;
    if (reading.current >= 0 && reading.target > 0) {
        if (std::abs(reading.current - reading.target) <= 1) {
            state = _("stable");
            state_color = text_dim;
        } else {
            state = reading.current < reading.target ? _("heating") : _("cooling");
        }
    } else if (heater != Heater::chamber && reading.current > 45) {
        state = _("cooling"); // chauffe arrêtée, encore chaud
    }
    if (heater == Heater::bed && chamber_climate::status().active) {
        state = _("slow cool-down"); // F29 : paliers de la descente douce (Refroidir l'arrête)
        state_color = style::accent;
    }
    draw_text(Rect16(x, body.Top() + 79, w, height(Font::small)), state, Font::small, state_color, bg);

    draw_graph(Rect16(x, body.Top() + 100, 128, 20), history.data(), history.size(), reading.target, bg);
}

void HeaterCard::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP) {
        bool changed = false;
        if (const Reading r = read(heater); !(r == reading)) {
            reading = r;
            changed = true;
        }
        if (heater == Heater::chamber && chamber_settable() != IsEnabled()) {
            set_enabled(!IsEnabled());
            changed = true;
        }
        if (const uint32_t now = ticks_ms(); ticks_diff(now, last_sample_ms) >= int32_t(sample_ms)) {
            last_sample_ms = now;
            std::rotate(history.begin(), history.begin() + 1, history.end());
            history.back() = reading.current;
            changed = true;
        }
        if (changed) {
            invalidate_content();
        }
    }
    Card::windowEvent(sender, event, param);
}

// ---- Ventilateurs et filtration ----

FanCard::FanCard(window_t *parent, Fan fan)
    : Card(parent, style::with_ring(fan_body(int(fan))), fan_actions[int(fan)])
    , fan(fan)
    , value(read(fan)) {
    set_enabled(fan != Fan::chamber || chamber_fans_settable());
}

int16_t FanCard::read(Fan fan) {
    switch (fan) {
    case Fan::print:
        return val_mapping(false, marlin_vars().print_fan_speed, 255, 100);
    case Fan::chamber: {
        auto &exb = buddy::xbuddy_extension();
        return exb.fan_target_pwm(buddy::XBuddyExtension::Fan::cooling_fan_1).transform(buddy::XBuddyExtension::FanPWM::to_percent_static).value_or(int8_t(-1));
    }
    case Fan::filtration:
        return config_store().chamber_print_filtration_enable.get();
    }
    return 0;
}

void FanCard::draw_content(Rect16 body) {
    static const img::Resource *const icons[] = { &img::fan_16x16, &img::turbine_16x16, &img::enclosure_16x16 };
    static const char *const labels[] = { N_("Part fan"), N_("Chamber fans"), N_("Filtration") };
    const Color bg = fill();
    const int16_t x = body.Left() + 10;
    const int16_t text_x = x + 22;
    const uint16_t text_w = body.Left() + body.Width() - 8 - text_x;

    const bool on = IsEnabled();
    draw_icon({ x, int16_t(body.Top() + (body.Height() - 16) / 2) }, *icons[int(fan)], bg, on ? ropfn {} : ropfn { .shadow = is_shadowed::yes });
    draw_text(Rect16(text_x, body.Top() + 4, text_w, height(Font::small)), _(labels[int(fan)]), Font::small, on ? style::text_muted : style::text_faint, bg);

    const Rect16 value_rect(text_x, body.Top() + 21, text_w, height(Font::small));
    if (fan == Fan::filtration) {
        draw_icon({ text_x, int16_t(body.Top() + 21) }, value ? img::switch_on_36x18 : img::switch_off_36x18, bg);
        return;
    }
    if (fan == Fan::chamber && value < 0) {
        draw_text(value_rect, _("Auto"), Font::small, style::text_strong, bg);
        return;
    }
    char text[8];
    snprintf(text, sizeof(text), "%d %%", value);
    draw_text(value_rect, string_view_utf8::MakeRAM(text), Font::small, style::text_strong, bg);
}

void FanCard::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP) {
        if (const int16_t v = read(fan); v != value) {
            value = v;
            invalidate_content();
        }
        if (fan == Fan::chamber && chamber_fans_settable() != IsEnabled()) {
            set_enabled(!IsEnabled());
            invalidate_content();
        }
    }
    Card::windowEvent(sender, event, param);
}

// ---- Boutons ----

TextButton::TextButton(window_t *parent, Rect16 body, Action action, const string_view_utf8 &label, const img::Resource *icon, Color body_color, Color fg)
    : Card(parent, style::with_ring(body), action, body_color, style::screen, style::radius_big)
    , label(label)
    , icon(icon)
    , fg(fg) {}

void TextButton::enable(bool set) {
    if (set == IsEnabled()) {
        return;
    }
    set_enabled(set);
    if (fg == style::on_accent) {
        body_color = set ? style::accent : style::secondary;
    }
    Invalidate();
}

void TextButton::draw_content(Rect16 body) {
    const Color color = IsEnabled() ? fg : style::text_faint;
    const uint16_t text_w = std::min<int>(label.computeNumUtf8Chars() * width(Font::small), body.Width() - 16);
    const uint16_t icon_w = icon ? icon->w + 8 : 0;
    const int16_t x = body.Left() + (body.Width() - icon_w - text_w) / 2;
    const int16_t y = body.Top() + (body.Height() - height(Font::small)) / 2;
    if (icon) {
        // Sur l'orange, l'icône blanche est inversée (noire)
        const ropfn rop = !IsEnabled() ? ropfn { .shadow = is_shadowed::yes } : fg == style::on_accent ? ropfn { .swap_bw = has_swapped_bw::yes } : ropfn {};
        draw_icon({ x, y }, *icon, fill(), rop);
    }
    draw_text(Rect16(x + icon_w, y, text_w, height(Font::small)), label, Font::small, color, fill());
}

// ---- Écran ----

ScreenTemperatures::ScreenTemperatures()
    : Screen(true)
    , header(this, _("Temperatures"), true)
    , nozzle(this, HeaterCard::Heater::nozzle)
    , bed(this, HeaterCard::Heater::bed)
    , chamber(this, HeaterCard::Heater::chamber)
    , fan(this, FanCard::Fan::print)
    , chamber_fans(this, FanCard::Fan::chamber)
    , filtration(this, FanCard::Fan::filtration)
    , preheat(this, preheat_body, od3d::preheat, _("Preheat..."), &img::preheat_white_16x16, style::accent, style::on_accent)
    , presets(this, presets_body, open_presets, _("Presets"), nullptr, style::secondary, style::text_strong)
    , cooldown(this, cooldown_body, od3d::cooldown, _("Cool down"), nullptr, style::secondary, style::text_strong)
    , tabs(this, section_tab()) {
    update_buttons();
    nozzle.SetFocus();
}

void ScreenTemperatures::update_buttons() {
    // Comme Prusa : pas de préchauffage (M1700) ni de « Refroidir » pendant une impression
    const bool idle = !marlin_client::is_printing();
    for (TextButton *button : { &preheat, &cooldown }) {
        button->enable(idle);
    }
    if (window_t *focused = get_focused_child_window(); focused && !focused->IsEnabled()) {
        nozzle.SetFocus();
    }
}

void ScreenTemperatures::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::HELD_RELEASED && GetFirstDialog()) {
        return; // sous l'éditeur, l'appui long ne referme pas l'écran
    }
    if (event == GUI_event_t::LOOP) {
        update_buttons();
    }
    Screen::windowEvent(sender, event, param);
}

} // namespace od3d
