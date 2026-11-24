/// @file
#include "screen_mmu_speed_od3d.hpp"
#include "screen_calib_assistant_od3d.hpp" // confirm_stop_measure
#include "menu_od3d.hpp"

#include <cstdio>
#include <cstring>
#include <span>
#include <display.hpp>
#include <feature/od3d/tip_od3d.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    namespace ms = mmu_speed;

    // Géométrie (zone des éléments : y 46 à 274)
    constexpr Rect16 panel_rect = Rect16(style::margin, style::top, 464, 58);
    constexpr Rect16 levels_rect = Rect16(style::margin, 108, 464, 60);
    constexpr Rect16 info_rect = Rect16(style::margin, 172, 464, 54); // 3 lignes
    constexpr Rect16 button_body(int i) {
        return Rect16(18 + 150 * i, 232, 142, 40);
    }
    static_assert(levels_rect.Top() > panel_rect.Bottom() && info_rect.Top() > levels_rect.Bottom());
    static_assert(style::with_ring(button_body(0)).Top() > info_rect.Bottom());
    static_assert(style::inside(style::content, style::with_ring(button_body(2))));

    constexpr uint8_t tag_action = 0;
    constexpr uint8_t tag_slot = 1;
    constexpr uint8_t tag_prusa = 2;

    uint8_t chosen_slot = 0; ///< gardé tant que l'imprimante reste allumée

    void draw_panel_back(Rect16 r) {
        display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    }

    /// Texte traduit au format printf, écrit dans out
    template <class... Args>
    void put(std::span<char> out, const char *msgid, Args... args) {
        char fmt[96];
        _(msgid).copyToRAM(fmt, sizeof(fmt));
        snprintf(out.data(), out.size(), fmt, args...);
    }

} // namespace

const char *mmu_speed_result_text(mmu_speed::Result result) {
    switch (result) {
    case ms::Result::none:
    case ms::Result::running:
    case ms::Result::saved:
        return nullptr;
    case ms::Result::stopped:
        return N_("Stopped: speed unchanged.");
    case ms::Result::refused_printing:
        return N_("Not while printing.");
    case ms::Result::refused_mmu:
        return N_("Refused: MMU not ready");
    case ms::Result::refused_sensor:
        return N_("Refused: extruder sensor inactive");
    case ms::Result::refused_extruder:
        return N_("Refused: filament in the extruder");
    case ms::Result::refused_selector:
        return N_("Refused: filament in the selector");
    case ms::Result::failed:
        return N_("Error at Prusa speed: speed unchanged. Is the spool preloaded?");
    case ms::Result::failed_check:
        return N_("Error at the kept speed: back to Prusa speed.");
    }
    return nullptr;
}

// ---- Vitesse ----

MmuSpeedPanel::MmuSpeedPanel(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void MmuSpeedPanel::unconditionalDraw() {
    const Rect16 r = GetRect();
    draw_panel_back(r);
    const ms::Saved s = ms::saved();
    const uint16_t hs = height(Font::small);
    const int16_t half = r.Width() / 2;

    draw_text(Rect16(r.Left() + 12, r.Top() + 5, half, hs), _("Pulley speed"), Font::small, style::text_strong, style::surface);
    char text[64];
    put(text, N_("Prusa %d mm/s"), int(ms::prusa_speed));
    draw_text(Rect16(r.Left() + half, r.Top() + 5, half - 12, hs), string_view_utf8::MakeRAM(text), Font::small, style::text_muted, style::surface, Align_t::RightCenter());

    snprintf(text, sizeof(text), "%d mm/s", int(ms::speed_of(s)));
    constexpr int16_t value_w = 128; // « 152 mm/s » en grand
    draw_text(Rect16(r.Left() + 12, r.Top() + 26, value_w, height(Font::big)), string_view_utf8::MakeRAM(text), Font::big, s.speed > ms::prusa_speed ? style::accent : style::text_strong, style::surface);

    // Gain mesuré : durée d'un cycle chargement + déchargement à la vitesse de Prusa, puis à la vitesse gardée
    if (s.speed > ms::prusa_speed && s.prusa_ds && s.cycle_ds) {
        put(text, N_("Each change %+.1f s (measured)"), (int(s.cycle_ds) - int(s.prusa_ds)) / 10.0);
    } else {
        _(s.prusa_ds ? N_("No faster speed passed") : N_("Not measured: Prusa speed")).copyToRAM(text, sizeof(text));
    }
    const int16_t gain_x = r.Left() + 12 + value_w;
    draw_text(Rect16(gain_x, r.Top() + 30, r.Right() - 12 - gain_x, hs), string_view_utf8::MakeRAM(text), Font::small, style::text_soft, style::surface, Align_t::RightCenter());
}

// ---- Paliers ----

MmuSpeedLevels::MmuSpeedLevels(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void MmuSpeedLevels::unconditionalDraw() {
    const Rect16 r = GetRect();
    draw_panel_back(r);
    const ms::Measurement &m = ms::last();
    const bool running = ms::is_running();
    const uint16_t hs = height(Font::small);
    draw_text(Rect16(r.Left() + 12, r.Top() + 4, r.Width() - 24, hs), _("Cycle per speed, then the kept one (mm/s, s)"), Font::small, style::text_strong, style::surface);

    constexpr int16_t column = 64;
    static_assert(8 + column * ms::cycles <= 464);
    for (uint8_t i = 0; i < ms::cycles; ++i) {
        const Rect16 col(r.Left() + 8 + column * i, r.Top() + 22, column, hs);
        char text[12];
        const bool check = i == ms::levels.size(); // vérification à la vitesse gardée, en orange
        if (!check) {
            snprintf(text, sizeof(text), "%d", int(ms::levels[i]));
        } else if (m.speed) {
            snprintf(text, sizeof(text), "%d", int(m.speed));
        } else {
            strlcpy(text, "-5", sizeof(text)); // le palier d'avant le dernier passé (paliers de 5 mm/s)
        }
        draw_text(col, string_view_utf8::MakeRAM(text), Font::small, check ? style::accent : style::text_muted, style::surface, Align_t::Center());

        Color color = style::text_faint;
        if (m.ds[i]) {
            snprintf(text, sizeof(text), "%.1f", m.ds[i] / 10.0);
            color = m.passed[i] ? style::text_strong : style::danger;
        } else if (running && m.step == i) {
            strlcpy(text, "...", sizeof(text));
            color = style::accent;
        } else {
            strlcpy(text, "--", sizeof(text));
        }
        draw_text(Rect16(col.Left(), r.Top() + 38, column, height(Font::normal)), string_view_utf8::MakeRAM(text), Font::normal, color, style::surface, Align_t::Center());
    }
}

// ---- Écran ----

ScreenMmuSpeed::ScreenMmuSpeed()
    : Screen(true)
    , header(this, _("Pulley speed"), true)
    , panel(this, panel_rect)
    , levels(this, levels_rect)
    , info(this, info_rect, is_multiline::yes)
    , action(this, button_body(0), tag_action, Button::Look::primary, _("Start"), &img::print_16x16, style::screen)
    , slot_button(this, button_body(1), tag_slot, Button::Look::secondary, {}, &img::spool_16x16, style::screen)
    , prusa_button(this, button_body(2), tag_prusa, Button::Look::outline, _("Prusa value"), &img::arrow_left_16x16, style::screen)
    , tabs(this, section_tab()) {
    ClrMenuTimeoutClose(); // l'écran reste ouvert pendant la mesure
    info.set_font(Font::small);
    info.SetAlignment(Align_t::LeftCenter());
    info.SetTextColor(style::text_soft);
    info.SetBackColor(style::screen);
    info.SetPadding({ 0, 0, 0, 0 });
    slot_button.set(Button::Look::secondary, _("Slot %i").formatted(slot_params, chosen_slot + 1), &img::spool_16x16);
    update();
    action.SetFocus();
}

void ScreenMmuSpeed::show_info() {
    const ms::Measurement &m = ms::last();
    const auto text = [&](const char *msgid) {
        _(msgid).copyToRAM(info_text.data(), info_text.size());
    };
    if (pending) {
        text(N_("Starting..."));
    } else if (ms::is_running()) {
        if (m.step < ms::levels.size()) {
            put(info_text, N_("Cycle %d/%d at %d mm/s. Stop: after this cycle."), m.step + 1, int(ms::levels.size()), int(ms::levels[m.step]));
        } else {
            put(info_text, N_("Checking %d mm/s..."), int(m.speed));
        }
    } else {
        if (const char *why = mmu_speed_result_text(m.result)) {
            text(why);
        } else if (m.result != ms::Result::saved) {
            text(N_("Preload the spool, then Start: up to 7 cycles from 95 to 120 mm/s, the level before the last clean one is kept (~4 min)."));
        } else if (m.speed > ms::prusa_speed) {
            put(info_text, N_("Saved: %d mm/s, sent to the MMU at each start."), int(m.speed));
        } else {
            text(N_("No faster speed passed without error: Prusa speed kept."));
        }
    }
    info.SetText(string_view_utf8::MakeRAM(info_text.data()));
    info.Invalidate();
}

void ScreenMmuSpeed::update() {
    const bool running = ms::is_running();
    const uint16_t revision = ms::revision();
    if (pending && (running || revision != revision_at_request)) {
        pending = false;
    }
    const bool busy = running || pending;
    const ms::Saved s = ms::saved();
    if (revision != shown_revision || busy != shown_busy || !(s == shown_saved)) {
        shown_revision = revision;
        shown_saved = s;
        panel.Invalidate();
        levels.Invalidate();
        show_info();
        if (busy != shown_busy) {
            shown_busy = busy;
            action.set(busy ? Button::Look::outline : Button::Look::primary, busy ? _("Cancel") : _("Start"), busy ? &img::cancel_16x16 : &img::print_16x16);
        }
    }
    const bool can_edit = !busy && !marlin_client::is_printing();
    action.enable(busy || !marlin_client::is_printing());
    slot_button.enable(can_edit);
    prusa_button.enable(can_edit && s.speed && s.speed != ms::prusa_speed);
}

void ScreenMmuSpeed::start() {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    StringViewUtf8Parameters<4> params;
    const string_view_utf8 question = _("The MMU loads the spool of slot %d to the extruder gears and unloads it, up to 7 times, faster each time. Is that spool preloaded?").formatted(params, chosen_slot + 1);
    if (MsgBoxQuestion(question, Responses_YesNo) != Response::Yes) {
        return;
    }
    char gcode[16];
    snprintf(gcode, sizeof(gcode), "M2002 P%u", unsigned(chosen_slot));
    const uint16_t before = ms::revision();
    if (marlin_client::gcode_try(gcode) == marlin_client::GcodeTryResult::Submitted) {
        revision_at_request = before;
        pending = true;
    }
}

void ScreenMmuSpeed::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag == tag_action) {
            if (ms::is_running() || pending) {
                if (confirm_stop_measure()) { // pendant une série de l'assistant : il passe à la suivante
                    ms::request_stop();
                }
            } else {
                start();
            }
        } else if (tag == tag_slot) {
            chosen_slot = uint8_t((chosen_slot + 1) % ms::slots);
            slot_button.set(Button::Look::secondary, _("Slot %i").formatted(slot_params, chosen_slot + 1), &img::spool_16x16);
        } else if (tag == tag_prusa) {
            marlin_client::gcode("M2002 R");
        }
        update();
        return;
    }

    default:
        break;
    }
    Screen::windowEvent(sender, event, param);
}

// ---- Atelier › Réglages du MMU ----

MI_MMU_SPEED::MI_MMU_SPEED()
    : IWindowMenuItem(_("Pulley speed"), &img::speed_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {
    const ms::Saved s = ms::saved();
    put(text, s.speed ? N_("Pulley speed\n%d mm/s, measured") : N_("Pulley speed\n%d mm/s (Prusa), not measured"), int(ms::speed_of(s)));
    SetLabel(string_view_utf8::MakeRAM(text.data()));
}

void MI_MMU_SPEED::click(IWindowMenu &) {
    open_screen<ScreenMmuSpeed>();
}

// ---- Atelier › Infos du MMU ----

MI_TIP_SLOT::MI_TIP_SLOT(uint8_t slot)
    : IWindowMenuItem({}, &img::info_16x16, is_enabled_t::yes, is_hidden_t::no) {
    const tip::Stats s = tip::stats(slot);
    put(text, N_("Filament tip, slot %d"), slot + 1);
    const size_t n = strlen(text.data());
    if (n + 1 >= text.size()) {
        return;
    }
    text[n] = '\n';
    const std::span<char> rest(text.data() + n + 1, text.size() - n - 1);
    if (!s.count) {
        _("No tool change in a print yet").copyToRAM(rest.data(), rest.size());
    } else if (tip::drifting(s)) {
        put(rest, N_("Drift: %.1f mm, usual %.1f \xC2\xB1 %.1f"), double(tip::mm_of(s.recent)), double(tip::mm_of(s.mean)), double(tip::sigma(s)));
        SetIconId(&img::warning_16x16);
    } else {
        put(rest, N_("%.1f \xC2\xB1 %.1f mm, %u changes"), double(tip::mm_of(s.mean)), double(tip::sigma(s)), unsigned(s.count));
    }
    SetLabel(string_view_utf8::MakeRAM(text.data()));
}

} // namespace od3d
