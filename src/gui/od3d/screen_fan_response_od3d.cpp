/// @file
#include "screen_fan_response_od3d.hpp"
#include "screen_calib_assistant_od3d.hpp" // confirm_stop_measure
#include "menu_items_od3d.hpp"
#include "menu_od3d.hpp"
#include "thumbnail_od3d.hpp" // draw_pixels

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <feature/od3d/lookahead_od3d.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <option/has_input_shaper_calibration.h>
#include <option/has_manual_belt_tuning.h>
#include <window_msgbox.hpp>
#if HAS_INPUT_SHAPER_CALIBRATION() && HAS_MANUAL_BELT_TUNING()
    #include <feature/od3d/vibration_od3d.hpp>
#endif

namespace od3d {

namespace {

    /// M2001 V : les vibrations des ventilateurs (~1 min) suivent le temps de montée dans la même commande ; l'écran
    /// reste occupé jusqu'à leur fin (ECR-4)
    bool vibration_running() {
#if HAS_INPUT_SHAPER_CALIBRATION() && HAS_MANUAL_BELT_TUNING()
        return fan_vibration::is_running();
#else
        return false;
#endif
    }

    /// Arrête la partie de M2001 qui tourne (un arrêt donné à l'autre resterait pour sa prochaine mesure)
    void stop_running() {
#if HAS_INPUT_SHAPER_CALIBRATION() && HAS_MANUAL_BELT_TUNING()
        if (fan_vibration::is_running()) {
            fan_vibration::request_stop();
            return;
        }
#endif
        fan_response::request_stop();
    }

    // Géométrie (zone des éléments : y 46 à 274)
    constexpr Rect16 panel_rect = Rect16(style::margin, style::top, 464, 60);
    constexpr Rect16 curve_back_rect = Rect16(style::margin, 112, 464, 82);
    constexpr Rect16 curve_rect = Rect16(20, 134, 440, 54);
    constexpr Rect16 info_rect = Rect16(style::margin, 198, 464, 30);
    constexpr Rect16 button_rect = Rect16(18, 232, 142, 40);
    static_assert(style::inside(style::content, panel_rect) && panel_rect.Right() == style::screen_w - 1 - style::margin);
    static_assert(curve_back_rect.Top() > panel_rect.Bottom() && style::inside(curve_back_rect, curve_rect));
    static_assert(info_rect.Top() > curve_back_rect.Bottom() && style::with_ring(button_rect).Top() > info_rect.Bottom());
    static_assert(style::inside(style::content, style::with_ring(button_rect)));

    constexpr uint8_t tag_action = 0;

    /// Créneaux d'une mesure complète : l'axe des temps de la courbe
    constexpr size_t plot_bins = (fan_response::baseline_ms + fan_response::high_ms) / 50;
    static_assert(plot_bins <= fan_response::max_bins);

    /// Hauteur de la courbe par colonne (-1 : pas encore mesuré), calculée une fois par dessin dans le tampon partagé
    using CurveHeights = std::array<int8_t, curve_rect.Width()>;
    static_assert(curve_rect.Height() <= INT8_MAX);

    void draw_panel_back(Rect16 r) {
        display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    }

    /// « 0.7 s », ou « Arrêt » pour une avance nulle (comme Prusa)
    void lead_text(uint8_t ds, char *out, size_t size) {
        if (ds == 0) {
            _("Off").copyToRAM(out, size);
        } else {
            snprintf(out, size, "%u.%u s", unsigned(ds / 10), unsigned(ds % 10));
        }
    }

    bool measured() {
        return !fan_response::is_running() && fan_response::status() == fan_response::Status::done;
    }

} // namespace

// ---- Panneau des valeurs ----

FanResponsePanel::FanResponsePanel(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void FanResponsePanel::unconditionalDraw() {
    const Rect16 r = GetRect();
    draw_panel_back(r);
    const fan_response::Result &res = fan_response::result();
    const uint16_t hs = height(Font::small);
    char text[24];

    draw_text(Rect16(r.Left() + 12, r.Top() + 6, 200, hs), _("Rise time"), Font::small, style::text_muted, style::surface);
    if (measured()) {
        snprintf(text, sizeof(text), "%.2f s", static_cast<double>(res.rise.rise_s));
    } else {
        strlcpy(text, "--", sizeof(text));
    }
    draw_text(Rect16(r.Left() + 12, r.Top() + 26, 200, height(Font::big)), string_view_utf8::MakeRAM(text), Font::big, style::text_strong, style::surface);

    const int16_t right = r.Right() - 12;
    draw_text(Rect16(right - 220, r.Top() + 6, 220, hs), _("Fan lead"), Font::small, style::text_muted, style::surface, Align_t::RightCenter());
    lead_text(lookahead::fan_lead_ds(), text, sizeof(text));
    draw_text(Rect16(right - 220, r.Top() + 22, 220, hs), string_view_utf8::MakeRAM(text), Font::small, style::text_strong, style::surface, Align_t::RightCenter());
    if (measured()) {
        snprintf(text, sizeof(text), "%u RPM", unsigned(std::lround(res.rise.rpm)));
        draw_text(Rect16(right - 220, r.Top() + 38, 220, hs), string_view_utf8::MakeRAM(text), Font::small, style::text_muted, style::surface, Align_t::RightCenter());
    }
}

// ---- Courbe ----

FanResponseCurveBack::FanResponseCurveBack(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void FanResponseCurveBack::unconditionalDraw() {
    const Rect16 r = GetRect();
    draw_panel_back(r);
    const uint16_t hs = height(Font::small);
    draw_text(Rect16(r.Left() + 12, r.Top() + 5, 300, hs), _("Part fan speed"), Font::small, style::text_strong, style::surface);
    char range[16];
    snprintf(range, sizeof(range), "0-%.0f s", static_cast<double>(plot_bins * lookahead::fan_bin_s));
    draw_text(Rect16(r.Right() - 12 - 72, r.Top() + 5, 72, hs), string_view_utf8::MakeRAM(range), Font::small, style::text_muted, style::surface, Align_t::RightCenter());
}

FanResponseCurve::FanResponseCurve(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void FanResponseCurve::unconditionalDraw() {
    const Rect16 r = GetRect();
    const int w = r.Width(), h = r.Height();
    const auto &edges = fan_response::edges();
    const size_t bins = std::min<size_t>(fan_response::bins(), plot_bins);
    const fan_response::Result &res = fan_response::result();

    // Échelle : le plus haut créneau, ou le maximum de Prusa (6 850 tr/min) tant que rien ne tourne
    float top = 0.f;
    for (size_t i = 0; i < bins; ++i) {
        top = std::max(top, float(edges[i]));
    }
    top = std::max(top, 6850.f / lookahead::fan_rpm(1.f)) * 1.1f;
    CurveHeights &curve_heights = scratch_as<CurveHeights>();
    for (int x = 0; x < w; ++x) {
        const size_t i = size_t(x) * plot_bins / size_t(w);
        curve_heights[x] = i < bins ? int8_t(std::lround(float(edges[i]) / top * float(h - 1))) : int8_t(-1);
    }
    const int command_x = res.command_s >= 0.f ? int(res.command_s / (plot_bins * lookahead::fan_bin_s) * w) : -1;
    const int threshold_y = measured() ? int(std::lround(0.9f * res.rise.rpm / lookahead::fan_rpm(1.f) / top * float(h - 1))) : -1;

    draw_pixels(r, [&](int x, int y) {
        const int up = h - 1 - y;
        if (up == threshold_y && (x / 4) % 2 == 0) {
            return style::warning; // 90 % du régime final, en pointillés
        }
        if (curve_heights[x] >= 0 && up <= curve_heights[x]) {
            return style::accent;
        }
        if (x == command_x) {
            return style::text_faint; // commande 0 → 100 %
        }
        return up == 0 ? style::line : style::surface;
    });
}

// ---- Écran ----

ScreenFanResponse::ScreenFanResponse()
    : Screen(true)
    , header(this, _("Part fan response"), true)
    , panel(this, panel_rect)
    , curve_back(this, curve_back_rect)
    , curve(this, curve_rect)
    , info(this, info_rect, is_multiline::yes)
    , action(this, button_rect, tag_action, Button::Look::primary, _("Start"), &img::print_16x16, style::screen)
    , tabs(this, section_tab()) {
    ClrMenuTimeoutClose(); // l'écran reste ouvert pendant la mesure
    info.set_font(Font::small);
    info.SetAlignment(Align_t::LeftCenter());
    info.SetTextColor(style::text_soft);
    info.SetBackColor(style::screen);
    info.SetPadding({ 0, 0, 0, 0 });
    update();
    action.SetFocus();
}

void ScreenFanResponse::show_info() {
    using fan_response::Status;
    string_view_utf8 text;
    if (shown_pending) {
        text = _("Starting...");
    } else if (shown_running) {
        text = _("Measuring the part fan... %d%%").formatted(info_params, int(shown_progress));
    } else {
        switch (shown_status) {
        case Status::idle:
        case Status::running:
            text = _("Part fan started at full speed, no filament or heating (about 8 s). Sets the fan lead.");
            break;
        case Status::done: {
            char before[12];
            lead_text(fan_response::result().lead_before_ds, before, sizeof(before));
            text = _("Fan lead set from the rise time (before: %s).").formatted(info_params, before);
            break;
        }
        case Status::stopped:
            text = _("Stopped");
            break;
        case Status::printing:
            text = _("Not while printing.");
            break;
        case Status::no_signal:
            text = _("No fan speed read: fan lead unchanged.");
            break;
        case Status::hot_nozzle:
            text = _("Let the nozzle cool below 50°C, then start again.");
            break;
        }
    }
    info.SetText(text);
    info.Invalidate();
}

void ScreenFanResponse::update() {
    const bool now_running = fan_response::is_running() || vibration_running();
    if (pending && (now_running || fan_response::run_count() != runs_at_request)) {
        pending = false;
    }
    const fan_response::Status s = fan_response::status();
    const uint8_t progress = fan_response::progress_percent();
    const uint8_t bins = fan_response::bins();

    if (bins != shown_bins || s != shown_status || now_running != shown_running) {
        shown_bins = bins;
        curve.Invalidate();
        panel.Invalidate();
    }
    if (now_running != shown_running || pending != shown_pending || s != shown_status || progress != shown_progress) {
        const bool was_busy = shown_running || shown_pending;
        shown_running = now_running;
        shown_pending = pending;
        shown_status = s;
        shown_progress = progress;
        show_info();
        const bool busy = now_running || pending;
        if (busy != was_busy) {
            action.set(busy ? Button::Look::outline : Button::Look::primary, busy ? _("Cancel") : _("Start"), busy ? &img::cancel_16x16 : &img::print_16x16);
        }
    }
    action.enable(now_running || pending || !marlin_client::is_printing());
}

void ScreenFanResponse::start() {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    const uint16_t before = fan_response::run_count();
    if (marlin_client::gcode_try("M2001") == marlin_client::GcodeTryResult::Submitted) {
        runs_at_request = before;
        pending = true;
    }
}

void ScreenFanResponse::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag == tag_action) {
            if (fan_response::is_running() || vibration_running() || pending) {
                if (confirm_stop_measure()) { // pendant une série de l'assistant : il passe à la suivante
                    stop_running();
                }
            } else {
                start();
            }
        }
        update();
        return;
    }

    default:
        break;
    }
    Screen::windowEvent(sender, event, param);
}

// ---- Réglages › Fonctions OneDrop3D ----

MI_FAN_RESPONSE::MI_FAN_RESPONSE()
    : IWindowMenuItem(_("Part fan response\nMeasured, sets the fan lead"), &img::fan_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_FAN_RESPONSE::click(IWindowMenu &) {
    open_screen<ScreenFanResponse>();
}

} // namespace od3d
