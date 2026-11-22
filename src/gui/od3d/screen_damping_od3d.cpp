/// @file
#include "screen_damping_od3d.hpp"
#include "accelerometer_guide_od3d.hpp"
#include "menu_items_od3d.hpp"
#include "menu_od3d.hpp"
#include "thumbnail_od3d.hpp" // draw_pixels

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <feature/od3d/belt_tension_math_od3d.hpp> // curve_height
#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <display.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <Marlin/src/feature/input_shaper/input_shaper_config.hpp>
#include <MItem_tools.hpp> // gui_try_gcode_with_msg
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    // Géométrie (zone des éléments : y 46 à 274)
    constexpr Rect16 panel_rect(int i) {
        return Rect16(style::margin + 236 * i, style::top, 228, 68);
    }
    constexpr Rect16 curves_back_rect = Rect16(style::margin, 120, 464, 72);
    constexpr Rect16 curves_rect = Rect16(20, 142, 440, 44);
    constexpr Rect16 info_rect = Rect16(style::margin, 196, 464, 32);
    constexpr Rect16 button_body(int i) {
        return Rect16(18 + 150 * i, 232, 142, 40);
    }
    static_assert(style::inside(style::content, panel_rect(1)) && panel_rect(1).Right() == style::screen_w - 1 - style::margin);
    static_assert(curves_back_rect.Top() > panel_rect(0).Bottom() && style::inside(curves_back_rect, curves_rect));
    static_assert(info_rect.Top() > curves_back_rect.Bottom() && style::with_ring(button_body(0)).Top() > info_rect.Bottom());
    static_assert(style::inside(style::content, style::with_ring(button_body(2))));

    constexpr Color curve_y = Color::from_raw(0x4FA3F7);
    constexpr Color curve_colors[damping::axis_count] = { style::accent, curve_y };

    constexpr uint8_t tag_action = 0;
    constexpr uint8_t tag_apply = 1;
    constexpr uint8_t tag_prusa = 2;

    constexpr AxisEnum axis_of(uint8_t a) {
        return a == 0 ? X_AXIS : Y_AXIS;
    }

    float current_zeta(uint8_t a) {
        return config_store().get_input_shaper_axis_config(axis_of(a)).damping_ratio;
    }

    float prusa_zeta(uint8_t a) {
        return input_shaper::axis_defaults[axis_of(a)].damping_ratio;
    }

    bool differs(float a, float b) {
        return std::fabs(a - b) > 0.0005f; // l'écran montre 3 décimales
    }

    /// Hauteur de chaque courbe par colonne, calculée une fois par dessin (voir screen_belts_od3d.cpp)
    std::array<std::array<int8_t, curves_rect.Width() + 1>, damping::axis_count> curve_heights;
    static_assert(curves_rect.Height() <= INT8_MAX);

    void draw_panel_back(Rect16 r) {
        display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    }

    /// L'écran a été fermé pendant sa mesure (retour, ou un autre écran ouvert par-dessus : en-tête, pastille) et l'a
    /// donc arrêtée ; gardé hors de l'écran, recréé au retour, pour le dire à l'utilisateur
    bool stopped_by_leaving = false;

} // namespace

// ---- Panneau d'un axe ----

DampingPanel::DampingPanel(window_t *parent, Rect16 rect, uint8_t axis)
    : window_t(parent, rect)
    , axis(axis) {}

void DampingPanel::unconditionalDraw() {
    const Rect16 r = GetRect();
    draw_panel_back(r);
    const damping::Measurement &m = damping::last();
    const damping::Resonance &res = m.results[axis];
    const bool measured = m.points[axis] == damping::sweep_points;
    const uint16_t hs = height(Font::small);

    display::fill_rect(Rect16(r.Left() + 12, r.Top() + 9, 10, 10), curve_colors[axis]);
    draw_text(Rect16(r.Left() + 28, r.Top() + 6, 100, hs), axis == 0 ? _("X axis") : _("Y axis"), Font::small, style::text_strong, style::surface);

    char text[24];
    if (res.found) {
        snprintf(text, sizeof(text), "%.1f Hz", static_cast<double>(res.frequency));
        draw_text(Rect16(r.Right() - 12 - 96, r.Top() + 6, 96, hs), string_view_utf8::MakeRAM(text), Font::small, style::text_muted, style::surface, Align_t::RightCenter());
        snprintf(text, sizeof(text), "zeta %.3f", static_cast<double>(res.zeta));
    } else {
        strlcpy(text, "zeta --", sizeof(text));
    }
    const Color value_color = damping::applicable(res) ? style::text_strong : (res.found ? style::warning : style::text_faint);
    draw_text(Rect16(r.Left() + 12, r.Top() + 24, r.Width() - 24, height(Font::big)), string_view_utf8::MakeRAM(text), Font::big, value_color, style::surface);

    const Rect16 bottom(r.Left() + 12, r.Top() + 48, r.Width() - 24, hs);
    if (res.found) {
        snprintf(text, sizeof(text), "Q %.1f   -3 dB %.1f Hz", static_cast<double>(res.q), static_cast<double>(res.width));
        draw_text(bottom, string_view_utf8::MakeRAM(text), Font::small, style::text_muted, style::surface);
    } else if (measured) {
        draw_text(bottom, _("weak peak"), Font::small, style::text_muted, style::surface);
    }
}

// ---- Courbes ----

DampingCurvesBack::DampingCurvesBack(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void DampingCurvesBack::unconditionalDraw() {
    const Rect16 r = GetRect();
    draw_panel_back(r);
    const uint16_t hs = height(Font::small);
    const int16_t y = r.Top() + 5;
    draw_text(Rect16(r.Left() + 12, y, 150, hs), _("Spectrum"), Font::small, style::text_strong, style::surface);
    const int16_t right = r.Right() - 12;
    char range[16];
    snprintf(range, sizeof(range), "%d-%d Hz", int(damping::sweep_start_hz), int(damping::sweep_end_hz));
    draw_text(Rect16(right - 72, y, 72, hs), string_view_utf8::MakeRAM(range), Font::small, style::text_muted, style::surface, Align_t::RightCenter());
    for (uint8_t a = 0; a < damping::axis_count; ++a) {
        const int16_t x = right - 148 + 32 * a;
        display::fill_rect(Rect16(x, y + 3, 10, 10), curve_colors[a]);
        draw_text(Rect16(x + 14, y, 12, hs), string_view_utf8::MakeCPUFLASH(a == 0 ? "X" : "Y"), Font::small, style::text, style::surface);
    }
}

DampingCurves::DampingCurves(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void DampingCurves::unconditionalDraw() {
    const damping::Measurement &m = damping::last();
    const Rect16 r = GetRect();
    const int w = r.Width(), h = r.Height();

    // Chaque courbe à sa propre échelle : c'est la largeur du pic qui compte, pas sa hauteur
    for (uint8_t a = 0; a < damping::axis_count; ++a) {
        float scale = 0.f;
        for (size_t i = 0; i < std::min<size_t>(m.points[a], damping::sweep_points); ++i) {
            if (std::isfinite(m.psd[a][i])) {
                scale = std::max(scale, m.psd[a][i]);
            }
        }
        scale *= 1.08f;
        for (int x = 0; x <= w; ++x) {
            curve_heights[a][x] = int8_t(belts::curve_height(m.psd[a], m.points[a], x, w, h, scale));
        }
    }

    draw_pixels(r, [&](int x, int y) {
        const int up = h - 1 - y;
        for (uint8_t a = 0; a < damping::axis_count; ++a) {
            const int y0 = curve_heights[a][x];
            if (y0 < 0) {
                continue;
            }
            const int y1 = curve_heights[a][x + 1] < 0 ? y0 : curve_heights[a][x + 1];
            if (up >= std::min(y0, y1) && up <= std::max(y0, y1) + 1) {
                return curve_colors[a];
            }
        }
        return up == 0 ? style::line : style::surface;
    });
}

// ---- Écran ----

ScreenDamping::ScreenDamping()
    : Screen(true)
    , header(this, _("Axis damping"), true)
    , panel_x(this, panel_rect(0), 0)
    , panel_y(this, panel_rect(1), 1)
    , curves_back(this, curves_back_rect)
    , curves(this, curves_rect)
    , info(this, info_rect, is_multiline::yes)
    , action(this, button_body(0), tag_action, Button::Look::primary, _("Start"), &img::print_16x16, style::screen, 10)
    , apply_button(this, button_body(1), tag_apply, Button::Look::secondary, _("Apply"), &img::ok_16x16, style::screen, 10)
    , prusa_button(this, button_body(2), tag_prusa, Button::Look::outline, _("Prusa value"), &img::arrow_left_16x16, style::screen, 10)
    , tabs(this, section_tab()) {
    ClrMenuTimeoutClose(); // l'écran reste ouvert pendant la mesure
    info.set_font(Font::small);
    info.SetAlignment(Align_t::LeftCenter());
    info.SetTextColor(style::text_soft);
    info.SetBackColor(style::screen);
    info.SetPadding({ 0, 0, 0, 0 });
    update(); // warned_accelerometer vaut true : aucun dialogue depuis le constructeur (Screens::Get() y est nul)
    action.SetFocus();
}

ScreenDamping::~ScreenDamping() {
    if (damping::is_running() || pending) {
        damping::request_stop();
        stopped_by_leaving = true;
    }
}

void ScreenDamping::show_info() {
    using damping::Status;
    string_view_utf8 text;
    if (shown_pending) {
        text = _("Starting...");
    } else if (shown_running) {
        text = shown_calibrating
            ? _("Calibrating accelerometer... %d%%").formatted(info_params, int(shown_progress))
            : _("Measuring %c axis... %d%%").formatted(info_params, shown_axis == 0 ? 'X' : 'Y', int(shown_progress));
    } else {
        switch (shown_status) {
        case Status::idle:
        case Status::running:
            text = _("Attach the accelerometer to the nozzle, then Start (X and Y, about 4 min).");
            break;
        case Status::done: {
            char x[8], y[8], prusa[8]; // recopiés dans info_params
            snprintf(x, sizeof(x), "%.3f", static_cast<double>(current_zeta(0)));
            snprintf(y, sizeof(y), "%.3f", static_cast<double>(current_zeta(1)));
            snprintf(prusa, sizeof(prusa), "%.3f", static_cast<double>(prusa_zeta(0)));
            text = _("Input shaper damping now: X %s, Y %s (Prusa %s).").formatted(info_params, x, y, prusa);
            break;
        }
        case Status::stopped:
            text = stopped_by_leaving ? _("Stopped: leaving this screen stops the measurement.") : _("Stopped");
            break;
        case Status::printing:
            text = _("Not while printing.");
            break;
        case Status::hot_nozzle:
            text = _("Let the nozzle cool below 50°C, then start again.");
            break;
        case Status::no_accelerometer:
            text = _("Accelerometer not responding.");
            break;
        case Status::failed:
            text = _("Measurement failed.");
            break;
        case Status::not_homed:
            text = _("Homing failed.");
            break;
        }
    }
    info.SetText(text);
    info.Invalidate();
}

void ScreenDamping::update() {
    const bool now_running = damping::is_running();
    if (pending && (now_running || damping::run_count() != runs_at_request)) {
        pending = false;
    }
    const damping::Status s = damping::status();
    const uint8_t axis = damping::current_axis();
    const uint8_t progress = damping::progress_percent();
    const bool calibrating = damping::calibrating();
    const uint16_t revision = damping::revision();

    if (revision != shown_revision) {
        shown_revision = revision;
        panel_x.Invalidate();
        panel_y.Invalidate();
        curves.Invalidate();
    }

    if (now_running != shown_running || pending != shown_pending || s != shown_status || axis != shown_axis || progress != shown_progress || calibrating != shown_calibrating) {
        const bool was_busy = shown_running || shown_pending;
        shown_running = now_running;
        shown_pending = pending;
        shown_status = s;
        shown_axis = axis;
        shown_progress = progress;
        shown_calibrating = calibrating;
        show_info();
        const bool busy = now_running || pending;
        if (busy != was_busy) {
            action.set(busy ? Button::Look::outline : Button::Look::primary, busy ? _("Cancel") : _("Start"), busy ? &img::cancel_16x16 : &img::print_16x16);
        }
    }

    const bool can_edit = !now_running && !pending && !marlin_client::is_printing();
    bool can_apply = false, can_restore = false;
    for (uint8_t a = 0; a < damping::axis_count; ++a) {
        const damping::Resonance &res = damping::last().results[a];
        can_apply |= damping::applicable(res) && differs(res.zeta, current_zeta(a));
        can_restore |= differs(current_zeta(a), prusa_zeta(a));
    }
    action.enable(now_running || pending || !marlin_client::is_printing());
    apply_button.enable(can_edit && can_apply);
    prusa_button.enable(can_edit && can_restore);

    // ACC-1 (une fois par mesure) : refusée sans bouger, l'accéléromètre ne répond pas
    if (!now_running && !pending && s == damping::Status::no_accelerometer && !warned_accelerometer && !HasDialog()) {
        warned_accelerometer = true;
        if (ask_accelerometer_retry()) {
            start();
        }
    }
}

void ScreenDamping::start() {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    auto &hotend = marlin_vars().active_hotend();
    if (hotend.temp_nozzle.get() > damping::max_nozzle_temperature || hotend.target_nozzle.get() > damping::max_nozzle_temperature) {
        MsgBoxWarning(_("Let the nozzle cool below 50°C, then start again."), Responses_Ok);
        return;
    }
    // ACC-1 : M1997 vérifie l'accéléromètre avant tout mouvement ; s'il ne répond pas, update() guide l'utilisateur
    const uint16_t before = damping::run_count();
    if (marlin_client::gcode_try("M1997") == marlin_client::GcodeTryResult::Submitted) {
        runs_at_request = before;
        pending = true;
        warned_accelerometer = false;
        stopped_by_leaving = false;
    }
}

void ScreenDamping::apply(bool measured) {
    const damping::Measurement &m = damping::last();
    for (uint8_t a = 0; a < damping::axis_count; ++a) {
        if (measured && !damping::applicable(m.results[a])) {
            continue; // cet axe garde sa valeur
        }
        auto config = config_store().get_input_shaper_axis_config(axis_of(a));
        config.damping_ratio = measured ? m.results[a].zeta : prusa_zeta(a);
        config_store().set_input_shaper_axis_config(axis_of(a), config);
    }
    // Comme le menu input shaper de Prusa : l'input shaper relit sa configuration
    gui_try_gcode_with_msg("M9200");
    show_info();
}

void ScreenDamping::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag == tag_action) {
            if (damping::is_running() || pending) {
                damping::request_stop();
            } else {
                start();
            }
        } else if (tag == tag_apply || tag == tag_prusa) {
            apply(tag == tag_apply);
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

MI_DAMPING::MI_DAMPING()
    : IWindowMenuItem(_("Axis damping\nInput shaper damping, measured"), &img::wizard_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_DAMPING::click(IWindowMenu &) {
    open_screen<ScreenDamping>();
}

} // namespace od3d
