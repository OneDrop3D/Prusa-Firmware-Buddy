/// @file
#include "screen_speed_map_od3d.hpp"
#include "screen_calib_assistant_od3d.hpp" // confirm_stop_measure
#include "accelerometer_guide_od3d.hpp"
#include "menu_items_od3d.hpp"
#include "menu_od3d.hpp"
#include "thumbnail_od3d.hpp" // draw_pixels

#include <algorithm>
#include <cmath>
#include <common/marlin_vars.hpp>
#include <display.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    using speed_map::axis_count;
    using speed_map::speed_count;

    // Géométrie (zone des éléments : y 44 à 274)
    constexpr Rect16 panel_rect(int i) {
        return Rect16(style::margin, style::top + 82 * i, 464, 78);
    }
    constexpr Rect16 note_rect(int i) {
        return Rect16(panel_rect(i).Left() + 92, panel_rect(i).Top() + 7, 360, height(Font::small));
    }
    constexpr Rect16 bars_rect(int i) {
        return Rect16(20, panel_rect(i).Top() + 26, 440, 44);
    }
    constexpr Rect16 info_rect = Rect16(style::margin, 214, 318, 56);
    constexpr Rect16 action_body = Rect16(336, 222, 136, 44);
    static_assert(style::inside(panel_rect(0), bars_rect(0)) && style::inside(panel_rect(1), bars_rect(1)) && style::inside(panel_rect(1), note_rect(1)));
    static_assert(info_rect.Top() > panel_rect(1).Bottom() && style::inside(style::content, info_rect));
    static_assert(style::inside(style::content, style::with_ring(action_body)) && style::with_ring(action_body).Left() > info_rect.Right());

    constexpr Color bar_normal = style::text_faint;
    constexpr uint8_t tag_action = 0;

    /// Échelle commune aux deux axes : on compare X et Y
    float common_scale(const speed_map::Map &m) {
        float scale = 0.f;
        for (uint8_t a = 0; a < axis_count; ++a) {
            for (size_t i = 0; i < std::min<size_t>(m.points[a], speed_count); ++i) {
                if (std::isfinite(m.rms[a][i])) {
                    scale = std::max(scale, m.rms[a][i]);
                }
            }
        }
        return scale * 1.05f;
    }

    /// Mesure finie : les vitesses critiques ont un sens
    bool finished() {
        return !speed_map::is_running() && speed_map::status() == speed_map::Status::done;
    }

} // namespace

// ---- Panneau, barres ----

SpeedPanel::SpeedPanel(window_t *parent, Rect16 rect, const string_view_utf8 &title)
    : window_t(parent, rect)
    , title(title) {}

void SpeedPanel::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    draw_text(Rect16(r.Left() + 12, r.Top() + 7, 76, height(Font::small)), title, Font::small, style::text_strong, style::surface);
}

SpeedBars::SpeedBars(window_t *parent, Rect16 rect, uint8_t axis)
    : window_t(parent, rect)
    , axis(axis) {}

void SpeedBars::unconditionalDraw() {
    const speed_map::Map &m = speed_map::last();
    const Rect16 r = GetRect();
    const int w = r.Width(), h = r.Height();
    const float scale = common_scale(m);
    const size_t valid = std::min<size_t>(m.points[axis], speed_count);

    std::array<bool, speed_count> critical {};
    if (finished()) {
        std::array<uint8_t, speed_map::max_critical> idx;
        const size_t n = speed_map::critical(m, axis, idx);
        for (size_t i = 0; i < n; ++i) {
            critical[idx[i]] = true;
        }
    }
    std::array<int16_t, speed_count> heights;
    for (size_t i = 0; i < speed_count; ++i) {
        heights[i] = i < valid ? speed_map::bar_height(m.rms[axis][i], scale, int16_t(h - 1)) : int16_t(0);
    }

    draw_pixels(r, [&](int x, int y) {
        const int up = h - 1 - y; // hauteur depuis le bas
        if (up == 0) {
            return style::line;
        }
        // Une case par vitesse, 3 px d'écart entre les barres
        const size_t i = std::min<size_t>(size_t(x) * speed_count / size_t(w), speed_count - 1);
        const int next = int((i + 1) * size_t(w) / speed_count);
        if (x >= next - 3 || up > heights[i]) {
            return style::surface;
        }
        return critical[i] ? style::accent : bar_normal;
    });
}

// ---- Écran ----

ScreenSpeedMap::ScreenSpeedMap()
    : Screen(true)
    , header(this, _("Critical speeds"), true)
    , panel_x(this, panel_rect(0), _("X axis"))
    , note_x(this, note_rect(0), is_multiline::no)
    , bars_x(this, bars_rect(0), 0)
    , panel_y(this, panel_rect(1), _("Y axis"))
    , note_y(this, note_rect(1), is_multiline::no)
    , bars_y(this, bars_rect(1), 1)
    , info(this, info_rect, is_multiline::yes)
    , action(this, action_body, tag_action, Button::Look::primary, _("Start"), &img::print_16x16, style::screen)
    , tabs(this, section_tab()) {
    ClrMenuTimeoutClose(); // l'écran reste ouvert pendant la mesure
    for (window_text_t *note : { &note_x, &note_y }) {
        note->set_font(Font::small);
        note->SetAlignment(Align_t::RightCenter());
        note->SetTextColor(style::text_muted);
        note->SetBackColor(style::surface);
        note->SetPadding({ 0, 0, 0, 0 });
    }
    info.set_font(Font::small);
    info.SetAlignment(Align_t::LeftCenter());
    info.SetTextColor(style::text_soft);
    info.SetBackColor(style::screen);
    info.SetPadding({ 0, 0, 0, 0 });
    update(); // warned_accelerometer vaut true : aucun dialogue depuis le constructeur (Screens::Get() y est nul)
    action.SetFocus();
}

void ScreenSpeedMap::show_notes() {
    const speed_map::Map &m = speed_map::last();
    window_text_t *notes[axis_count] = { &note_x, &note_y };
    for (uint8_t a = 0; a < axis_count; ++a) {
        string_view_utf8 text = string_view_utf8::MakeNULLSTR();
        Color color = style::text_muted;
        if (finished()) {
            std::array<uint8_t, speed_map::max_critical> idx;
            const size_t n = speed_map::critical(m, a, idx);
            if (n == 0) {
                text = _("No critical speed");
            } else {
                char list[32];
                speed_map::format_speeds(std::span<const uint8_t>(idx.data(), n), speed_map::speed_start, speed_map::speed_step, list, sizeof(list));
                text = _("Avoid: %s mm/s").formatted(note_params[a], list);
                color = style::accent;
            }
        }
        notes[a]->SetTextColor(color);
        notes[a]->SetText(text);
        notes[a]->Invalidate();
    }
}

void ScreenSpeedMap::show_info() {
    using speed_map::Status;
    string_view_utf8 text;
    if (shown_pending) {
        text = _("Starting...");
    } else if (shown_running) {
        text = _("Measuring %c axis... %d%%").formatted(info_params, shown_axis == 0 ? 'X' : 'Y', int(shown_progress));
    } else {
        switch (shown_status) {
        case Status::idle:
        case Status::running:
            text = shown_homing ? _(shown_homing) : _("Attach the accelerometer to the extruder, then Start (X and Y, about 1 min).");
            break;
        case Status::done: {
            const speed_map::Map &m = speed_map::last();
            std::array<uint8_t, speed_map::max_critical> idx;
            const bool any = speed_map::critical(m, 0, idx) > 0 || speed_map::critical(m, 1, idx) > 0;
            // V-07 : la plage s'arrête où l'accéléromètre ne capte plus les vibrations des moteurs (speed_map_od3d.hpp)
            text = (any ? _("Orange bars: speeds to avoid. Measured up to %d mm/s, the limit of the accelerometer.")
                        : _("No critical speed up to %d mm/s, the limit of the accelerometer."))
                       .formatted(info_params, int(speed_map::speed_end));
            break;
        }
        case Status::stopped:
            text = _("Stopped");
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
        case Status::not_homed: // perdue depuis : jamais refaite accéléromètre fixé
            text = shown_homing ? _(shown_homing) : _("Attach the accelerometer to the extruder, then Start (X and Y, about 1 min).");
            break;
        case Status::no_room:
            text = _("No room on this axis");
            break;
        }
    }
    info.SetText(text);
    info.Invalidate();
}

void ScreenSpeedMap::update() {
    const bool now_running = speed_map::is_running();
    if (pending && (now_running || speed_map::run_count() != runs_at_request)) {
        pending = false;
    }
    const speed_map::Status s = speed_map::status();
    const uint8_t axis = speed_map::current_axis();
    const uint8_t progress = speed_map::progress_percent();
    const uint16_t revision = speed_map::revision();

    if (revision != shown_revision) {
        shown_revision = revision;
        bars_x.Invalidate();
        bars_y.Invalidate();
        show_notes();
    }

    const char *homing = now_running || pending ? nullptr : accelerometer_homing_text();
    if (now_running != shown_running || pending != shown_pending || s != shown_status || axis != shown_axis || progress != shown_progress || homing != shown_homing) {
        const bool was_busy = shown_running || shown_pending;
        shown_running = now_running;
        shown_pending = pending;
        shown_status = s;
        shown_axis = axis;
        shown_homing = homing;
        shown_progress = progress;
        show_info();
        const bool busy = now_running || pending;
        if (busy != was_busy) {
            action.set(busy ? Button::Look::outline : Button::Look::primary, busy ? _("Cancel") : _("Start"), busy ? &img::cancel_16x16 : &img::print_16x16);
        }
    }
    action.enable(now_running || pending || !marlin_client::is_printing());

    // ACC-1 (une fois par mesure) : refusée sans bouger, l'accéléromètre ne répond pas
    if (!now_running && !pending && s == speed_map::Status::no_accelerometer && !warned_accelerometer && !HasDialog()) {
        warned_accelerometer = true;
        if (ask_accelerometer_retry()) {
            start();
        }
    }
}

void ScreenSpeedMap::start() {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    auto &hotend = marlin_vars().active_hotend();
    if (hotend.temp_nozzle.get() > speed_map::max_nozzle_temperature || hotend.target_nozzle.get() > speed_map::max_nozzle_temperature) {
        MsgBoxWarning(_("Let the nozzle cool below 50°C, then start again."), Responses_Ok);
        return;
    }
    if (!accelerometer_homed_or_home()) {
        return; // origine d'abord, accéléromètre pas encore fixé : l'écran demande ensuite de le fixer
    }
    // ACC-1 : M1993 vérifie l'accéléromètre avant tout mouvement ; s'il ne répond pas, update() guide l'utilisateur
    const uint16_t before = speed_map::run_count();
    if (marlin_client::gcode_try("M1993") == marlin_client::GcodeTryResult::Submitted) {
        runs_at_request = before;
        pending = true;
        warned_accelerometer = false;
    }
}

void ScreenSpeedMap::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag == tag_action) {
            if (speed_map::is_running() || pending) {
                if (confirm_stop_measure()) { // pendant une série de l'assistant : il passe à la suivante
                    speed_map::request_stop();
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

MI_SPEED_MAP::MI_SPEED_MAP()
    : IWindowMenuItem(_("Critical speeds\nSpeeds that make the printer vibrate"), &img::wizard_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_SPEED_MAP::click(IWindowMenu &) {
    open_screen<ScreenSpeedMap>();
}

} // namespace od3d
