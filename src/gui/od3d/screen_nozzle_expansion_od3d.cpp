/// @file
#include "screen_nozzle_expansion_od3d.hpp"
#include "screen_calib_assistant_od3d.hpp" // confirm_stop_measure
#include "menu_items_od3d.hpp"
#include "menu_od3d.hpp"
#include "thumbnail_od3d.hpp" // draw_pixels

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <display.hpp>
#include <feature/nozzle_thermal_compensation/nozzle_thermal_compensation.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <MItem_tools.hpp> // gui_try_gcode_with_msg
#include <window_msgbox.hpp>

namespace od3d {

namespace ne = nozzle_expansion;
namespace comp = buddy::nozzle_thermal_compensation;

const char *nozzle_expansion_result_text(nozzle_expansion::Status status) {
    using ne::Status;
    switch (status) {
    case Status::idle:
    case Status::running:
        break;
    case Status::saved:
        return N_("Saved for this nozzle: Z follows it from now on. The saved bed mesh is cleared.");
    case Status::stopped:
        return N_("Stopped");
    case Status::printing:
        return N_("Not while printing.");
    case Status::filament:
        return N_("Unload the filament first: the nozzle must be empty.");
    case Status::heating:
        return N_("Heating failed.");
    case Status::homing:
        return N_("Homing failed.");
    case Status::probing:
        return N_("Probing failed.");
    case Status::spread:
        return N_("Rejected: probes too far apart. Clean the nozzle tip and the bed.");
    case Status::residual:
        return N_("Rejected: steps off the line. Retry with the printer warm.");
    case Status::slope:
        return N_("Rejected: implausible slope. Nothing changed.");
    }
    return N_("Empty nozzle, clear bed. Center probed 170-260\xC2\xB0""C and back, cleaned on the way up, ~10 min.");
}

namespace {

    // Géométrie (zone des éléments : y 46 à 274)
    constexpr Rect16 panel_rect = Rect16(style::margin, style::top, 196, 140);
    constexpr Rect16 chart_back_rect = Rect16(212, style::top, 260, 140);
    constexpr Rect16 chart_rect = Rect16(224, 70, 236, 96);
    constexpr Rect16 info_rect = Rect16(style::margin, 190, 464, 36);
    constexpr Rect16 button_body(int i) {
        return Rect16(18 + 150 * i, 232, 142, 40);
    }
    static_assert(chart_back_rect.Left() > panel_rect.Right() && chart_back_rect.Right() == style::screen_w - 1 - style::margin);
    static_assert(style::inside(chart_back_rect, chart_rect) && chart_rect.Bottom() + 16 <= chart_back_rect.Bottom());
    static_assert(info_rect.Top() > panel_rect.Bottom() && style::with_ring(button_body(0)).Top() > info_rect.Bottom());
    static_assert(style::inside(style::content, style::with_ring(button_body(1))));
    static_assert(chart_rect.Height() <= INT8_MAX);

    constexpr Color back_color = Color::from_raw(0x4FA3F7); ///< retour (comme Y dans l'amortissement)
    constexpr float chart_t0 = 160.f, chart_t1 = 270.f; ///< axe des températures du graphique [°C]
    constexpr float at_c = 250.f; ///< écart montré à cette température (PETG, ABS)

    constexpr uint8_t tag_action = 0;
    constexpr uint8_t tag_prusa = 1;

    /// Paliers mesurés et leur droite (au moins deux températures)
    struct Points {
        std::array<float, ne::stages> t {}, z {};
        uint8_t n = 0;
        ne::Line line;
    };
    Points points() {
        Points p;
        const ne::Measurement &m = ne::last();
        p.n = std::min<uint8_t>(m.done, ne::stages);
        for (uint8_t i = 0; i < p.n; ++i) {
            p.t[i] = float(ne::temperatures[i]);
            p.z[i] = m.stage[i].z_mm;
        }
        p.line = ne::fit(std::span(p.t).first(p.n), std::span(p.z).first(p.n));
        return p;
    }

    /// Écart avec Prusa à 250 °C, buse palpée à la référence (170 °C) [µm]
    float delta_um(float slope_mm_per_c) {
        return (slope_mm_per_c - comp::prusa_slope_mm_per_c) * (at_c - comp::reference_temperature_c) * 1000.f;
    }

    bool rejected(ne::Status s) {
        return s == ne::Status::spread || s == ne::Status::residual || s == ne::Status::slope;
    }

    void draw_panel_back(Rect16 r) {
        display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    }

    /// Hauteur des droites par colonne (calculée une fois par dessin) ; -1 : hors du graphique
    std::array<std::array<int8_t, chart_rect.Width() + 1>, 2> line_heights;

} // namespace

// ---- Panneau ----

ExpansionPanel::ExpansionPanel(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void ExpansionPanel::unconditionalDraw() {
    const Rect16 r = GetRect();
    draw_panel_back(r);
    const uint16_t hs = height(Font::small);
    const int16_t x = r.Left() + 12, w = r.Width() - 24;
    const Points p = points();
    const ne::Measurement &m = ne::last();
    const ne::Status s = ne::status();

    // En direct ou après une mesure : sa pente ; sinon celle qu'utilise la compensation
    const bool measured = p.line.valid;
    const float slope = measured ? p.line.slope : comp::slope_mm_per_c();
    const bool prusa_in_use = !measured && slope == comp::prusa_slope_mm_per_c;
    draw_text(Rect16(x, r.Top() + 6, w, hs), measured ? _("Measured") : (prusa_in_use ? _("Prusa model") : _("Saved")), Font::small, style::text_strong, style::surface);

    char text[32];
    snprintf(text, sizeof(text), "%.3f um/\xC2\xB0""C", static_cast<double>(slope * 1000.f));
    const Color value_color = rejected(s) && m.done == ne::stages ? style::warning : (ne::is_running() ? style::text_soft : style::text_strong);
    draw_text(Rect16(x, r.Top() + 26, w, height(Font::big)), string_view_utf8::MakeRAM(text), Font::big, value_color, style::surface);

    snprintf(text, sizeof(text), "Prusa %.3f um/\xC2\xB0""C", static_cast<double>(comp::prusa_slope_mm_per_c * 1000.f));
    draw_text(Rect16(x, r.Top() + 56, w, hs), string_view_utf8::MakeRAM(text), Font::small, style::text_muted, style::surface);

    char format[32];
    _("At 250\xC2\xB0""C: %+.1f um").copyToRAM(format, sizeof(format));
    snprintf(text, sizeof(text), format, static_cast<double>(delta_um(slope)));
    draw_text(Rect16(x, r.Top() + 80, w, hs), string_view_utf8::MakeRAM(text), Font::small, style::text, style::surface);

    if (m.done == ne::stages && std::isfinite(m.outcome.hysteresis_mm)) {
        // Refusée pour la dispersion : celle-ci ; sinon l'écart entre l'aller et le retour
        const bool spread = s == ne::Status::spread;
        (spread ? _("Spread %.1f um") : _("Hysteresis %.1f um")).copyToRAM(format, sizeof(format));
        snprintf(text, sizeof(text), format, static_cast<double>((spread ? m.outcome.spread_mm : m.outcome.hysteresis_mm) * 1000.f));
        draw_text(Rect16(x, r.Top() + 104, w, hs), string_view_utf8::MakeRAM(text), Font::small, style::text_muted, style::surface);
    }
}

// ---- Graphique ----

ExpansionChartBack::ExpansionChartBack(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void ExpansionChartBack::unconditionalDraw() {
    const Rect16 r = GetRect();
    draw_panel_back(r);
    const uint16_t hs = height(Font::small);
    const int16_t y = r.Top() + 5;
    // Légende : aller, retour, droite de Prusa (la mesurée est blanche)
    const int16_t x = r.Left() + 12;
    display::fill_rect(Rect16(x, y + 4, 8, 8), style::accent);
    draw_text(Rect16(x + 12, y, 54, hs), _("up"), Font::small, style::text, style::surface);
    display::fill_rect(Rect16(x + 70, y + 4, 8, 8), back_color);
    draw_text(Rect16(x + 82, y, 60, hs), _("back"), Font::small, style::text, style::surface);
    display::fill_rect(Rect16(x + 146, y + 7, 12, 2), style::text_faint);
    draw_text(Rect16(x + 162, y, 72, hs), string_view_utf8::MakeCPUFLASH("Prusa"), Font::small, style::text_muted, style::surface);
    // Températures des paliers sous l'axe
    for (const int16_t t : { int16_t(170), int16_t(215), int16_t(260) }) {
        const int16_t cx = int16_t(chart_rect.Left() + std::lround((t - chart_t0) / (chart_t1 - chart_t0) * (chart_rect.Width() - 1)));
        char label[8];
        snprintf(label, sizeof(label), "%d", int(t));
        draw_text(Rect16(cx - 20, chart_rect.Bottom() + 2, 40, hs), string_view_utf8::MakeRAM(label), Font::small, style::text_muted, style::surface, Align_t::Center());
    }
}

ExpansionChart::ExpansionChart(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void ExpansionChart::unconditionalDraw() {
    const Rect16 r = GetRect();
    const int w = r.Width(), h = r.Height();
    const Points p = points();

    // Droites : la mesurée, et celle de Prusa passant par elle (ou par le 1er palier) à 170 °C
    const float z_ref = p.line.valid ? p.line.slope * comp::reference_temperature_c + p.line.intercept : (p.n ? p.z[0] : 0.f);
    const auto prusa_z = [&](float t) { return z_ref + comp::prusa_slope_mm_per_c * (t - comp::reference_temperature_c); };
    float lo = std::min(prusa_z(chart_t0), prusa_z(chart_t1)), hi = std::max(prusa_z(chart_t0), prusa_z(chart_t1));
    for (uint8_t i = 0; i < p.n; ++i) {
        lo = std::min(lo, p.z[i]);
        hi = std::max(hi, p.z[i]);
    }
    if (p.line.valid) {
        for (const float t : { chart_t0, chart_t1 }) {
            lo = std::min(lo, p.line.slope * t + p.line.intercept);
            hi = std::max(hi, p.line.slope * t + p.line.intercept);
        }
    }
    const float margin = std::max(0.002f, (hi - lo) * 0.06f);
    lo -= margin;
    hi += margin;
    const auto row = [&](float z) { return std::isfinite(z) ? int(std::lround((z - lo) / (hi - lo) * float(h - 1))) : -1; };
    const auto col_t = [&](int x) { return chart_t0 + (chart_t1 - chart_t0) * float(x) / float(w - 1); };
    for (int x = 0; x <= w; ++x) {
        const float t = col_t(x);
        line_heights[0][x] = int8_t(p.line.valid ? std::clamp(row(p.line.slope * t + p.line.intercept), -1, h - 1) : -1);
        line_heights[1][x] = int8_t(p.n ? std::clamp(row(prusa_z(t)), -1, h - 1) : -1);
    }
    std::array<int16_t, ne::stages> px {}, py {};
    for (uint8_t i = 0; i < p.n; ++i) {
        px[i] = int16_t(std::lround((p.t[i] - chart_t0) / (chart_t1 - chart_t0) * float(w - 1)));
        py[i] = int16_t(row(p.z[i]));
    }

    draw_pixels(r, [&](int x, int y) {
        const int up = h - 1 - y;
        for (uint8_t i = 0; i < p.n; ++i) {
            if (std::abs(x - px[i]) <= 2 && std::abs(up - py[i]) <= 2) {
                return i < 3 ? style::accent : back_color; // 170, 215, 260 à l'aller, puis le retour
            }
        }
        for (uint8_t k = 0; k < 2; ++k) {
            const int y0 = line_heights[k][x];
            const int y1 = line_heights[k][x + 1] < 0 ? y0 : line_heights[k][x + 1];
            if (y0 >= 0 && up >= std::min(y0, y1) && up <= std::max(y0, y1)) {
                return k == 0 ? style::text_strong : style::text_faint;
            }
        }
        return up == 0 ? style::line : style::surface;
    });
}

// ---- Écran ----

ScreenNozzleExpansion::ScreenNozzleExpansion()
    : Screen(true)
    , header(this, _("Nozzle expansion"), true)
    , panel(this, panel_rect)
    , chart_back(this, chart_back_rect)
    , chart(this, chart_rect)
    , info(this, info_rect, is_multiline::yes)
    , action(this, button_body(0), tag_action, Button::Look::primary, _("Start"), &img::print_16x16, style::screen, 10)
    , prusa_button(this, button_body(1), tag_prusa, Button::Look::outline, _("Prusa model"), &img::arrow_left_16x16, style::screen, 10)
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

void ScreenNozzleExpansion::show_info() {
    string_view_utf8 text;
    if (shown_pending) {
        text = _("Starting...");
    } else if (shown_running) {
        const uint8_t i = ne::current_stage();
        const int t = ne::temperatures[i % ne::stages];
        switch (ne::phase()) {
        case ne::Phase::bed:
            text = _("Bed to 60\xC2\xB0""C and steady, homing, nozzle cleaning... %d%%").formatted(info_params, int(shown_progress));
            break;
        case ne::Phase::heating:
            text = _("%d/%d: nozzle to %d\xC2\xB0""C, waiting until steady... %d%%").formatted(info_params, i + 1, int(ne::stages), t, int(shown_progress));
            break;
        case ne::Phase::probing:
            text = _("%d/%d: probing at %d\xC2\xB0""C... %d%%").formatted(info_params, i + 1, int(ne::stages), t, int(shown_progress));
            break;
        }
    } else {
        text = _(nozzle_expansion_result_text(shown_status));
    }
    info.SetText(text);
    info.Invalidate();
}

void ScreenNozzleExpansion::update() {
    const bool now_running = ne::is_running();
    if (pending && (now_running || ne::run_count() != runs_at_request)) {
        pending = false;
    }
    const ne::Status s = ne::status();
    const uint8_t progress = ne::progress_percent();
    const uint16_t revision = ne::revision();
    const float slope = comp::slope_mm_per_c();

    const bool new_revision = revision != shown_revision; // aussi d'une phase à l'autre (plateau, chauffe, palpage)
    if (new_revision || slope != shown_slope) {
        shown_revision = revision;
        shown_slope = slope;
        panel.Invalidate();
        chart.Invalidate();
    }

    if (new_revision || now_running != shown_running || pending != shown_pending || s != shown_status || progress != shown_progress) {
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

    const bool idle = !now_running && !pending && !marlin_client::is_printing();
    action.enable(now_running || pending || !marlin_client::is_printing());
    prusa_button.enable(idle && slope != comp::prusa_slope_mm_per_c);
}

void ScreenNozzleExpansion::start() {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    if (MsgBoxQuestion(_("Unload the filament and clear the bed: the nozzle probes the bed center up to 260°C. Start?"), Responses_YesNo) != Response::Yes) {
        return;
    }
    const uint16_t before = ne::run_count();
    if (marlin_client::gcode_try("M2003") == marlin_client::GcodeTryResult::Submitted) {
        runs_at_request = before;
        pending = true;
    }
}

void ScreenNozzleExpansion::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag == tag_action) {
            if (ne::is_running() || pending) {
                if (confirm_stop_measure()) { // pendant une série de l'assistant : il passe à la suivante
                    ne::request_stop();
                }
            } else {
                start();
            }
        } else if (tag == tag_prusa) {
            gui_try_gcode_with_msg("M2003 R"); // comme « Valeur Prusa » de l'amortissement
        }
        update();
        return;
    }

    default:
        break;
    }
    Screen::windowEvent(sender, event, param);
}

// ---- Réglages › Fonctions OneDrop3D › Température de la buse ----

MI_NOZZLE_EXPANSION::MI_NOZZLE_EXPANSION()
    : IWindowMenuItem(_("Measure the expansion\nProbes the bed from 170 to 260°C"), &img::wizard_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_NOZZLE_EXPANSION::click(IWindowMenu &) {
    open_screen<ScreenNozzleExpansion>();
}

} // namespace od3d
