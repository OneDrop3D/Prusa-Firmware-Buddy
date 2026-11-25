/// @file
#include "screen_belts_od3d.hpp"
#include "screen_calib_assistant_od3d.hpp" // confirm_stop_measure
#include "accelerometer_guide_od3d.hpp"
#include <option/has_attachable_accelerometer.h>
#include "menu_items_od3d.hpp"
#include "menu_od3d.hpp"
#include "thumbnail_od3d.hpp" // draw_pixels

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <common/marlin_vars.hpp>
#include <display.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <ScreenHandler.hpp>
#include <span>
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    // Géométrie (zone des éléments : y 44 à 274)
    constexpr Rect16 gauge_rect(int i) {
        return Rect16(style::margin + 236 * i, style::top, 228, 76);
    }
    constexpr Rect16 spectrum_rect = Rect16(style::margin, 128, 464, 90);
    constexpr Rect16 curve_rect = Rect16(20, 152, 440, 58);
    constexpr Rect16 info_rect = Rect16(style::margin, 224, 318, 48);
    constexpr Rect16 action_body = Rect16(336, 226, 136, 44);
    static_assert(style::inside(style::content, gauge_rect(1)) && gauge_rect(1).Right() == style::screen_w - 1 - style::margin);
    static_assert(style::inside(spectrum_rect, curve_rect) && spectrum_rect.Top() > gauge_rect(0).Bottom());
    static_assert(info_rect.Top() > spectrum_rect.Bottom() && style::inside(style::content, info_rect));
    static_assert(style::inside(style::content, style::with_ring(action_body)) && style::with_ring(action_body).Left() > info_rect.Right());

    constexpr Color ok_green = style::curve_green;
    constexpr Color band_green = style::band_green; ///< plage de Prusa sur la jauge
    constexpr Color band_tint = style::band_tint; ///< plage de Prusa sous la courbe
    constexpr Color curve_b = style::curve_blue;
    constexpr Color curve_colors[belts::belt_count] = { style::accent, curve_b };

    constexpr uint8_t tag_action = 0;

    bool live_on_open = false; ///< open_belts_live() : le réglage en direct part à l'ouverture
    /// Conseil de Prusa (les deux vis, serrer / desserrer) d'après les pics de la dernière mesure
    belts::Advice current_advice() {
        const belts::Measurement &m = belts::last();
        return belts::advise(m.peaks[0].frequency, m.peaks[1].frequency, belts::targets());
    }

    /// Bornes de la jauge : la plage de Prusa élargie (CORE One : 80 à 108 Hz)
    float gauge_lo(const belts::Targets &t) {
        return t.result_min - 9.f;
    }
    float gauge_hi(const belts::Targets &t) {
        return t.result_max + 10.f;
    }

    /// Hauteur de la courbe par colonne (une de plus pour le segment de la dernière), -1 sans point : calculées une fois
    /// par dessin plutôt qu'à chaque pixel
    std::array<std::array<int8_t, curve_rect.Width() + 1>, belts::belt_count> curve_heights;
    static_assert(curve_rect.Height() <= INT8_MAX);

    void draw_panel_back(Rect16 r) {
        display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    }

} // namespace

// ---- Jauge ----

BeltGauge::BeltGauge(window_t *parent, Rect16 rect, const string_view_utf8 &title)
    : window_t(parent, rect)
    , title(title) {}

void BeltGauge::set(float set_frequency, bool set_found) {
    const bool same_nan = std::isnan(set_frequency) && std::isnan(frequency);
    if ((same_nan || set_frequency == frequency) && set_found == found) {
        return;
    }
    frequency = set_frequency;
    found = set_found;
    Invalidate();
}

void BeltGauge::unconditionalDraw() {
    const Rect16 r = GetRect();
    draw_panel_back(r);
    const uint16_t hs = height(Font::small);
    draw_text(Rect16(r.Left() + 12, r.Top() + 7, 92, hs), title, Font::small, style::text_strong, style::surface);

    const belts::Targets t = belts::targets();
    const bool has = !std::isnan(frequency);
    string_view_utf8 state = string_view_utf8::MakeNULLSTR();
    Color value_color = style::text_faint;
    if (has && !found) {
        state = _("inconclusive");
        value_color = style::text_muted;
    } else if (has && belts::advice_validated) {
        switch (belts::zone(frequency, t)) {
        case belts::Zone::low:
            state = _("too low");
            value_color = style::warning;
            break;
        case belts::Zone::ok:
            state = _("in range");
            value_color = ok_green;
            break;
        case belts::Zone::high:
            state = _("too high");
            value_color = style::warning;
            break;
        }
    }
    draw_text(Rect16(r.Right() - 12 - 112, r.Top() + 7, 112, hs), state, Font::small, value_color, style::surface, Align_t::RightCenter());

    char value[16];
    if (has) {
        snprintf(value, sizeof(value), "%.1f Hz", static_cast<double>(frequency));
    } else {
        strlcpy(value, "-- Hz", sizeof(value));
    }
    draw_text(Rect16(r.Left() + 12, r.Top() + 25, r.Width() - 24, height(Font::big)), string_view_utf8::MakeRAM(value), Font::big,
        has ? style::text_strong : style::text_faint, style::surface);

    // Barre : plage de Prusa en vert (mesure validée seulement), repère blanc de 3 px sur la fréquence mesurée
    const Rect16 bar(r.Left() + 12, r.Top() + 54, r.Width() - 24, 12);
    const int16_t w = bar.Width();
    const float lo = gauge_lo(t), hi = gauge_hi(t);
    const int16_t band0 = belts::gauge_x(t.result_min, lo, hi, w);
    const int16_t band1 = belts::gauge_x(t.result_max, lo, hi, w);
    const int16_t mark = has ? belts::gauge_x(frequency, lo, hi, w) : int16_t(-10);
    const Color mark_color = found ? style::text_strong : style::text_muted;
    draw_pixels(bar, [&](int x, int) {
        if (std::abs(x - mark) <= 1) {
            return mark_color;
        }
        return (belts::advice_validated && x >= band0 && x <= band1) ? band_green : style::secondary;
    });
}

// ---- Spectre ----

SpectrumPanel::SpectrumPanel(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void SpectrumPanel::unconditionalDraw() {
    const Rect16 r = GetRect();
    draw_panel_back(r);
    const uint16_t hs = height(Font::small);
    const int16_t y = r.Top() + 6;
    draw_text(Rect16(r.Left() + 12, y, 150, hs), _("Spectrum"), Font::small, style::text_strong, style::surface);

    // Légende : A orange, B bleu, puis la plage balayée
    const int16_t right = r.Right() - 12;
    char range[16];
    snprintf(range, sizeof(range), "%d-%d Hz", int(belts::sweep_start_hz), int(belts::sweep_end_hz));
    draw_text(Rect16(right - 81, y, 81, hs), string_view_utf8::MakeRAM(range), Font::small, style::text_muted, style::surface, Align_t::RightCenter());
    for (uint8_t b = 0; b < belts::belt_count; ++b) {
        const int16_t x = right - 157 + 32 * b;
        display::fill_rect(Rect16(x, y + 3, 10, 10), curve_colors[b]);
        draw_text(Rect16(x + 14, y, 12, hs), string_view_utf8::MakeCPUFLASH(b == 0 ? "A" : "B"), Font::small, style::text, style::surface);
    }
}

SpectrumCurve::SpectrumCurve(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void SpectrumCurve::unconditionalDraw() {
    const belts::Measurement &m = belts::last();
    const belts::Targets t = belts::targets();
    const Rect16 r = GetRect();
    const int w = r.Width(), h = r.Height();

    // Même échelle pour A et B : on compare les deux courbes
    float scale = 0.f;
    for (uint8_t b = 0; b < belts::belt_count; ++b) {
        for (size_t i = 0; i < std::min<size_t>(m.points[b], belts::sweep_points); ++i) {
            if (std::isfinite(m.spectra[b][i])) {
                scale = std::max(scale, m.spectra[b][i]);
            }
        }
    }
    scale *= 1.08f; // un peu d'air au-dessus du pic
    for (uint8_t b = 0; b < belts::belt_count; ++b) {
        for (int x = 0; x <= w; ++x) {
            curve_heights[b][x] = int8_t(belts::curve_height(m.spectra[b], m.points[b], x, w, h, scale));
        }
    }

    const int16_t band0 = belts::plot_x(t.result_min, belts::sweep_start_hz, belts::sweep_end_hz, w);
    const int16_t band1 = belts::plot_x(t.result_max, belts::sweep_start_hz, belts::sweep_end_hz, w);
    draw_pixels(r, [&](int x, int y) {
        const int up = h - 1 - y; // hauteur depuis le bas
        for (uint8_t b = 0; b < belts::belt_count; ++b) {
            const int y0 = curve_heights[b][x];
            if (y0 < 0) {
                continue;
            }
            const int y1 = curve_heights[b][x + 1] < 0 ? y0 : curve_heights[b][x + 1];
            // Segment vertical jusqu'à la colonne suivante : pas de trou dans les pentes raides ; 2 px d'épaisseur
            if (up >= std::min(y0, y1) && up <= std::max(y0, y1) + 1) {
                return curve_colors[b];
            }
        }
        if (up == 0) {
            return style::line;
        }
        return (belts::advice_validated && x >= band0 && x <= band1) ? band_tint : style::surface;
    });
}

// ---- Écran ----

ScreenBelts::ScreenBelts()
    : Screen(true)
    , header(this, _("Belt tension"), true)
    , gauge_a(this, gauge_rect(0), _("Belt A"))
    , gauge_b(this, gauge_rect(1), _("Belt B"))
    , spectrum_panel(this, spectrum_rect)
    , curve(this, curve_rect)
    , info(this, info_rect, is_multiline::yes)
    , action(this, action_body, tag_action, Button::Look::primary, _("Start"), &img::print_16x16, style::screen)
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

void ScreenBelts::show_info() {
    using belts::Stage;
    using belts::Status;
    const belts::Measurement &m = belts::last();
    string_view_utf8 text;

    if (shown_pending) {
        text = _("Starting...");
    } else if (shown_running) {
        switch (shown_stage) {
        case Stage::preparing:
            text = _("Moving to the measuring position...");
            break;
        case Stage::calibrating:
            text = _("Calibrating accelerometer... %d%%").formatted(info_params, int(shown_progress));
            break;
        case Stage::belt_a:
        case Stage::belt_b:
            if (belts::comparing()) { // V-03 : le mode en cours sur les quatre, l'avancement de toute la comparaison
                text = _("Comparison %d/%d, belt %c: %d%%")
                           .formatted(info_params, int(belts::compared()) + 1, int(belts::compare_modes.size()), shown_stage == Stage::belt_a ? 'A' : 'B', int(shown_progress));
                break;
            }
            if (live_requested) {
                // Réglage en direct : le conseil suit chaque tour ; il change de sens dès qu'on dépasse la cible
                const belts::Advice advice = current_advice();
                if (advice.verdict == belts::Verdict::misaligned) {
                    text = _("A and B differ by more than %d Hz: check the gantry alignment.").formatted(info_params, int(belts::targets().diff_limit));
                } else if (advice.verdict == belts::Verdict::ok) {
                    text = _("Both belts in range (%d/%d in a row): leave the screws, Done stops.").formatted(info_params, int(belts::live_in_range()), int(belts::live_ok_rounds));
                } else {
                    (advice.turn_eighths > 0 ? _("tighten") : _("loosen")).copyToRAM(word.data(), word.size());
                    belts::format_turns(advice.turn_eighths, turns.data(), turns.size());
                    text = _("Both screws: %s %s turn. Measured again every ~10 s, Done stops.").formatted(info_params, word.data(), turns.data());
                }
                break;
            }
            // Retour du 23/11 (« un drôle de bruit au début ») : c'est l'excitation, plus forte aux basses fréquences
            text = _("Belt %c rings, the motor hums: normal. %d%%").formatted(info_params, shown_stage == Stage::belt_a ? 'A' : 'B', int(shown_progress));
            break;
        }
    } else {
        switch (shown_status) {
        case Status::idle:
        case Status::running:
            text = shown_homing ? _(shown_homing) : (HAS_ATTACHABLE_ACCELEROMETER() ? _("Attach the accelerometer to the extruder, then Start: 4 ways of measuring, about 5 min.") : _("Start: 4 ways of measuring, about 5 min."));
            break;
        case Status::done: {
            if (!belts::advice_validated) { // seulement les fréquences (jauges) ; le réglage chez Prusa
                text = _("Measurement being validated: adjust with the Prusa app or the Prusa wizard (M961).");
                break;
            }
            if (!m.peaks[0].found || !m.peaks[1].found) {
                // Jamais de conseil sur une mesure non concluante (retour du 23/11 : 136 Hz lus 80 Hz, « serrer »)
                text = _("Inconclusive (belt %c): peak out of range, belt maybe too tight or too loose.").formatted(info_params, m.peaks[0].found ? 'B' : 'A');
                break;
            }
            const belts::Targets t = belts::targets();
            const belts::Advice advice = belts::advise(m.peaks[0].frequency, m.peaks[1].frequency, t);
            switch (advice.verdict) {
            case belts::Verdict::ok:
                text = _("Both belts are in range: no need to turn the screws.");
                break;
            case belts::Verdict::misaligned:
                text = _("A and B differ by more than %d Hz: check the gantry alignment.").formatted(info_params, int(t.diff_limit));
                break;
            case belts::Verdict::adjust:
                // Textes « serrer » / « desserrer » de l'assistant de Prusa
                (advice.turn_eighths > 0 ? _("tighten") : _("loosen")).copyToRAM(word.data(), word.size());
                belts::format_turns(advice.turn_eighths, turns.data(), turns.size());
                text = _("Both screws: %s %s turn, then measure again.").formatted(info_params, word.data(), turns.data());
                break;
            }
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
            text = shown_homing ? _(shown_homing) : (HAS_ATTACHABLE_ACCELEROMETER() ? _("Attach the accelerometer to the extruder, then Start: 4 ways of measuring, about 5 min.") : _("Start: 4 ways of measuring, about 5 min."));
            break;
        }
    }
    info.SetText(text);
    info.Invalidate();
}

void ScreenBelts::update() {
    const bool now_running = belts::is_running();
    if (pending && (now_running || belts::run_count() != runs_at_request)) {
        pending = false;
    }
    const belts::Status s = belts::status();
    const belts::Stage stage = belts::stage();
    const uint8_t progress = belts::progress_percent();
    const uint16_t revision = belts::revision();

    if (revision != shown_revision) {
        shown_revision = revision;
        if (live_requested && now_running) {
            shown_progress = 0xff; // le conseil suit chaque pic : texte redessiné plus bas
        }
        const belts::Measurement &m = belts::last();
        // Fréquence seulement quand la courroie est balayée et son pic calculé, sinon « -- »
        const auto frequency = [&](uint8_t b) {
            return m.points[b] == belts::sweep_points && m.peaks[b].frequency > 0.f ? m.peaks[b].frequency : NAN;
        };
        gauge_a.set(frequency(0), m.peaks[0].found);
        gauge_b.set(frequency(1), m.peaks[1].found);
        curve.Invalidate();
    }

    const char *homing = now_running || pending ? nullptr : accelerometer_homing_text();
    const uint8_t in_range = belts::live_in_range();
    if (now_running != shown_running || pending != shown_pending || s != shown_status || stage != shown_stage || progress != shown_progress || homing != shown_homing
        || in_range != shown_in_range) {
        shown_in_range = in_range;
        const bool was_busy = shown_running || shown_pending;
        shown_running = now_running;
        shown_pending = pending;
        shown_status = s;
        shown_stage = stage;
        shown_homing = homing;
        shown_progress = progress;
        const bool busy = now_running || pending;
        const bool full_done = was_busy && !busy && !live_requested && s == belts::Status::done;
        if (!busy) {
            live_requested = false;
        }
        show_info();
        if (busy != was_busy) {
            action.set(busy ? Button::Look::outline : Button::Look::primary, busy ? (live_requested ? _("Done") : _("Cancel")) : _("Start"), busy ? &img::cancel_16x16 : &img::print_16x16);
        }
        // V-03 : la comparaison finie, tous ses pics, à confronter à l'appli de Prusa (aussi au bilan de l'assistant)
        if (full_done && belts::compared() == belts::compare_modes.size() && !HasDialog() && !calibration_series_running()) {
            static char peaks[320]; // la boîte le lit tant qu'elle est ouverte
            belt_peaks_text(peaks);
            MsgBoxInfo(string_view_utf8::MakeRAM(peaks), Responses_Ok);
        }
        // Hors de la plage après une mesure complète : le réglage en direct est proposé (retour du 23/11)
        const belts::Measurement &m = belts::last();
        if (belts::advice_validated && full_done && !HasDialog() && m.peaks[0].found && m.peaks[1].found && current_advice().verdict == belts::Verdict::adjust && !calibration_series_running()
            && MsgBoxQuestion(_("Belts out of range. Adjust them live? Both belts are measured again every ~10 s while you turn both screws (Prusa guide: prusa.io/core-belt-calibration-tensioning)."), Responses_YesNo)
                == Response::Yes) {
            start_live();
        }
    }
    if (live_on_open && !HasDialog()) {
        live_on_open = false;
        start_live();
    }
    action.enable(now_running || pending || !marlin_client::is_printing());

    // ACC-1 (une fois par mesure) : refusée sans bouger, l'accéléromètre ne répond pas
    if (!now_running && !pending && s == belts::Status::no_accelerometer && !warned_accelerometer && !HasDialog()) {
        warned_accelerometer = true;
        if (ask_accelerometer_retry()) {
            start();
        }
    }
}

void ScreenBelts::start() {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    auto &hotend = marlin_vars().active_hotend();
    if (hotend.temp_nozzle.get() > belts::max_nozzle_temperature || hotend.target_nozzle.get() > belts::max_nozzle_temperature) {
        MsgBoxWarning(_("Let the nozzle cool below 50°C, then start again."), Responses_Ok);
        return;
    }
    if (!accelerometer_homed_or_home()) {
        return; // origine d'abord, accéléromètre pas encore fixé : l'écran demande ensuite de le fixer
    }
    // ACC-1 : M1992 vérifie l'accéléromètre avant tout mouvement ; s'il ne répond pas, update() guide l'utilisateur.
    // V-03 : tant que la mesure n'est pas validée (advice_validated), la comparaison de tous les modes
    const uint16_t before = belts::run_count();
    if (marlin_client::gcode_try("M1992 C") == marlin_client::GcodeTryResult::Submitted) {
        runs_at_request = before;
        pending = true;
        warned_accelerometer = false;
    }
}

void ScreenBelts::start_live() {
    if (marlin_client::is_printing() || belts::is_running() || !accelerometer_homed_or_home()) {
        return; // sans origine : l'écran la fait prendre d'abord, accéléromètre retiré
    }
    const uint16_t before = belts::run_count();
    if (marlin_client::gcode_try("M1992 L") == marlin_client::GcodeTryResult::Submitted) {
        runs_at_request = before;
        pending = true;
        live_requested = true;
        warned_accelerometer = false;
    }
}

void open_belts_live() {
    live_on_open = true;
    Screens::Access()->Open(ScreenFactory::Screen<ScreenBelts>);
}

void belt_peaks_text(std::span<char> out) {
    // Ordre de compare_modes ; deux pics par courroie, du plus proéminent au moins, tient sur une ligne (le CSV a les trois)
    static constexpr const char *labels[belts::compare_modes.size()] = { N_("After the stop (R):"), N_("Prusa excitation (P):"), N_("2nd harmonic (I2):"), N_("Usual measurement:") };
    constexpr size_t shown = 2;
    size_t n = 0;
    const auto put = [&](const char *s) { // coupé à la fin du tampon
        n = std::min(n + strlcpy(out.data() + n, s, out.size() - n), out.size() - 1);
    };
    out[0] = '\0';
    if (belts::compared() == 0) {
        return;
    }
    char text[64];
    _("Peaks in Hz, to compare with the Prusa app:").copyToRAM(text, sizeof(text));
    put(text);
    const belts::Comparison &c = belts::comparison();
    for (uint8_t k = 0; k < belts::compared(); ++k) {
        put("\n");
        _(labels[k]).copyToRAM(text, sizeof(text));
        put(text);
        for (uint8_t b = 0; b < belts::belt_count; ++b) {
            snprintf(text, sizeof(text), b ? ", %c" : " %c", 'A' + b);
            put(text);
            if (!std::isfinite(c[k][b][0])) {
                put(" --");
            }
            for (size_t i = 0; i < shown && std::isfinite(c[k][b][i]); ++i) {
                snprintf(text, sizeof(text), " %.1f", double(c[k][b][i]));
                put(text);
            }
        }
    }
}

void ScreenBelts::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag == tag_action) {
            if (belts::is_running() || pending) {
                if (live_requested || confirm_stop_measure()) { // série de l'assistant : il passe à la suivante
                    belts::request_stop();
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

MI_BELT_TENSION::MI_BELT_TENSION()
    : IWindowMenuItem(_("Belt tension\nMeasured with the accelerometer"), &img::wizard_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_BELT_TENSION::click(IWindowMenu &) {
    open_screen<ScreenBelts>();
}

} // namespace od3d
