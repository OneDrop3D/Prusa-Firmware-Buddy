/// @file
#include "screen_pa_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <filament.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    // Sous l'en-tête de 40 px : carte de la valeur et courbe côte à côte, une ligne d'état, deux rangées de boutons
    constexpr Rect16 card_rect = { style::margin, style::top, 184, 128 };
    constexpr Rect16 graph_rect = { 200, style::top, 272, 128 };
    constexpr Rect16 note_rect = { style::margin, 180, 464, 16 };
    constexpr Rect16 wins_body = { style::margin, 202, 228, 30 };
    constexpr Rect16 csv_body = { 244, 202, 228, 30 };
    constexpr Rect16 measure_body = { style::margin, 238, 228, 34 };
    constexpr Rect16 second_body = { 244, 238, 228, 34 };
    static_assert(card_rect.Bottom() < note_rect.Top() && note_rect.Bottom() < style::with_ring(wins_body).Top());
    static_assert(style::with_ring(wins_body).Bottom() < style::with_ring(measure_body).Top());
    static_assert(style::inside(style::content, style::with_ring(second_body)) && style::inside(style::content, graph_rect));

    enum Tag : uint8_t {
        tag_wins,
        tag_csv,
        tag_measure,
        tag_second,
    };

    constexpr Color color_low = Color::from_raw(0x4A90D9); ///< K le plus bas : bleu
    constexpr Color color_best = style::accent;
    constexpr Color color_high = style::danger;
    constexpr Color color_command = Color::from_raw(0x8A8E94);

    /// Courbes exportées sur la clé : choix gardé jusqu'au redémarrage
    bool csv_on = true;

    /// « 0.0350 » ; « - » sans valeur
    void format_k(char *buf, size_t size, uint16_t k_e4) {
        if (k_e4 == pa::k_none) {
            strlcpy(buf, "-", size);
        } else {
            snprintf(buf, size, "%.4f", double(pa::k_from_e4(k_e4)));
        }
    }

    bool loaded(uint8_t slot) {
        return Slot::read(slot).loaded;
    }

} // namespace

void open_pa(uint8_t slot) {
    Screens::Access()->Open(ScreenFactory::ScreenWithArg<ScreenPa>(slot));
}

// ---- Carte de la valeur ----

PaValueCard::PaValueCard(window_t *parent, Rect16 rect, uint8_t slot)
    : window_t(parent, rect)
    , slot(slot) {}

void PaValueCard::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    const int16_t x = r.Left() + 12;
    const uint16_t w = r.Width() - 24;
    draw_text(Rect16(x, r.Top() + 8, w, height(Font::small)), _("Pressure advance"), Font::small, style::text_muted, style::surface);

    const pa::Record rec = pa::record(slot);
    char value[12];
    format_k(value, sizeof(value), rec.k_e4);
    draw_text(Rect16(x, r.Top() + 30, w, height(Font::big)), string_view_utf8::MakeRAM(value), Font::big, rec.has_k() ? style::text_strong : style::text_faint, style::surface);

    StringViewUtf8Parameters<4> params;
    if (rec.has_k()) {
        draw_text(Rect16(x, r.Top() + 60, w, height(Font::small)), _("Confidence %d/100").formatted(params, int(rec.confidence)), Font::small, style::text_soft, style::surface);
    } else {
        draw_text(Rect16(x, r.Top() + 60, w, height(Font::small)), _("Not measured"), Font::small, style::text_soft, style::surface);
    }

    char name[24];
    atelier::spool_name(name, FilamentType::for_tool(VirtualToolIndex::from_raw(slot)));
    draw_text(Rect16(x, r.Top() + 82, w, height(Font::small)), string_view_utf8::MakeRAM(name), Font::small, style::text_muted, style::surface);

    if (pa::needs_measure(slot)) {
        draw_text(Rect16(x, r.Top() + 104, w, height(Font::small)), _("PA to measure"), Font::small, style::warning, style::surface);
    }
}

// ---- Courbe de force ----

PaGraph::PaGraph(window_t *parent, Rect16 rect, uint8_t slot)
    : window_t(parent, rect)
    , slot(slot) {}

void PaGraph::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    const pa::LastRun &run = pa::last_run();
    const bool has = !pa::is_running() && run.slot == slot && run.best.step_s > 0;
    if (!has) {
        draw_text(Rect16(r.Left() + 8, r.Top() + (r.Height() - height(Font::small)) / 2, r.Width() - 16, height(Font::small)), _("No force curve yet"), Font::small, style::text_faint, style::surface, Align_t::Center());
        return;
    }

    // Légende : les trois K, chacun dans la couleur de sa courbe
    struct Curve {
        const pa::Waveform *wave;
        float k;
        Color color;
    };
    const Curve curves[] = {
        { &run.low, run.k_low, color_low },
        { &run.high, run.k_high, color_high },
        { &run.best, run.k_best, color_best }, // dernière : dessinée par-dessus
    };
    const uint16_t cell = (r.Width() - 16) / 3;
    for (int i = 0; i < 3; ++i) {
        const Curve &c = curves[i == 1 ? 2 : (i == 2 ? 1 : 0)]; // ordre de lecture : bas, meilleur, haut
        char text[12];
        snprintf(text, sizeof(text), "K %.3f", double(c.k));
        draw_text(Rect16(r.Left() + 8 + i * cell, r.Top() + 6, cell, height(Font::small)), string_view_utf8::MakeRAM(text), Font::small, c.wave->step_s > 0 ? c.color : style::text_faint, style::surface, Align_t::Center());
    }

    // Zone de tracé ; échelle commune : 0 (palier lent) et 64 (palier rapide) toujours visibles
    const int16_t left = r.Left() + 8, top = r.Top() + 28;
    const int16_t w = r.Width() - 16, h = r.Height() - 36;
    int lo = 0, hi = 64;
    for (const Curve &c : curves) {
        if (c.wave->step_s > 0) {
            for (const int8_t v : c.wave->force) {
                lo = std::min<int>(lo, v);
                hi = std::max<int>(hi, v);
            }
        }
    }
    const auto y_of = [&](int v) { return uint16_t(top + h - 1 - (v - lo) * (h - 1) / std::max(1, hi - lo)); };
    const auto x_of = [&](size_t j) { return uint16_t(left + int(j) * (w - 1) / int(pa::Waveform::points - 1)); };
    display::fill_rect(Rect16(left, y_of(0), w, 1), style::line);
    display::fill_rect(Rect16(left, y_of(64), w, 1), style::line);

    const auto polyline = [&](const std::array<int8_t, pa::Waveform::points> &v, Color color, bool bold) {
        for (size_t j = 1; j < v.size(); ++j) {
            const point_ui16_t a { x_of(j - 1), y_of(v[j - 1]) }, b { x_of(j), y_of(v[j]) };
            display::draw_line(a, b, color);
            if (bold) {
                display::draw_line({ a.x, uint16_t(a.y + 1) }, { b.x, uint16_t(b.y + 1) }, color);
            }
        }
    };
    polyline(run.best.command, color_command, false);
    for (const Curve &c : curves) {
        if (c.wave->step_s > 0) {
            polyline(c.wave->force, c.color, c.wave == &run.best);
        }
    }
}

// ---- Écran ----

ScreenPa::ScreenPa(uint8_t slot)
    : Screen(true)
    , slot(slot)
    , header(this, _("Slot %d · PA").formatted(title_params, slot + 1), true)
    , card(this, card_rect, slot)
    , graph(this, graph_rect, slot)
    , note(this, note_rect, is_multiline::no)
    , wins(this, wins_body, tag_wins, Button::Look::secondary, {}, nullptr, style::screen)
    , csv(this, csv_body, tag_csv, Button::Look::secondary, {}, nullptr, style::screen)
    , measure_button(this, measure_body, tag_measure, Button::Look::primary, _("Measure"), &img::nozzle_16x16, style::screen)
    , second(this, second_body, tag_second, Button::Look::outline, _("Clear"), nullptr, style::screen)
    , tabs(this, TabBar::Tab::workshop) {
    ClrMenuTimeoutClose(); // l'écran reste ouvert pendant la mesure
    note.set_font(Font::small);
    note.SetTextColor(style::text_soft);
    note.SetBackColor(style::screen);
    note.SetPadding({ 0, 0, 0, 0 });
    shown = pa::status();
    shown_running = pa::is_running();
    shown_runs = pa::run_count();
    show_toggles();
    show_note();
    update();
    if (measure_button.IsEnabled()) {
        measure_button.SetFocus();
    } else if (window_t *first = GetFirstEnabledSubWin()) {
        first->SetFocus();
    }
}

void ScreenPa::show_toggles() {
    const bool on = pa::printer_wins();
    wins.set(on ? Button::Look::selected : Button::Look::secondary, on ? _("Printer PA: on") : _("Printer PA: off"));
    csv.set(csv_on ? Button::Look::selected : Button::Look::secondary, csv_on ? _("USB curves: on") : _("USB curves: off"));
}

void ScreenPa::show_note() {
    StringViewUtf8Parameters<24> params;
    char k[12];
    string_view_utf8 text;
    const bool mine = shown.slot == slot;
    if (shown_pending) {
        text = _("Starting...");
    } else if (shown_running && mine) {
        switch (shown.phase) {
        case pa::Phase::heating:
            text = _("Heating the nozzle to %d°").formatted(params, int(shown.temperature));
            break;
        case pa::Phase::priming:
            text = _("Purging...");
            break;
        case pa::Phase::measuring:
            format_k(k, sizeof(k), shown.k_e4);
            text = _("Measuring K = %s (%d/%d)").formatted(params, k, int(shown.step), int(shown.steps));
            break;
        default:
            text = _("Homing and moving...");
            break;
        }
    } else if (marlin_client::is_printing()) {
        text = _("Not while printing.");
    } else if (mine && shown.phase == pa::Phase::done) {
        format_k(k, sizeof(k), shown.k_e4);
        text = shown.csv ? _("Measured: K = %s. Curves on the USB drive.").formatted(params, k) : _("Measured: K = %s").formatted(params, k);
    } else if (mine && shown.phase == pa::Phase::failed && shown.error != pa::Error::not_loaded) {
        format_k(k, sizeof(k), shown.k_e4);
        switch (shown.error) {
        case pa::Error::printing:
            text = _("Not while printing.");
            break;
        case pa::Error::homing:
            text = _("Homing failed.");
            break;
        case pa::Error::no_room:
            text = _("No room for the test.");
            break;
        case pa::Error::heating:
            text = _("The nozzle did not heat up.");
            break;
        case pa::Error::stopped:
            text = _("Measurement cancelled.");
            break;
        case pa::Error::weak_signal:
            text = _("Force signal too weak: nothing saved.");
            break;
        case pa::Error::no_crossing:
        case pa::Error::too_few_points:
            text = _("Force does not follow K: nothing saved.");
            break;
        case pa::Error::out_of_range:
            text = _("K out of range (%s): nothing saved.").formatted(params, k);
            break;
        case pa::Error::low_confidence:
            text = _("Unsure result (K = %s): nothing saved.").formatted(params, k);
            break;
        default:
            text = _("Invalid request.");
            break;
        }
    } else if (!loaded(slot)) {
        text = _("Load this spool into the nozzle to measure it.");
    } else {
        text = _("Heats, then extrudes ~1 g at the bed front.");
    }
    text.copyToRAM(note_text.data(), note_text.size());
    note.SetText(string_view_utf8::MakeRAM(note_text.data()));
    note.Invalidate();
}

void ScreenPa::update() {
    const bool running = pa::is_running();
    const uint16_t runs = pa::run_count();
    if (pending && (running || runs != runs_at_request)) {
        pending = false;
    }
    const pa::Status s = pa::status();
    const bool finished = runs != shown_runs || (shown_running && !running);
    if (running != shown_running || pending != shown_pending || !(s == shown) || finished) {
        shown_running = running;
        shown_pending = pending;
        shown = s;
        shown_runs = runs;
        show_note();
        if (finished) {
            card.Invalidate();
            graph.Invalidate();
        }
    }

    const bool busy = running || pending;
    const bool printing = marlin_client::is_printing();
    const bool can_measure = !busy && !printing && loaded(slot) && FilamentType::for_tool(VirtualToolIndex::from_raw(slot));
    measure_button.enable(can_measure);
    if (busy != second_cancels) {
        second_cancels = busy; // libellé changé seulement quand il le faut : pas de redessin à chaque passage
        second.set(Button::Look::outline, busy ? _("Cancel") : _("Clear"));
    }
    second.enable(busy || pa::record(slot).has_k());
    wins.enable(true);
    csv.enable(!busy);

    // Un élément désactivé ne garde pas le focus
    if (window_t *focused = get_focused_child_window(); !focused || !focused->IsEnabled()) {
        if (window_t *first = GetFirstEnabledSubWin()) {
            first->SetFocus();
        }
    }
}

void ScreenPa::measure() {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    if (!loaded(slot)) {
        MsgBoxWarning(_("Load this spool into the nozzle to measure it."), Responses_Ok);
        return;
    }
    if (MsgBoxQuestion(_("The nozzle heats up, then extrudes about 1 g of filament in the air above the front of the bed (2 to 3 min). Clear the bed. Start?"), Responses_YesNo) != Response::Yes) {
        return;
    }
    char gcode[24];
    snprintf(gcode, sizeof(gcode), "M1992 P%u O%u", unsigned(slot), csv_on ? 1u : 0u);
    const uint16_t before = pa::run_count();
    if (marlin_client::gcode_try(gcode) == marlin_client::GcodeTryResult::Submitted) {
        runs_at_request = before;
        pending = true;
    }
}

void ScreenPa::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        switch (static_cast<Tag>(reinterpret_cast<uintptr_t>(param))) {
        case tag_wins:
            pa::set_printer_wins(!pa::printer_wins());
            show_toggles();
            card.Invalidate(); // « PA à mesurer » suit le réglage
            break;
        case tag_csv:
            csv_on = !csv_on;
            show_toggles();
            break;
        case tag_measure:
            measure();
            break;
        case tag_second:
            if (pa::is_running() || pending) {
                pa::request_stop();
            } else {
                pa::clear(slot);
                card.Invalidate();
            }
            break;
        }
        update();
        return;
    }

    default:
        break;
    }
    Screen::windowEvent(sender, event, param); // BUG-2 : la base reçoit tous les autres événements
}

} // namespace od3d
