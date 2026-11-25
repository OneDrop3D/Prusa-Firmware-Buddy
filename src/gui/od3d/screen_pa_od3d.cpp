/// @file
#include "screen_pa_od3d.hpp"
#include "screen_calib_assistant_od3d.hpp" // confirm_stop_measure
#include "screen_spool_calib_od3d.hpp"

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
    // Rangée du milieu, en petite police : « PA imprimante : oui » (19 caractères de 9 px), motif, « Meilleure ligne »
    constexpr Rect16 wins_body = { style::margin, 202, 184, 30 };
    constexpr Rect16 pattern_body = { 200, 202, 96, 30 };
    constexpr Rect16 lines_body = { 304, 202, 168, 30 };
    // Rangée du bas : trois boutons de 148 px
    constexpr Rect16 measure_body = { style::margin, 238, 148, 34 };
    constexpr Rect16 second_body = { 166, 238, 148, 34 };
    constexpr Rect16 calib_body = { 324, 238, 148, 34 };
    static_assert(card_rect.Bottom() < note_rect.Top() && note_rect.Bottom() < style::with_ring(wins_body).Top());
    static_assert(style::with_ring(wins_body).Bottom() < style::with_ring(measure_body).Top());
    static_assert(style::inside(style::content, style::with_ring(calib_body)) && style::inside(style::content, graph_rect));
    static_assert(style::with_ring(measure_body).Right() < style::with_ring(second_body).Left() && style::with_ring(second_body).Right() < style::with_ring(calib_body).Left());
    static_assert(style::with_ring(wins_body).Right() < style::with_ring(pattern_body).Left() && style::with_ring(pattern_body).Right() < style::with_ring(lines_body).Left());
    static_assert(style::inside(style::content, style::with_ring(lines_body)));

    enum Tag : uint8_t {
        tag_wins,
        tag_pattern,
        tag_lines,
        tag_measure,
        tag_second,
        tag_calib,
    };

    constexpr Color color_low = style::curve_blue; ///< K le plus bas : le bleu des autres courbes (amortissement, courroies, dilatation)
    constexpr Color color_best = style::accent;
    constexpr Color color_high = style::danger;
    constexpr Color color_command = style::curve_grey;

    /// Courbes exportées sur la clé : choix gardé jusqu'au redémarrage (réglé sur l'écran Débit/T°)
    bool csv_on = true;

    /// La dernière mesure de l'emplacement a imprimé au moins une ligne : le choix à l'œil est possible
    bool has_lines(uint8_t slot) {
        const pa::LastRun &run = pa::last_run();
        return !pa::is_running() && run.slot == slot && run.points > 0;
    }

    // Écran « Meilleure ligne » : aide sur 3 lignes, puis les lignes en colonnes de 5
    constexpr Rect16 lines_help_rect = { style::margin, style::top, 464, 48 };
    constexpr int16_t line_top = 100;
    constexpr uint16_t line_w = 148, line_h = 28, line_gap_y = 6;
    constexpr uint8_t lines_per_column = 5;
    constexpr Rect16 line_body(uint8_t index) {
        const uint8_t column = index / lines_per_column, row = index % lines_per_column;
        return { int16_t(style::margin + column * 158), int16_t(line_top + row * (line_h + line_gap_y)), line_w, line_h };
    }
    static_assert(pa::max_lines <= 3 * lines_per_column);
    static_assert(lines_help_rect.Bottom() < style::with_ring(line_body(0)).Top());
    static_assert(style::inside(style::content, style::with_ring(line_body(pa::max_lines - 1))));
    static_assert(style::with_ring(line_body(0)).Bottom() < style::with_ring(line_body(1)).Top());
    static_assert(style::with_ring(line_body(0)).Right() < style::with_ring(line_body(lines_per_column)).Left());

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

    /// La mesure peut amener la bobine dans la buse (MMU actif), ou elle y est déjà
    bool reachable(uint8_t slot) {
        return loaded(slot) || atelier::mmu_active();
    }

    /// Emplacement dont l'écran doit proposer la mesure dès son ouverture (0xff : aucun)
    uint8_t autostart_slot = 0xff;

} // namespace

void open_pa(uint8_t slot, bool start) {
    autostart_slot = start ? slot : 0xff;
    Screens::Access()->Open(ScreenFactory::ScreenWithArg<ScreenPa>(slot));
}

void pa_progress(StringBuilder &sb, const pa::Status &s) {
    StringViewUtf8Parameters<24> params;
    char k[12];
    switch (s.phase) {
    case pa::Phase::bed:
        sb.append_string_view(_("Heating the bed..."));
        break;
    case pa::Phase::leveling:
        sb.append_string_view(_("Probing bed"));
        break;
    case pa::Phase::heating:
        sb.append_string_view(_("Heating the nozzle to %d°").formatted(params, int(s.temperature)));
        break;
    case pa::Phase::loading:
        sb.append_string_view(_("Loading filament"));
        break;
    case pa::Phase::priming:
        sb.append_string_view(_("Purging..."));
        break;
    case pa::Phase::measuring:
        format_k(k, sizeof(k), s.k_e4);
        sb.append_string_view(_("Line %d/%d: K = %s").formatted(params, int(s.step), int(s.steps), k));
        break;
    default:
        sb.append_string_view(_("Homing and moving..."));
        break;
    }
}

bool usb_curves() {
    return csv_on;
}

void set_usb_curves(bool set) {
    csv_on = set;
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
    if (rec.has_k() && rec.source == pa::Record::chosen) {
        draw_text(Rect16(x, r.Top() + 60, w, height(Font::small)), _("Chosen by eye"), Font::small, style::text_soft, style::surface);
    } else if (rec.has_k()) {
        draw_text(Rect16(x, r.Top() + 60, w, height(Font::small)), _("Confidence %d/100").formatted(params, int(rec.confidence)), Font::small, style::text_soft, style::surface);
    } else {
        draw_text(Rect16(x, r.Top() + 60, w, height(Font::small)), _("Not measured"), Font::small, style::text_soft, style::surface);
    }

    char name[24];
    atelier::spool_name(name, FilamentType::for_tool(VirtualToolIndex::from_raw(slot)));
    draw_text(Rect16(x, r.Top() + 82, w, height(Font::small)), string_view_utf8::MakeRAM(name), Font::small, style::text_muted, style::surface);

    if (rec.has_k() && !rec.applies()) {
        // M-10 : un K trop peu sûr compte comme mesuré, mais celui du trancheur sert
        draw_text(Rect16(x, r.Top() + 104, w, height(Font::small)), _("Unsure: slicer PA used"), Font::small, style::warning, style::surface);
    } else if (pa::needs_measure(slot)) {
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
    , pattern(this, pattern_body, tag_pattern, Button::Look::secondary, {}, nullptr, style::screen)
    , lines(this, lines_body, tag_lines, Button::Look::secondary, _("Best line"), nullptr, style::screen)
    , measure_button(this, measure_body, tag_measure, Button::Look::primary, _("Measure"), &img::nozzle_16x16, style::screen)
    , second(this, second_body, tag_second, Button::Look::outline, _("Clear"), nullptr, style::screen)
    , calib(this, calib_body, tag_calib, Button::Look::secondary, _("Flow, temp."), nullptr, style::screen)
    , tabs(this, TabBar::Tab::workshop) {
    ClrMenuTimeoutClose(); // l'écran reste ouvert pendant la mesure
    note.set_font(Font::small);
    note.SetTextColor(style::text_soft);
    note.SetBackColor(style::screen);
    note.SetPadding({ 0, 0, 0, 0 });
    for (Button *b : { &wins, &pattern, &lines }) {
        b->set_font(Font::small);
    }
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
    pattern.set(Button::Look::secondary, pa::large_pattern() ? _("Large") : _("Compact"));
}

void ScreenPa::show_note() {
    StringViewUtf8Parameters<24> params;
    char k[12];
    string_view_utf8 text;
    const bool mine = shown.slot == slot;
    if (shown_pending) {
        text = _("Starting...");
    } else if (shown_running && mine) {
        StringBuilder sb(note_text);
        pa_progress(sb, shown);
        note.SetText(string_view_utf8::MakeRAM(note_text.data()));
        note.Invalidate();
        return;
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
            text = _("The nozzle or the bed did not heat up.");
            break;
        case pa::Error::leveling:
            text = _("Bed probing failed.");
            break;
        case pa::Error::stopped:
            text = _("Measurement cancelled.");
            break;
        case pa::Error::weak_signal:
            text = _("Weak force: pick the best line by eye.");
            break;
        case pa::Error::no_crossing:
        case pa::Error::too_few_points:
            text = _("Force does not follow K: pick the best line.");
            break;
        case pa::Error::out_of_range:
            text = _("K out of range (%s): pick the best line.").formatted(params, k);
            break;
        case pa::Error::low_confidence:
            text = _("Unsure (K = %s): pick the best line.").formatted(params, k);
            break;
        case pa::Error::mmu:
            text = _("MMU not ready.");
            break;
        case pa::Error::load_failed:
            text = _("Spool not loaded: nothing measured.");
            break;
        case pa::Error::no_force:
            text = _("No force: the nozzle did not touch the bed.");
            break;
        case pa::Error::flexible:
            text = _("Flexible filament: not measured.");
            break;
        case pa::Error::no_memory:
            text = _("Not enough memory, try again later.");
            break;
        default:
            text = _("Invalid request.");
            break;
        }
    } else if (!reachable(slot)) {
        text = _("Load this spool into the nozzle to measure it.");
    } else if (!loaded(slot)) {
        text = _("Loads the spool, then prints test lines in front.");
    } else {
        text = _("Prints one test line per K at the bed front.");
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
    // Pas pendant une mesure du débit ou de la température (F5, F6) : elle passerait après
    const bool can_measure = !busy && !calib::is_running() && !printing && reachable(slot) && FilamentType::for_tool(VirtualToolIndex::from_raw(slot));
    measure_button.enable(can_measure);
    if (busy != second_cancels) {
        second_cancels = busy; // libellé changé seulement quand il le faut : pas de redessin à chaque passage
        second.set(Button::Look::outline, busy ? _("Cancel") : _("Clear"));
    }
    second.enable(busy || pa::record(slot).has_k());
    wins.enable(true);
    pattern.enable(!busy); // la mesure en cours garde son motif
    lines.enable(has_lines(slot));
    calib.enable(true); // on y regarde les valeurs même pendant une mesure ou une impression

    // Un élément désactivé ne garde pas le focus ; pendant la mesure il va à « Annuler » (le retour de l'en-tête,
    // premier élément, ferait quitter l'écran au prochain clic)
    if (window_t *focused = get_focused_child_window(); !focused || !focused->IsEnabled()) {
        if (window_t *next = busy && second.IsEnabled() ? &second : GetFirstEnabledSubWin()) {
            next->SetFocus();
        }
    }
}

void ScreenPa::measure() {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    if (!reachable(slot)) {
        MsgBoxWarning(_("Load this spool into the nozzle to measure it."), Responses_Ok);
        return;
    }
    const bool large = pa::large_pattern();
    const string_view_utf8 question = loaded(slot)
        ? (large ? _("Clean sheet in place, front of the bed free? The bed heats up, its front is probed, then one test line per K is printed there (6 to 10 min). Start?")
                 : _("Clean sheet in place, front of the bed free? The bed heats up while homing, its front is probed, then 4 to 8 short test lines are printed there (4 to 6 min). Start?"))
        : (large ? _("Clean sheet in place, front of the bed free? This spool is loaded into the nozzle, the bed heats up, its front is probed, then one test line per K is printed there (8 to 12 min). Start?")
                 : _("Clean sheet in place, front of the bed free? This spool is loaded into the nozzle while the bed heats up, its front is probed, then 4 to 8 short test lines are printed there (6 to 8 min). Start?"));
    if (MsgBoxQuestion(question, Responses_YesNo) != Response::Yes) {
        return;
    }
    char gcode[24];
    snprintf(gcode, sizeof(gcode), "M1994 P%u V%u O%u", unsigned(slot), large ? 0u : 1u, csv_on ? 1u : 0u);
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
        // « Calibrer le PA » de la fiche de la bobine : la mesure est proposée une fois, l'écran déjà affiché
        if (autostart_slot == slot) {
            autostart_slot = 0xff;
            if (measure_button.IsEnabled()) {
                measure();
            }
        }
        break;

    case GUI_event_t::CHILD_CLICK: {
        switch (static_cast<Tag>(reinterpret_cast<uintptr_t>(param))) {
        case tag_wins:
            pa::set_printer_wins(!pa::printer_wins());
            show_toggles();
            card.Invalidate(); // « PA à mesurer » suit le réglage
            break;
        case tag_pattern:
            pa::set_large_pattern(!pa::large_pattern());
            show_toggles();
            break;
        case tag_lines:
            if (has_lines(slot)) {
                Screens::Access()->Open(ScreenFactory::ScreenWithArg<ScreenPaLines>(slot));
                return;
            }
            break;
        case tag_measure:
            measure();
            break;
        case tag_second:
            // Le clic fait ce que dit le libellé affiché : « Annuler » devenu « Effacer » sous le doigt (et le focus) à la
            // fin d'une mesure n'efface rien, et l'effacement se confirme (retour du 23/11 : un K mesuré perdu)
            if (second_cancels) {
                if ((pa::is_running() || pending) && confirm_stop_measure()) { // série de l'assistant : il passe à la suivante
                    pa::request_stop();
                }
            } else if (MsgBoxQuestion(_("Forget the measured values of this spool?"), Responses_YesNo, 1) == Response::Yes) {
                pa::clear(slot);
                card.Invalidate();
            }
            break;
        case tag_calib:
            open_spool_calib(slot);
            return;
        }
        update();
        return;
    }

    default:
        break;
    }
    Screen::windowEvent(sender, event, param); // BUG-2 : la base reçoit tous les autres événements
}

// ---- Meilleure ligne ----

ScreenPaLines::LineButton::LineButton(window_t *parent, uint8_t index)
    : Button(parent, line_body(index), index, Look::card, {}, nullptr, style::screen) {}

ScreenPaLines::ScreenPaLines(uint8_t slot)
    : Screen(true)
    , slot(slot)
    , header(this, _("Slot %d · Best line").formatted(title_params, slot + 1), true)
    , help(this, lines_help_rect, is_multiline::yes, is_closed_on_click_t::no,
          pa::last_run().compact ? _("Line 1 is at the front; lines follow the test order, not K (see each button). Pick the line with the most even width.")
                                 : _("Line 1 is at the front; count the ticks right of each line (long tick = 5). Pick the line with the most even width."))
    , buttons { { this, 0 }, { this, 1 }, { this, 2 }, { this, 3 }, { this, 4 }, { this, 5 }, { this, 6 }, { this, 7 }, { this, 8 }, { this, 9 }, { this, 10 }, { this, 11 }, { this, 12 }, { this, 13 }, { this, 14 } }
    , tabs(this, TabBar::Tab::workshop) {
    static_assert(pa::max_lines == 15, "une entrée de buttons par ligne");
    help.set_font(Font::small);
    help.SetTextColor(style::text_soft);
    help.SetBackColor(style::screen);
    help.SetPadding({ 0, 0, 0, 0 });

    const pa::LastRun &run = pa::last_run();
    const uint8_t count = run.slot == slot ? std::min<uint8_t>(run.points, pa::max_lines) : 0;
    const pa::Record rec = pa::record(slot);
    window_t *focus = nullptr;
    for (uint8_t i = 0; i < pa::max_lines; ++i) {
        LineButton &b = buttons[i];
        if (i >= count) {
            b.Hide();
            b.enable(false);
            continue;
        }
        snprintf(b.text.data(), b.text.size(), "%u: K %.3f", unsigned(i + 1), double(run.k[i]));
        const bool saved = rec.has_k() && rec.k_e4 == pa::k_to_e4(run.k[i]);
        b.set(saved ? Button::Look::selected : Button::Look::card, string_view_utf8::MakeRAM(b.text.data()));
        if (!focus || saved) {
            focus = &b;
        }
    }
    if (focus) {
        focus->SetFocus();
    }
}

void ScreenPaLines::choose(uint8_t index) {
    const pa::LastRun &run = pa::last_run();
    if (run.slot != slot || index >= run.points) {
        return;
    }
    char k[12];
    format_k(k, sizeof(k), pa::k_to_e4(run.k[index]));
    StringViewUtf8Parameters<16> params;
    if (MsgBoxQuestion(_("Save K = %s (line %d) for this spool?").formatted(params, k, int(index + 1)), Responses_YesNo) != Response::Yes) {
        return;
    }
    if (pa::choose_line(slot, index)) {
        Screens::Access()->close_screen();
    }
}

void ScreenPaLines::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::CHILD_CLICK) {
        const uintptr_t tag = reinterpret_cast<uintptr_t>(param);
        if (tag < pa::max_lines) {
            choose(uint8_t(tag));
        }
        return;
    }
    Screen::windowEvent(sender, event, param); // BUG-2 : la base reçoit tous les autres événements
}

} // namespace od3d
