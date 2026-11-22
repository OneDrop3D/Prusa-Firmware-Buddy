/// @file
#include "screen_spool_calib_od3d.hpp"
#include "screen_pa_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <feature/od3d/pa_od3d.hpp>
#include <filament.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    // Même géométrie que l'écran PA : carte et courbe côte à côte, une ligne d'état, deux rangées de boutons
    constexpr Rect16 card_rect = { style::margin, style::top, 184, 128 };
    constexpr Rect16 graph_rect = { 200, style::top, 272, 128 };
    constexpr Rect16 note_rect = { style::margin, 180, 464, 16 };
    constexpr Rect16 limit_body = { style::margin, 202, 228, 30 };
    constexpr Rect16 csv_body = { 244, 202, 228, 30 };
    constexpr Rect16 flow_body = { style::margin, 238, 148, 34 };
    constexpr Rect16 temp_body = { 166, 238, 148, 34 };
    constexpr Rect16 second_body = { 324, 238, 148, 34 };
    static_assert(card_rect.Bottom() < note_rect.Top() && note_rect.Bottom() < style::with_ring(limit_body).Top());
    static_assert(style::with_ring(limit_body).Bottom() < style::with_ring(flow_body).Top());
    static_assert(style::inside(style::content, style::with_ring(second_body)) && style::inside(style::content, graph_rect));
    static_assert(style::with_ring(flow_body).Right() < style::with_ring(temp_body).Left() && style::with_ring(temp_body).Right() < style::with_ring(second_body).Left());

    enum Tag : uint8_t {
        tag_limit,
        tag_csv,
        tag_flow,
        tag_temp,
        tag_second,
    };

    constexpr Color color_points = style::text_soft;
    constexpr Color color_reference = Color::from_raw(0x8A8E94);
    constexpr Color color_result = style::accent;
    constexpr Color color_threshold = style::danger;

    /// « 18.5 » ; « >=30.0 » pour une borne basse
    void format_flow(char *buf, size_t size, const calib::Record &r) {
        if (!r.has_flow()) {
            strlcpy(buf, "-", size);
        } else {
            snprintf(buf, size, "%s%.1f mm3/s", (r.flags & calib::Record::flow_lower_bound) ? ">=" : "", double(calib::flow_from_e1(r.max_flow_e1)));
        }
    }

    /// « 195° » ; « <=180° » quand le plancher est atteint (on pourrait descendre plus bas)
    void format_temperature(char *buf, size_t size, const calib::Record &r) {
        if (!r.has_min_temperature()) {
            strlcpy(buf, "-", size);
        } else {
            snprintf(buf, size, "%s%d\xC2\xB0", (r.flags & calib::Record::temperature_floor) ? "<=" : "", int(r.min_temperature));
        }
    }

    bool loaded(uint8_t slot) {
        return Slot::read(slot).loaded;
    }

    /// Une mesure par la cellule de charge occupe l'imprimante (la nôtre ou celle du PA)
    bool any_running() {
        return calib::is_running() || pa::is_running();
    }

} // namespace

void open_spool_calib(uint8_t slot) {
    Screens::Access()->Open(ScreenFactory::ScreenWithArg<ScreenSpoolCalib>(slot));
}

// ---- Carte des valeurs ----

SpoolCalibCard::SpoolCalibCard(window_t *parent, Rect16 rect, uint8_t slot)
    : window_t(parent, rect)
    , slot(slot) {}

void SpoolCalibCard::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    const int16_t x = r.Left() + 12;
    const uint16_t w = r.Width() - 24;
    const calib::Record rec = calib::record(slot);
    char value[20];

    draw_text(Rect16(x, r.Top() + 6, w, height(Font::small)), _("Max flow"), Font::small, style::text_muted, style::surface);
    format_flow(value, sizeof(value), rec);
    draw_text(Rect16(x, r.Top() + 24, w, height(Font::big)), string_view_utf8::MakeRAM(value), Font::big, rec.has_flow() ? style::text_strong : style::text_faint, style::surface);

    draw_text(Rect16(x, r.Top() + 50, w, height(Font::small)), _("Min. temperature"), Font::small, style::text_muted, style::surface);
    format_temperature(value, sizeof(value), rec);
    draw_text(Rect16(x, r.Top() + 68, w, height(Font::big)), string_view_utf8::MakeRAM(value), Font::big, rec.has_min_temperature() ? style::text_strong : style::text_faint, style::surface);

    // Limite en vigueur si le réglage est actif, sinon la matière
    const Rect16 line(x, r.Top() + 100, w, height(Font::small));
    if (calib::flow_limit_enabled() && rec.has_flow()) {
        StringViewUtf8Parameters<12> params;
        snprintf(value, sizeof(value), "%.1f", double(std::clamp(calib::flow_from_e1(rec.max_flow_e1), calib::flow_limit_floor, calib::flow_ceiling)));
        draw_text(line, _("Limit: %s mm3/s").formatted(params, value), Font::small, style::accent, style::surface);
    } else {
        char name[24];
        atelier::spool_name(name, FilamentType::for_tool(VirtualToolIndex::from_raw(slot)));
        draw_text(line, string_view_utf8::MakeRAM(name), Font::small, style::text_muted, style::surface);
    }
}

// ---- Courbe de force ----

SpoolCalibGraph::SpoolCalibGraph(window_t *parent, Rect16 rect, uint8_t slot)
    : window_t(parent, rect)
    , slot(slot) {}

void SpoolCalibGraph::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    const calib::LastRun &run = calib::last_run();
    const bool has = !calib::is_running() && run.slot == slot && run.points >= 2;
    if (!has) {
        draw_text(Rect16(r.Left() + 8, r.Top() + (r.Height() - height(Font::small)) / 2, r.Width() - 16, height(Font::small)), _("No force curve yet"), Font::small, style::text_faint, style::surface, Align_t::Center());
        return;
    }
    const bool flow = run.kind == calib::Kind::flow;
    const float sign = flow ? run.knee.sign : run.min_temp.sign;
    const size_t n = run.points;
    const float x_first = run.point[0].x, x_last = run.point[n - 1].x;

    // Titre à gauche, étendue des paliers à droite
    const int16_t left = r.Left() + 8;
    const uint16_t inner_w = r.Width() - 16;
    // Deux zones distinctes : draw_text repeint tout son rect
    constexpr uint16_t range_w = 100; ///< 11 caractères : « 2-30 mm3/s », « 215-180° »
    draw_text(Rect16(left, r.Top() + 6, inner_w - range_w - 4, height(Font::small)), flow ? _("Force/flow") : _("Force/temp."), Font::small, style::text_muted, style::surface);
    char range[20];
    if (flow) {
        snprintf(range, sizeof(range), "%.0f-%.0f mm3/s", double(x_first), double(x_last));
    } else {
        snprintf(range, sizeof(range), "%.0f-%.0f\xC2\xB0", double(x_first), double(x_last));
    }
    draw_text(Rect16(left + inner_w - range_w, r.Top() + 6, range_w, height(Font::small)), string_view_utf8::MakeRAM(range), Font::small, style::text_faint, style::surface, Align_t::RightCenter());

    // Zone de tracé : X des paliers dans l'ordre de la mesure (débit croissant, température décroissante), force de 0
    // au plus fort (avec le seuil de F6)
    const int16_t top = r.Top() + 28;
    const int16_t w = int16_t(inner_w), h = r.Height() - 36;
    float hi = 1.f;
    for (size_t i = 0; i < n; ++i) {
        hi = std::max(hi, sign * run.point[i].force_g);
    }
    const float threshold = run.max_ratio * run.min_temp.reference_g;
    if (!flow && run.min_temp.reference_g > 0) {
        hi = std::max(hi, threshold);
    }
    hi *= 1.1f;
    const float span = x_last - x_first;
    const auto x_of = [&](float v) {
        const float t = std::fabs(span) > 1e-3f ? (v - x_first) / span : 0.5f;
        return uint16_t(left + std::lround(std::clamp(t, 0.f, 1.f) * float(w - 1)));
    };
    const auto y_of = [&](float f) {
        const float t = std::clamp(f / hi, 0.f, 1.f);
        return uint16_t(top + h - 1 - std::lround(t * float(h - 1)));
    };
    display::fill_rect(Rect16(left, y_of(0.f), w, 1), style::line);

    // Loi de référence de F5 (gris), seuil de F6 (rouge)
    if (flow && run.knee.a > 0) {
        constexpr int samples = 32;
        point_ui16_t prev {};
        for (int j = 0; j <= samples; ++j) {
            const float q = x_first + span * float(j) / float(samples);
            const point_ui16_t p { x_of(q), y_of(run.knee.reference(q)) };
            if (j > 0) {
                display::draw_line(prev, p, color_reference);
            }
            prev = p;
        }
    }
    if (!flow && run.min_temp.reference_g > 0) {
        display::fill_rect(Rect16(left, y_of(threshold), w, 1), color_threshold);
    }

    // Résultat : trait vertical au coude (F5) ou à la température retenue (F6)
    float result = NAN;
    if (flow && (run.knee.fit == calib::KneeFit::found || run.knee.fit == calib::KneeFit::not_reached)) {
        result = run.knee.flow;
    } else if (!flow && (run.min_temp.fit == calib::MinTempFit::found || run.min_temp.fit == calib::MinTempFit::floor_reached)) {
        result = run.min_temp.temperature;
    }
    if (std::isfinite(result)) {
        display::fill_rect(Rect16(x_of(result), top, 2, h), color_result);
    }

    // Paliers mesurés : ligne et carrés
    for (size_t i = 0; i < n; ++i) {
        const point_ui16_t p { x_of(run.point[i].x), y_of(sign * run.point[i].force_g) };
        if (i > 0) {
            const point_ui16_t a { x_of(run.point[i - 1].x), y_of(sign * run.point[i - 1].force_g) };
            display::draw_line(a, p, color_points);
        }
        display::fill_rect(Rect16(int16_t(p.x - 1), int16_t(p.y - 1), 3, 3), color_points);
    }
}

// ---- Écran ----

ScreenSpoolCalib::ScreenSpoolCalib(uint8_t slot)
    : Screen(true)
    , slot(slot)
    , header(this, _("Slot %d · Flow/temp").formatted(title_params, slot + 1), true)
    , card(this, card_rect, slot)
    , graph(this, graph_rect, slot)
    , note(this, note_rect, is_multiline::no)
    , limit(this, limit_body, tag_limit, Button::Look::secondary, {}, nullptr, style::screen)
    , csv(this, csv_body, tag_csv, Button::Look::secondary, {}, nullptr, style::screen)
    , flow_button(this, flow_body, tag_flow, Button::Look::primary, _("Max flow"), nullptr, style::screen)
    , temp_button(this, temp_body, tag_temp, Button::Look::secondary, _("Min. temp."), nullptr, style::screen)
    , second(this, second_body, tag_second, Button::Look::outline, _("Clear"), nullptr, style::screen)
    , tabs(this, TabBar::Tab::workshop) {
    ClrMenuTimeoutClose(); // l'écran reste ouvert pendant la mesure
    note.set_font(Font::small);
    note.SetTextColor(style::text_soft);
    note.SetBackColor(style::screen);
    note.SetPadding({ 0, 0, 0, 0 });
    shown = calib::status();
    shown_running = calib::is_running();
    shown_runs = calib::run_count();
    shown_pa_running = pa::is_running();
    show_toggles();
    show_note();
    update();
    if (flow_button.IsEnabled()) {
        flow_button.SetFocus();
    } else if (window_t *first = GetFirstEnabledSubWin()) {
        first->SetFocus();
    }
}

void ScreenSpoolCalib::show_toggles() {
    const bool on = calib::flow_limit_enabled();
    limit.set(on ? Button::Look::selected : Button::Look::secondary, on ? _("Flow limit: on") : _("Flow limit: off"));
    const bool usb = usb_curves();
    csv.set(usb ? Button::Look::selected : Button::Look::secondary, usb ? _("USB curves: on") : _("USB curves: off"));
}

void ScreenSpoolCalib::show_note() {
    StringViewUtf8Parameters<24> params;
    char value[12];
    string_view_utf8 text;
    const bool mine = shown.slot == slot;
    const bool flow = shown.kind == calib::Kind::flow;
    if (shown_pending) {
        text = _("Starting...");
    } else if (shown_running && mine) {
        switch (shown.phase) {
        case calib::Phase::bed:
            text = _("Heating the bed...");
            break;
        case calib::Phase::leveling:
            text = _("Probing bed");
            break;
        case calib::Phase::heating:
            text = _("Heating the nozzle to %d°").formatted(params, int(shown.temperature));
            break;
        case calib::Phase::priming:
            text = _("Purging...");
            break;
        case calib::Phase::measuring:
            if (flow) {
                snprintf(value, sizeof(value), "%.1f", double(shown.value) / 10.0);
                text = _("Line at %s mm3/s (%d/%d)").formatted(params, value, int(shown.step), int(shown.steps));
            } else {
                text = _("Line at %d° (%d/%d)").formatted(params, int(shown.value), int(shown.step), int(shown.steps));
            }
            break;
        case calib::Phase::settling:
            text = _("Nozzle down to %d° (%d/%d)").formatted(params, int(shown.value), int(shown.step), int(shown.steps));
            break;
        default:
            text = _("Homing and moving...");
            break;
        }
    } else if (shown_pa_running) {
        text = _("Another measurement is running.");
    } else if (marlin_client::is_printing()) {
        text = _("Not while printing.");
    } else if (mine && shown.phase == calib::Phase::done) {
        if (flow) {
            snprintf(value, sizeof(value), "%.1f", double(shown.value) / 10.0);
            text = _("Max flow measured: %s mm3/s").formatted(params, value);
        } else {
            text = _("Min. temperature measured: %d°").formatted(params, int(shown.value));
        }
    } else if (mine && shown.phase == calib::Phase::failed && shown.error != calib::Error::not_loaded) {
        switch (shown.error) {
        case calib::Error::printing:
            text = _("Not while printing.");
            break;
        case calib::Error::homing:
            text = _("Homing failed.");
            break;
        case calib::Error::no_room:
            text = _("No room for the test.");
            break;
        case calib::Error::heating:
            text = _("The nozzle or the bed did not heat up.");
            break;
        case calib::Error::leveling:
            text = _("Bed probing failed.");
            break;
        case calib::Error::stopped:
            text = _("Measurement cancelled.");
            break;
        case calib::Error::weak_signal:
            text = _("Force signal too weak: nothing saved.");
            break;
        case calib::Error::too_few_points:
            text = _("Too few steps: nothing saved.");
            break;
        default:
            text = _("Invalid request.");
            break;
        }
    } else if (!loaded(slot)) {
        text = _("Load this spool into the nozzle to measure it.");
    } else {
        text = _("Prints test lines at the bed front.");
    }
    text.copyToRAM(note_text.data(), note_text.size());
    note.SetText(string_view_utf8::MakeRAM(note_text.data()));
    note.Invalidate();
}

void ScreenSpoolCalib::update() {
    const bool running = calib::is_running();
    const uint16_t runs = calib::run_count();
    if (pending && (running || runs != runs_at_request)) {
        pending = false;
    }
    const calib::Status s = calib::status();
    const bool finished = runs != shown_runs || (shown_running && !running);
    const bool pa_running = pa::is_running();
    if (running != shown_running || pending != shown_pending || !(s == shown) || finished || pa_running != shown_pa_running) {
        shown_pa_running = pa_running;
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

    const bool ours = running || pending;
    const bool busy = ours || pa_running;
    const bool printing = marlin_client::is_printing();
    const bool can_measure = !busy && !printing && loaded(slot) && FilamentType::for_tool(VirtualToolIndex::from_raw(slot));
    flow_button.enable(can_measure);
    temp_button.enable(can_measure);
    if (ours != second_cancels) {
        second_cancels = ours; // libellé changé seulement quand il le faut : pas de redessin à chaque passage
        second.set(Button::Look::outline, ours ? _("Cancel") : _("Clear"));
    }
    const calib::Record rec = calib::record(slot);
    second.enable(ours || (!busy && (rec.has_flow() || rec.has_min_temperature())));
    limit.enable(true);
    csv.enable(!busy);

    // Un élément désactivé ne garde pas le focus ; pendant la mesure il va à « Annuler » (le retour de l'en-tête,
    // premier élément, ferait quitter l'écran au prochain clic)
    if (window_t *focused = get_focused_child_window(); !focused || !focused->IsEnabled()) {
        if (window_t *next = ours && second.IsEnabled() ? &second : GetFirstEnabledSubWin()) {
            next->SetFocus();
        }
    }
}

void ScreenSpoolCalib::measure(calib::Kind kind) {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    if (!loaded(slot)) {
        MsgBoxWarning(_("Load this spool into the nozzle to measure it."), Responses_Ok);
        return;
    }
    const bool flow = kind == calib::Kind::flow;
    const string_view_utf8 question = flow
        ? _("Clean sheet in place, front of the bed free? The bed heats up, its front is probed, then lines are printed there faster and faster (5 to 8 min). Start?")
        : _("Clean sheet in place, front of the bed free? The bed heats up, its front is probed, then lines are printed there while the nozzle cools down step by step (8 to 12 min). Start?");
    if (MsgBoxQuestion(question, Responses_YesNo) != Response::Yes) {
        return;
    }
    if (any_running()) {
        return; // une autre mesure a démarré pendant la question
    }
    char gcode[24];
    snprintf(gcode, sizeof(gcode), "%s P%u O%u", flow ? "M1995" : "M1996", unsigned(slot), usb_curves() ? 1u : 0u);
    const uint16_t before = calib::run_count();
    if (marlin_client::gcode_try(gcode) == marlin_client::GcodeTryResult::Submitted) {
        runs_at_request = before;
        pending = true;
    }
}

void ScreenSpoolCalib::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        switch (static_cast<Tag>(reinterpret_cast<uintptr_t>(param))) {
        case tag_limit:
            calib::set_flow_limit_enabled(!calib::flow_limit_enabled());
            show_toggles();
            card.Invalidate(); // « Limite : … » suit le réglage
            break;
        case tag_csv:
            set_usb_curves(!usb_curves());
            show_toggles();
            break;
        case tag_flow:
            measure(calib::Kind::flow);
            break;
        case tag_temp:
            measure(calib::Kind::temperature);
            break;
        case tag_second:
            if (calib::is_running() || pending) {
                calib::request_stop();
            } else {
                calib::clear(slot);
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
