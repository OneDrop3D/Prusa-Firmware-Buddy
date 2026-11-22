/// @file
#include "screen_e_jump_od3d.hpp"
#include "screen_calib_assistant_od3d.hpp" // confirm_stop_measure
#include "menu_items_od3d.hpp"
#include "menu_od3d.hpp" // section_tab

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <filament.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <tool_index.hpp>
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    // Carte et barres côte à côte, une note sur trois lignes, deux boutons (contenu : y 46 à 274)
    constexpr Rect16 card_rect = { style::margin, style::top, 228, 128 };
    constexpr Rect16 steps_rect = { 244, style::top, 228, 128 };
    constexpr Rect16 note_rect = { style::margin, 180, 464, 50 };
    constexpr Rect16 action_body = { style::margin, 238, 228, 34 };
    constexpr Rect16 clear_body = { 244, 238, 228, 34 };
    static_assert(card_rect.Bottom() < note_rect.Top() && note_rect.Bottom() < style::with_ring(action_body).Top());
    static_assert(style::inside(style::content, steps_rect) && style::inside(style::content, style::with_ring(clear_body)));
    static_assert(style::with_ring(action_body).Right() < style::with_ring(clear_body).Left());

    enum Tag : uint8_t {
        tag_action,
        tag_clear,
    };

    constexpr Color ok_green = Color::from_raw(0x5FBF6A);

    bool loaded(uint8_t slot) {
        return Slot::read(slot).loaded;
    }

    Color step_color(const e_jump::Step &s) {
        switch (e_jump::state(s)) {
        case e_jump::StepState::clean:
            return ok_green;
        case e_jump::StepState::doubtful:
            return style::warning;
        case e_jump::StepState::stalled:
            return style::danger;
        }
        return style::secondary;
    }

} // namespace

// ---- Carte ----

EJumpCard::EJumpCard(window_t *parent, Rect16 rect, uint8_t slot)
    : window_t(parent, rect)
    , slot(slot) {}

void EJumpCard::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    const int16_t x = r.Left() + 12;
    const uint16_t w = r.Width() - 24;
    const uint16_t hs = height(Font::small), hb = height(Font::big);
    const e_jump::Record rec = e_jump::record(slot);
    const bool has = rec.has_jump();
    char value[20];

    draw_text(Rect16(x, r.Top() + 4, w, hs), _("Safe jump"), Font::small, style::text_muted, style::surface);
    if (has) {
        snprintf(value, sizeof(value), "%s%.1f mm/s", (rec.flags & e_jump::Record::lower_bound) ? ">=" : "", double(rec.jump()));
    } else {
        strlcpy(value, "-", sizeof(value));
    }
    draw_text(Rect16(x, r.Top() + 20, w, hb), string_view_utf8::MakeRAM(value), Font::big, has ? style::text_strong : style::text_faint, style::surface);

    // F avec le K de l'emplacement ; sans mesure, celle de Prusa (grisée)
    const float f = e_jump::filament_accel_for(slot);
    draw_text(Rect16(x, r.Top() + 44, w, hs), _("Max filament accel."), Font::small, style::text_muted, style::surface);
    snprintf(value, sizeof(value), "%ld mm/s2", std::lround(f));
    draw_text(Rect16(x, r.Top() + 60, w, hb), string_view_utf8::MakeRAM(value), Font::big, has ? style::text_strong : style::text_muted, style::surface);

    draw_text(Rect16(x, r.Top() + 84, w, hs), _("Line accel. (estimated)"), Font::small, style::text_muted, style::surface);
    snprintf(value, sizeof(value), "%ld mm/s2", std::lround(e_jump::line_accel(f)));
    draw_text(Rect16(x, r.Top() + 100, w, hb), string_view_utf8::MakeRAM(value), Font::big, has ? style::accent : style::text_muted, style::surface);
}

// ---- Paliers ----

EJumpSteps::EJumpSteps(window_t *parent, Rect16 rect, uint8_t slot)
    : window_t(parent, rect)
    , slot(slot) {}

void EJumpSteps::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::draw_rounded_rect(r, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
    const int16_t left = r.Left() + 12;
    const uint16_t inner_w = r.Width() - 24;
    const uint16_t hs = height(Font::small);
    constexpr uint16_t range_w = 90;
    draw_text(Rect16(left, r.Top() + 6, inner_w - range_w - 4, hs), _("Jump steps"), Font::small, style::text_muted, style::surface);
    std::array<float, e_jump::max_steps> jumps;
    const size_t n = e_jump::jump_steps(jumps);
    char range[16];
    snprintf(range, sizeof(range), "%.0f-%.0f mm/s", double(jumps[0]), double(jumps[n - 1]));
    draw_text(Rect16(left + inner_w - range_w, r.Top() + 6, range_w, hs), string_view_utf8::MakeRAM(range), Font::small, style::text_faint, style::surface, Align_t::RightCenter());

    // Une barre par palier, hauteur selon le saut ; gris : pas encore fait (ou pas de mesure de cet emplacement)
    const e_jump::LastRun &run = e_jump::last_run();
    const size_t done = run.slot == slot ? std::min<size_t>(run.points, n) : 0;
    const int16_t top = r.Top() + 28;
    const int16_t h = r.Height() - 36;
    constexpr int16_t gap = 4;
    const int16_t bar_w = int16_t((inner_w - gap * (n - 1)) / n);
    const auto y_of = [&](float jump) {
        return int16_t(top + h - std::lround(std::clamp(jump / jumps[n - 1], 0.f, 1.f) * float(h)));
    };
    display::fill_rect(Rect16(left, top + h, inner_w, 1), style::line);
    for (size_t i = 0; i < n; ++i) {
        const int16_t y = y_of(jumps[i]);
        const Color c = i < done ? step_color(run.step[i]) : style::secondary;
        display::fill_rect(Rect16(int16_t(left + i * (bar_w + gap)), y, bar_w, uint16_t(top + h - y)), c);
    }
    // Saut gardé (marge comprise) : trait orange
    if (const e_jump::Record rec = e_jump::record(slot); rec.has_jump()) {
        display::fill_rect(Rect16(left, y_of(rec.jump()), inner_w, 2), style::accent);
    }
}

// ---- Écran ----

ScreenEJump::ScreenEJump(uint8_t slot)
    : Screen(true)
    , slot(slot)
    , header(this, _("Slot %d · Fil. accel.").formatted(title_params, slot + 1), true)
    , card(this, card_rect, slot)
    , steps(this, steps_rect, slot)
    , note(this, note_rect, is_multiline::yes)
    , action(this, action_body, tag_action, Button::Look::primary, _("Start"), &img::print_16x16, style::screen)
    , clear(this, clear_body, tag_clear, Button::Look::outline, _("Clear"), nullptr, style::screen)
    , tabs(this, section_tab()) {
    ClrMenuTimeoutClose(); // l'écran reste ouvert pendant la mesure
    note.set_font(Font::small);
    note.SetAlignment(Align_t::LeftTop());
    note.SetTextColor(style::text_soft);
    note.SetBackColor(style::screen);
    note.SetPadding({ 0, 0, 0, 0 });
    shown = e_jump::status();
    shown_running = e_jump::is_running();
    shown_runs = e_jump::run_count();
    show_note();
    update();
    if (action.IsEnabled()) {
        action.SetFocus();
    } else if (window_t *first = GetFirstEnabledSubWin()) {
        first->SetFocus();
    }
}

void ScreenEJump::show_note() {
    StringViewUtf8Parameters<24> params;
    char value[12];
    string_view_utf8 text;
    const bool mine = shown.slot == slot;
    if (shown_pending) {
        text = _("Starting...");
    } else if (shown_running && mine) {
        switch (shown.phase) {
        case e_jump::Phase::heating:
            text = _("Heating the nozzle to %d°").formatted(params, int(shown.temperature));
            break;
        case e_jump::Phase::measuring:
            snprintf(value, sizeof(value), "%.1f", double(shown.jump_e1) / 10.0);
            text = _("Extruder jump %s mm/s (%d/%d)").formatted(params, value, int(shown.step), int(shown.steps));
            break;
        default:
            text = _("Homing and moving...");
            break;
        }
    } else if (shown_running) {
        text = _("Another measurement is running.");
    } else if (marlin_client::is_printing()) {
        text = _("Not while printing.");
    } else if (mine && shown.phase == e_jump::Phase::done) {
        snprintf(value, sizeof(value), "%.1f", double(shown.jump_e1) / 10.0);
        text = _("Safe jump saved: %s mm/s. Remove the purged filament.").formatted(params, value);
    } else if (mine && shown.phase == e_jump::Phase::failed && shown.error != e_jump::Error::not_loaded) {
        switch (shown.error) {
        case e_jump::Error::printing:
            text = _("Not while printing.");
            break;
        case e_jump::Error::homing:
            text = _("Homing failed.");
            break;
        case e_jump::Error::heating:
            text = _("The nozzle or the bed did not heat up.");
            break;
        case e_jump::Error::stopped:
            text = _("Measurement cancelled.");
            break;
        case e_jump::Error::too_low:
            text = _("Stalled from the first step: nothing saved. Check the extruder.");
            break;
        default:
            text = _("Invalid request.");
            break;
        }
    } else if (!loaded(slot)) {
        text = _("Load this spool into the nozzle to measure it.");
    } else {
        text = _("Extruder jumps at the purge spot (~60 mm of filament, 1 min). Then filament accel. = jump / K. Line 0.45 x 0.2 mm.");
    }
    text.copyToRAM(note_text.data(), note_text.size());
    note.SetText(string_view_utf8::MakeRAM(note_text.data()));
    note.Invalidate();
}

void ScreenEJump::update() {
    const bool running = e_jump::is_running();
    const uint16_t runs = e_jump::run_count();
    if (pending && (running || runs != runs_at_request)) {
        pending = false;
    }
    const e_jump::Status s = e_jump::status();
    const bool finished = runs != shown_runs || (shown_running && !running);
    if (running != shown_running || pending != shown_pending || !(s == shown) || finished) {
        shown_running = running;
        shown_pending = pending;
        shown = s;
        shown_runs = runs;
        show_note();
        if (finished) {
            card.Invalidate();
        }
    }
    if (const uint16_t rev = e_jump::revision(); rev != shown_revision) {
        shown_revision = rev;
        steps.Invalidate();
        card.Invalidate();
    }

    const bool ours = (running && s.slot == slot) || pending;
    if (ours != action_cancels) {
        action_cancels = ours; // libellé changé seulement quand il le faut : pas de redessin à chaque passage
        action.set(ours ? Button::Look::outline : Button::Look::primary, ours ? _("Cancel") : _("Start"), ours ? &img::cancel_16x16 : &img::print_16x16);
    }
    const bool printing = marlin_client::is_printing();
    action.enable(ours || (!running && !printing && loaded(slot) && FilamentType::for_tool(VirtualToolIndex::from_raw(slot))));
    clear.enable(!running && !pending && e_jump::record(slot).has_jump());

    // Un élément désactivé ne garde pas le focus (le retour de l'en-tête, premier élément, ferait quitter l'écran)
    if (window_t *focused = get_focused_child_window(); !focused || !focused->IsEnabled()) {
        if (window_t *next = action.IsEnabled() ? &action : GetFirstEnabledSubWin()) {
            next->SetFocus();
        }
    }
}

void ScreenEJump::start() {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    if (!loaded(slot)) {
        MsgBoxWarning(_("Load this spool into the nozzle to measure it."), Responses_Ok);
        return;
    }
    char gcode[16];
    snprintf(gcode, sizeof(gcode), "M2000 S%u", unsigned(slot));
    const uint16_t before = e_jump::run_count();
    if (marlin_client::gcode_try(gcode) == marlin_client::GcodeTryResult::Submitted) {
        runs_at_request = before;
        pending = true;
    }
}

void ScreenEJump::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        switch (static_cast<Tag>(reinterpret_cast<uintptr_t>(param))) {
        case tag_action:
            if (e_jump::is_running() || pending) {
                if (confirm_stop_measure()) { // pendant une série de l'assistant : il passe à la suivante
                    e_jump::request_stop();
                }
            } else {
                start();
            }
            break;
        case tag_clear:
            e_jump::clear(slot); // F de Prusa au prochain changement d'outil ou de K
            card.Invalidate();
            steps.Invalidate();
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

// ---- Réglages › Fonctions OneDrop3D ----

MI_E_JUMP::MI_E_JUMP()
    : IWindowMenuItem(_("Filament accel.\nExtruder jump, loaded spool"), &img::wizard_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_E_JUMP::click(IWindowMenu &) {
    uint8_t slot = 0;
    for (uint8_t i = 0; i < VirtualToolIndex::count; ++i) {
        if (loaded(i)) {
            slot = i;
            break;
        }
    }
    Screens::Access()->Open(ScreenFactory::ScreenWithArg<ScreenEJump>(slot));
}

} // namespace od3d
