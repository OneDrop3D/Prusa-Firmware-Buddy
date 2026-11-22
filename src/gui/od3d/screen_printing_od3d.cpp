/// @file
#include "screen_printing_od3d.hpp"

#include "print_info_od3d.hpp"
#include "screen_files_od3d.hpp"
#include "value_editor_od3d.hpp"

#include <algorithm>
#include <common/gcode/gcode_info.hpp>
#include <common/gcode/gcode_info_scan.hpp>
#include <common/marlin_server_shared.h>
#include <common/marlin_vars.hpp>
#include <common/thumbnail_sizes.hpp>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <display.hpp>
#include <display_helper.h>
#include <feature/print_status_message/print_status_message_formatter_buddy.hpp>
#include <feature/print_status_message/print_status_message_mgr.hpp>
#include <gui/event/gui_event.hpp>
#include <gui/event/knob_event.hpp>
#include <i18n.h>
#include <IDialog.hpp>
#include <img_resources.hpp>
#include <liveadjust_z.hpp>
#include <marlin_client.hpp>
#include <marlin_stubs/skippable_gcode.hpp>
#include <odometer.hpp>
#include <option/buddy_enable_connect.h>
#include <option/has_indx.h>
#include <option/has_tool_crash_recovery.h>
#include <option/has_tool_mapping.h>
#include <screen_menu_tune.hpp>
#include <time_tools.hpp>
#include <utils/string_builder.hpp>
#include <window_msgbox.hpp>

#if BUDDY_ENABLE_CONNECT()
    #include <connect/connect.hpp>
    #include <connect/marlin_printer.hpp>
#endif

namespace od3d {

namespace {

    // Géométrie (SPEC_G.md §3.12), sous l'en-tête de 40 px : aperçu 134 px de haut (140 dans la maquette)
    constexpr Rect16 thumb_rect = { style::margin, style::top, 200, 134 };
    constexpr Rect16 info_rect = { 218, 42, 254, 138 }; ///< colonne de droite : progression, ligne, bobine (rien d'actif)
    constexpr Rect16 progress_rect = { info_rect.Left(), info_rect.Top(), info_rect.Width(), 56 };
    constexpr Rect16 line_rect = { info_rect.Left(), progress_rect.Bottom() + 1, info_rect.Width(), 16 };
    constexpr Rect16 spool_rect = Rect16::fromLTRB(info_rect.Left(), line_rect.Bottom() + 1, info_rect.Left() + info_rect.Width(), info_rect.Top() + info_rect.Height());
    static_assert(progress_rect.Bottom() + 1 == line_rect.Top() && line_rect.Bottom() + 1 == spool_rect.Top());
    static_assert(spool_rect.Height() == 66 && info_rect.Top() > style::header_line.Top());
    constexpr Rect16 strip_rect = { style::margin, 184, 464, 20 };
    constexpr Rect16 button_body(int i) {
        return Rect16(style::margin + i * 94, 210, 88, 62);
    }
    constexpr uint8_t thumb_factor = 2;
    constexpr uint16_t img_w = thumbnail_sizes::preview_thumbnail_width / thumb_factor;
    constexpr uint16_t img_h = thumbnail_sizes::preview_thumbnail_height / thumb_factor;
    static_assert(img_w <= thumb_rect.Width() && img_h <= thumb_rect.Height());
    static_assert(style::inside(style::content, style::with_ring(button_body(4))));
    static_assert(thumb_rect.Right() < info_rect.Left() && info_rect.Bottom() < strip_rect.Top());

    constexpr const char *dot = " \xC2\xB7 ";

    bool waiting_for_connect_set_ready() {
#if BUDDY_ENABLE_CONNECT()
        return connect_client::is_connect_registered() && !connect_client::MarlinPrinter::is_printer_ready();
#else
        return false;
#endif
    }

    /// Heure locale de fin, « 21:48 » (ou « 9:48pm » en 12 h) ; false si l'heure ou l'estimation manque
    bool format_end_time(uint32_t time_to_end, std::span<char> out) {
        if (time_to_end == marlin_server::TIME_TO_END_INVALID || time_to_end > 60 * 60 * 24 * 365) {
            return false;
        }
        const time_t now = time_tools::get_local_time();
        if (now == time_tools::invalid_time) {
            return false;
        }
        const time_t end = now + time_to_end;
        struct tm tm;
        localtime_r(&end, &tm);
        if (time_tools::get_time_format() == time_tools::TimeFormat::_24h) {
            snprintf(out.data(), out.size(), "%02d:%02d", tm.tm_hour, tm.tm_min);
        } else {
            snprintf(out.data(), out.size(), "%d:%02d%s", (tm.tm_hour + 11) % 12 + 1, tm.tm_min, tm.tm_hour < 12 ? "am" : "pm");
        }
        return true;
    }

} // namespace

bool print_active() {
    return marlin_vars().peek_fsm_states([](const auto &states) { return states.is_active(ClientFSM::Printing); });
}

void open_print_tab() {
    Screens::Access()->CloseAll(); // avant Open(), qui serait sinon oublié
    if (print_active()) {
        open_screen<ScreenPrinting>();
    } else {
        open_screen<ScreenFiles>();
    }
}

// ---- Boutons ----

bool PrintingButton::Face::same(const Face &o) const {
    return icon == o.icon && glyph == o.glyph && text.is_same_ref(o.text) && value == o.value && active == o.active && danger == o.danger;
}

PrintingButton::PrintingButton(ScreenPrinting &screen, Rect16 body, Handler handler)
    : Card(&screen, style::with_ring(body), nullptr)
    , screen(screen)
    , handler(handler) {}

void PrintingButton::set(const Face &f) {
    if (!face.same(f)) {
        face = f;
        Invalidate();
    }
}

void PrintingButton::show(const img::Resource *icon, const char *glyph, const string_view_utf8 &label, bool active, bool danger) {
    set(Face { .icon = icon, .glyph = glyph, .text = label, .active = active, .danger = danger });
}

void PrintingButton::show_value(const string_view_utf8 &caption, const char *value, bool active) {
    Face f { .text = caption, .active = active };
    strlcpy(f.value.data(), value, f.value.size());
    set(f);
}

void PrintingButton::unconditionalDraw() {
    if (!face.danger) {
        Card::unconditionalDraw();
        return;
    }
    // Bouton danger (§1.5) : contour 2 px #E74626 autour du corps, d'une passe
    draw_frame(GetRect(), back, style::danger, fill(), radius, 2);
    draw_content(body());
}

void PrintingButton::draw_content(Rect16 body) {
    const Rect16 top(body.Left() + 4, body.Top() + 12, body.Width() - 8, 16);
    const Color fg = face.active ? style::text : style::text_faint;
    if (face.value[0]) {
        draw_text(top, face.text, Font::small, face.active ? style::text_muted : style::text_faint, fill(), Align_t::Center());
        draw_text(Rect16(top.Left(), body.Top() + 31, top.Width(), height(Font::normal)), string_view_utf8::MakeRAM(face.value.data()), Font::normal, fg, fill(), Align_t::Center());
        return;
    }
    if (face.icon) {
        draw_icon({ int16_t(body.Left() + (body.Width() - face.icon->w) / 2), int16_t(top.Top()) }, *face.icon, fill(), face.active ? ropfn {} : ropfn { .shadow = is_shadowed::yes });
    } else if (face.glyph) {
        draw_text(top, string_view_utf8::MakeCPUFLASH(face.glyph), Font::small, fg, fill(), Align_t::Center());
    } else if (face.danger) {
        display::draw_rounded_rect(Rect16(body.Left() + (body.Width() - 14) / 2, top.Top() + 1, 14, 14), fill(), style::danger, 3, MIC_ALL_CORNERS);
    }
    const Color label_fg = !face.active ? style::text_faint : (face.danger ? Color::from_raw(0xFF8A73) : style::text);
    draw_text(Rect16(top.Left(), body.Top() + 33, top.Width(), height(Font::small)), face.text, Font::small, label_fg, fill(), Align_t::Center());
}

void PrintingButton::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::CLICK || event == GUI_event_t::TOUCH_CLICK) {
        if (face.active) {
            (screen.*handler)();
        }
        return; // consommé, comme Card
    }
    Card::windowEvent(sender, event, param);
}

// ---- Aperçu, colonne de droite, frise ----

PrintThumb::PrintThumb(ScreenPrinting &screen, Rect16 rect)
    : window_t(&screen, rect)
    , screen(screen)
    , grey_rows(img_h) {}

void PrintThumb::set_progress(uint8_t percent) {
    const uint16_t rows = img_h * (100 - std::min<int>(percent, 100)) / 100;
    if (rows != grey_rows) {
        grey_rows = rows; // la vignette n'est relue sur la clé qu'à chaque ligne gagnée
        // La vignette seule, sans repeindre sa case (elle clignotait à chaque ligne)
        const bool background_valid = HasValidBackground();
        Invalidate();
        if (background_valid) {
            ValidateBackground();
        }
    }
}

void PrintThumb::unconditionalDraw() {
    const Rect16 box = GetRect();
    if (!HasValidBackground()) {
        display::draw_rounded_rect(box, style::screen, style::thumb_back, style::radius_big, MIC_ALL_CORNERS);
    } else if (line_y) {
        display::fill_rect(Rect16(box.Left(), line_y, box.Width(), 1), style::thumb_back); // le trait précédent, hors de l'image
    }
    line_y = 0;
    if (!draw_thumbnail(GCodeInfo::getInstance().GetGcodeFilepath(), box, thumb_factor, screen.scaler, grey_rows)) {
        draw_icon({ int16_t(box.Left() + (box.Width() - 16) / 2), int16_t(box.Top() + (box.Height() - 16) / 2) }, img::print_16x16, style::thumb_back);
        return;
    }
    if (grey_rows > 0) {
        // Trait orange à la hauteur imprimée, sur toute la largeur de la case (§3.12)
        line_y = box.Top() + (box.Height() - img_h) / 2 + grey_rows;
        display::fill_rect(Rect16(box.Left(), line_y, box.Width(), 1), style::accent);
    }
}

PrintInfo::PrintInfo(window_t *parent, Rect16 rect, Part part)
    : window_t(parent, rect)
    , part(part) {}

void PrintInfo::set(const Data &d) {
    const bool changed = part == Part::progress
        ? d.percent != data.percent || d.state != data.state
        : d.slot != data.slot || !(d.look == data.look) || d.material != data.material || d.bottom != data.bottom || d.bottom_strong != data.bottom_strong;
    data = d;
    if (changed) {
        Invalidate();
    }
}

void PrintInfo::unconditionalDraw() {
    // Des cases qui couvrent tout le rect, chacune avec son fond : pas d'effacement préalable, « 47 % » ne clignote plus
    const Rect16 r = GetRect();
    const int16_t x = r.Left();
    const uint16_t w = r.Width();

    if (part == Part::progress) {
        // « 47 % » : chiffres et % de la police large, puis l'état à côté
        char percent[8];
        snprintf(percent, sizeof(percent), "%u %%", data.percent);
        const uint16_t percent_w = strlen(percent) * width(Font::large);
        render_text_align(Rect16(x, r.Top(), percent_w, r.Height()), string_view_utf8::MakeRAM(percent), Font::large, style::screen, style::text_strong, {}, text_flags(Align_t::LeftTop()));
        render_text_align(Rect16(x + percent_w, r.Top(), w - percent_w, r.Height()), string_view_utf8::MakeRAM(data.state.data()), Font::small, style::screen, style::text_muted,
            padding_ui8(8, 29, 0, 0), text_flags(Align_t::LeftTop()));
        return;
    }

    // Haut (34 px) : bobine chargée et ses deux lignes ; bas : message sur deux lignes
    constexpr uint8_t d = 26;
    constexpr int16_t text_dx = d + 8;
    const Rect16 top_rect(x, r.Top(), w, 34);
    if (data.slot != 0xff) {
        const spool::Disc disc(data.look, d);
        draw_pixels(Rect16(x, r.Top(), text_dx, top_rect.Height()), [&](int px, int py) { return disc.pixel(px, py - 2).value_or(style::screen); });
        StringViewUtf8Parameters<4> params;
        draw_text(Rect16(x + text_dx, r.Top(), w - text_dx, height(Font::small)), _("Now: slot %d").formatted(params, data.slot + 1), Font::small, style::text_muted, style::screen);
        render_text_align(Rect16(x + text_dx, r.Top() + 16, w - text_dx, top_rect.Height() - 16), string_view_utf8::MakeRAM(data.material.data()), Font::small, style::screen, style::text_strong, {}, text_flags(Align_t::LeftTop()));
    } else {
        display::fill_rect(top_rect, style::screen);
    }
    render_text_align(Rect16(x, r.Top() + top_rect.Height(), w, r.Height() - top_rect.Height()), string_view_utf8::MakeRAM(data.bottom.data()), Font::small, style::screen, data.bottom_strong ? style::text_soft : style::text_muted, {}, text_flags(Align_t::LeftTop(), is_multiline::yes));
}

PrintStrip::PrintStrip(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void PrintStrip::set_progress(uint8_t p) {
    if (p != percent) {
        percent = p;
        Invalidate();
    }
}

void PrintStrip::unconditionalDraw() {
    // Piste arrondie, partie faite orange, curseur blanc de 3 px : une image d'une passe (la barre clignotait)
    const Rect16 r = GetRect();
    const int w = r.Width(), h = r.Height();
    constexpr int track_top = 4, track_h = 12, track_r = 6;
    const int done = w * std::min<int>(percent, 100) / 100;
    const int cursor = std::clamp(done - 1, 0, w - 3);
    draw_pixels(r, [&](int x, int y) {
        if (x >= cursor && x < cursor + 3) {
            return (x != cursor + 1 && (y == 0 || y == h - 1)) ? style::screen : style::text_strong; // coins arrondis de 1 px
        }
        const int ty = y - track_top;
        const int inset = ty >= 0 && ty < track_h ? print_info::corner_inset(ty, track_h, track_r) : w;
        if (x < inset || x >= w - inset) {
            return style::screen;
        }
        return x < done - inset ? style::accent : style::line;
    });
}

// ---- Écran ----

ScreenPrinting::ScreenPrinting()
    : Screen(true)
    , header(this, _("Printing"), true)
    , thumb(*this, thumb_rect)
    , progress(this, progress_rect, PrintInfo::Part::progress)
    , line(this, line_rect, is_multiline::no)
    , slot_info(this, spool_rect, PrintInfo::Part::spool)
    , strip(this, strip_rect)
    , speed(*this, button_body(0), &ScreenPrinting::on_speed)
    , live_z(*this, button_body(1), &ScreenPrinting::on_live_z)
    , tune(*this, button_body(2), &ScreenPrinting::on_tune)
    , pause(*this, button_body(3), &ScreenPrinting::on_pause)
    , stop(*this, button_body(4), &ScreenPrinting::on_stop)
    , tabs(this, TabBar::Tab::print) {
    ClrMenuTimeoutClose(); // comme screen_printing_data_t : ne se ferme pas tout seul
    line.set_font(Font::small);
    line.SetAlignment(Align_t::LeftCenter());
    line.SetTextColor(style::text_soft);
    line.SetBackColor(style::screen);
    line.SetPadding({ 0, 0, 0, 0 });
    line.SetText(string_view_utf8::MakeRAM(line_text.data()));
    if (!GCodeInfo::getInstance().is_loaded()) {
        gcode_info_scan::start_scan(); // comme PrintProgress::init_gcode_info()
    }
    update();
    speed.SetFocus();
}

void ScreenPrinting::update_state() {
    using marlin_server::State;
    // Reprise de screen_printing_data_t::change_print_state()
    PrintState st = [] {
        switch (marlin_vars().print_state) {
        case State::Idle:
        case State::PrintPreviewInit:
        case State::PrintPreviewImage:
        case State::PrintPreviewConfirmed:
        case State::PrintPreviewQuestions:
#if HAS_TOOL_MAPPING()
        case State::PrintPreviewToolsMapping:
#endif
        case State::PrintInit:
            return PrintState::initial;
        case State::Printing:
            return PrintState::printing;
        case State::PowerPanic_AwaitingResume:
        case State::MediaErrorRecovery_BufferData:
        case State::Paused:
            return PrintState::paused;
        case State::Pausing_Begin:
        case State::Pausing_WaitIdle:
        case State::Pausing_ParkHead:
            return PrintState::pausing;
        case State::Resuming_Reheating:
            return PrintState::reheating;
        case State::Resuming_BufferData:
        case State::Resuming_Begin:
        case State::Resuming_ExecutingGCodeInterrupt:
        case State::Resuming_UnparkHead_XY:
        case State::Resuming_UnparkHead_ZE:
        case State::CrashRecovery_Begin:
        case State::CrashRecovery_Retracting:
        case State::CrashRecovery_Lifting:
        case State::CrashRecovery_ToolchangePowerPanic:
#if HAS_INDX()
        case State::PowerPanic_FinishIndxToolchange:
#endif
        case State::CrashRecovery_XY_Measure:
#if HAS_TOOL_CRASH_RECOVERY()
        case State::CrashRecovery_Tool_Pickup:
#endif
        case State::CrashRecovery_XY_HOME:
        case State::CrashRecovery_HOMEFAIL:
        case State::CrashRecovery_Axis_NOK:
        case State::CrashRecovery_Repeated_Crash:
        case State::PowerPanic_Resume:
            return PrintState::resuming;
        case State::Aborting_Begin:
        case State::Aborting_WaitIdle:
        case State::Aborting_UnloadFilament:
        case State::Aborting_ParkHead:
        case State::Aborting_Preview:
            return PrintState::aborting;
        case State::Finishing_WaitIdle:
        case State::Finishing_UnloadFilament:
        case State::Finishing_ParkHead:
            return PrintState::printing;
        case State::Aborted:
            return PrintState::stopped;
        case State::Finished:
        case State::Exit:
            return PrintState::printed;
        case State::PowerPanic_acFault:
        case State::SerialPrintInit:
            // Comme Prusa : peut arriver pendant une coupure, l'écran est alors éteint
            return PrintState::printing;
        }
        return PrintState::printing;
    }();

    switch (st) {
    case PrintState::reheating:
    case PrintState::resuming:
    case PrintState::aborting:
    case PrintState::stopped:
    case PrintState::printed:
        stop_pressed = false;
        break;
    default:
        break;
    }
    if (stop_pressed) {
        st = PrintState::aborting;
    }
    if (skippable_gcode().is_running()) {
        st = PrintState::skippable;
    }
    state = st;
    if (st == PrintState::printed || st == PrintState::stopped || st == PrintState::paused) {
        Odometer_s::instance().force_to_eeprom();
    }
}

void ScreenPrinting::update_buttons() {
    const bool tunable = state == PrintState::printing || state == PrintState::skippable || state == PrintState::paused;
    auto &vars = marlin_vars();

    char value[12];
    snprintf(value, sizeof(value), "%u %%", unsigned(vars.print_speed.get()));
    speed.show_value(_("Speed"), value, tunable);

    // Live Z : comme l'appui long de screen_printing_data_t, en début d'impression seulement (Z ≤ 1 mm)
    snprintf(value, sizeof(value), "%.3f", double(vars.z_offset.get()));
    live_z.show_value(_("Live Z"), value, state == PrintState::printing && vars.logical_curr_pos[2] <= 1.0f);

    if (state == PrintState::printed && waiting_for_connect_set_ready()) {
        tune.show(&img::set_ready_16x16, nullptr, _("Set Ready"), true);
    } else {
        tune.show(&img::settings_16x16, nullptr, _("Adjust"), tunable);
    }

    static constexpr const char *pause_glyph = "II";
    switch (state) {
    case PrintState::initial:
    case PrintState::printing:
        pause.show(nullptr, pause_glyph, _("Pause"), true);
        break;
    case PrintState::skippable:
        pause.show(&img::print_16x16, nullptr, _("Skip"), true);
        break;
    case PrintState::pausing:
        pause.show(nullptr, pause_glyph, _("Pausing..."), false);
        break;
    case PrintState::paused:
        pause.show(&img::print_16x16, nullptr, _("Resume print"), vars.media_inserted);
        break;
    case PrintState::resuming:
        pause.show(&img::print_16x16, nullptr, _("Resuming..."), false);
        break;
    case PrintState::reheating:
        pause.show(&img::print_16x16, nullptr, _("Heating..."), false);
        break;
    case PrintState::stopped:
    case PrintState::printed:
        pause.show(&img::print_16x16, nullptr, _("Reprint"), true);
        break;
    case PrintState::aborting:
        pause.show(nullptr, pause_glyph, _("Pause"), false);
        break;
    }

    switch (state) {
    case PrintState::stopped:
    case PrintState::printed:
        stop.show(&img::home_full_16x16, nullptr, _("Home"), true);
        break;
    case PrintState::pausing:
    case PrintState::resuming:
    case PrintState::aborting:
        stop.show(nullptr, nullptr, _("Stop print"), false, true);
        break;
    default:
        stop.show(nullptr, nullptr, _("Stop print"), true, true);
        break;
    }
}

void ScreenPrinting::update_info() {
    auto &vars = marlin_vars();
    const bool ended = state == PrintState::printed || state == PrintState::stopped;
    PrintInfo::Data d;
    d.percent = state == PrintState::printed ? 100 : std::min<uint8_t>(vars.sd_percent_done, 100);

    std::array<char, 12> time;
    StringViewUtf8Parameters<12> params;
    switch (state) {
    case PrintState::pausing:
    case PrintState::paused:
        _("paused").copyToRAM(d.state);
        break;
    case PrintState::aborting:
    case PrintState::stopped:
        _("stopped").copyToRAM(d.state);
        break;
    case PrintState::printed:
        _("finished").copyToRAM(d.state);
        break;
    default:
        if (format_end_time(vars.time_to_end, time)) {
            _("ends %s").formatted(params, time.data()).copyToRAM(d.state);
        }
        break;
    }

    // « Reste 1 h 12 · Z 12.4 mm » ; à la fin, la durée de l'impression
    std::array<char, 40> line_new {};
    StringBuilder line_sb(line_new);
    if (ended) {
        print_info::format_hm(vars.print_duration, time.data(), time.size());
        line_sb.append_string_view(_("Printing time"));
        line_sb.append_printf(" %s", time.data());
    } else {
        if (const uint32_t left = vars.time_to_end; left != marlin_server::TIME_TO_END_INVALID) {
            print_info::format_hm(left, time.data(), time.size());
            StringViewUtf8Parameters<12> left_params;
            line_sb.append_string_view(_("%s left").formatted(left_params, time.data()));
            line_sb.append_string(dot);
        }
        line_sb.append_printf("Z %.1f mm", double(vars.logical_curr_pos[2]));
    }

    // Bobine chargée dans la buse
    for (uint8_t i = 0; i < print_info::max_tools; ++i) {
        if (const Slot slot = Slot::read(i); slot.loaded) {
            d.slot = i;
            d.look = slot.look;
            d.material = slot.material;
            break;
        }
    }

    // En bas : message d'état de Prusa (chauffe, palpage...), sinon prochain changement (M600), sinon le nom du fichier
    if (const auto msg = print_status_message().current_message(); msg && !ended) {
        StringBuilder sb(d.bottom);
        PrintStatusMessageFormatterBuddy::format(sb, msg.message);
        d.bottom_strong = true;
    } else if (const uint32_t change = vars.time_to_pause; change != marlin_server::TIME_TO_END_INVALID && !ended) {
        print_info::format_hm(change, time.data(), time.size());
        StringBuilder sb(d.bottom);
        sb.append_string_view(_("Next change in"));
        sb.append_printf(" %s", time.data());
    } else {
        strlcpy(d.bottom.data(), GCodeInfo::getInstance().GetGcodeFilename(), d.bottom.size());
        if (char *ext = strrchr(d.bottom.data(), '.'); ext && ext != d.bottom.data()) {
            *ext = '\0';
        }
    }
    if (line_new != line_text) {
        line_text = line_new;
        line.Invalidate(); // même tampon, autre texte
    }
    progress.set(d);
    slot_info.set(d);
    thumb.set_progress(d.percent);
    strip.set_progress(d.percent);
}

void ScreenPrinting::update() {
    update_state();
    update_buttons();
    update_info();
}

void ScreenPrinting::reprint() {
    // Comme screen_printing_reprint() : aperçu sauté, association des outils gardée
    marlin_client::print_start(GCodeInfo::getInstance().GetGcodeFilepath(), marlin_client::PreviewSkipIfAble::preview, marlin_client::ResetToolMapping::no);
}

void ScreenPrinting::on_speed() {
    // Éditeur de valeur commun (value_editor_od3d.hpp), appliqué à chaque pas comme dans la maquette
    static constexpr NumericInputConfig config = {
        .min_value = 50,
        .max_value = 200,
        .step = 5,
        .unit = Unit::percent,
    };
    ValueEditor::exec(_("Speed"), marlin_vars().print_speed, config, [](float v) {
        marlin_client::set_print_speed(static_cast<uint16_t>(v));
    });
}

void ScreenPrinting::on_live_z() {
    open_live_adjust_z_screen(); // l'écran de Prusa, comme l'appui long de screen_printing_data_t
}

void ScreenPrinting::on_tune() {
    switch (state) {
    case PrintState::printing:
    case PrintState::skippable:
    case PrintState::paused:
        open_screen<ScreenMenuTune>(); // le menu Tune complet de Prusa, en attendant « Ajuster » (§3.13)
        break;
    case PrintState::printed:
#if BUDDY_ENABLE_CONNECT()
        if (waiting_for_connect_set_ready()) {
            connect_client::MarlinPrinter::set_printer_ready(true);
        }
#endif
        break;
    default:
        break;
    }
    update();
}

void ScreenPrinting::on_pause() {
    // Reprise de screen_printing_data_t::pauseAction()
    switch (state) {
    case PrintState::printing:
        marlin_client::print_pause();
        break;
    case PrintState::skippable:
        skippable_gcode().request_skip();
        break;
    case PrintState::paused:
        marlin_client::print_resume();
        break;
    case PrintState::stopped:
    case PrintState::printed:
        reprint();
        break;
    default:
        break;
    }
    update();
}

void ScreenPrinting::on_stop() {
    // Reprise de screen_printing_data_t::stopAction(), avec sa confirmation
    switch (state) {
    case PrintState::stopped:
    case PrintState::printed:
        marlin_client::print_exit();
        return;
    case PrintState::pausing:
    case PrintState::resuming:
        return;
    default:
        if (MsgBoxWarning(_("Are you sure to stop this printing?"), Responses_YesNo, 1) == Response::Yes) {
            stop_pressed = true;
            waiting_for_abort = true;
            marlin_client::print_abort();
            update();
        }
        return;
    }
}

void ScreenPrinting::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP) {
        // Arrêt demandé pendant le palpage : renvoyé quand il se termine (screen_printing_data_t)
        if (stop_pressed && waiting_for_abort && marlin_vars().gcode_command.get() != marlin_server::Cmd::G29
            && (state == PrintState::aborting || state == PrintState::paused)) {
            marlin_client::print_abort();
            waiting_for_abort = false;
        } else {
            update();
            // Clé retirée après la fin : l'impression se ferme (retour à l'accueil)
            if (!marlin_vars().media_inserted && (state == PrintState::printed || state == PrintState::stopped)) {
                marlin_client::print_exit();
            }
        }
    } else if (event == GUI_event_t::MEDIA) {
        update_buttons(); // « Reprendre » demande la clé
    }
    Screen::windowEvent(sender, event, param);
}

} // namespace od3d
