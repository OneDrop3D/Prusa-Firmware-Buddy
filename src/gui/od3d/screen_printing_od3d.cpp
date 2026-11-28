/// @file
#include "screen_printing_od3d.hpp"

#include "atelier_od3d.hpp"

#include "print_info_od3d.hpp"
#include "screen_files_od3d.hpp"
#include "screen_pa_od3d.hpp"
#include "sleep_gui_od3d.hpp"
#include "menu_items_od3d.hpp"
#include "text_od3d.hpp"

#include <algorithm>
#include <array>
#include <basic_screen_menu.hpp>
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
#include <feature/od3d/speed_mode_od3d.hpp>
#include <feature/print_status_message/print_status_message_formatter_buddy.hpp>
#include <feature/print_status_message/print_status_message_mgr.hpp>
#include <gui/event/gui_event.hpp>
#include <gui/event/knob_event.hpp>
#include <gui.hpp>
#include <i18n.h>
#include <IDialog.hpp>
#include <img_resources.hpp>
#include <liveadjust_z.hpp>
#include <marlin_client.hpp>
#include <marlin_stubs/skippable_gcode.hpp>
#include <odometer.hpp>
#include <option/buddy_enable_connect.h>
#include <option/has_indx.h>
#include <option/has_loadcell.h>
#include <option/has_mmu2.h>
#include <option/has_tool_crash_recovery.h>
#include <option/has_tool_mapping.h>
#include <screen_menu_tune.hpp>
#include <time_tools.hpp>
#include <option/has_toolchanger.h>
#include <tool_index.hpp>
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
    /// L'image dans sa case, centrée ; la même que celle d'« Avant l'impression » (gardée sous la même clé)
    constexpr Rect16 img_rect = { int16_t(thumb_rect.Left() + (thumb_rect.Width() - img_w) / 2), int16_t(thumb_rect.Top() + (thumb_rect.Height() - img_h) / 2), img_w, img_h };
    static_assert(img_rect.Left() - thumb_rect.Left() >= style::radius_big, "les coins de la case tiennent à côté de l'image");
    constexpr uint16_t no_image = img_h + 1;
    // Un pas d'1 % fait passer au plus une ligne à la couleur : la ligne et le trait sur toute la case, ~1,2 Ko ; avec la
    // frise (< 1 Ko, testé dans print_info_test), un pas reste sous 3 Ko (les chiffres de « 47 % » à part : 4,8 Ko chacun)
    static_assert(img_h <= 100 && 2 * thumb_rect.Width() * 3 + 1024 < 3 * 1024);
    constexpr ScaledQoiReader::Params thumb_params(uint16_t grey_rows) {
        return { .factor = thumb_factor, .back = style::thumb_back, .radius = 0, .outside = style::thumb_back, .grey_rows = grey_rows };
    }
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

bool print_ended() {
    const marlin_server::State s = marlin_vars().print_state;
    return s == marlin_server::State::Finished || s == marlin_server::State::Aborted || s == marlin_server::State::Exit;
}

void open_print_tab() {
    Screens::Access()->CloseAll(); // avant Open(), qui serait sinon oublié
    if (print_active() && !print_ended()) {
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
    if (face.danger && !HasValidBackground()) {
        draw_card(style::danger, 2); // bouton danger (§1.5) : contour 2 px #E74626 autour du corps, d'une passe
        return;
    }
    Card::unconditionalDraw();
}

void PrintingButton::draw_content(Rect16 body) {
    const Rect16 top(body.Left() + 4, body.Top() + 12, body.Width() - 8, 16);
    const Color fg = face.active ? style::text : style::text_faint;
    if (face.value[0]) {
        // 9 caractères au plus, « ... » compris (« Geschwindigkeit » : « Geschw... »)
        const Rect16 caption_rect(body.Left() + 2, top.Top(), body.Width() - 4, top.Height());
        std::array<char, 32> caption;
        face.text.copyToRAM(caption.data(), caption.size());
        text::ellipsize(caption.data(), caption_rect.Width() / width(Font::small));
        draw_text(caption_rect, string_view_utf8::MakeRAM(caption.data()), Font::small, face.active ? style::text_muted : style::text_faint, fill(), Align_t::Center());
        draw_text(Rect16(top.Left(), body.Top() + 31, top.Width(), height(Font::normal)), string_view_utf8::MakeRAM(face.value.data()), Font::normal, fg, fill(), Align_t::Center());
        return;
    }
    const Color label_fg = !face.active ? style::text_faint : (face.danger ? style::text_danger : style::text);
    if (face.text.computeNumUtf8Chars() * width(Font::small) > top.Width()) {
        // Plus de 8 caractères : deux lignes de 9 sans l'icône (« Mise en pause », « Ekran główny »)
        draw_label(Rect16(body.Left() + 2, body.Top(), body.Width() - 4, body.Height()), face.text, nullptr, Font::small, label_fg, fill());
        return;
    }
    if (face.icon) {
        draw_icon({ int16_t(body.Left() + (body.Width() - face.icon->w) / 2), int16_t(top.Top()) }, *face.icon, fill(), face.active ? ropfn {} : ropfn { .shadow = is_shadowed::yes });
    } else if (face.glyph) {
        draw_text(top, string_view_utf8::MakeCPUFLASH(face.glyph), Font::small, fg, fill(), Align_t::Center());
    } else if (const Rect16 square(body.Left() + (body.Width() - 14) / 2, top.Top() + 1, 14, 14); face.danger && !noted(square)) {
        display::draw_rounded_rect(square, fill(), style::danger, 3, MIC_ALL_CORNERS);
    }
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
    , grey_rows(img_h)
    , drawn_rows(no_image) {}

PrintThumb::~PrintThumb() {
    if (loading) {
        big_thumb::stop(screen.scaler);
    }
    big_thumb::release();
}

void PrintThumb::set_progress(uint8_t percent) {
    const uint16_t rows = img_h * (100 - std::min<int>(percent, 100)) / 100;
    if (rows != grey_rows) {
        grey_rows = rows;
        if (drawn_rows != no_image) {
            // Les lignes passées à la couleur et le trait seulement, sans repeindre la case
            const bool background_valid = HasValidBackground();
            Invalidate();
            if (background_valid) {
                ValidateBackground();
            }
        }
    }
}

void PrintThumb::unconditionalDraw() {
    const Rect16 box = GetRect();
    const char *path = GCodeInfo::getInstance().GetGcodeFilepath();
    const point_ui16_t at = point_ui16(img_rect.Left(), img_rect.Top());
    const auto draw_line = [&](uint16_t rows, Color color) {
        if (rows > 0 && rows < img_h) { // dans l'image : les marges seulement, l'image est repeinte à part
            display::fill_rect(Rect16::fromLTRB(box.Left(), img_rect.Top() + rows, img_rect.Left(), img_rect.Top() + rows + 1), color);
            display::fill_rect(Rect16::fromLTRB(img_rect.Right() + 1, img_rect.Top() + rows, box.Right() + 1, img_rect.Top() + rows + 1), color);
        } else if (rows == img_h) {
            display::fill_rect(Rect16(box.Left(), img_rect.Top() + rows, box.Width(), 1), color); // sous l'image
        }
    };

    if (HasValidBackground() && drawn_rows != no_image && grey_rows <= drawn_rows && big_thumb::cached(path, thumb_params(0))) {
        // Un pas (~1,2 Ko au lieu de ~40 Ko et d'une lecture de la clé) : les lignes passées du gris à la couleur, depuis
        // la mémoire ; l'ancien trait effacé, le nouveau
        if (grey_rows < drawn_rows) {
            const uint16_t first = grey_rows ? grey_rows + 1 : 0; // la ligne grey_rows est sous le trait
            const uint16_t end = std::min<uint16_t>(drawn_rows + 1, img_h);
            if (first < end) {
                big_thumb::draw_cached(path, thumb_params(grey_rows), at, screen.scaler, first, end - first);
            }
            draw_line(drawn_rows, style::thumb_back);
            if (grey_rows > 0) {
                display::fill_rect(Rect16(box.Left(), img_rect.Top() + grey_rows, box.Width(), 1), style::accent);
            }
            drawn_rows = grey_rows;
        }
        return;
    }

    if (loading) {
        big_thumb::stop(screen.scaler);
        loading = false;
    }
    if (big_thumb::cached(path, thumb_params(0))) {
        // La case autour de l'image, l'image grisée en haut, le trait orange à la hauteur imprimée (§3.12)
        big_thumb::draw_box(box, img_rect, MIC_ALL_CORNERS);
        big_thumb::draw_cached(path, thumb_params(grey_rows), at, screen.scaler, 0, img_h);
        if (grey_rows > 0) {
            display::fill_rect(Rect16(box.Left(), img_rect.Top() + grey_rows, box.Width(), 1), style::accent);
        }
        drawn_rows = grey_rows;
        return;
    }
    display::draw_rounded_rect(box, style::screen, style::thumb_back, style::radius_big, MIC_ALL_CORNERS);
    draw_icon({ int16_t(box.Left() + (box.Width() - 16) / 2), int16_t(box.Top() + (box.Height() - 16) / 2) }, img::print_16x16, style::thumb_back);
    drawn_rows = no_image;
}

void PrintThumb::load_step() {
    if (loading) {
        // Gardée sans être dessinée (le gris n'est pas gardé), puis dessinée d'une passe depuis la mémoire
        loading = big_thumb::step(screen.scaler, {}, false);
        if (loading) {
            gui_reset_loop_timer();
        } else {
            Invalidate();
        }
        return;
    }
    // Pas encore gardée (impression lancée par le réseau) : une lecture, au tout début de l'impression seulement (rien
    // d'imprimé), s'il y a la place de la garder ; sinon l'icône reste
    const char *path = GCodeInfo::getInstance().GetGcodeFilepath();
    if (tried || drawn_rows != no_image || IsInvalid() || !path[0] || grey_rows != img_h || !big_thumb::can_keep()) {
        return;
    }
    tried = true;
    loading = big_thumb::start(path, thumb_params(0), screen.scaler);
    if (loading) {
        gui_reset_loop_timer();
    }
}

PrintInfo::PrintInfo(window_t *parent, Rect16 rect, Part part)
    : window_t(parent, rect)
    , part(part) {}

void PrintInfo::set(const Data &d) {
    const bool changed = part == Part::progress
        ? d.percent != data.percent || d.state != data.state
        : d.slot != data.slot || !(d.look == data.look) || d.material != data.material || d.bottom != data.bottom || d.bottom_strong != data.bottom_strong;
    state_changed = state_changed || d.state != data.state;
    data = d;
    if (changed) {
        const bool background_valid = HasValidBackground();
        Invalidate();
        if (background_valid && part == Part::progress) {
            ValidateBackground(); // un pas : ce qui a changé seulement
        }
    }
}

void PrintInfo::unconditionalDraw() {
    // Des cases qui couvrent tout le rect, chacune avec son fond : pas d'effacement préalable, « 47 % » ne clignote plus
    const Rect16 r = GetRect();
    const int16_t x = r.Left();
    const uint16_t w = r.Width();

    if (part == Part::progress) {
        // « 47 % » : chiffres et % de la police large, puis l'état à côté. Un pas ne repeint que les chiffres qui changent
        // (un chiffre : 4,8 Ko ; avant, « 47 % » et l'état à chaque pas : ~41 Ko), l'état seulement s'il change ou bouge.
        char percent[8], shown[8];
        snprintf(percent, sizeof(percent), "%u %%", data.percent);
        snprintf(shown, sizeof(shown), "%u %%", shown_percent);
        const size_t len = strlen(percent);
        const bool step = HasValidBackground() && !state_changed && strlen(shown) == len;
        size_t from = 0, to = len; // caractères [from, to) à repeindre
        while (step && from < to && shown[from] == percent[from]) {
            ++from;
        }
        while (step && to > from && shown[to - 1] == percent[to - 1]) {
            --to;
        }
        shown_percent = data.percent;
        state_changed = false;
        const uint16_t char_w = width(Font::large), percent_w = len * char_w;
        if (from < to) {
            percent[to] = '\0';
            render_text_align(Rect16(x + from * char_w, r.Top(), (to - from) * char_w, r.Height()), string_view_utf8::MakeRAM(percent + from), Font::large, style::screen, style::text_strong, {}, text_flags(Align_t::LeftTop()));
        }
        if (!step) {
            render_text_align(Rect16(x + percent_w, r.Top(), w - percent_w, r.Height()), string_view_utf8::MakeRAM(data.state.data()), Font::small, style::screen, style::text_muted,
                padding_ui8(8, 29, 0, 0), text_flags(Align_t::LeftTop()));
        }
        return;
    }

    // Haut (34 px) : bobine chargée et ses deux lignes ; bas : message sur deux lignes
    constexpr uint8_t d = 26;
    constexpr int16_t text_dx = d + 8;
    const Rect16 top_rect(x, r.Top(), w, 34);
    if (data.slot != 0xff) {
        const spool::Disc disc(data.look, d);
        draw_pixels(Rect16(x, r.Top(), text_dx, top_rect.Height()), [&](int px, int py) { return disc.pixel(px, py - 2, style::screen); });
        StringViewUtf8Parameters<4> params;
        draw_text(Rect16(x + text_dx, r.Top(), w - text_dx, height(Font::small)), (HAS_TOOLCHANGER() ? _("Now: head %d") : _("Now: slot %d")).formatted(params, data.slot + 1), Font::small, style::text_muted, style::screen);
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
        const bool background_valid = HasValidBackground();
        Invalidate();
        if (background_valid) {
            ValidateBackground(); // les colonnes qui changent seulement
        }
    }
}

void PrintStrip::unconditionalDraw() {
    // Une image d'une passe (la barre clignotait) ; un pas : les colonnes gagnées et le curseur (print_info::strip_part)
    const Rect16 r = GetRect();
    const int w = r.Width(), h = r.Height();
    const std::pair<int, int> cols = HasValidBackground() ? print_info::strip_changed(w, drawn, percent) : std::pair(0, w);
    const int first = cols.first;
    drawn = percent;
    if (first >= cols.second) {
        return;
    }
    draw_pixels(Rect16(r.Left() + first, r.Top(), cols.second - first, h), [&](int x, int y) {
        switch (print_info::strip_part(x + first, y, w, h, percent)) {
        case print_info::StripPart::outside:
            return style::screen;
        case print_info::StripPart::track:
            return style::line;
        case print_info::StripPart::done:
            return style::accent;
        case print_info::StripPart::cursor:
            break;
        }
        return style::text_strong;
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
        tune.show(&img::set_ready_16x16, nullptr, _("Mark ready"), true); // boutons de 2 lignes de 9 caractères
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
        pause.show(&img::print_16x16, nullptr, _("Skip wait"), true);
        break;
    case PrintState::pausing:
        pause.show(nullptr, pause_glyph, _("Pausing"), false);
        break;
    case PrintState::paused:
        pause.show(&img::print_16x16, nullptr, _("Resume printing"), vars.media_inserted);
        break;
    case PrintState::resuming:
        pause.show(&img::print_16x16, nullptr, _("Resuming"), false);
        break;
    case PrintState::reheating:
        pause.show(&img::print_16x16, nullptr, _("Reheating"), false);
        break;
    case PrintState::stopped:
    case PrintState::printed:
        pause.show(&img::print_16x16, nullptr, _("Print again"), true);
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
        StringViewUtf8Parameters<12> time_params;
        line_sb.append_string_view(_("Printing time %s").formatted(time_params, time.data()));
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
    for (uint8_t i = 0; i < VirtualToolIndex::count; ++i) {
        // INDX : chaque tête garde son filament ; celle qui imprime est la tête en service (IF-02 : les 8)
        if (const Slot slot = Slot::read(i); slot.loaded && (HAS_MMU2() || VirtualToolIndex::currently_selected_opt() == VirtualToolIndex::from_raw(i))) {
            d.slot = i;
            d.look = slot.look;
            d.material = slot.material;
            break;
        }
    }

    // En bas : mesure automatique du PA (F4, « Passer » à la place de « Pause »), sinon message d'état de Prusa (chauffe,
    // palpage...), sinon prochain changement (M600), sinon le nom du fichier
#if HAS_LOADCELL() && HAS_MMU2() // INDX : pas de PA mesuré
    if (pa::is_running() && !ended) {
        const pa::Status s = pa::status();
        StringBuilder sb(d.bottom);
        sb.append_printf("PA %u%s", unsigned(s.slot + 1), dot);
        pa_progress(sb, s);
        d.bottom_strong = true;
    } else
#endif
        if (const auto msg = print_status_message().current_message(); msg && !ended) {
        StringBuilder sb(d.bottom);
        PrintStatusMessageFormatterBuddy::format(sb, msg.message);
        d.bottom_strong = true;
    } else if (const uint32_t change = vars.time_to_pause; change != marlin_server::TIME_TO_END_INVALID && !ended) {
        print_info::format_hm(change, time.data(), time.size());
        StringBuilder sb(d.bottom);
        StringViewUtf8Parameters<12> time_params;
        sb.append_string_view(_("Next change in %s").formatted(time_params, time.data()));
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

namespace {

    /// Libellés des modes de vitesse (speed_mode_od3d.hpp), dans l'ordre de speed_mode::Mode
    constexpr std::array<const char *, speed_mode::percents.size()> mode_labels {
        N_("Quiet\n80%, quieter fans"),
        N_("Normal\n100%"),
        N_("Fast\n130%"),
        N_("Very fast\n160%"),
        N_("Ultra fast\n250%"),
    };

    /// Un mode : sa vitesse tout de suite, et les ventilateurs bridés pour Silencieux seulement
    template <speed_mode::Mode M>
    class MI_SPEED_MODE final : public IWindowMenuItem {
    public:
        MI_SPEED_MODE()
            : IWindowMenuItem(_(mode_labels[std::to_underlying(M)])) {}

        void click(IWindowMenu &) override {
            marlin_client::set_print_speed(speed_mode::percent(M));
            speed_mode::quiet = (M == speed_mode::Mode::quiet);
            Screens::Access()->close_screen();
        }
    };

    /// « Vitesse » : les modes, puis le réglage à la main de Prusa (MI_SPEED, 10 à 300 % ; le nôtre s'arrêtait à 200)
    class ScreenSpeedModes final : public BasicScreenMenu<MI_SPEED_MODE<speed_mode::Mode::quiet>, MI_SPEED_MODE<speed_mode::Mode::normal>,
                                       MI_SPEED_MODE<speed_mode::Mode::fast>, MI_SPEED_MODE<speed_mode::Mode::faster>, MI_SPEED_MODE<speed_mode::Mode::fastest>, MI_SPEED> {
    public:
        ScreenSpeedModes()
            : BasicScreenMenu(_("Speed")) {}
    };

} // namespace

MI_SPEED_MODES::MI_SPEED_MODES()
    : IWindowMenuItem(_("Speed\nModes, or set by hand"), &img::speed_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_SPEED_MODES::click(IWindowMenu &) {
    open_screen<ScreenSpeedModes>();
}

void ScreenPrinting::on_speed() {
    open_screen<ScreenSpeedModes>(); // retour du 26/11 : des modes, et plus de 200 % à la main
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
    if (event == GUI_event_t::LOOP && !GetFirstDialog() && !sleep::screen_off()) {
        thumb.load_step();
    }
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
