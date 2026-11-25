/// @file
#include "screen_mpc_od3d.hpp"
#include "screen_calib_assistant_od3d.hpp" // confirm_stop_measure
#include "atelier_od3d.hpp"
#include "menu_od3d.hpp"
#include "screen_mmu_od3d.hpp" // Panel
#include "value_editor_od3d.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <feature/od3d/mpc_control_od3d.hpp>
#include <common/marlin_vars.hpp>
#include <cmath>
#include <cstdio>
#include <display.hpp>
#include <i18n.h>
#include <initializer_list>
#include <img_resources.hpp>
#include <inc/MarlinConfig.h>
#include <marlin_client.hpp>
#include <window_msgbox.hpp>

static_assert(ENABLED(OD3D_MPCTEMP), "Régulation MPC (F7) : CORE One sans INDX");

namespace od3d {

/// Calibration du modèle thermique : étapes, température de la buse, barre, « Lancer » et « Annuler »
class ScreenMpcCalibration final : public Screen {
public:
    ScreenMpcCalibration();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    /// Les quatre étapes de la maquette, avec leurs repères ✓ ● ○
    class Steps : public window_t {
    public:
        Steps(window_t *parent, Rect16 rect);
        void set_phase(mpc::Phase phase, bool failed);

    protected:
        void unconditionalDraw() override;

    private:
        int8_t current = -1; ///< étape en cours, -1 : aucune ; 4 : toutes faites
        bool failed = false;
    };

    class Bar : public window_t {
    public:
        Bar(window_t *parent, Rect16 rect);
        void set_progress(uint8_t set);

    protected:
        void unconditionalDraw() override;

    private:
        uint8_t progress = 0;
    };

    void update();
    void start();
    void show_status();

    // Ordre de construction = ordre de la molette
    Header header;
    Panel panel;
    window_text_t temp_note;
    Steps steps;
    window_text_t status;
    Bar bar;
    Button start_button;
    Button cancel_button;
    TabBar tabs;

    std::array<char, 24> temp_text {};
    std::array<char, 64> status_text {};
    bool pending = false; ///< M306 T envoyé, pas encore commencé
    uint16_t runs_at_request = 0;
    mpc::TuneStatus shown {};
    bool shown_pending = false;
    int16_t shown_temp = INT16_MIN;
};


namespace {

    // Un bandeau (étapes, note de la buse, état), la barre, puis les boutons (SPEC_G.md §1.9, en plein écran)
    constexpr Rect16 panel_rect = Rect16(style::margin, style::top, 464, 150);
    constexpr Rect16 note_rect = Rect16(150, panel_rect.Top() + 7, 310, height(Font::small));
    constexpr int16_t step_h = 22;
    constexpr Rect16 steps_rect = Rect16(16, panel_rect.Top() + 28, 448, 4 * step_h);
    constexpr Rect16 status_rect = Rect16(22, steps_rect.Bottom() + 6, 436, height(Font::small));
    constexpr Rect16 bar_rect = Rect16(18, panel_rect.Bottom() + 10, 444, 6);
    constexpr Rect16 button_body(int i) {
        return Rect16(18 + 150 * i, bar_rect.Bottom() + 12, 142, 40);
    }
    static_assert(style::inside(panel_rect, steps_rect) && style::inside(panel_rect, status_rect));
    static_assert(style::inside(style::content, style::with_ring(button_body(1))));

    enum Tag : uint8_t {
        tag_start = 0,
        tag_cancel,
    };

    constexpr const char *step_texts[4] = {
        N_("Cooling"),
        N_("Free heating"),
        N_("Measuring losses (fan)"),
        N_("Computing the model"),
    };

    /// Étape de la maquette pour une phase de la calibration (-1 : pas commencée, 4 : toutes faites)
    int8_t step_of(mpc::Phase phase) {
        switch (phase) {
        case mpc::Phase::idle:
            return -1;
        case mpc::Phase::cooling:
        case mpc::Phase::spin_down:
            return 0;
        case mpc::Phase::heating:
            return 1;
        case mpc::Phase::hold:
        case mpc::Phase::hold_fan:
            return 2;
        case mpc::Phase::done:
            return 4;
        case mpc::Phase::failed:
            break;
        }
        return -1;
    }

    enum class Mark : uint8_t {
        done,
        current,
        upcoming,
    };

    /// Repère d'une étape, comme l'écran de progression (screen_progress_od3d.cpp) : ✓ orange, ● blanc, ○ gris
    void draw_mark(int16_t x, int16_t y, Mark mark, Color current_color) {
        switch (mark) {
        case Mark::done:
            for (int16_t t = 0; t < 2; ++t) {
                display::draw_line(point_ui16(x, y + 5 + t), point_ui16(x + 3, y + 8 + t), style::accent);
                display::draw_line(point_ui16(x + 3, y + 8 + t), point_ui16(x + 9, y + 2 + t), style::accent);
            }
            break;
        case Mark::current:
            display::draw_rounded_rect(Rect16(x + 1, y + 2, 8, 8), style::surface, current_color, 4, MIC_ALL_CORNERS);
            break;
        case Mark::upcoming:
            display::draw_rounded_rect(Rect16(x + 1, y + 2, 8, 8), style::surface, style::text_faint, 4, MIC_ALL_CORNERS);
            display::draw_rounded_rect(Rect16(x + 3, y + 4, 4, 4), style::text_faint, style::surface, 2, MIC_ALL_CORNERS);
            break;
        }
    }

    string_view_utf8 error_text(mpc::Error e) {
        switch (e) {
        case mpc::Error::none:
            break;
        case mpc::Error::cancelled:
            return _("Cancelled");
        case mpc::Error::refused:
            return _("Not while printing.");
        case mpc::Error::interrupted:
            return _("Interrupted: nozzle target changed");
        case mpc::Error::cooling_timeout:
            return _("The nozzle does not cool down steadily");
        case mpc::Error::heating_timeout:
            return _("Calibration temperature not reached");
        case mpc::Error::overheat:
            return _("Nozzle too hot: stopped");
        case mpc::Error::hold_timeout:
            return _("The temperature does not settle");
        case mpc::Error::bad_result:
            return _("Inconsistent measurements, try again");
        case mpc::Error::no_memory:
            return _("Not enough memory, try again later.");
        }
        return string_view_utf8::MakeNULLSTR();
    }

    /// Bascule possible maintenant : pas d'impression, pas de calibration en cours
    bool can_change() {
        return !marlin_client::is_printing() && !mpc::tune_status().running;
    }

} // namespace

// ---- Régulation : PID / MPC ----

MI_NOZZLE_CONTROL::MI_NOZZLE_CONTROL()
    : MenuItemSelectMenu(_("Control\nPID (Prusa) or MPC, experimental")) {
    SetIconId(&img::temperature_16x16);
    set_behavior(Behavior::quick_cycle);
    refresh();
}

int MI_NOZZLE_CONTROL::item_count() const {
    return 2;
}

string_view_utf8 MI_NOZZLE_CONTROL::build_item_text(int index, StringViewUtf8ParamBase &) const {
    return string_view_utf8::MakeCPUFLASH(index == 1 ? "MPC (exp.)" : "PID");
}

void MI_NOZZLE_CONTROL::refresh() {
    const bool mpc_on = mpc::is_mpc_selected();
    if (current_item() != (mpc_on ? 1 : 0)) {
        set_current_item(mpc_on ? 1 : 0);
    }
    // Sans calibration valable, le MPC est grisé (on peut toujours revenir au PID s'il était choisi)
    set_enabled(can_change() && (mpc_on || mpc::has_valid_calibration()));
}

void MI_NOZZLE_CONTROL::Loop() {
    refresh();
}

bool MI_NOZZLE_CONTROL::on_item_selected(const OnItemSelectedArgs &args) {
    switch (mpc::set_mode(args.new_index == 1, marlin_client::is_printing())) {
    case mpc::ModeResult::ok:
        set_current_item(mpc::is_mpc_selected() ? 1 : 0);
        break;
    case mpc::ModeResult::printing:
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        break;
    case mpc::ModeResult::not_calibrated:
        MsgBoxWarning(_("Calibrate the MPC model first."), Responses_Ok);
        break;
    case mpc::ModeResult::tuning:
        MsgBoxWarning(_("Calibration in progress."), Responses_Ok);
        break;
    }
    return false; // valeur déjà posée d'après le mode réellement en service
}

MI_MPC_CALIBRATE::MI_MPC_CALIBRATE()
    : IWindowMenuItem(_("Calibrate MPC model\nAbout 10 min, nozzle up to 200 °C"), &img::calibrate_white_16x16,
        marlin_client::is_printing() ? is_enabled_t::no : is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_MPC_CALIBRATE::Loop() {
    set_enabled(!marlin_client::is_printing());
}

void MI_MPC_CALIBRATE::click(IWindowMenu &) {
    open_screen<ScreenMpcCalibration>();
}

ScreenFactory::Creator mpc_calibration_screen() {
    return ScreenFactory::Screen<ScreenMpcCalibration>;
}

// ---- F8 : température selon le débit ----

namespace {

    constexpr NumericInputConfig flow_threshold_config {
        .min_value = mpc::flow_threshold_range.min,
        .max_value = mpc::flow_threshold_range.max,
    };

    constexpr NumericInputConfig flow_max_boost_config {
        .min_value = mpc::flow_max_boost_range.min,
        .max_value = mpc::flow_max_boost_range.max,
    };

} // namespace

// Vaut tout de suite, impression comprise : à l'arrêt, la hausse redescend en rampe
MI_FLOW_TEMP::MI_FLOW_TEMP()
    : WI_ICON_SWITCH_OFF_ON_t(mpc::flow_temp_enabled(), _("Flow temperature\nHotter before fast moves, MPC only"), &img::speed_16x16,
        mpc::is_mpc_selected() ? is_enabled_t::yes : is_enabled_t::no) {}

void MI_FLOW_TEMP::OnChange(size_t) {
    mpc::set_flow_temp_enabled(value());
}

void MI_FLOW_TEMP::Loop() {
    set_enabled(mpc::is_mpc_selected());
}

MI_FLOW_TEMP_THRESHOLD::MI_FLOW_TEMP_THRESHOLD()
    : MI_OD3D_NUMBER(_("Flow threshold\nThe temperature rises above it"), &img::nozzle_16x16, N_("Flow threshold"), flow_threshold_config, "mm3/s",
        mpc::flow_temp_threshold, mpc::set_flow_temp_threshold) {}

MI_FLOW_TEMP_MAX_BOOST::MI_FLOW_TEMP_MAX_BOOST()
    : MI_OD3D_NUMBER(_("Maximum rise\n+1 °C per mm3/s above the threshold"), &img::temperature_16x16, N_("Maximum rise"), flow_max_boost_config, "\xC2\xB0" "C",
        mpc::flow_temp_max_boost, mpc::set_flow_temp_max_boost) {}

MI_FLOW_TEMP_BOOST::MI_FLOW_TEMP_BOOST()
    : MenuItemAutoUpdatingLabel(_("Current rise"), "+%d \xC2\xB0" "C", [](auto *) -> int {
        return mpc::flow_temp_current_boost();
    }) {
    SetIconId(&img::temperature_16x16);
}

// ---- Écran de la calibration ----

ScreenMpcCalibration::Steps::Steps(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void ScreenMpcCalibration::Steps::set_phase(mpc::Phase phase, bool set_failed) {
    const int8_t step = set_failed ? current : step_of(phase);
    if (step != current || set_failed != failed) {
        current = step;
        failed = set_failed;
        Invalidate();
    }
}

void ScreenMpcCalibration::Steps::unconditionalDraw() {
    const Rect16 r = GetRect();
    display::fill_rect(r, style::surface);
    for (int8_t i = 0; i < 4; ++i) {
        const int16_t y = r.Top() + i * step_h;
        const Mark mark = i < current ? Mark::done : (i == current ? Mark::current : Mark::upcoming);
        draw_mark(r.Left() + 6, y + 3, mark, failed ? style::danger : style::text);
        draw_text(Rect16(r.Left() + 24, y, r.Width() - 24, height(Font::small)), _(step_texts[i]), Font::small,
            mark == Mark::upcoming ? style::text_muted : style::text, style::surface);
    }
}

ScreenMpcCalibration::Bar::Bar(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void ScreenMpcCalibration::Bar::set_progress(uint8_t set) {
    set = std::min<uint8_t>(set, 100);
    if (set != progress) {
        progress = set;
        Invalidate();
    }
}

void ScreenMpcCalibration::Bar::unconditionalDraw() {
    const Rect16 r = GetRect();
    const int16_t done = r.Width() * progress / 100;
    if (done > 0) {
        display::fill_rect(Rect16(r.Left(), r.Top(), done, r.Height()), style::accent);
    }
    if (done < r.Width()) {
        display::fill_rect(Rect16(r.Left() + done, r.Top(), r.Width() - done, r.Height()), style::line);
    }
}

ScreenMpcCalibration::ScreenMpcCalibration()
    : Screen(true)
    , header(this, _("MPC calibration"), true)
    , panel(this, panel_rect, _("Thermal model"))
    , temp_note(this, note_rect, is_multiline::no)
    , steps(this, steps_rect)
    , status(this, status_rect, is_multiline::no)
    , bar(this, bar_rect)
    , start_button(this, button_body(0), tag_start, Button::Look::primary, _("Calibrate"), &img::calibrate_white_16x16, style::screen)
    , cancel_button(this, button_body(1), tag_cancel, Button::Look::outline, _("Cancel"), &img::cancel_16x16, style::screen)
    , tabs(this, section_tab()) {
    ClrMenuTimeoutClose(); // l'écran reste ouvert pendant la calibration
    for (window_text_t *t : { &temp_note, &status }) {
        t->set_font(Font::small);
        t->SetBackColor(style::surface);
        t->SetPadding({ 0, 0, 0, 0 });
    }
    temp_note.SetAlignment(Align_t::RightCenter());
    temp_note.SetTextColor(style::text_muted);
    status.SetAlignment(Align_t::LeftCenter());
    status.SetTextColor(style::text_soft);

    shown = mpc::tune_status();
    steps.set_phase(shown.phase, shown.phase == mpc::Phase::failed);
    bar.set_progress(shown.phase == mpc::Phase::idle ? 0 : shown.progress);
    show_status();
    update();
    if (start_button.IsEnabled()) {
        start_button.SetFocus();
    } else if (window_t *first = GetFirstEnabledSubWin()) {
        first->SetFocus();
    }
}

void ScreenMpcCalibration::show_status() {
    if (shown_pending) {
        status.SetText(_("Starting..."));
    } else if (shown.running) {
        StringViewUtf8Parameters<8> params;
        // Texte formaté copié : le tampon des paramètres ne survit pas à la fonction
        _("In progress: %d %%").formatted(params, int(shown.progress)).copyToRAM(status_text.data(), status_text.size());
        status.SetText(string_view_utf8::MakeRAM(status_text.data()));
    } else if (shown.phase == mpc::Phase::done) {
        status.SetText(_("Model saved: MPC can now be chosen"));
    } else if (shown.phase == mpc::Phase::failed) {
        status.SetText(error_text(shown.error));
    } else {
        status.SetText(_("Keep the bed clear; the fan runs at times"));
    }
    status.SetTextColor(!shown.running && shown.phase == mpc::Phase::failed ? style::warning : style::text_soft);
    status.Invalidate();
}

void ScreenMpcCalibration::update() {
    const mpc::TuneStatus st = mpc::tune_status();
    if (pending && (st.running || st.runs != runs_at_request)) {
        pending = false;
    }
    if (st.phase != shown.phase || st.error != shown.error || st.progress != shown.progress || st.running != shown.running || pending != shown_pending) {
        shown = st;
        shown_pending = pending;
        steps.set_phase(st.phase, !st.running && st.phase == mpc::Phase::failed);
        bar.set_progress(st.progress);
        show_status();
    }

    const int16_t temp = int16_t(std::lround(marlin_vars().active_hotend().temp_nozzle.get()));
    if (temp != shown_temp) {
        shown_temp = temp;
        snprintf(temp_text.data(), temp_text.size(), "%d °C", int(temp));
        temp_note.SetText(string_view_utf8::MakeRAM(temp_text.data()));
        temp_note.Invalidate();
    }

    const bool busy = st.running || pending;
    start_button.enable(!busy && !marlin_client::is_printing());
    cancel_button.enable(busy);

    // Un élément désactivé ne garde pas le focus : il va à « Annuler » au lancement, à « Calibrer » à la fin, pas au
    // retour de l'en-tête
    if (window_t *focused = get_focused_child_window(); !focused || !focused->IsEnabled()) {
        if (window_t *to = busy ? &cancel_button : start_button.IsEnabled() ? &start_button : GetFirstEnabledSubWin()) {
            to->SetFocus();
        }
    }
}

void ScreenMpcCalibration::start() {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    const uint16_t before = mpc::tune_status().runs;
    if (marlin_client::gcode_try("M306 T") == marlin_client::GcodeTryResult::Submitted) {
        runs_at_request = before;
        pending = true;
    }
}

void ScreenMpcCalibration::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag == tag_start) {
            start();
        } else if (tag == tag_cancel) {
            if (confirm_stop_measure()) { // pendant une série de l'assistant : il passe à la suivante
                mpc::request_cancel(); // aussi avant le début : M306 T s'arrêtera aussitôt
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

} // namespace od3d
