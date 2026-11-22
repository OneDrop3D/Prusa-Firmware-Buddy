/// @file
#include "screen_mmu_usb_od3d.hpp"
#include "menu_od3d.hpp"

#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <cstdio>
#include <feature/od3d/mmu_usb_od3d.hpp>
#include <feature/prusa/MMU2/mmu2_mk4.h>
#include <feature/prusa/MMU2/mmu2_state.h>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <ScreenHandler.hpp>
#include <timing.h>
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    constexpr Rect16 info_rect = Rect16(style::margin, style::top + 6, style::screen_w - 2 * style::margin, 120);
    constexpr Rect16 state_rect = Rect16(style::margin, 180, style::screen_w - 2 * style::margin, 24);
    constexpr Rect16 action_body = Rect16(336, 226, 136, 44);
    constexpr uint8_t tag_action = 0;
    constexpr uint32_t pending_timeout_ms = 3'000; ///< file des G-codes pleine ou commande refusée : on cesse d'attendre

    void setup_text(window_text_t &text, Font font, Color color) {
        text.set_font(font);
        text.SetAlignment(Align_t::LeftTop());
        text.SetTextColor(color);
        text.SetBackColor(style::screen);
        text.SetPadding({ 0, 0, 0, 0 });
    }

} // namespace

MI_MMU_USB::MI_MMU_USB()
    : IWindowMenuItem(_("Prepare MMU USB update\nPrinter stops talking to the MMU"), &img::spool_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_MMU_USB::click(IWindowMenu &) {
    if (!mmu_usb::active()) {
        if (marlin_client::is_printing()) {
            MsgBoxWarning(_("Not while printing."), Responses_Ok);
            return;
        }
        if (!config_store().mmu2_enabled.get()) {
            MsgBoxWarning(_("Turn the MMU on first (MMU settings)."), Responses_Ok);
            return;
        }
        if (marlin_client::gcode_try("M709 S2") != marlin_client::GcodeTryResult::Submitted) {
            return;
        }
    }
    open_screen<ScreenMmuUsb>();
}

ScreenMmuUsb::ScreenMmuUsb()
    : Screen(true)
    , header(this, _("MMU USB update"), true)
    , info(this, info_rect, is_multiline::yes, is_closed_on_click_t::no,
          _("The MMU stays powered and the printer no longer talks to it. Connect the MMU micro-USB to the computer and start the installation in PrintForge or PrusaSlicer."))
    , state(this, state_rect, is_multiline::no)
    , action(this, action_body, tag_action, Button::Look::primary, _("Finish"), nullptr, style::screen)
    , tabs(this, section_tab()) {
    ClrMenuTimeoutClose(); // l'écran reste ouvert pendant l'installation
    setup_text(info, Font::normal, style::text_soft);
    setup_text(state, Font::small, style::text_strong);
    if (!mmu_usb::active()) {
        pending = Pending::start; // ouvert par MI_MMU_USB, qui vient d'envoyer M709 S2
        pending_ms = ticks_ms();
    }
    update();
    action.SetFocus();
}

void ScreenMmuUsb::update() {
    const bool on = mmu_usb::active();
    const auto mmu = MMU2::xState(marlin_vars().mmu2_state.get());
    if (pending != Pending::none && ((pending == Pending::start) == on || ticks_diff(ticks_ms(), pending_ms) > int32_t(pending_timeout_ms))) {
        pending = Pending::none;
    }

    const bool waiting = pending == Pending::start || (on && pending != Pending::finish);
    std::array<char, 48> text {};
    if (pending == Pending::start) {
        _("Stopping the communication...").copyToRAM(text.data(), text.size());
    } else if (waiting) {
        _("The MMU waits for the computer.").copyToRAM(text.data(), text.size());
    } else if (mmu == MMU2::xState::Active) {
        // Après « Terminer » : la version relue par la poignée de main, build compris
        const auto v = MMU2::mmu2.GetMMUFWVersion();
        char prefix[32];
        _("Installed:").copyToRAM(prefix, sizeof(prefix));
        snprintf(text.data(), text.size(), "%s MMU %d.%d.%d+%u", prefix, v.major, v.minor, v.build, unsigned(MMU2::mmu2.GetMMUFWVersionBuild()));
    } else if (!config_store().mmu2_enabled.get()) {
        _("MMU off").copyToRAM(text.data(), text.size());
    } else {
        _("Restarting the MMU...").copyToRAM(text.data(), text.size());
    }
    if (text != state_text || !shown) { // le texte ne change que quelques fois : pas de dessin à chaque tour
        state_text = text;
        state.SetText(string_view_utf8::MakeRAM(state_text.data()));
    }

    if (waiting != shown_waiting || !shown) {
        shown_waiting = waiting;
        action.set(waiting ? Button::Look::primary : Button::Look::outline, waiting ? _("Finish") : _("Close"));
    }
    shown = true;
    action.enable(pending == Pending::none);
}

void ScreenMmuUsb::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag == tag_action) {
            if (mmu_usb::active()) {
                // Coupure de l'alimentation, puis reprise du protocole : le MMU démarre sur son nouveau firmware
                if (marlin_client::gcode_try("M709 X2") == marlin_client::GcodeTryResult::Submitted) {
                    pending = Pending::finish;
                    pending_ms = ticks_ms();
                }
            } else {
                Screens::Access()->close_screen();
                return;
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
