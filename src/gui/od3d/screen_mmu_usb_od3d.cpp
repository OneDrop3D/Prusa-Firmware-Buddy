/// @file
#include "screen_mmu_usb_od3d.hpp"
#include "menu_od3d.hpp"

#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <cstdarg>
#include <cstdio>
#include <feature/od3d/calib_assistant_od3d.hpp>
#include <feature/od3d/mmu_flash_od3d.hpp>
#include <feature/od3d/mmu_usb_od3d.hpp>
#include <feature/prusa/MMU2/mmu2_mk4.h>
#include <feature/prusa/MMU2/mmu2_state.h>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <ScreenHandler.hpp>
#include <timing.h>
#include <usb_host.h>
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    constexpr Rect16 info_rect = Rect16(style::margin, style::top + 6, style::screen_w - 2 * style::margin, 120);
    constexpr Rect16 bar_rect = Rect16(style::margin, 170, style::screen_w - 2 * style::margin, 6);
    constexpr Rect16 state_rect = Rect16(style::margin, 180, style::screen_w - 2 * style::margin, 24);
    constexpr Rect16 install_body = Rect16(192, 226, 136, 44);
    constexpr Rect16 action_body = Rect16(336, 226, 136, 44);
    constexpr uint8_t tag_action = 0;
    constexpr uint8_t tag_install = 1;
    constexpr uint32_t pending_timeout_ms = 3'000; ///< file des G-codes pleine ou commande refusée : on cesse d'attendre

    void setup_text(window_text_t &text, Font font, Color color) {
        text.set_font(font);
        text.SetAlignment(Align_t::LeftTop());
        text.SetTextColor(color);
        text.SetBackColor(style::screen);
        text.SetPadding({ 0, 0, 0, 0 });
    }

    /// M709 S2 si le mode n'est pas déjà actif ; faux s'il est refusé
    bool enter_usb_mode() {
        if (mmu_usb::active()) {
            return true;
        }
        if (marlin_client::is_printing()) {
            MsgBoxWarning(_("Not while printing."), Responses_Ok);
            return false;
        }
        if (calib_assistant::mmu_in_use()) { // la série reprend le MMU ensuite : les autres tests peuvent tourner pendant le flash
            MsgBoxWarning(_("The calibration assistant is using the MMU. Try again after this test."), Responses_Ok);
            return false;
        }
        if (!config_store().mmu2_enabled.get()) {
            MsgBoxWarning(_("Turn the MMU on first (MMU settings)."), Responses_Ok);
            return false;
        }
        return marlin_client::gcode_try("M709 S2") == marlin_client::GcodeTryResult::Submitted;
    }

    using Text = std::array<char, 64>;

    void put(Text &text, const string_view_utf8 &label) {
        label.copyToRAM(text);
    }

    /// `label` traduit, puis la valeur
    __attribute__((format(printf, 3, 4))) void compose(Text &text, const string_view_utf8 &label, const char *format, ...) {
        char prefix[48];
        label.copyToRAM(prefix, sizeof(prefix));
        char value[24];
        va_list args;
        va_start(args, format);
        vsnprintf(value, sizeof(value), format, args);
        va_end(args);
        snprintf(text.data(), text.size(), "%s %s", prefix, value);
    }

} // namespace

MI_MMU_USB::MI_MMU_USB()
    : IWindowMenuItem(_("Prepare MMU USB update\nPrinter stops talking to the MMU"), &img::spool_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_MMU_USB::click(IWindowMenu &) {
    if (enter_usb_mode()) {
        ScreenMmuUsb::open_front = false;
        open_screen<ScreenMmuUsb>();
    }
}

MI_MMU_USB_FRONT::MI_MMU_USB_FRONT()
    : IWindowMenuItem(_("Update the MMU via the front USB\nMMU micro-USB into the front port"), &img::spool_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_MMU_USB_FRONT::click(IWindowMenu &) {
    if (enter_usb_mode()) {
        ScreenMmuUsb::open_front = true;
        open_screen<ScreenMmuUsb>();
    }
}

ScreenMmuUsb::ScreenMmuUsb()
    : Screen(true)
    , front(open_front)
    , header(this, front ? _("MMU via front USB") : _("MMU USB update"), true)
    , info(this, info_rect, is_multiline::yes, is_closed_on_click_t::no,
          front ? _("Remove the USB drive, then plug the micro-USB of the MMU board into the front USB port of the printer. The printer writes the MMU firmware it carries, then reads it back.")
                : _("The MMU stays powered and the printer no longer talks to it. Connect the MMU micro-USB to the computer and start the installation in PrintForge or PrusaSlicer."))
    , state(this, state_rect, is_multiline::no)
    , bar(this, bar_rect, style::accent, style::surface, 3)
    , install_button(this, install_body, tag_install, Button::Look::primary, _("Install"), nullptr, style::screen)
    , action(this, action_body, tag_action, Button::Look::primary, _("Finish"), nullptr, style::screen)
    , tabs(this, section_tab()) {
    ClrMenuTimeoutClose(); // l'écran reste ouvert pendant l'installation
    setup_text(info, Font::normal, style::text_soft);
    setup_text(state, Font::small, style::text_strong);
    bar.Hide();
    if (mmu_flash::Version v; front && mmu_flash::bundled(v)) {
        snprintf(version_text.data(), version_text.size(), "%u.%u.%u+%u", v.major, v.minor, v.patch, v.build);
    }
    if (!front) {
        install_button.Hide();
    }
    if (!mmu_usb::active()) {
        pending = Pending::start; // ouvert par MI_MMU_USB, qui vient d'envoyer M709 S2
        pending_ms = ticks_ms();
    }
    update();
    action.SetFocus();
}

void ScreenMmuUsb::front_state(Text &text, bool &busy) {
    using mmu_flash::Error;
    using mmu_flash::Phase;
    const auto st = mmu_flash::status();
    busy = st.phase == Phase::connecting || st.phase == Phase::writing;

    const bool show_bar = busy || st.phase == Phase::done;
    if (show_bar != shown_bar || !shown) {
        shown_bar = show_bar;
        bar.set_visible(show_bar);
    }
    bar.set_progress_percent(st.percent);

    const bool retry = st.phase == Phase::failed;
    if (retry != shown_retry || !shown) {
        shown_retry = retry;
        install_button.set(Button::Look::primary, retry ? _("Retry") : _("Install"));
    }
    install_button.enable(!busy && st.phase != Phase::done && mmu_flash::device() != mmu_flash::Device::none && version_text[0]);

    switch (st.phase) {
    case Phase::connecting:
        return put(text, _("Starting the MMU bootloader..."));
    case Phase::writing:
        return compose(text, _("Writing and reading back:"), "%u %%", unsigned(st.percent));
    case Phase::done:
        return put(text, _("Verified. Unplug the cable, then Finish."));
    case Phase::failed:
        switch (st.error) {
        case Error::file:
            return put(text, _("MMU firmware missing in the printer."));
        case Error::no_mmu:
            return put(text, _("MMU disconnected."));
        case Error::no_bootloader:
            return put(text, _("The MMU did not enter its bootloader."));
        case Error::no_answer:
            return compose(text, _("MMU bootloader not answering:"), "%c", st.command);
        case Error::refused:
            return compose(text, _("Command refused by the MMU:"), "%c", st.command);
        case Error::wrong_chip:
            return put(text, _("Not an MMU3 board: nothing written."));
        case Error::verify:
            return compose(text, _("Read-back differs at"), "0x%04X", unsigned(st.address));
        case Error::none:
            break;
        }
        return;
    case Phase::idle:
        break;
    }
    if (mmu_flash::device() != mmu_flash::Device::none) {
        compose(text, _("MMU connected, firmware to install:"), "%s", version_text.data());
    } else if (usb_host::is_media_inserted()) {
        put(text, _("Remove the USB drive from the front port."));
    } else {
        put(text, _("Waiting for the MMU on the front USB port..."));
    }
}

void ScreenMmuUsb::update() {
    const bool on = mmu_usb::active();
    const auto mmu = MMU2::xState(marlin_vars().mmu2_state.get());
    if (pending != Pending::none && ((pending == Pending::start) == on || ticks_diff(ticks_ms(), pending_ms) > int32_t(pending_timeout_ms))) {
        pending = Pending::none;
    }

    const bool waiting = pending == Pending::start || (on && pending != Pending::finish);
    bool busy = false; ///< écriture par le port avant en cours : rien ne l'interrompt
    Text text {};
    if (front && on && pending == Pending::none) {
        front_state(text, busy);
    } else if (pending == Pending::start) {
        put(text, _("Stopping the communication..."));
    } else if (waiting) {
        put(text, _("The MMU waits for the computer."));
    } else if (mmu == MMU2::xState::Active) {
        // Après « Terminer » : la version relue par la poignée de main, build compris
        const auto v = MMU2::mmu2.GetMMUFWVersion();
        char prefix[32];
        _("Installed:").copyToRAM(prefix, sizeof(prefix));
        snprintf(text.data(), text.size(), "%s MMU %d.%d.%d+%u", prefix, v.major, v.minor, v.build, unsigned(MMU2::mmu2.GetMMUFWVersionBuild()));
    } else if (!config_store().mmu2_enabled.get()) {
        put(text, _("MMU off"));
    } else {
        put(text, _("Restarting the MMU..."));
    }
    if (front && !(on && pending == Pending::none)) {
        bar.set_visible(shown_bar = false);
        install_button.enable(false);
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
    action.enable(pending == Pending::none && !busy);
}

void ScreenMmuUsb::install() {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    StringViewUtf8Parameters<16> params;
    if (MsgBoxQuestion(_("Write MMU firmware %s from the printer? Do not unplug anything until it is verified.").formatted(params, version_text.data()), Responses_YesNo) == Response::Yes) {
        mmu_flash::start();
    }
}

void ScreenMmuUsb::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag == tag_install) {
            install();
        } else if (tag == tag_action) {
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
