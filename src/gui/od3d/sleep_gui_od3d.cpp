/// @file
#include "sleep_gui_od3d.hpp"

#include <feature/od3d/sleep_od3d.hpp>

#include <ScreenHandler.hpp>
#include <common/marlin_vars.hpp>
#include <display.hpp>
#include <ili9488.hpp>
#include <leds/led_manager.hpp>
#include <marlin_client.hpp>
#include <option/has_serial_print.h>
#include <timing.h>
#include <utils/timing/rate_limiter.hpp>

namespace od3d::sleep {

namespace {

    Idle idle;
    bool dark = false; ///< écran éteint
    uint8_t saved_brightness = 0xff;
    RateLimiter<uint32_t> watch_limiter { 100 };

    Watch watch() {
        auto &vars = marlin_vars();
        const auto top = vars.peek_fsm_states([](const fsm::States &states) { return states.get_top(); });

        uint8_t fsm = 0;
        bool fsm_busy = false;
        if (top) {
            fsm = static_cast<uint8_t>(top->fsm_type) + 1;
            // L'écran de fin d'impression reste ouvert jusqu'à ce qu'on le quitte : il n'empêche pas la veille
            fsm_busy = top->fsm_type != ClientFSM::Printing
#if HAS_SERIAL_PRINT()
                && top->fsm_type != ClientFSM::Serial_printing
#endif
                ;
        }

        return {
            .busy = marlin_client::is_printing() || vars.is_processing.get() || fsm_busy, // is_printing : pause comprise
            .print_state = static_cast<uint8_t>(vars.print_state.get()),
            .fsm = fsm,
            .usb = vars.media_inserted.get() != 0,
            .mmu = static_cast<uint16_t>(vars.mmu2_state.get() << 8 | vars.mmu2_finda.get()),
        };
    }

    void set_backlight(uint8_t brightness) {
        ili9488_brightness_set(brightness); // écrans récents (Configuration::has_display_backlight_control)
        leds::LEDManager::instance().set_lcd_brightness(brightness ? 100 : 0); // anciens : 4e sortie des LED d'état
    }

    void apply(const Settings &s) {
        const bool want_dark = idle.asleep() && s.screen;
        if (want_dark != dark) {
            dark = want_dark;
            if (dark) {
                const uint8_t brightness = ili9488_brightness_get();
                saved_brightness = brightness ? brightness : 0xff; // jamais 0 : l'écran resterait noir au réveil
                display::fill_rect(GuiDefaults::RectScreen, COLOR_BLACK);
            } else {
                set_backlight(saved_brightness);
                if (auto *screen = Screens::Access()->Get()) {
                    screen->Invalidate(); // tout l'écran courant, dialogues compris
                }
            }
        }
        if (dark && ili9488_brightness_get() != 0) { // à l'endormissement, ou l'écran a été réinitialisé
            set_backlight(0);
        }
        publish(s.enabled, idle.asleep() && s.light, idle.asleep() && s.fans, idle.asleep());
    }

} // namespace

void gui_step() {
    const Settings s = settings();
    const uint32_t now = ticks_ms();
    if (watch_limiter.check(now)) {
        idle.step(now, s, watch());
        if (const Due due = idle.take_due(now, s); due.motors || due.heaters) {
            request(due);
        }
    }
    apply(s);
}

void filter_knob(int32_t &encoder_diff, bool &is_btn, BtnState_t btn) {
    const uint32_t now = ticks_ms();
    if (is_btn) {
        const bool release = btn == BtnState_t::Released || btn == BtnState_t::HeldAndReleased;
        is_btn = !idle.input(now, release ? Input::button_release : Input::button, dark);
    }
    if (encoder_diff != 0 && idle.input(now, Input::encoder, dark)) {
        encoder_diff = 0;
    }
}

bool swallow_touch() {
    return idle.input(ticks_ms(), Input::touch, dark);
}

bool screen_off() {
    return dark;
}

bool sleep_now() {
    return idle.sleep_now(ticks_ms());
}

} // namespace od3d::sleep
