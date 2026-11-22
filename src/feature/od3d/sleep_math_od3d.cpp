/// @file
#include "sleep_math_od3d.hpp"

namespace od3d::sleep {

namespace {
    constexpr uint32_t ms_per_min = 60'000;
} // namespace

bool Idle::step(uint32_t now_ms, const Settings &s, const Watch &w) {
    const bool was_asleep = asleep_;
    if (!started) {
        started = true;
        last = w;
        activity_ms = now_ms;
    }

    const bool activity = w.busy
        || w.print_state != last.print_state
        || w.fsm != last.fsm
        || (w.usb && !last.usb) // insertion seulement
        || w.mmu != last.mmu;
    last = w;

    if (activity) {
        activity_ms = now_ms;
        asleep_ = false;
    } else if (!asleep_ && s.enabled && now_ms - activity_ms >= (s.delay_min ? s.delay_min : 1u) * ms_per_min) { // 0 (mémoire abîmée) : l'écran resterait inutilisable
        fall_asleep(now_ms);
    }
    return asleep_ != was_asleep;
}

bool Idle::input(uint32_t now_ms, Input in, bool screen_off) {
    activity_ms = now_ms;
    const bool is_button = in == Input::button || in == Input::button_release;

    if (asleep_) {
        asleep_ = false;
        if (!screen_off) {
            return false; // écran resté allumé : l'entrée agit normalement
        }
        woke_ms = now_ms;
        grace = true;
        swallow_button = in == Input::button;
        return true;
    }

    if (is_button) {
        if (!swallow_button) {
            return false;
        }
        swallow_button = in != Input::button_release;
        return true;
    }

    if (grace && now_ms - woke_ms < grace_ms) {
        return true;
    }
    grace = false;
    return false;
}

bool Idle::sleep_now(uint32_t now_ms) {
    if (last.busy) {
        return false;
    }
    if (!asleep_) {
        fall_asleep(now_ms);
    }
    return true;
}

Due Idle::take_due(uint32_t now_ms, const Settings &s) {
    Due due;
    if (!asleep_) {
        return due;
    }
    if (s.motors && !motors_done) {
        motors_done = due.motors = true;
    }
    if (s.heaters && !heaters_done && now_ms - asleep_since_ms >= s.heaters_delay_min * ms_per_min) {
        heaters_done = due.heaters = true;
    }
    return due;
}

void Idle::fall_asleep(uint32_t now_ms) {
    asleep_ = true;
    asleep_since_ms = now_ms;
    grace = false;
    swallow_button = false;
    motors_done = false;
    heaters_done = false;
}

} // namespace od3d::sleep
