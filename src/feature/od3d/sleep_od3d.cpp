/// @file
#include "sleep_od3d.hpp"

#include <atomic>
#include <optional>

#include <config_store/store_instance.hpp>
#include <feature/stepper_timeout/stepper_timeout.hpp>
#include <marlin_server.hpp>
#include <module/temperature.h>

namespace od3d::sleep {

namespace {

    std::optional<Settings> cached; ///< fil de l'interface seulement

    std::atomic<bool> takeover_flag = false;
    std::atomic<bool> lights_flag = false;
    std::atomic<bool> fans_flag = false;
    std::atomic<bool> motors_request = false;
    std::atomic<bool> heaters_request = false;
    std::atomic<int8_t> motor_holds = 0;

} // namespace

Settings settings() {
    if (!cached) {
        cached = config_store().od3d_sleep.get();
    }
    return *cached;
}

void set_settings(const Settings &s) {
    cached = s;
    config_store().od3d_sleep.set(s);
}

void publish(bool takeover, bool lights_off, bool fans_off) {
    takeover_flag = takeover;
    lights_flag = lights_off;
    fans_flag = fans_off;
}

void hold_motors(bool hold) {
    motor_holds += hold ? 1 : -1;
}

void request(Due due) {
    if (due.motors) {
        motors_request = true;
    }
    if (due.heaters) {
        heaters_request = true;
    }
}

bool lights_off() {
    return lights_flag;
}

bool fans_off() {
    return fans_flag && !marlin_server::is_printing();
}

void step() {
    const bool held = motor_holds > 0;
    if (takeover_flag || held) {
        buddy::stepper_timeout().reset(); // la veille décide seule de couper les moteurs ; tenus : jamais
    }

    const bool motors = motors_request.exchange(false) && !held;
    const bool heaters = heaters_request.exchange(false);
    if ((!motors && !heaters) || marlin_server::is_printing() || marlin_server::is_processing()) {
        return; // occupé (pause comprise) : l'interface se réveille de toute façon
    }
    if (motors) {
        buddy::stepper_timeout().trigger(); // les axes que Prusa coupe d'habitude (pas Z sur la CORE One)
    }
    if (heaters) {
        Temperature::disable_all_heaters(); // seule action thermique : couper, comme le SafetyTimer hors impression
    }
}

} // namespace od3d::sleep
