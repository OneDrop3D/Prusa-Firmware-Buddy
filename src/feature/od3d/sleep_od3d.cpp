/// @file
#include "sleep_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <optional>

#include <config_store/store_instance.hpp>
#include <feature/stepper_timeout/stepper_timeout.hpp>
#include <marlin_server.hpp>
#include <module/stepper.h>
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
    std::atomic<bool> asleep_flag = false;
    std::atomic<uint8_t> z_hold_pct = 100; ///< publié par l'interface (z_hold)
    std::optional<uint8_t> cached_z_hold; ///< fil de l'interface seulement

    // Fil de Marlin : maintien de Z baissé (ou Z coupé) pendant cette veille
    bool z_quiet = false;
    uint8_t z_saved_ihold = 0;
#if AXIS_IS_TMC(Z2)
    uint8_t z2_saved_ihold = 0;
#endif

    void quiet_z(uint8_t pct) {
        z_quiet = true;
        if (pct == 0) {
            z_saved_ihold = 0;
            disable_Z(); // l'origine Z est perdue : la prochaine impression la reprend
            return;
        }
        z_saved_ihold = stepperZ.ihold();
        stepperZ.ihold(reduced_ihold(z_saved_ihold, pct)); // le courant de marche (IRUN) ne change pas
#if AXIS_IS_TMC(Z2)
        z2_saved_ihold = stepperZ2.ihold();
        stepperZ2.ihold(reduced_ihold(z2_saved_ihold, pct));
#endif
    }

    void restore_z() {
        z_quiet = false;
        if (z_saved_ihold == 0) {
            return; // Z coupé : il se réalimente au prochain mouvement, avec son courant normal
        }
        stepperZ.ihold(z_saved_ihold);
#if AXIS_IS_TMC(Z2)
        stepperZ2.ihold(z2_saved_ihold);
#endif
    }

} // namespace

uint8_t z_hold() {
    if (!cached_z_hold) {
        cached_z_hold = std::min<uint8_t>(config_store().od3d_sleep_z_hold.get(), 100);
    }
    return *cached_z_hold;
}

void set_z_hold(uint8_t pct) {
    cached_z_hold = std::min<uint8_t>(pct, 100);
    config_store().od3d_sleep_z_hold.set(*cached_z_hold);
}

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

void publish(bool takeover, bool lights_off, bool fans_off, bool asleep) {
    takeover_flag = takeover;
    lights_flag = lights_off;
    fans_flag = fans_off;
    z_hold_pct = z_hold();
    asleep_flag = asleep;
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

    const bool busy = marlin_server::is_printing() || marlin_server::is_processing();
    // Maintien de Z : baissé une fois endormi, rétabli au réveil, avant tout travail ou quand un écran tient les moteurs
    if (z_quiet && (!asleep_flag || busy || held)) {
        restore_z();
    } else if (!z_quiet && asleep_flag && !busy && !held && z_hold_pct < 100) {
        quiet_z(z_hold_pct);
    }

    const bool motors = motors_request.exchange(false) && !held;
    const bool heaters = heaters_request.exchange(false);
    if ((!motors && !heaters) || busy) {
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
