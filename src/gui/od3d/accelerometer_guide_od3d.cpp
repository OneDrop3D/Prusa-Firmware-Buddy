/// @file
#include "accelerometer_guide_od3d.hpp"

#include <common/marlin_vars.hpp>
#include <i18n.h>
#include <inc/MarlinConfig.h>
#include <marlin_client.hpp>
#include <module/motion.h>
#include <window_msgbox.hpp>

namespace od3d {

bool ask_accelerometer_retry() {
    return MsgBoxWarning(_("The accelerometer does not answer. Attach it to the nozzle and plug it in, then Retry."), Responses_RetryCancel)
        == Response::Retry;
}

const char *accelerometer_homing_text() {
    if (!axes_need_homing()) { // lu dans les octets de Marlin (autre tâche), comme l'écran Déplacer les axes
        return nullptr;
    }
    return marlin_vars().is_processing.get() ? N_("Homing without the accelerometer, please wait...")
                                             : N_("Start homes the printer first: accelerometer not on the nozzle yet (remove it if it is).");
}

bool accelerometer_homed_or_home() {
    if (!axes_need_homing()) {
        return true;
    }
    if (!marlin_vars().is_processing.get()) {
        marlin_client::gcode("M1989 H"); // comme l'assistant : chauffe coupée, ventilateur si chaude, origine, tête au centre
    }
    return false;
}

} // namespace od3d
