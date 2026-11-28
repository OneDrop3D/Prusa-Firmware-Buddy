/// @file
/// OneDrop3D (F7) : M306, régulation MPC de la buse (Marlin 2.1 : MPCTEMP), absente du Marlin de Prusa.
#include "PrusaGcodeSuite.hpp"

#include <common/marlin_server.hpp>
#include <feature/od3d/mpc_control_od3d.hpp>
#include <gcode/gcode.h>
#include <gcode/parser.h>

#if ENABLED(OD3D_MPCTEMP)

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M306: OneDrop3D — régulation MPC de la buse (expérimentale)
 *
 *#### Usage
 *
 *    M306 [T [S<temp>]] [M<0|1>] [P<W>] [C<J/K>] [R<1/s>] [A<W/K>] [F<W/K>] [H<J/K/mm>]
 *
 *#### Parameters
 *
 * - `T` - Calibration automatique (≈ 5 à 15 min) : refroidissement ventilateur à 100 %, chauffe à pleine
 *         puissance, paliers ventilateur arrêté puis à 100 %. Refusée pendant une impression.
 * - `S` - Avec T : température de calibration (150 à 250 °C, 200 par défaut)
 * - `M` - Régulation : 0 = PID de Prusa (par défaut), 1 = MPC (refusé sans calibration valable et pendant une
 *         impression). Enregistrée.
 * - `P` `C` `R` `A` `F` `H` - Paramètres du modèle, comme Marlin : puissance de la cartouche, capacité thermique du
 *         bloc, réactivité de la sonde, pertes ventilateur arrêté, pertes en plus ventilateur à 100 %, capacité
 *         thermique du filament par mm. Enregistrés ; s'ils ne sont pas valables, refusés sans rien changer.
 *
 * Sans paramètre : affiche les paramètres, l'état de la calibration et la régulation en service.
 * Les protections thermiques de Prusa (emballement, MINTEMP/MAXTEMP, consigne maximale, ventilateur du
 * heatbreak) sont les mêmes avec le PID et le MPC.
 */
void PrusaGcodeSuite::M306() {
    namespace mpc = od3d::mpc;

    if (parser.seen('T')) {
        mpc::run_autotune(parser.seenval('S') ? parser.value_float() : 200.0f);
        return;
    }

    if (parser.seen("PCRAFH")) {
        if (marlin_server::is_printing()) {
            SERIAL_ECHO_MSG("?MPC parameters: not while printing");
            return;
        }
        mpc::Params p = mpc::params();
        if (parser.seenval('P')) {
            p.heater_power = parser.value_float();
        }
        if (parser.seenval('C')) {
            p.block_heat_capacity = parser.value_float();
        }
        if (parser.seenval('R')) {
            p.sensor_responsiveness = parser.value_float();
        }
        if (parser.seenval('A')) {
            p.ambient_xfer_coeff = parser.value_float();
        }
        if (parser.seenval('F')) {
            p.fan_xfer_coeff = parser.value_float();
        }
        if (parser.seenval('H')) {
            p.filament_heat_capacity_permm = parser.value_float();
        }
        // MAT-4 : refusés sans rien enregistrer, la calibration et la régulation en service restent
        if (mpc::is_valid(p)) {
            mpc::set_params(p);
        } else {
            SERIAL_ECHO_MSG("?MPC parameters out of range: not saved");
        }
    }

    if (parser.seenval('M')) {
        switch (mpc::set_mode(parser.value_bool(), marlin_server::is_printing())) {
        case mpc::ModeResult::ok:
            break;
        case mpc::ModeResult::printing:
            SERIAL_ECHO_MSG("?MPC: control mode cannot change while printing");
            break;
        case mpc::ModeResult::not_calibrated:
            SERIAL_ECHO_MSG("?MPC: calibrate first (M306 T)");
            break;
        case mpc::ModeResult::tuning:
            SERIAL_ECHO_MSG("?MPC: calibration in progress");
            break;
        }
    }

    mpc::report();
}

/** @}*/

#endif
