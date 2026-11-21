/// @file
/// OneDrop3D (F29) : M1998, descente douce du plateau (démarrer, arrêter, état).
#include "PrusaGcodeSuite.hpp"

#include <option/has_chamber_api.h>
#if HAS_CHAMBER_API()

#include <feature/od3d/chamber_climate_od3d.hpp>
#include <gcode/parser.h>
#include <core/serial.h>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M1998: OneDrop3D — descente douce du plateau
 *
 * Internal GCode (Fonctions OneDrop3D › Climat de la chambre, et essais à la main).
 *
 *#### Usage
 *
 *    M1998 [S<0|1>]
 *
 *#### Parameters
 *
 * - `S1` - démarre la descente depuis la température actuelle du plateau (refusé pendant une impression ou si une
 *          buse chauffe) ; paliers des réglages (Climat de la chambre)
 * - `S0` - arrête la descente et coupe le plateau
 * - sans paramètre : affiche l'état
 *
 * La descente ne fait que baisser la consigne du plateau, jamais au-dessus de sa température mesurée. Toute autre
 * consigne (M140, « Refroidir », préchauffage, minuteur de sécurité) l'arrête ; une impression aussi.
 */
void PrusaGcodeSuite::M1998() {
    namespace cc = od3d::chamber_climate;
    if (parser.seenval('S')) {
        if (parser.value_bool()) {
            if (!cc::start_now()) {
                SERIAL_ECHOLNPGM("M1998: nothing to do (printing, nozzle heating or bed below the floor)");
            }
        } else {
            cc::request_stop();
        }
    }
    const cc::Status s = cc::status();
    if (s.active) {
        SERIAL_ECHOLNPAIR("M1998 active, bed target:", s.target, " next step in s:", s.next_step_s, " steps left:", s.steps_left, " floor:", s.floor_c);
    } else {
        SERIAL_ECHOLNPGM("M1998 inactive");
    }
}

/** @}*/

#endif // HAS_CHAMBER_API()
