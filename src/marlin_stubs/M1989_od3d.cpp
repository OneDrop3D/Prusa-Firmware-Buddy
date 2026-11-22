/// @file
/// OneDrop3D : M1989, assistant de calibration (feature/od3d/calib_assistant_od3d.hpp).
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/calib_assistant_od3d.hpp>
#include <gcode/parser.h>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M1989: OneDrop3D — assistant de calibration
 *
 * Internal GCode (écran « Assistant de calibration »)
 *
 *#### Usage
 *
 *    M1989 H | A | R<bits> | N<i>
 *
 *#### Parameters
 *
 * - `H` - avant de fixer l'accéléromètre : chauffe de la buse coupée, ventilateur à fond si elle est chaude, origine si
 *   besoin, buse au centre à mi-hauteur (comme le début de M1959)
 * - `A` - vérifie, sans bouger, que la buse est froide (< 40 °C, chauffe coupée), l'origine faite et l'accéléromètre
 *   présent (od3d::accel::ready())
 * - `R` - lance la série des tests automatiques dont les bits (od3d::calib_registry::Test) sont donnés : une étape mise
 *   en file à la fois, suivie de `M1989 N<i>`
 * - `N` - repère de fin de l'étape i (mis en file par la série) : lit son résultat, met en file l'étape suivante
 *
 * Refusé pendant une impression.
 */
void PrusaGcodeSuite::M1989() {
    namespace assistant = od3d::calib_assistant;
    if (parser.seenval('N')) {
        assistant::step_end(parser.value_byte());
    } else if (parser.seenval('R')) {
        assistant::run(parser.value_ulong());
    } else if (parser.seen('A')) {
        assistant::check();
    } else if (parser.seen('H')) {
        assistant::prepare();
    }
}

/** @}*/
