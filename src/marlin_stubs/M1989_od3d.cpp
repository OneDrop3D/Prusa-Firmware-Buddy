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
 *    M1989 H | A | R<bits> [S<bits>] | N<i> | L<slot> | C
 *
 *#### Parameters
 *
 * - `H` - avant de fixer l'accéléromètre : chauffe de la buse coupée, ventilateur à fond si elle est chaude, origine si
 *   besoin, buse au centre à mi-hauteur (comme le début de M1959)
 * - `A` - vérifie, sans bouger, que la buse est froide (< 40 °C, chauffe coupée), l'origine faite et l'accéléromètre
 *   présent (od3d::accel::ready())
 * - `R` - lance la série des tests automatiques dont les bits (od3d::calib_registry::Test) sont donnés : une étape mise
 *   en file à la fois, suivie de `M1989 N<i>` ; tests par bobine : seulement les bobines sans valeur, sauf ceux de `S`
 *   (bits des tests à refaire sur toutes les bobines)
 * - `N` - repère de fin de l'étape i (mis en file par la série) : lit son résultat, met en file l'étape suivante
 * - `L` - avant le chargement d'une bobine pour un test qui imprime (mis en file par la série) : plateau entier palpé
 *   une fois pour la série, à la température de la matière de l'emplacement
 * - `C` - plateau nettoyé après « Plateau plein » : place libre partout, l'étape en attente repart
 *
 * Refusé pendant une impression.
 */
void PrusaGcodeSuite::M1989() {
    namespace assistant = od3d::calib_assistant;
    if (parser.seenval('N')) {
        assistant::step_end(parser.value_byte());
    } else if (parser.seenval('L')) {
        assistant::level_bed(parser.value_byte());
    } else if (parser.seen('C')) {
        assistant::clean_done();
    } else if (parser.seenval('R')) {
        const uint32_t mask = parser.value_ulong();
        assistant::run(mask, parser.seenval('S') ? parser.value_ulong() : 0);
    } else if (parser.seen('A')) {
        assistant::check();
    } else if (parser.seen('H')) {
        assistant::prepare();
    }
}

/** @}*/
