/// @file
/// OneDrop3D (F14, « Tension des courroies ») : M1992, mesure de la fréquence des courroies par l'accéléromètre.
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/belt_tension_od3d.hpp>
#include <gcode/parser.h>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M1992: OneDrop3D — tension des courroies (accéléromètre)
 *
 * Internal GCode (écran Réglages › Fonctions OneDrop3D › « Tension des courroies »)
 *
 *#### Usage
 *
 *    M1992 [I<1|2>]
 *
 *#### Parameters
 *
 * - `I` - harmonique mesurée : 1 (par défaut) = la fréquence d'excitation, 2 = son double (essai)
 *
 * L'accéléromètre de l'input shaper doit être fixé sur l'extrudeur. Refusé pendant une impression et si la buse dépasse
 * 50 °C, et sans origine (jamais prise accéléromètre fixé). Buse à 10 mm au moins, position du réglage manuel des courroies de Prusa (M961),
 * puis chaque moteur seul (A, puis B) fait vibrer la tête de 70 à 120 Hz ; le pic de la réponse donne la fréquence
 * de la courroie. S'arrête depuis l'écran.
 */
void PrusaGcodeSuite::M1992() {
    const uint8_t harmonic = parser.seenval('I') && parser.value_byte() == 2 ? 2 : 1;
    od3d::belts::run(harmonic);
}

/** @}*/
