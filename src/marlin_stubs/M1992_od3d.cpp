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
 *    M1992 [I<1|2>] [L] [P] [R] [C]
 *
 *#### Parameters
 *
 * - `I` - harmonique mesurée : 1 (par défaut) = la fréquence d'excitation, 2 = son double (essai)
 * - `L` - réglage en direct après une mesure : les deux courroies remesurées en boucle autour de leur pic pendant qu'on
 *         tourne les vis, jusqu'à « Terminé » ou deux tours de suite dans la plage
 * - `P` - diagnostic : excitation de l'assistant de Prusa (M961), les deux moteurs, tête d'avant en arrière
 * - `R` - diagnostic : réponse mesurée 10 périodes après l'arrêt des moteurs, sur 20 périodes
 * - `C` - comparaison (V-03, ~5 min) : R, P, I2 puis la mesure habituelle, chacun sur les deux courroies ; tous les pics
 *         de chaque mode à l'écran et dans le CSV, à comparer avec l'appli de Prusa (I, P et R ignorés)
 *
 * Chaque mesure ajoute son spectre brut (gain par axe et par point) et tous ses pics à /usb/OD3D_BELTS.CSV si une clé
 * est branchée ; la comparaison y ajoute le tableau de ses pics.
 *
 * L'accéléromètre de l'input shaper doit être fixé sur l'extrudeur. Refusé pendant une impression et si la buse dépasse
 * 50 °C, et sans origine (jamais prise accéléromètre fixé). Buse à 10 mm au moins, position du réglage manuel des courroies de Prusa (M961),
 * puis chaque moteur seul (A, puis B) fait vibrer la tête de 70 à 120 Hz ; le pic de la réponse donne la fréquence
 * de la courroie. S'arrête depuis l'écran.
 */
void PrusaGcodeSuite::M1992() {
    const uint8_t harmonic = parser.seenval('I') && parser.value_byte() == 2 ? 2 : 1;
    od3d::belts::run(harmonic, parser.seen('L'), parser.seen('P'), parser.seen('R'), parser.seen('C'));
}

/** @}*/
