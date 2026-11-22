/// @file
/// OneDrop3D (F33, « Amortissement des axes ») : M1997, amortissement réel ζ et facteur Q de X et Y par l'accéléromètre.
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/damping_od3d.hpp>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M1997: OneDrop3D — amortissement des axes X et Y (accéléromètre)
 *
 * Internal GCode (écran Réglages › Fonctions OneDrop3D › « Amortissement des axes »)
 *
 *#### Usage
 *
 *    M1997
 *
 * L'accéléromètre doit être fixé sur la buse et branché : il est vérifié avant tout mouvement, sinon rien ne bouge.
 * Refusé pendant une impression et si la buse dépasse 50 °C. Prise d'origine si nécessaire, centre du plateau à
 * mi-hauteur (comme M1959), puis X et Y sont excités de 30 à 90 Hz par pas de 0,5 Hz (excitation de M1959). ζ et Q
 * sont tirés de la largeur du pic à −3 dB ; rien n'est écrit dans la configuration de l'input shaper. S'arrête depuis
 * l'écran.
 */
void PrusaGcodeSuite::M1997() {
    od3d::damping::run();
}

/** @}*/
