/// @file
/// OneDrop3D : M2004, accélération max utile de X et Y par l'accéléromètre (feature/od3d/vibration_od3d.hpp).
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/vibration_od3d.hpp>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M2004: OneDrop3D — accélération max utile (accéléromètre)
 *
 * Internal GCode (assistant de calibration, série « Accéléromètre »)
 *
 *#### Usage
 *
 *    M2004
 *
 * L'accéléromètre doit être fixé sur la buse et branché : il est vérifié avant tout mouvement, sinon rien ne bouge.
 * Refusé pendant une impression, si la buse dépasse 50 °C ou sans origine. Au centre du plateau à mi-hauteur (comme
 * M1959), X puis Y font des déplacements courts de 2 000 à 10 000 mm/s², input shaper en place ; les vibrations qui
 * restent après l'arrêt donnent la plus haute accélération où elles ne remontent pas. Rien n'est appliqué : l'assistant
 * propose M201 X Y. Mesures brutes dans /usb/OD3D_ACCEL.CSV.
 */
void PrusaGcodeSuite::M2004() {
    od3d::accel_limit::run();
}

/** @}*/
