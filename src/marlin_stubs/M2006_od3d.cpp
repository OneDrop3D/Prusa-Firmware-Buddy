/// @file
/// OneDrop3D : M2006, vérification de l'input shaper par l'accéléromètre (feature/od3d/vibration_od3d.hpp).
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/vibration_od3d.hpp>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M2006: OneDrop3D — vérification de l'input shaper
 *
 * Internal GCode (assistant de calibration, série « Accéléromètre », après l'accélération max utile)
 *
 *#### Usage
 *
 *    M2006
 *
 * L'accéléromètre doit être fixé sur la buse et branché : il est vérifié avant tout mouvement, sinon rien ne bouge.
 * Refusé pendant une impression, si la buse dépasse 50 °C ou sans origine. Au centre du plateau à mi-hauteur (comme
 * M1959), X puis Y font quatre déplacements courts à 5 000 mm/s² avec leur filtre, puis quatre sans (le filtre est
 * toujours remis) : la part des vibrations après l'arrêt que le filtre retire. Rien n'est réglé.
 * Mesures dans /usb/OD3D_SHAPER.CSV.
 */
void PrusaGcodeSuite::M2006() {
    od3d::shaper_check::run();
}

/** @}*/
