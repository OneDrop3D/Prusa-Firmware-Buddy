/// @file
/// OneDrop3D : M2005, pose de l'accéléromètre (feature/od3d/vibration_od3d.hpp).
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/vibration_od3d.hpp>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M2005: OneDrop3D — pose de l'accéléromètre
 *
 * Internal GCode (assistant de calibration, série « Accéléromètre », en premier)
 *
 *#### Usage
 *
 *    M2005
 *
 * L'accéléromètre doit être fixé sur la buse et branché : il est vérifié avant tout mouvement, sinon rien ne bouge.
 * Refusé pendant une impression, si la buse dépasse 50 °C ou sans origine. Au centre du plateau à mi-hauteur, tête au
 * repos : inclinaison (pesanteur) et bruit ; puis un aller-retour court sur X et sur Y : la part du mouvement dans le
 * plan horizontal. Échoue si l'accéléromètre est clipsé de travers ou mal fixé (son orientation à plat n'est pas
 * jugée : Prusa ne s'en sert pas). Rien n'est réglé.
 * Mesures dans /usb/OD3D_MOUNT.CSV.
 */
void PrusaGcodeSuite::M2005() {
    od3d::accel_mount::run();
}

/** @}*/
