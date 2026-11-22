/// @file
/// OneDrop3D (N3, « Réponse du ventilateur ») : M2001, temps de montée du ventilateur pièce.
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/fan_response_od3d.hpp>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M2001: OneDrop3D — réponse du ventilateur pièce
 *
 * Internal GCode (écran Réglages › Fonctions OneDrop3D › « Réponse du ventilateur », assistant de calibration)
 *
 *#### Usage
 *
 *    M2001
 *
 * Sans filament ni chauffe : ventilateur pièce arrêté 5 s, puis commandé de 0 à 100 % ; ses fronts de tachymètre sont
 * comptés par 50 ms pendant 3 s. Le temps de montée (commande → 90 % du régime final) règle l'avance du ventilateur
 * (« Ventilation anticipée », F26) ; sans régime lisible, elle ne change pas. Ventilateur remis comme avant. Refusé
 * pendant une impression ; s'arrête depuis l'écran.
 */
void PrusaGcodeSuite::M2001() {
    od3d::fan_response::run();
}

/** @}*/
