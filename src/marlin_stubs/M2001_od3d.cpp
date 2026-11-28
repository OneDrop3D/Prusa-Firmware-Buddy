/// @file
/// OneDrop3D (N3, « Réponse du ventilateur ») : M2001, temps de montée du ventilateur pièce.
#include "PrusaGcodeSuite.hpp"

#include <core/serial.h>
#include <feature/od3d/fan_response_od3d.hpp>
#include <gcode/parser.h>
#include <option/has_indx.h>
#include <option/has_input_shaper_calibration.h>
#include <option/has_manual_belt_tuning.h>
#if HAS_INPUT_SHAPER_CALIBRATION() && HAS_MANUAL_BELT_TUNING()
    #include <feature/od3d/vibration_od3d.hpp>
#endif

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
 *    M2001 [V]
 *
 *#### Parameters
 *
 * - `V` - puis les vibrations de chaque ventilateur seul (pièce, tête, chambre) à 50, 75 et 100 %, moteurs à l'arrêt,
 *   par l'accéléromètre fixé sur la buse (vérifié avant ; buse sous 50 °C) : amplitude à la fréquence de rotation,
 *   comparée à la première mesure captée de chaque ventilateur (sa référence). /usb/OD3D_FANS.CSV
 *
 * Sans filament ni chauffe : ventilateur pièce arrêté 5 s, puis commandé de 0 à 100 % ; ses fronts de tachymètre sont
 * comptés par 50 ms pendant 3 s. Le temps de montée (commande → 90 % du régime final) règle l'avance du ventilateur
 * (« Ventilation anticipée », F26) ; sans régime lisible, elle ne change pas. Ventilateur remis comme avant. Refusé
 * pendant une impression ; s'arrête depuis l'écran.
 */
void PrusaGcodeSuite::M2001() {
#if HAS_INDX()
    // IC-7 : le ventilateur pièce est sur la tête et son tachymètre n'est pas lu ici (fanctl_instances.cpp) : refusé
    // plutôt qu'un « pas de signal » après 8 s de ventilateur ; la ventilation anticipée (F26) se règle à la main
    SERIAL_ERROR_MSG("M2001: part fan rise time not measurable on INDX");
#else
    od3d::fan_response::run();
#endif
#if HAS_INPUT_SHAPER_CALIBRATION() && HAS_MANUAL_BELT_TUNING()
    if (parser.seen('V') && od3d::fan_response::status() != od3d::fan_response::Status::stopped) {
        od3d::fan_vibration::run();
    }
#endif
}

/** @}*/
