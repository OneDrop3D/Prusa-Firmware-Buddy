/// @file
/// OneDrop3D : M2003, dilatation réelle de la buse mesurée par la cellule de charge (feature/od3d/nozzle_expansion_od3d.hpp).
#include "PrusaGcodeSuite.hpp"

#include <core/serial.h>
#include <feature/nozzle_thermal_compensation/nozzle_thermal_compensation.hpp>
#include <feature/od3d/nozzle_expansion_od3d.hpp>
#include <gcode/parser.h>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M2003: OneDrop3D — dilatation de la buse mesurée
 *
 * Internal GCode (Réglages › Fonctions OneDrop3D › Température de la buse › « Mesurer la dilatation », assistant de
 * calibration).
 *
 *#### Usage
 *
 *    M2003 [R]
 *
 *#### Parameters
 *
 * - sans paramètre : buse vide (refusé si le capteur de l'extrudeur voit du filament), plateau à 60 °C, origine,
 *   puis le centre du plateau palpé 3 fois à 170, 215, 260, 215 et 170 °C, buse nettoyée à chaque nouvelle température
 *   (170, 215, 260 °C) et stable 30 s à chaque palier. La pente est gardée pour la buse réglée dans Prusa (type de
 *   hotend, haut débit, trempée) et remplace le modèle de Prusa pour elle ; le maillage gardé est oublié. Environ
 *   10 min (estimé). S'arrête depuis l'écran.
 * - `R` - la buse réglée revient au modèle de Prusa
 */
void PrusaGcodeSuite::M2003() {
    namespace ne = od3d::nozzle_expansion;
    if (parser.seen('R')) {
        ne::forget();
    } else {
        ne::run();
    }
    SERIAL_ECHOLNPAIR("M2003 result:", uint8_t(ne::status()), " nm/C:", buddy::nozzle_thermal_compensation::measured_nm_per_c(buddy::nozzle_thermal_compensation::current_nozzle_key()));
}

/** @}*/
