/// @file
/// OneDrop3D : M2000, saut de vitesse de l'extrudeur d'un emplacement, d'où l'accélération max du filament (M204 F).
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/e_jump_od3d.hpp>
#include <feature/od3d/filament_memory_od3d.hpp>
#include <gcode/parser.h>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M2000: OneDrop3D — accélération du filament d'un emplacement (cellule de charge)
 *
 * Internal GCode (Réglages › Fonctions OneDrop3D › « Accélération du filament », assistant de calibration)
 *
 *#### Usage
 *
 *    M2000 [S<slot>]
 *
 *#### Parameters
 *
 * - `S` - emplacement 0 à 4 (celui chargé dans la buse, obligatoire) ; sans S, l'emplacement chargé
 *
 * Buse à la température de la matière, tête à la position de purge de Prusa, PA coupé ; par palier de saut (4 à
 * 47 mm/s, ×1,25), deux impulsions de l'extrudeur seul ; le détecteur de filament coincé de Prusa dit s'il a calé. 70 %
 * du dernier palier sûr est enregistré pour l'emplacement ; à l'impression, M204 F = saut / K (120 à 1500 mm/s²).
 * Refusé pendant une impression ; arrêtable depuis l'écran. Réglages remis à la fin. Retirer le filament extrudé.
 */
void PrusaGcodeSuite::M2000() {
    uint8_t slot = 0xff; // refusé : rien de chargé
    if (parser.seenval('S')) {
        slot = parser.value_byte();
    } else if (const auto loaded = od3d::filament_memory::loaded_slot()) {
        slot = *loaded;
    }
    od3d::e_jump::calibrate(slot);
}

/** @}*/
