/// @file
/// OneDrop3D (F1, « Déplacer les axes ») : M1990, déplacer X ou Y sans origine, arrêt au contact d'un bord.
#include "PrusaGcodeSuite.hpp"

#include <algorithm>
#include <gcode/parser.h>
#include <module/motion.h>
#include <module/planner.h>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M1990: OneDrop3D — déplacer un axe sans origine
 *
 * Internal GCode (écran « Déplacer les axes » de OneDrop3D)
 *
 *#### Usage
 *
 *    M1990 [X<mm>] [Y<mm>]
 *
 * Déplacement relatif de X, puis de Y, fait comme le mouvement de la prise d'origine (do_homing_move) : vitesse de
 * la prise d'origine et détection de calage des moteurs, donc la tête s'arrête si elle force contre un bord. L'axe
 * reste sans origine. Un axe qui a déjà son origine est ignoré : le G1 normal, avec ses butées logicielles, s'en
 * charge. 50 mm au plus par axe et par commande.
 */
void PrusaGcodeSuite::M1990() {
    constexpr float max_distance = 50.f;
    constexpr std::pair<AxisEnum, char> axes[] = { { X_AXIS, 'X' }, { Y_AXIS, 'Y' } };
    for (const auto &[axis, letter] : axes) {
        if (!parser.seenval(letter) || !axes_need_homing(_BV(axis))) {
            continue;
        }
        const float distance = std::clamp(parser.value_linear_units(), -max_distance, max_distance);
        if (distance == 0) {
            continue;
        }
        // Vitesse de la prise d'origine (fr = 0) : c'est à elle que la détection de calage est réglée
        (void)do_homing_move(axis, distance);
        if (planner.draining()) {
            return; // arrêt demandé
        }
    }
}

/** @}*/
