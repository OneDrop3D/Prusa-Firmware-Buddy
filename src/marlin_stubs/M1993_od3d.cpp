/// @file
/// OneDrop3D (F22, « Vitesses critiques ») : M1993, carte des vitesses critiques (VFA) par l'accéléromètre.
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/speed_map_od3d.hpp>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M1993: OneDrop3D — vitesses critiques (accéléromètre)
 *
 * Internal GCode (écran Réglages › Fonctions OneDrop3D › « Vitesses critiques »)
 *
 *#### Usage
 *
 *    M1993
 *
 * L'accéléromètre de l'input shaper doit être fixé sur l'extrudeur. Refusé pendant une impression et si la buse dépasse
 * 50 °C, et sans origine (jamais prise accéléromètre fixé). Buse à 10 mm au moins, puis en X et en Y, de 20 à 80 mm/s par pas de 5 (sans
 * dépasser la vitesse maximale de la machine), un aller et un retour à vitesse constante autour du centre du plateau
 * (±100 mm au plus, 10 mm gardés aux bords), accélération 4 000 mm/s² ; mode furtif coupé puis remis, réglages de
 * mouvement rétablis. S'arrête depuis l'écran.
 */
void PrusaGcodeSuite::M1993() {
    od3d::speed_map::run();
}

/** @}*/
