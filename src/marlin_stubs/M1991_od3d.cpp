/// @file
/// OneDrop3D (F18, « Musique des moteurs ») : M1991, jouer un fichier MIDI de la clé USB avec les moteurs.
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/music_player_od3d.hpp>
#include <gcode/parser.h>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M1991: OneDrop3D — musique des moteurs
 *
 * Internal GCode (écran Contrôle › « Musique des moteurs » de OneDrop3D, et fin d'impression si l'option est active)
 *
 *#### Usage
 *
 *    M1991 [A<0|1|2>]
 *
 *#### Parameters
 *
 * - `A` - voix : 0 = 3 voix (mélodie et 2e voix sur les moteurs du haut, basse sur le plateau), 1 = 2 voix (mélodie
 *   et basse sur les moteurs du haut), 2 = la mélodie seule ; sans A, le réglage de l'écran
 *
 * Joue le fichier .mid choisi dans l'écran (racine de la clé, format 0 ou 1 ; à chaque instant la plus aiguë, la
 * 2e plus aiguë et la plus grave, accords de moins de 40 ms lissés), lu en flux. Prise d'origine si nécessaire, hauteur sûre, allers-retours autour du centre de la course,
 * vitesses et accélérations bornées, mode furtif coupé pendant la lecture puis remis. Refusé pendant une
 * impression ; s'arrête depuis l'écran, ou si une impression est lancée.
 */
void PrusaGcodeSuite::M1991() {
    const uint8_t motors = parser.seenval('A') ? parser.value_byte() : 0xff;
    od3d::music::play(motors);
}

/** @}*/
