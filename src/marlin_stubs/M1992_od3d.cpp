/// @file
/// OneDrop3D (F4) : M1992, mesure du pressure advance d'un emplacement par la cellule de charge.
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/pa_od3d.hpp>
#include <gcode/parser.h>
#include <tool_index.hpp>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M1992: OneDrop3D — mesure du pressure advance (PA) d'un emplacement
 *
 * Internal GCode (fiche de la bobine › PA › « Mesurer » de OneDrop3D) ; les paramètres servent à régler la séquence.
 *
 *#### Usage
 *
 *    M1992 [P<slot>] [L<k>] [H<k>] [S<k>] [R<n>] [B<mm3/s>] [F<mm3/s>] [U<s>] [D<s>] [E<mm/mm>] [C<n>] [T<C>] [X<mm>] [Y<mm>] [Z<mm>] [O<0|1>]
 *
 *#### Parameters
 *
 * - `P` - emplacement 0 à 4 (celui chargé dans la buse, obligatoire) ; sans P, l'emplacement chargé
 * - `L`, `H`, `S` - balayage de K : de L à H par pas de S (0, 0,10, 0,02)
 * - `R` - points du second balayage au pas S/4 autour du premier choix (5 ; 0 : aucun)
 * - `B`, `F` - débits lent et rapide (2 et 15 mm³/s)
 * - `U`, `D` - durées des paliers lent et rapide (1 et 0,3 s)
 * - `E` - filament par mm de déplacement en X (0,08)
 * - `C` - créneaux par K (5)
 * - `T` - température de la buse (celle de la matière)
 * - `X`, `Y`, `Z` - centre de la séquence (milieu du plateau), avant du plateau (10 mm), hauteur minimale (20 mm)
 * - `O1` - écrit la trace et les résultats dans /usb/OD3D_PA<n>.CSV
 *
 * Buse chauffée à la température de la matière, purge puis extrusion en créneaux dans le vide avec un déplacement en
 * X (le PA n'agit qu'avec du XY), K balayé, réponse de la cellule de charge analysée (méthode de PrusaPATuner,
 * réimplémentée). Le K retenu est enregistré pour l'emplacement. Refusé pendant une impression ; arrêtable depuis
 * l'écran. Réglages (PA, accélérations, consigne de la buse) remis à la fin.
 */
void PrusaGcodeSuite::M1992() {
    od3d::pa::Request rq;
    if (parser.seenval('P')) {
        rq.slot = parser.value_byte();
    } else if (const auto tool = VirtualToolIndex::currently_selected_opt()) {
        rq.slot = tool->to_raw();
    } else {
        rq.slot = 0xff; // refusé : rien de chargé
    }
    const auto read = [](char c, float &v) {
        if (parser.seenval(c)) {
            v = parser.value_float();
        }
    };
    read('L', rq.k_min);
    read('H', rq.k_max);
    read('S', rq.k_step);
    read('B', rq.params.slow_flow);
    read('F', rq.params.fast_flow);
    read('U', rq.params.slow_s);
    read('D', rq.params.fast_s);
    read('E', rq.params.e_per_mm);
    read('X', rq.x);
    read('Y', rq.y);
    read('Z', rq.z);
    if (parser.seenval('R')) {
        rq.refine = parser.value_byte();
    }
    if (parser.seenval('C')) {
        rq.params.cycles = parser.value_byte();
    }
    if (parser.seenval('T')) {
        rq.temperature = parser.value_int();
    }
    rq.csv = parser.seen('O') && parser.value_bool();
    od3d::pa::calibrate(rq);
}

/** @}*/
