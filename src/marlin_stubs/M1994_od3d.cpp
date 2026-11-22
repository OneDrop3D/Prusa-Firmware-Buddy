/// @file
/// OneDrop3D (F4) : M1994, mesure du pressure advance d'un emplacement par la cellule de charge.
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/pa_od3d.hpp>
#include <gcode/parser.h>
#include <tool_index.hpp>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M1994: OneDrop3D — mesure du pressure advance (PA) d'un emplacement
 *
 * Internal GCode (fiche de la bobine › PA › « Mesurer », début d'impression) ; les paramètres servent à régler la
 * séquence.
 *
 *#### Usage
 *
 *    M1994 [P<slot>] [A] [V1] [L<k>] [H<k>] [S<k>] [R<n>] [B<mm3/s>] [F<mm3/s>] [U<s>] [D<s>] [W<mm>] [J<mm/s2>] [C<n>] [T<C>] [Z<mm>] [O<0|1>] [Q<C>] [X<mm>] [Y<mm>] [I<mm>]
 *
 *#### Parameters
 *
 * - `P` - emplacement 0 à 4 ; sans P, l'emplacement chargé. Chargé dans la buse par le MMU s'il ne l'est pas.
 * - `A` - au début d'une impression (mis en file par l'imprimante) : « Passer » possible, l'impression continue
 * - `L`, `H`, `S` - balayage de K : de L à H par pas de S (0, 0,10, 0,01)
 * - `R` - séquences du second balayage au pas S/2 autour du premier choix (4 ; 0 : aucune)
 * - `B`, `F` - débits lent et rapide (3 et 14 mm³/s, PrusaPATuner)
 * - `U`, `D` - durées des paliers lent et rapide (2 et 0,714 s)
 * - `W` - aller-retour en X (1 mm), `J` - accélération (500 mm/s²), `C` - créneaux par K (5)
 * - `T` - température de la buse (celle de la matière), `Z` - hauteur de la buse pendant la mesure (50 mm)
 * - `O1` - écrit la trace brute, la commande et les résultats dans /usb/OD3D_PA<n>.CSV
 * - `V1` - repli visuel : imprime une ligne par K (K de L à H par pas de S) à l'avant du plateau, après palpage,
 *   comme une vraie impression ; `Q` plateau (celle de la matière), `X`, `Y`, `I` début des lignes et écart (20, 12, 5)
 *
 * Mesure dans le vide à la position de purge de Prusa (méthode PrusaPATuner, réimplémentée) : créneaux d'extrusion
 * lent / rapide avec un petit aller-retour en X, force de la cellule de charge analysée pour chaque K, K retenu au
 * minimum du coût de la réponse, enregistré pour l'emplacement. Refusé pendant une impression (sauf `A`) ; arrêtable
 * depuis l'écran. Réglages (PA, accélérations, consignes) remis à la fin, buse levée.
 */
void PrusaGcodeSuite::M1994() {
    od3d::pa::Request rq;
    if (parser.seenval('P')) {
        rq.slot = parser.value_byte();
    } else if (const auto tool = VirtualToolIndex::currently_selected_opt()) {
        rq.slot = tool->to_raw();
    } else {
        rq.slot = 0xff; // refusé : rien de chargé, aucun emplacement donné
    }
    rq.automatic = parser.seen('A');
    rq.visual = parser.seen('V') && parser.value_bool();
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
    read('W', rq.params.wiggle);
    read('J', rq.params.accel);
    read('Z', rq.park_z);
    read('X', rq.x);
    read('Y', rq.y);
    read('I', rq.pitch);
    if (parser.seenval('R')) {
        rq.refine = parser.value_byte();
    }
    if (parser.seenval('C')) {
        rq.params.cycles = parser.value_byte();
    }
    if (parser.seenval('T')) {
        rq.temperature = parser.value_int();
    }
    if (parser.seenval('Q')) {
        rq.bed_temperature = parser.value_int();
    }
    rq.csv = parser.seen('O') && parser.value_bool();
    od3d::pa::calibrate(rq);
}

/** @}*/
