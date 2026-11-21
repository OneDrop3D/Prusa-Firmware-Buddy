/// @file
/// OneDrop3D (F5) : M1995, débit maximal d'un emplacement mesuré par la cellule de charge.
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/spool_calib_od3d.hpp>
#include <gcode/parser.h>
#include <tool_index.hpp>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M1995: OneDrop3D — débit maximal (volumétrique) d'un emplacement
 *
 * Internal GCode (fiche de la bobine › PA › « Débit, temp. » › « Débit max ») ; les paramètres servent à régler la
 * mesure.
 *
 *#### Usage
 *
 *    M1995 [P<slot>] [L<mm3/s>] [H<mm3/s>] [S<mm3/s>] [D<s>] [K<excess>] [T<C>] [X<mm>] [Y<mm>] [Z<mm>] [O<0|1>]
 *
 *#### Parameters
 *
 * - `P` - emplacement 0 à 4 (celui chargé dans la buse, obligatoire) ; sans P, l'emplacement chargé
 * - `L`, `H`, `S` - paliers de débit : de L à H par pas de S (2, 30, 2 mm³/s) ; H borné à 40 mm³/s, 24 paliers au plus
 * - `D` - durée d'un palier (3 s, de 2 à 6 s)
 * - `K` - seuil du coude : dépassement relatif de la loi des premiers paliers (0,25)
 * - `T` - température de la buse (celle de la matière)
 * - `X`, `Y`, `Z` - centre des paliers (milieu du plateau), avant du plateau (10 mm), hauteur minimale (20 mm)
 * - `O1` - écrit les paliers et le résultat dans /usb/OD3D_FL<n>.CSV
 *
 * Buse chauffée, purge, puis paliers d'extrusion à débit croissant (la tête avance lentement en X pour étaler le
 * filament), force moyenne de chaque palier ; la rampe s'arrête au coude de la courbe force/débit, enregistré comme
 * débit maximal de l'emplacement. Refusé pendant une impression ; arrêtable depuis l'écran. Réglages remis à la fin.
 */
void PrusaGcodeSuite::M1995() {
    od3d::calib::FlowRequest rq;
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
    read('L', rq.flow_min);
    read('H', rq.flow_max);
    read('S', rq.flow_step);
    read('D', rq.step_s);
    read('K', rq.excess);
    read('X', rq.x);
    read('Y', rq.y);
    read('Z', rq.z);
    if (parser.seenval('T')) {
        rq.temperature = parser.value_int();
    }
    rq.csv = parser.seen('O') && parser.value_bool();
    od3d::calib::measure_flow(rq);
}

/** @}*/
