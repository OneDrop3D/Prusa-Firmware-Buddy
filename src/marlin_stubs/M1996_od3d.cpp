/// @file
/// OneDrop3D (F6) : M1996, température minimale d'un emplacement mesurée par la cellule de charge.
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/spool_calib_od3d.hpp>
#include <gcode/parser.h>
#include <tool_index.hpp>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M1996: OneDrop3D — température minimale d'un emplacement
 *
 * Internal GCode (fiche de la bobine › PA › « Débit, temp. » › « Temp. min ») ; les paramètres servent à régler la
 * mesure. Le résultat est seulement affiché : il n'impose rien aux impressions.
 *
 *#### Usage
 *
 *    M1996 [P<slot>] [T<C>] [L<C>] [S<C>] [F<mm3/s>] [R<ratio>] [D<s>] [W<s>] [Q<C>] [B<mm>] [Z<mm>] [X<mm>] [Y<mm>] [I<mm>] [M<mm>] [O<0|1>]
 *
 *#### Parameters
 *
 * - `P` - emplacement 0 à 4 (celui chargé dans la buse, obligatoire) ; sans P, l'emplacement chargé
 * - `T` - température de départ (celle de la matière, jamais au-dessus)
 * - `L` - plancher (T − 60 ; jamais sous la température minimale d'extrusion + 10 °C)
 * - `S` - pas de la descente (5 °C, de 2 à 20)
 * - `F` - débit constant (5 mm³/s, de 1 à 15)
 * - `R` - force acceptable jusqu'à R × la force à la température de départ (1,5)
 * - `D` - durée d'un palier d'extrusion (3 s, de 2 à 6 s)
 * - `W` - stabilisation à ±1 °C avant chaque palier (5 s, de 0 à 30 s)
 * - `Q` - température du plateau (celle de la matière)
 * - `B`, `Z` - largeur et hauteur du cordon imprimé (0,6 et 0,3 mm)
 * - `X`, `Y`, `I`, `M` - centre des lignes (milieu du plateau), première ligne à l'avant (Y12), écart entre les lignes
 *   (3 mm), longueur maximale d'une ligne (180 mm)
 * - `O1` - écrit les paliers et le résultat dans /usb/OD3D_TM<n>.CSV
 *
 * Plateau chauffé, origine, nivellement local, purge au bord avant (comme le G-code de départ de la CORE One), buse à
 * la température de départ, puis une ligne imprimée par palier à débit constant ; entre deux lignes, la buse levée
 * descend au palier suivant et se stabilise. La descente s'arrête dès que la force dépasse le seuil. La plus basse
 * température acceptable est enregistrée pour l'emplacement. Refusé pendant une impression ; arrêtable depuis l'écran.
 * Réglages et consignes remis et buse levée à la fin.
 */
void PrusaGcodeSuite::M1996() {
    od3d::calib::TempRequest rq;
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
    const auto read_int = [](char c, int16_t &v) {
        if (parser.seenval(c)) {
            v = int16_t(parser.value_int());
        }
    };
    read_int('T', rq.start);
    read_int('L', rq.floor);
    read_int('S', rq.step);
    read('F', rq.flow);
    read('R', rq.max_ratio);
    read('D', rq.step_s);
    read('W', rq.hold_s);
    read('X', rq.place.x);
    read('Y', rq.place.y);
    read('I', rq.place.pitch);
    read('M', rq.place.max_length);
    read('B', rq.place.bead.width);
    read('Z', rq.place.bead.height);
    if (parser.seenval('Q')) {
        rq.place.bed_temperature = int16_t(parser.value_int());
    }
    rq.csv = parser.seen('O') && parser.value_bool();
    od3d::calib::measure_temperature(rq);
}

/** @}*/
