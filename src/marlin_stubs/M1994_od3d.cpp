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
 * Internal GCode (fiche de la bobine › PA › « Mesurer » de OneDrop3D) ; les paramètres servent à régler la séquence.
 *
 *#### Usage
 *
 *    M1994 A
 *    M1994 P<slot> A1
 *    M1994 A2
 *    M1994 [P<slot>] [V<0|1>] [L<k>] [H<k>] [S<k>] [R<n>] [B<mm3/s>] [F<mm3/s>] [U<s>] [D<s>] [W<mm>] [Z<mm>] [C<n>] [T<C>] [Q<C>] [X<mm>] [Y<mm>] [I<mm>] [O<0|1>] [J1]
 *
 *#### Parameters
 *
 * - `A` - mesure automatique au départ d'une impression (mise en file par l'imprimante si le réglage « PA automatique »
 *   est actif) : choisit les emplacements utilisés à mesurer et leur place à côté de la zone d'impression (M555), puis
 *   met en file pour chacun « Charger dans la buse » de Prusa et `M1994 P<n> A1` (sa mesure courte), enfin le filament
 *   remis comme avant ; un échec ou « Passer » laisse le PA du trancheur, l'impression continue
 * - `P` - emplacement 0 à 4 ; s'il n'est pas dans la buse, « Charger dans la buse » de Prusa est mis en file (l'autre
 *   filament est déchargé) après l'origine si besoin et la position de chargement levée (G28 O I, G27 W2), suivi du
 *   même M1994 avec J1 ; sans P, l'emplacement chargé
 * - `V` - motif : `V1` compact (lignes serrées sans repères, K cherché ligne après ligne, 8 lignes au plus ; ses
 *   valeurs par défaut : C2, U0.6, D0.25, I2.5, R0), `V0` grand (balayage complet, repères) ; sans V, le réglage
 *   « Grand motif PA » (compact par défaut)
 * - `J1` - chargement déjà tenté (mis par M1994 lui-même) : sans la bobine dans la buse, échec sans autre tentative
 * - `L`, `H`, `S` - balayage de K : de L à H par pas de S (0, 0,10, 0,02)
 * - `R` - lignes du second balayage au pas S/4 autour du premier choix (5 ; 0 : aucune) ; 15 lignes au plus en tout
 * - `B`, `F` - débits lent et rapide (1,5 et 10 mm³/s)
 * - `U`, `D` - durées des tronçons lent et rapide (0,8 et 0,3 s)
 * - `W`, `Z` - largeur et hauteur du cordon (0,45 et 0,2 mm)
 * - `C` - tronçons rapides par ligne (3)
 * - `T`, `Q` - températures de la buse et du plateau (celles de la matière)
 * - `X`, `Y`, `I` - début des lignes (X20), première ligne à l'avant (Y12), écart entre les lignes (5 mm)
 * - `O1` - écrit la trace et les résultats dans /usb/OD3D_PA<n>.CSV
 *
 * Plateau et buse chauffés ensemble, origine pendant que le plateau chauffe, nettoyage et nivellement local de la zone
 * du motif une fois le plateau chaud (comme le G-code de départ de la CORE One), buse à la température de la matière
 * (purge pendant la montée, lignes une fois stable), chargement de la bobine si besoin, ligne de purge au bord avant, puis une ligne imprimée par K à hauteur de première
 * couche, lente / rapide / lente, avec à sa droite des traits qui donnent son numéro. La force verticale de la cellule
 * de charge pendant les changements de vitesse donne le K (méthode inspirée de PrusaPATuner, réimplémentée), enregistré
 * pour l'emplacement ; l'écran permet aussi de choisir la meilleure ligne à l'œil. Refusé pendant une impression ;
 * arrêtable depuis l'écran. Réglages (PA, accélérations, consignes, maillage) remis à la fin, buse levée.
 */
void PrusaGcodeSuite::M1994() {
    if (parser.seen('A')) {
        if (parser.has_value() && parser.value_byte() == 2) {
            od3d::pa::calibrate_auto_level();
        } else if (parser.seenval('P')) {
            od3d::pa::calibrate_auto_step(parser.value_byte());
        } else {
            od3d::pa::calibrate_auto();
        }
        return;
    }
    od3d::pa::Request rq;
    // Le motif d'abord : ses valeurs par défaut, puis les paramètres donnés
    if (parser.seen('V') ? parser.value_bool() : !od3d::pa::large_pattern()) {
        od3d::pa::make_compact(rq);
    }
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
    read('W', rq.params.bead.width);
    read('Z', rq.params.bead.height);
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
    od3d::pa::calibrate(rq, parser.seen('J') && parser.value_bool());
}

/** @}*/
