/// @file
/// OneDrop3D (F15) : M852, correction d'équerrage XY (Prusa a retiré celui de Marlin), et son application au démarrage.
#include "PrusaGcodeSuite.hpp"

#include <config_store/store_instance.hpp>
#include <cstdio>
#include <feature/od3d/skew_od3d.hpp>
#include <gcode/gcode.h>
#include <gcode/parser.h>
#include <module/motion.h>
#include <module/planner.h>

#if ENABLED(OD3D_SKEW_CORRECTION_XY)

namespace od3d::skew {

void set_factor(float factor) {
    if (!is_valid_factor(factor) || Planner::skew_xy == factor) {
        return;
    }
    if (planner.leveling_active) {
        // Comme set_bed_leveling_enabled : la tête ne bouge pas, c'est la position native qui change
        planner.synchronize();
        const auto machine = current_machine_position();
        Planner::skew_xy = factor;
        set_current_position(to_native_pos(machine));
        sync_plan_position();
    } else {
        Planner::skew_xy = factor; // sans MBL, pas de correction : rien à recalculer
    }
}

void load() {
    const float factor = config_store().od3d_skew_xy.get();
    set_factor(is_valid_factor(factor) ? factor : 0.0f);
}

} // namespace od3d::skew

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M852: OneDrop3D — correction d'équerrage XY
 *
 *#### Usage
 *
 *    M852 [I<factor>] [S<factor>]
 *
 *#### Parameters
 *
 * - `I` - Facteur XY (tangente de l'écart, |k| <= 0.01), enregistré et appliqué tout de suite (S : même chose,
 *         comme dans Marlin). Pas de J ni de K : XZ et YZ ne sont pas corrigés.
 *
 * Sans paramètre : affiche le facteur. La correction n'agit qu'avec le MBL actif (impressions).
 */
void PrusaGcodeSuite::M852() {
    if (parser.seenval('I') || parser.seenval('S')) {
        const float factor = parser.value_float();
        if (!od3d::skew::is_valid_factor(factor)) {
            SERIAL_ECHO_MSG("?Skew factor (I) out of range (-0.01 to 0.01)");
            return;
        }
        od3d::skew::set_factor(factor);
        config_store().od3d_skew_xy.set(factor);
    }
    char buf[48];
    snprintf(buf, sizeof(buf), "M852 I%.6f (%.3f deg)", static_cast<double>(Planner::skew_xy), static_cast<double>(od3d::skew::factor_to_degrees(Planner::skew_xy)));
    SERIAL_ECHO_START();
    SERIAL_ECHOLN(buf);
}

/** @}*/

#endif
