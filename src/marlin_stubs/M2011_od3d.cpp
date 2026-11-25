/// @file
/// OneDrop3D (F30) : M2011, type du tracé qui suit, rendu par la lecture du fichier (feature/od3d/feature_marks_od3d.hpp).
#include "PrusaGcodeSuite.hpp"

#include <feature/od3d/clean_resume_od3d.hpp>
#include <gcode/parser.h>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M2011: OneDrop3D — type de tracé
 *
 * Internal GCode : jamais écrit dans un fichier. Reprise sans cicatrice active, la lecture du fichier remplace chaque
 * « ;TYPE:… » du trancheur par M2011, à sa place ; la reprise après pause amorce ensuite dans le remplissage.
 *
 *#### Usage
 *
 *    M2011 S<n>
 *
 *#### Parameters
 *
 * - `S` - 1 : remplissage intérieur (PrusaSlicer « Internal infill ») ; 0 : tout autre tracé
 */
void PrusaGcodeSuite::M2011() {
    od3d::clean_resume::set_feature(parser.byteval('S', 0));
}

/** @}*/
