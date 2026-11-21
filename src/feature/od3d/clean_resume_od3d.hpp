/// @file
/// OneDrop3D (F30) : reprise sans cicatrice (option, coupée par défaut ; voir clean_resume_math_od3d.hpp).
///
/// Déjà chez Prusa : à la pause, rétraction de STANDARD_RETRACT_LENGTH, levée (au moins 50 mm au-dessus de la pièce)
/// et parcage à l'arrière ; à la reprise, réchauffe, retour en Z puis en XY 5 mm au-dessus de la pièce, descente à
/// 5 mm/s et désrétraction immobile au point de reprise. Après une coupure : réchauffe, origine X/Y, retour au point
/// et désrétraction. La purge hors pièce n'existe chez Prusa qu'avec le nettoyeur de buse de l'INDX.
///
/// Ajouté, quand l'option est active : avant ce retour, purge à la position de purge de Prusa (celle du M600),
/// rétraction, essuyage, recalage de E ; après une pause, descente lente avec la désrétraction pendant la descente.
/// Option coupée : aucun mouvement de plus, la reprise de Prusa est inchangée.
#pragma once

#include "clean_resume_math_od3d.hpp"

#include <optional>

namespace od3d::clean_resume {

Settings settings();
void set_settings(const Settings &s);

// ---- Thread de Marlin ----

/// Reprise après pause, juste avant le retour en XY de Prusa (unpark_head_XY). resume_e : E de la pièce à la pause
void before_unpark(float resume_e);

/// unpark_head_ZE : si before_unpark a purgé, vitesse de la descente avec désrétraction (consommé à la lecture)
std::optional<float> take_approach(float prusa_z_mm_s);

/// Reprise après coupure (power_panic, ResumeState::Unpark), avant le retour au point : monte d'au moins 5 mm
/// au-dessus du point de reprise, purge, essuie ; Prusa désrétracte ensuite pause_retract_mm comme d'habitude
void before_power_panic_unpark(float resume_z, float pause_retract_mm);

/// Vitesse Z du retour au point après une coupure : la nôtre si l'option est active, sinon celle de Prusa
float power_panic_z_feedrate(float prusa_mm_s);

} // namespace od3d::clean_resume
