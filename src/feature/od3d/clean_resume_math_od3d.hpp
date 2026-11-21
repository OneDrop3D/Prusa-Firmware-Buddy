/// @file
/// OneDrop3D (F30) : reprise sans cicatrice, partie sans matériel (testée en unitaire).
///
/// Avant de revenir sur la pièce (reprise après pause, après coupure) : purge à l'endroit où Prusa purge déjà
/// (position de purge du M600, devant le plateau sur la CORE One), rétraction, petit essuyage (aller-retour en X
/// qui casse le fil), puis retour de Prusa. La position de E est recalée pour que la désrétraction de Prusa pousse
/// exactement ce que l'on vient de rétracter dans une buse fraîchement purgée (« désrétraction calibrée »).
/// Après une pause, la dernière descente (5 mm au-dessus de la pièce) se fait lentement, la désrétraction pendant
/// la descente : la buse n'est jamais immobile sur la pièce pendant que la pression revient.
///
/// PA progressif sur les premiers millimètres : non. Le pressure advance de Prusa ne change qu'avec les files de
/// mouvements vides (M572_internal : planner.synchronize, set_axis_e_config : file vide exigée) ; une rampe de K
/// ferait plusieurs arrêts complets au point de reprise, donc de nouvelles marques.
#pragma once

#include <cstdint>
#include <optional>

namespace od3d::clean_resume {

struct Settings {
    bool enabled = false; ///< coupé par défaut : la reprise de Prusa reste telle quelle
    uint8_t purge_mm = 8; ///< filament poussé hors de la pièce, après la désrétraction de la pause
    uint8_t retract_tenths = 8; ///< rétraction après la purge, en dixièmes de mm (désrétractée au retour)
    uint8_t approach_mm_s = 2; ///< vitesse de la dernière descente sur la pièce (Prusa : 5 mm/s)

    static constexpr uint8_t purge_max = 30;
    static constexpr uint8_t retract_max = 20; ///< 2 mm
    static constexpr uint8_t approach_min = 1, approach_max = 5;

    Settings clamped() const;
};

/// Essuyage : aller-retour en X de cette longueur à la position de purge
inline constexpr float wipe_mm = 10.0f;

/// Désrétraction de la pause prise en compte au plus (au-delà, la position de E n'est pas crédible : rien n'est fait)
inline constexpr float max_unretract_mm = 10.0f;

/// Remontée minimale au-dessus du point de reprise avant d'aller purger après une coupure
inline constexpr float power_panic_clearance_mm = 5.0f;

struct Box {
    float x_min, x_max, y_min, y_max;
};

struct Plan {
    float purge_x, purge_y; ///< position de purge, dans les limites
    float wipe_x; ///< aller de l'essuyage (puis retour en purge_x)
    float extrude_mm; ///< désrétraction de la pause + purge
    float retract_mm; ///< rétraction après la purge
    float e_after; ///< position de E à poser à la fin (recalage)
};

/// Reprise après pause : resume_e = E de la pièce au moment de la pause, current_e = E maintenant (rétracté par la
/// pause). Après le plan, E vaut resume_e − retract : la désrétraction de Prusa pousse exactement retract_mm.
/// nullopt : option coupée ou position de E incohérente (on laisse la reprise de Prusa telle quelle).
std::optional<Plan> plan_pause(const Settings &settings, float purge_x, float purge_y, float resume_e, float current_e, const Box &limits);

/// Reprise après coupure : Prusa désrétracte toujours pause_retract_mm (STANDARD_RETRACT_LENGTH) au retour.
/// On désrétracte ce qu'il a rétracté à la coupure, on purge, puis on rétracte la même longueur ; E revient à
/// current_e (rien ne change pour la suite de Prusa).
std::optional<Plan> plan_power_panic(const Settings &settings, float purge_x, float purge_y, float pause_retract_mm, float current_e, const Box &limits);

/// Hauteur où monter avant d'aller purger après une coupure (jamais plus bas que maintenant, jamais au-delà de z_max)
float power_panic_lift_z(float current_z, float resume_z, float z_max);

/// Vitesse de la dernière descente : le réglage, jamais plus rapide que celle de Prusa
float approach_feedrate(const Settings &settings, float prusa_mm_s);

} // namespace od3d::clean_resume
