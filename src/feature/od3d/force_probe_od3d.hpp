/// @file
/// OneDrop3D (F4, et plus tard F5, F6, F9) : outil de mesure commun par la cellule de charge.
///
/// Enregistre la force de la cellule de charge pendant une séquence (lignes imprimées, balayage de température…) dans une
/// trace en mémoire (force_trace_od3d.hpp) : chaque échantillon de la cellule (320 Hz sur la CORE One en haute
/// précision) arrive par Loadcell::ProcessSample(), dans une interruption ; deux échantillons sont moyennés (160 Hz)
/// avec la position X de la tête au même instant (positionLookback de Prusa). Rien n'est fait hors enregistrement.
///
/// L'enregistrement peut être exporté en CSV sur la clé USB pour régler les seuils d'après de vraies courbes.
/// Les metrics de Prusa restent disponibles en parallèle (M334 hôte, M331 loadcell_value, pos_x : comme PrusaPATuner).
#pragma once

#include "force_trace_od3d.hpp"

#include <cstdint>
#include <cstdio>

namespace od3d::force_probe {

inline constexpr size_t capacity = 1536; ///< points (6 Ko) : 9,6 s à 160 Hz
inline constexpr uint8_t decimation = 2; ///< échantillons de la cellule moyennés par point

/// Appelée par Loadcell::ProcessSample (interruption) avec la charge tarée [g] (négative quand la buse appuie, comme au
/// palpage) et la position X [mm] à l'instant de l'échantillon ; ne fait rien hors enregistrement. La trace garde la
/// force **vers le haut positive** (plateau ou plastique écrasé qui repousse la buse).
void on_loadcell_sample(uint32_t time_us, float force_g, float x_mm);

/// Efface la trace et commence l'enregistrement (fil de Marlin) ; le premier échantillon sert de zéro (force et X)
void start();

/// Arrête l'enregistrement ; la trace reste lisible jusqu'au prochain start()
void stop();

bool recording();

/// Points enregistrés (lire après stop())
force::Trace trace();

/// Pas de temps moyen entre deux points [s], d'après les horodatages ; 0 avec moins de 2 points
float dt();

/// La mémoire a débordé (la fin de la séquence manque)
bool overflowed();

/// Fenêtre de mesure (F5, F6) : moyenne et écart type de la force sur les points enregistrés entre window_begin() et
/// window_end(), calculés en flottant dans l'interruption (sans la limite de ±655 g des points de la trace, et même
/// après le débordement de la mémoire). Force relative au premier échantillon, comme la trace.
struct WindowStats {
    uint32_t count = 0; ///< points dans la fenêtre
    float mean_g = 0; ///< force moyenne [g]
    float sd_g = 0; ///< écart type [g]
};

/// Les points suivants entrent dans la fenêtre (statistiques remises à zéro) ; à appeler pendant l'enregistrement
void window_begin();

/// Plus aucun point n'entre dans la fenêtre
void window_end();

/// Statistiques de la fenêtre (lire après window_end() ou stop())
WindowStats window();

/// Écrit la trace en CSV, une ligne « D,<étiquette>,t_s,x_mm,force_g » par point ; false si l'écriture échoue
bool write_csv(FILE *file, float label);

} // namespace od3d::force_probe
