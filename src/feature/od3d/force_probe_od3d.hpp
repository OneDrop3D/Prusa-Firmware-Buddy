/// @file
/// OneDrop3D (F4, et plus tard F5, F6, F9) : outil de mesure commun par la cellule de charge.
///
/// Enregistre la force de la cellule de charge pendant une séquence (créneaux d'extrusion, lignes imprimées, balayage
/// de température…) dans une trace en mémoire (force_trace_od3d.hpp) : chaque échantillon de la cellule (320 Hz sur la
/// CORE One en haute précision) arrive par Loadcell::ProcessSample(), dans une interruption ; trois échantillons sont
/// moyennés (~107 points/s) avec les positions de X et du moteur E au même instant (positionLookback de Prusa : X
/// donne les instants des changements de débit, E montre le PA réellement appliqué). C'est le signal `loadcell_value`
/// des metrics de Prusa (charge tarée), lu à la source au lieu de passer par l'UDP. Rien n'est fait hors enregistrement.
///
/// L'enregistrement peut être exporté en CSV sur la clé USB pour régler les seuils d'après de vraies courbes.
/// Les metrics de Prusa restent disponibles en parallèle (M334 hôte, M331 loadcell_value, pos_x : comme PrusaPATuner).
#pragma once

#include "force_trace_od3d.hpp"

#include <cstdint>
#include <cstdio>

namespace od3d::force_probe {

inline constexpr size_t capacity = 1920; ///< points (11,5 Ko) : 18 s à ~107 points/s (une séquence de F4 : 15,6 s)
inline constexpr uint8_t decimation = 3; ///< échantillons de la cellule moyennés par point

/// Appelée par Loadcell::ProcessSample (interruption) avec la charge tarée [g] (négative quand la buse appuie, comme au
/// palpage) et les positions X et E [mm] à l'instant de l'échantillon ; ne fait rien hors enregistrement. La trace
/// garde la force **vers le haut positive** (−loadcell_value : plateau ou plastique écrasé qui repousse la buse).
void on_loadcell_sample(uint32_t time_us, float force_g, float x_mm, float e_mm);

/// Efface la trace et commence l'enregistrement (fil de Marlin) ; le premier échantillon sert de zéro (force et X)
void start();

/// Arrête l'enregistrement ; la trace reste lisible jusqu'au prochain start()
void stop();

bool recording();

/// Points enregistrés (lire après stop())
force::Trace trace();

/// Pas de temps moyen entre deux points [s], d'après les horodatages ; 0 avec moins de 2 points
float dt();

/// Force du premier échantillon (zéro de la trace), vers le haut positive [g] : −loadcell_value à cet instant
float base_force_g();

/// La mémoire a débordé (la fin de la séquence manque)
bool overflowed();

/// Fenêtre de mesure (F5, F6) : moyenne et écart type de la force sur les points enregistrés entre window_begin() et
/// window_end(), calculés en flottant dans l'interruption (sans la limite de ±6,5 kg des points de la trace, et même
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

/// Écrit la trace en CSV, une ligne « D,<étiquette>,t_s,x_mm,e_mm,force_g » par point ; false si l'écriture échoue
bool write_csv(FILE *file, float label);

/// Une ligne de CSV, comme fprintf mais avec les décimales. Sur l'imprimante, fprintf vient de newlib-nano, qui les
/// laisse vides : seuls printf, snprintf et vsnprintf passent par lib/printf (lib/AddPrintf.cmake). Exemple : le relevé
/// OD3D_PA3.CSV du 22/11, 4 600 lignes sans un nombre, alors que les tests sur ordinateur passaient. 255 caractères
/// au plus ; rend le nombre écrit, ou -1. La CI refuse fprintf dans notre code.
[[gnu::format(printf, 2, 3)]] int csv_printf(FILE *file, const char *format, ...);

} // namespace od3d::force_probe
