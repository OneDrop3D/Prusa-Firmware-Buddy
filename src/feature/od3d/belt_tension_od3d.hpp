/// @file
/// OneDrop3D (F14) : tension des courroies mesurée par l'accéléromètre de l'input shaper (M1992).
///
/// Chaque courroie est excitée par son seul moteur (CoreXY : moteur A = pilote X, moteur B = pilote Y, mouvement en
/// diagonale), en balayant la fréquence de 60 à 150 Hz : un point sur deux (2 Hz) sur toute la plage, puis au pas de
/// 1 Hz à ±fine_half_hz du pic trouvé (retour du 23/11 : de 70 à 120 Hz, une courroie trop tendue à 136 Hz était lue
/// au bord, « trop lâche ») ; à chaque fréquence, la réponse est mesurée par vibrate_measure::measure_repeat() (le M958
/// de Prusa, celui de l'assistant input shaper M1959). Le pic du spectre (belt_peak(), sur toute la plage, refusé au
/// bord ou trop peu marqué), interpolé, donne la fréquence propre de la courroie ; le verdict et le conseil reprennent
/// le réglage manuel des courroies de Prusa (M961). Excitation : celle de M961 (calc_accel, bornée à sa plage de 60 à
/// 130 Hz), à la position de M961.
///
/// Les résultats restent en mémoire jusqu'au redémarrage ; l'interface les lit pendant la mesure (courbe en direct).
#pragma once

#include "belt_tension_math_od3d.hpp"

#include <array>
#include <cstdint>

namespace od3d::belts {

inline constexpr float sweep_start_hz = 60.f;
inline constexpr float sweep_end_hz = 150.f;
inline constexpr float sweep_step_hz = 1.f;
inline constexpr size_t sweep_points = 91;
inline constexpr size_t coarse_stride = 2; ///< 1er passage : un point sur deux (les autres, interpolés en attendant)
inline constexpr float fine_half_hz = 5.f; ///< 2e passage : les points sautés à ±5 Hz du pic du 1er
static_assert((sweep_points - 1) % coarse_stride == 0, "le 1er passage finit sur le dernier point");
static_assert(sweep_start_hz + (sweep_points - 1) * sweep_step_hz == sweep_end_hz);

/// Courroie, par son moteur
enum class Belt : uint8_t {
    a = 0, ///< moteur A (pilote X)
    b = 1, ///< moteur B (pilote Y)
};
inline constexpr uint8_t belt_count = 2;

enum class Status : uint8_t {
    idle, ///< rien lancé depuis le démarrage
    running,
    done, ///< mesure finie (les pics peuvent ne pas avoir été trouvés : voir Measurement::peaks)
    stopped, ///< annulée depuis l'écran
    printing, ///< refusé : une impression est en cours
    hot_nozzle, ///< refusé : buse au-dessus de 50 °C (l'accéléromètre se fixe sur l'extrudeur)
    no_accelerometer, ///< l'accéléromètre ne répond pas (texte de Prusa)
    failed, ///< la mesure a échoué en cours de route (accéléromètre)
    not_homed, ///< pas d'origine : refusé (jamais prise accéléromètre fixé)
};

/// Étape en cours, pour l'interface
enum class Stage : uint8_t {
    preparing, ///< déplacement à la position de mesure
    calibrating, ///< étalonnage de la fréquence d'échantillonnage de l'accéléromètre (~15 s)
    belt_a,
    belt_b,
};

struct Measurement {
    std::array<std::array<float, sweep_points>, belt_count> spectra {}; ///< réponse (gain²) par fréquence
    std::array<uint8_t, belt_count> points {}; ///< points déjà mesurés par courroie
    std::array<Peak, belt_count> peaks {};
    uint8_t harmonic = 1; ///< harmonique mesurée (1 : la fréquence d'excitation)
};

/// Corps de M1992 : bloque jusqu'à la fin de la mesure ou jusqu'à request_stop(). harmonic : 1 ou 2.
/// live (M1992 L, retour du 23/11) : réglage en direct, sans balayage complet ni étalonnage de l'accéléromètre (faits
/// par la mesure d'avant) : les deux courroies remesurées en boucle sur live_points points autour de leur dernier pic
/// (pics et spectre mis à jour à chaque tour), jusqu'à request_stop() (« Terminé » : les dernières valeurs, Status::done)
/// ou live_ok_rounds tours de suite dans la plage de Prusa
void run(uint8_t harmonic = 1, bool live = false);

/// Le réglage en direct tourne, et ses tours de suite dans la plage (0 à live_ok_rounds)
bool live();
uint8_t live_in_range();

/// Demande l'arrêt (appelable depuis l'interface) ; il prend effet en moins d'une seconde
void request_stop();

bool is_running();
Status status();
Stage stage();

/// Avancement de la mesure en cours, de 0 à 100
uint8_t progress_percent();

/// Nombre de M1992 traités (mesurés ou refusés) depuis le démarrage : l'interface sait quand sa demande a répondu
uint16_t run_count();

/// Change à chaque point mesuré et à chaque nouvelle mesure : l'interface redessine la courbe
uint16_t revision();

/// La dernière mesure (ou celle en cours)
const Measurement &last();

/// Les cibles de Prusa pour cette imprimante (manual_belt_tuning_config.hpp)
Targets targets();

/// Température de la buse au-delà de laquelle la mesure est refusée (comme l'assistant input shaper de Prusa)
inline constexpr float max_nozzle_temperature = 50.f;

} // namespace od3d::belts
