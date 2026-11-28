/// @file
/// OneDrop3D (F33, vecteur n° 03 du document v3) : amortissement réel ζ et facteur Q des axes X et Y, mesurés par
/// l'accéléromètre (M1997).
///
/// Chaque axe est excité comme dans l'assistant input shaper de Prusa (M1959 : 2,5 m/s², 50 périodes par fréquence,
/// mode « Klipper », moteurs de l'axe logique) par vibrate_measure::measure_repeat() (M958), de 30 à 90 Hz par pas de
/// 0,5 Hz, au centre du plateau. Le carré du gain de l'axe donne la courbe de résonance ; ζ et Q en sont tirés par la
/// bande à −3 dB (damping_math_od3d.hpp). L'écran propose ensuite d'écrire ζ dans la configuration de l'input shaper de
/// Prusa ; rien n'est changé sans l'accord de l'utilisateur.
///
/// Les résultats restent en mémoire jusqu'au redémarrage ; l'interface les lit pendant la mesure (courbe en direct).
#pragma once

#include "damping_math_od3d.hpp"

#include <array>
#include <cstdint>

namespace od3d::damping {

inline constexpr uint8_t axis_count = 2; ///< X, Y

enum class Status : uint8_t {
    idle,
    running,
    done,
    stopped, ///< annulée depuis l'écran
    printing, ///< refusé : une impression est en cours
    hot_nozzle, ///< refusé : buse au-dessus de 50 °C (l'accéléromètre se fixe sur la buse)
    no_accelerometer, ///< refusé sans bouger : l'accéléromètre ne répond pas (ACC-1)
    failed, ///< l'accéléromètre a cessé de répondre pendant la mesure
    not_homed,
    no_dock, ///< INDX, refusé sans bouger : aucune tête prise ni dock calibré (accel::head_available)
};

struct Measurement {
    std::array<std::array<float, sweep_points>, axis_count> psd {}; ///< gain² de l'axe par fréquence
    std::array<uint8_t, axis_count> points {}; ///< fréquences déjà mesurées par axe
    std::array<Resonance, axis_count> results {};
};

/// Corps de M1997 : bloque jusqu'à la fin de la mesure ou jusqu'à request_stop()
void run();

/// Demande l'arrêt (appelable depuis l'interface) ; il prend effet en moins d'une seconde
void request_stop();

bool is_running();
Status status();

/// Axe en cours (0 : X, 1 : Y)
uint8_t current_axis();

/// Étalonnage de la fréquence d'échantillonnage de l'accéléromètre en cours (~15 s, au début)
bool calibrating();

/// Avancement de 0 à 100 (pendant l'étalonnage : celui de l'étalonnage)
uint8_t progress_percent();

uint16_t run_count();
uint16_t revision();

const Measurement &last();

inline constexpr float max_nozzle_temperature = 50.f;

} // namespace od3d::damping
