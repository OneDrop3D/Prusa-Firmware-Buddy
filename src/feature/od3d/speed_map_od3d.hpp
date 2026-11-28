/// @file
/// OneDrop3D (F22) : carte des vitesses critiques (VFA) mesurée par l'accéléromètre de l'input shaper (M1993).
///
/// Pour chaque axe (X puis Y) et chaque vitesse de 20 à 80 mm/s par pas de 5, la tête fait un aller et un retour à
/// vitesse constante depuis un même bout (sweep_from : sans recentrage entre deux vitesses, ensemble centré sur le
/// plateau) ; l'énergie vibratoire (moyenne quadratique de l'accélération après un
/// passe-haut, speed_map_math_od3d.hpp) n'est mesurée que dans la partie à vitesse constante, repérée par la position
/// réelle des moteurs. Les vitesses dont l'énergie ressort nettement de celle de leurs voisines sont « à éviter ».
///
/// Les résultats restent en mémoire jusqu'au redémarrage ; l'interface les lit pendant la mesure (barres en direct).
#pragma once

#include "speed_map_math_od3d.hpp"

#include <array>
#include <cstdint>

namespace od3d::speed_map {

inline constexpr float speed_start = 20.f; ///< mm/s
inline constexpr float speed_step = 5.f;
/// 20 à 80 mm/s (V-07, étude F34) : la fréquence des pas complets des moteurs vaut 6,25 × v Hz (500 Hz à 80 mm/s ;
/// 6,35 × v avec les courroies 1,5GT de la Gen2, 101,587 pas/mm) et approche au-delà la moitié de l'échantillonnage de
/// l'accéléromètre (~1 344 Hz), qui ne la capte plus : une carte plus haute ne serait pas fiable. Rien n'en est calculé :
/// les vitesses sont mesurées. Le pas de 5 mm/s garde des voisines de chaque côté pour la base locale.
inline constexpr size_t speed_count = 13;
inline constexpr float speed_end = speed_start + (speed_count - 1) * speed_step;
static_assert(speed_end == 80.f);

inline constexpr uint8_t axis_count = 2; ///< X, Y

/// Seuil des vitesses critiques : énergie au-dessus de critical_factor × leur base locale (find_critical())
inline constexpr float critical_factor = 1.5f;
inline constexpr size_t max_critical = 5;

enum class Status : uint8_t {
    idle,
    running,
    done,
    stopped, ///< annulée depuis l'écran
    printing, ///< refusé : une impression est en cours
    hot_nozzle, ///< refusé : buse au-dessus de 50 °C (l'accéléromètre se fixe sur l'extrudeur)
    no_accelerometer,
    failed, ///< l'accéléromètre a cessé de répondre
    not_homed,
    no_room, ///< pas assez de course autour du centre
    no_dock, ///< INDX, refusé sans bouger : aucune tête prise ni dock calibré (accel::head_available)
};

struct Map {
    std::array<std::array<float, speed_count>, axis_count> rms {}; ///< m/s², NAN si la mesure est trop courte
    std::array<uint8_t, axis_count> points {}; ///< vitesses déjà mesurées par axe
};

/// Corps de M1993 : bloque jusqu'à la fin de la mesure ou jusqu'à request_stop()
void run();

/// Demande l'arrêt (appelable depuis l'interface) ; il prend effet à la fin du passage en cours (1 s au plus)
void request_stop();

bool is_running();
Status status();

/// Axe en cours de mesure (0 : X, 1 : Y)
uint8_t current_axis();

/// Avancement de 0 à 100
uint8_t progress_percent();

uint16_t run_count();
uint16_t revision();

const Map &last();

/// Vitesses critiques d'un axe de la carte (indices dans rms, du plus fort au plus faible) ; rend leur nombre
size_t critical(const Map &map, uint8_t axis, std::span<uint8_t> out);

inline constexpr float max_nozzle_temperature = 50.f;

} // namespace od3d::speed_map
