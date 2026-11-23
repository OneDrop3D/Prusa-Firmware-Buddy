/// @file
/// OneDrop3D : deux mesures de la série « Accéléromètre » de l'assistant (à froid, accéléromètre fixé sur la buse),
/// calculs dans vibration_math_od3d.hpp. Comme les autres tests à l'accéléromètre (accelerometer_od3d.hpp) : refusées
/// sans rien bouger si l'accéléromètre ne répond pas, si la buse dépasse 50 °C ou pendant une impression ; rien ne
/// chauffe. Les résultats restent en mémoire jusqu'au redémarrage ; mesures brutes ajoutées sur la clé (CSV).
///
/// - M2004, accélération max utile : à la position de l'assistant input shaper de Prusa (M1959 : centre du plateau,
///   mi-hauteur), X puis Y font quatre déplacements courts (aller, retour, aller, retour) à chaque palier de 2 000 à
///   10 000 mm/s², input shaper en place, mode furtif coupé ; le résidu est mesuré dans les 250 ms qui suivent l'arrêt.
///   Rien n'est appliqué : l'assistant propose ensuite d'en faire le plafond de M201 (accel_cap_od3d.hpp), gardé et
///   appliqué au démarrage comme à chaque M201 (celui du départ de PrusaSlicer compris), si l'utilisateur l'accepte.
///   /usb/OD3D_ACCEL.CSV.
/// - M2001 V, vibrations des ventilateurs (après le temps de montée de M2001) : chaque ventilateur seul (les autres
///   arrêtés par les commandes du test des ventilateurs de Prusa, M1978), à 50, 75 et 100 %, moteurs à l'arrêt ;
///   régime lu au tachymètre, amplitude à la fréquence de rotation par la transformée en un point de M958
///   (FourierSeries3d). La première mesure captée de chaque ventilateur devient sa référence, gardée sur 7 bits dans
///   « OD3D Calib Values » (case du ventilateur, [2] et [3]). /usb/OD3D_FANS.CSV.
#pragma once

#include "vibration_math_od3d.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace od3d::vibration {

enum class Status : uint8_t {
    idle,
    running,
    done,
    stopped, ///< arrêtée depuis l'écran (ou « Passer » de l'assistant)
    printing, ///< refusé : impression en cours
    hot_nozzle, ///< refusé : buse au-dessus de 50 °C (l'accéléromètre est sur la buse)
    no_accelerometer, ///< refusé sans bouger : l'accéléromètre ne répond pas (ACC-1)
    failed, ///< l'accéléromètre a cessé de répondre
    not_homed, ///< M2004 : pas d'origine (jamais prise accéléromètre fixé)
};

inline constexpr float max_nozzle_temperature = 50.f;

} // namespace od3d::vibration

namespace od3d::accel_limit {

using vibration::Status;

inline constexpr uint8_t axis_count = 2; ///< X, Y

struct Measurement {
    std::array<std::array<float, vibration::accel_levels>, axis_count> residual {}; ///< variance moyenne (m²/s⁴), NAN : pas mesuré
    float noise = NAN; ///< variance au repos
    std::array<uint16_t, axis_count> top {}; ///< plus haut palier essayé (limite matérielle de Prusa), 0 : pas d'axe
    std::array<vibration::Limit, axis_count> limits {};
};

/// Corps de M2004 : bloque jusqu'à la fin (~1 min) ou request_stop()
void run();
void request_stop();
bool is_running();
Status status();
uint8_t progress_percent();
const Measurement &last();

/// Accélération max utile d'un axe (mm/s²) à proposer pour M201 ; nullopt : non concluant (résidu dans le bruit)
std::optional<uint16_t> proposal(uint8_t axis);

} // namespace od3d::accel_limit

namespace od3d::fan_vibration {

using vibration::Status;

/// Les références gardées, dans cet ordre (deux codes par case de « OD3D Calib Values »)
enum class Fan : uint8_t {
    print, ///< ventilateur pièce
    hotend, ///< ventilateur de la tête (barrière thermique)
    chamber_1, ///< chambre (xBuddy Extension) : premier ventilateur, ou celui de filtration
    chamber_2,
};
inline constexpr size_t fan_count = 4;

struct FanResult {
    bool present = false;
    std::array<uint16_t, vibration::fan_pwm.size()> rpm {}; ///< par régime, au tachymètre
    std::array<vibration::Tone, vibration::fan_pwm.size()> tones {};
    vibration::FanLevel level {};
    std::optional<float> reference; ///< m/s², avant cette mesure
    vibration::Verdict verdict = vibration::Verdict::not_detected;
};

struct Measurement {
    std::array<FanResult, fan_count> fans {};
};

/// Corps de M2001 V après le temps de montée : bloque jusqu'à la fin (~1 min) ou request_stop()
void run();
void request_stop();
bool is_running();
Status status();
uint8_t progress_percent();
const Measurement &last();

/// Les références des ventilateurs telles que gardées (deux int16 de « OD3D Calib Values », case du ventilateur)
std::array<int16_t, 2> stored_references();

} // namespace od3d::fan_vibration
