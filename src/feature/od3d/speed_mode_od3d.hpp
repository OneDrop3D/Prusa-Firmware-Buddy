/// @file
/// OneDrop3D (retour du 26/11) : modes de vitesse, choisis pendant l'impression dans Ajuster › Vitesse ou depuis la
/// vitesse de l'écran d'impression (screen_printing_od3d.cpp), avec le réglage à la main (MI_SPEED, 10 à 300 %).
/// Un mode pose la vitesse (comme M220). Silencieux bride aussi le ventilateur de pièce (Temperature::manage_fans) et
/// le plafond de la régulation des ventilateurs de l'enceinte (XBuddyExtension::step) ; le ventilateur du radiateur de
/// la buse et les secours de surchauffe de l'enceinte ne changent pas. Le mode vaut pour l'impression en cours : le
/// bridage s'arrête à sa fin (finalize_print), et Prusa remet la vitesse à 100 % au départ de la suivante.
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <utility>

namespace od3d::speed_mode {

enum class Mode : uint8_t {
    quiet, ///< Silencieux
    normal,
    fast, ///< Rapide
    faster, ///< Très rapide
    fastest, ///< Ultra rapide
};

/// Vitesse de chaque mode [%], dans l'ordre de Mode (propositions de l'utilisateur ; Silencieux : 80)
inline constexpr std::array<uint16_t, 5> percents { 80, 100, 130, 160, 250 };

constexpr uint16_t percent(Mode m) {
    return percents[std::to_underlying(m)];
}

inline constexpr uint8_t quiet_print_fan_pwm = 153; ///< ventilateur de pièce en Silencieux : 60 % au plus
inline constexpr uint8_t quiet_chamber_fans_pwm = 102; ///< régulation de l'enceinte en Silencieux : 40 % au plus

/// Silencieux choisi : ventilateurs bridés jusqu'à la fin de l'impression
inline std::atomic<bool> quiet = false;

/// PWM du ventilateur de pièce, bridée en Silencieux
constexpr uint8_t print_fan_pwm(uint8_t pwm, bool is_quiet) {
    return is_quiet ? std::min(pwm, quiet_print_fan_pwm) : pwm;
}

/// Plafond de la régulation automatique des ventilateurs de l'enceinte (celui des réglages), bridé en Silencieux
constexpr uint8_t chamber_fans_max_pwm(uint8_t max_pwm, bool is_quiet) {
    return is_quiet ? std::min(max_pwm, quiet_chamber_fans_pwm) : max_pwm;
}

} // namespace od3d::speed_mode
