/// @file
/// OneDrop3D (N3) : réponse du ventilateur pièce (M2001), sans filament ni chauffe.
///
/// Prusa ne calcule le régime qu'une fois par seconde (CFanCtlTach) et partage le tachymètre avec le ventilateur de la
/// tête. Pendant la mesure seulement, Fans::tick() (1 kHz) garde le tachymètre sur le ventilateur pièce et ses fronts
/// sont comptés par créneaux de 50 ms. Le ventilateur est arrêté (il ralentit sans signal : son tachymètre est coupé
/// avec lui), puis commandé de 0 à 100 % comme M106 ; le temps de montée (commande → 90 % du régime final,
/// lookahead_math_od3d.hpp) règle l'avance du ventilateur de F26 (lookahead_od3d.hpp, « OD3D Fan Lead »). Sans mesure
/// réussie, l'avance reste ce qu'elle est. Ventilateur remis comme avant à la fin.
#pragma once

#include "lookahead_math_od3d.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace od3d::fan_response {

inline constexpr uint32_t spin_down_ms = 5000; ///< arrêt avant la mesure (un ventilateur de 5015 s'arrête en ~3 s)
inline constexpr uint32_t baseline_ms = 300; ///< comptage avant la commande
inline constexpr uint32_t high_ms = 2700; ///< à fond, régime final sur les 500 dernières ms
inline constexpr size_t max_bins = 64; ///< (baseline_ms + high_ms) / 50 ms = 60, et un peu de marge
inline constexpr float min_rpm = 1000.f; ///< à fond, en dessous : pas de signal (ventilateur ou tachymètre)

enum class Status : uint8_t {
    idle,
    running,
    done, ///< avance réglée
    stopped,
    printing, ///< refusé : impression en cours
    no_signal, ///< pas de régime lisible : avance inchangée
};

struct Result {
    lookahead::FanRise rise;
    uint8_t lead_before_ds = 0; ///< avance de F26 avant la mesure
    uint8_t lead_ds = 0; ///< avance appliquée (done)
    float command_s = -1; ///< commande 0 → 100 %, depuis le début du comptage ; négative : pas encore
};

/// Corps de M2001 : bloque jusqu'à la fin (~8 s) ou request_stop()
void run();
void request_stop();
bool is_running();
Status status();
uint8_t progress_percent();
uint16_t run_count();

/// Fronts par créneau de la dernière mesure (en cours ou finie, jusqu'au redémarrage) : les bins() premiers sont valides
const std::array<uint8_t, max_bins> &edges();
uint8_t bins();
const Result &result();

/// Fans::tick() (interruption, 1 kHz) : comptage en cours, le tachymètre reste sur le ventilateur pièce
bool sampling();

/// Fans::tick() pendant le comptage : niveau du tachymètre
void tick(bool level);

} // namespace od3d::fan_response
