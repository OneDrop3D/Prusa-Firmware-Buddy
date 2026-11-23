/// @file
/// OneDrop3D : accélération max utile (M2004) et vibrations des ventilateurs (M2001 V), calculs purs — sans Marlin ni
/// affichage, testés dans tests/unit/feature/od3d/vibration_math_test.cpp sur des signaux synthétiques.
///
/// Accélération max utile : à chaque palier a, un déplacement court en triangle (rampes de ramp_s, donc vitesse
/// a × ramp_s et longueur a × ramp_s²) : la forme de l'excitation est la même à tous les paliers, seul a la met à
/// l'échelle. Dans un système linéaire, l'input shaper en place laisse alors un résidu (vibrations après l'arrêt, dans
/// une fenêtre fixe) proportionnel à a : le résidu par unité d'accélération reste constant. Il remonte quand la machine
/// ne suit plus (moteurs à la limite du couple, courroies, cadre) : la limite retenue est le plus haut palier où il reste
/// sous rise_ratio fois celui du palier de référence (le plus bas qui sort du bruit).
///
/// Ventilateurs : amplitude de l'accélération à la fréquence de rotation (tr/min ÷ 60, lue au tachymètre), comparée au
/// bruit voisin pour savoir si l'accéléromètre la capte depuis la buse, puis à la référence du ventilateur (sa première
/// mesure captée) : une amplitude nettement plus haute signale un balourd ou une usure.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace od3d::vibration {

// ---- Accélération max utile (M2004) ----

inline constexpr uint16_t accel_first = 2000; ///< mm/s², premier palier
inline constexpr uint16_t accel_step = 1000;
inline constexpr size_t accel_levels = 9; ///< jusqu'à 10 000 (limite matérielle de Prusa en X/Y sur la CORE One)
inline constexpr float ramp_s = 0.03f; ///< durée de chaque rampe : 300 mm/s et 9 mm au dernier palier
inline constexpr float rise_ratio = 1.5f; ///< résidu par unité d'accélération : +50 % sur la référence = remontée
/// Référence : premier palier dont le résidu vaut au moins deux fois le bruit au repos (vibrations au moins égales au
/// bruit, qui est retiré : en dessous, le résidu par unité d'accélération ne serait que du bruit)
inline constexpr float reference_ratio = 2.f;

constexpr uint16_t level_accel(size_t i) {
    return uint16_t(accel_first + i * accel_step);
}
static_assert(level_accel(accel_levels - 1) == 10000);

/// Variance d'une fenêtre d'échantillons (m/s²) : moyenne de chaque axe retirée (pesanteur), somme des trois axes.
/// Pas de passe-haut : son régime transitoire après le freinage (1 g) noierait le résidu.
class WindowMeter {
public:
    void add(float x, float y, float z);
    uint32_t count() const { return n; }
    /// m²/s⁴ ; 0 sans échantillon
    float variance() const;

private:
    double sum[3] = { 0, 0, 0 };
    double sum_sq[3] = { 0, 0, 0 };
    uint32_t n = 0;
};

struct Limit {
    bool found = false; ///< un palier sort du bruit : la référence existe
    bool rose = false; ///< le résidu a remonté avant le dernier palier mesuré (sinon : pas de remontée jusqu'en haut)
    uint8_t reference = 0; ///< palier de référence
    uint8_t last_ok = 0; ///< plus haut palier sous le seuil : la limite (level_accel(last_ok))
};

/// residual[i] : résidu moyen (variance, m²/s⁴) au palier i (accel[i] mm/s², croissantes), NAN : pas mesuré (la suite
/// non plus) ; noise : variance au repos. Au premier palier qui dépasse le seuil, on s'arrête (le plus prudent).
Limit find_limit(std::span<const float> residual, std::span<const uint16_t> accel, float noise);

// ---- Ventilateurs (M2001 V) ----

inline constexpr std::array<uint8_t, 3> fan_pwm { 128, 191, 255 }; ///< 50, 75, 100 %
inline constexpr size_t peak_bins = 13; ///< autour de la fréquence de rotation, de −3 à +3 % (écart du tachymètre)
inline constexpr float peak_step = 0.005f; ///< demi-largeur du pic d'une fenêtre de 1 s à 100 Hz : 10 % perdus au plus
inline constexpr std::array<float, 6> noise_factors { 0.75f, 0.8f, 0.85f, 1.15f, 1.2f, 1.25f }; ///< bruit voisin
inline constexpr size_t fan_bins = peak_bins + noise_factors.size();
inline constexpr float fan_snr_min = 3.f; ///< capté : pic à 3 fois le bruit voisin au moins
inline constexpr float worse_ratio = 2.f; ///< balourd ou usure : amplitude deux fois celle de la référence

/// Fréquences des bins d'un régime : peak_bins autour de f0 (Hz), puis le bruit voisin
std::array<float, fan_bins> bin_frequencies(float f0);

struct Tone {
    float amplitude = 0.f; ///< m/s², le plus haut des bins du pic
    float noise = 0.f; ///< m/s², médiane des bins voisins
    bool detected = false;
};

/// magnitudes : celles des bins de bin_frequencies() (m/s²)
Tone tone(std::span<const float> magnitudes);

/// Un ventilateur sur ses régimes : moyenne quadratique des amplitudes (m/s²) ; capté si un régime l'est
struct FanLevel {
    float value = 0.f;
    bool detected = false;
};
FanLevel fan_level(std::span<const Tone> tones);

enum class Verdict : uint8_t {
    not_detected, ///< l'accéléromètre ne le capte pas depuis la buse : ni référence ni comparaison
    reference, ///< première mesure captée : elle devient la référence
    ok,
    worse, ///< nettement plus haut que la référence : balourd ou usure
};

/// reference : amplitude de référence (m/s²), s'il y en a une
Verdict compare(FanLevel now, std::optional<float> reference);

/// Référence gardée sur 7 bits dans « OD3D Calib Values » (la config_store est pleine) : 8 pas par octave depuis
/// code_min m/s² (9 % par pas, de 0,001 à ~60 m/s²) ; 0 : pas de référence
inline constexpr float code_min = 0.001f;
uint8_t encode(float amplitude);
std::optional<float> decode(uint8_t code);

/// Deux codes dans un int16 de « OD3D Calib Values » : octet haut ≤ 0x7F, jamais calib_registry::no_value (0x8000),
/// qui se lit comme deux codes absents (valeur d'une config_store neuve)
int16_t pack(uint8_t low, uint8_t high);
uint8_t unpack(int16_t packed, bool high);

/// Les codes des quatre ventilateurs depuis [2] et [3] de la case du ventilateur : M2001 V écrit toujours [3] (jamais
/// no_value). [3] à no_value : version 1 du test, dont [2] gardait le régime : aucune référence
std::array<uint8_t, 4> unpack_references(int16_t v2, int16_t v3);

} // namespace od3d::vibration
