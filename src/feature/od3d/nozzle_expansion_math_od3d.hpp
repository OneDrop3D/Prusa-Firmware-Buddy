/// @file
/// OneDrop3D (M2003) : pente réelle de la dilatation de la buse, tirée des palpages d'un même point du plateau à
/// plusieurs températures de la buse. Pur : ni Marlin, ni config_store (testé dans tests/unit/feature/od3d).
///
/// Buse plus longue de e : elle touche le plateau e plus tôt, le Z natif du contact monte de e. La pente de z(T) est donc
/// celle de la dilatation, tout ce qui pend sous la cellule de charge compris (buse, bloc de chauffe, radiateur) : c'est
/// ce que la compensation de Prusa doit corriger.
///
/// Ce qui pourrait fausser la mesure, et pourquoi c'est écarté :
/// - le plateau garde une seule consigne du début à la fin, et c'est toujours le même point : sa hauteur et sa
///   planéité sortent de la pente ;
/// - ce qui chauffe lentement pendant la mesure (plateau et cadre qui finissent de prendre leur température, cellule de
///   charge et radiateur qui tiédissent avec la buse) dérive avec le temps, pas avec la température. Aller-retour
///   170 → 215 → 260 → 215 → 170 à paliers réguliers : les écarts des températures à leur moyenne et des instants à leur
///   moyenne sont orthogonaux, une dérive régulière pèse autant sur chaque température et ne change pas la pente ;
/// - ce qui n'est pas régulier (hystérésis, dérive qui ralentit) sépare l'aller du retour : écart à la droite, mesure
///   refusée au-delà de max_residual_mm, et l'écart retour − aller est montré.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

namespace od3d::nozzle_expansion {

/// Paliers de la buse [°C], dans l'ordre
inline constexpr std::array<int16_t, 5> temperatures { 170, 215, 260, 215, 170 };
inline constexpr size_t stages = temperatures.size();
inline constexpr uint8_t taps = 3; ///< palpages par palier, au même point

/// Le palier i est à une température jamais atteinte avant dans la mesure (170, 215, 260 °C) : la buse y suinte en
/// chauffant, elle est nettoyée avant d'être palpée (retour du 23/11)
constexpr bool new_high(size_t i) {
    for (size_t k = 0; k < i && i < stages; ++k) {
        if (temperatures[k] >= temperatures[i]) {
            return false;
        }
    }
    return i < stages;
}

inline constexpr float max_spread_mm = 0.020f; ///< écart max − min des palpages d'un palier
inline constexpr float max_residual_mm = 0.010f; ///< écart d'un palier à la droite
inline constexpr float min_slope_mm_per_c = 0.00005f; ///< 0,05 µm/°C ; Prusa : 0,41
inline constexpr float max_slope_mm_per_c = 0.002f; ///< 2 µm/°C

/// Un palier : z natif du contact, moyenne des palpages (sans correction), et leur écart max − min
struct Stage {
    float z_mm = NAN;
    float spread_mm = 0.f;
};

/// Moyenne et écart max − min de n palpages
Stage stage_of(std::span<const float> z_mm);

struct Line {
    float slope = NAN;
    float intercept = NAN;
    bool valid = false; ///< au moins deux x différents
};

/// Droite des moindres carrés y = slope · x + intercept
Line fit(std::span<const float> x, std::span<const float> y);

enum class Verdict : uint8_t {
    ok,
    spread, ///< palpages d'un palier trop dispersés
    residual, ///< un palier loin de la droite (aller et retour différents)
    slope, ///< pente impossible (négative, nulle, trop forte)
};

struct Outcome {
    Verdict verdict = Verdict::slope;
    float slope_mm_per_c = NAN;
    float intercept_mm = NAN;
    float spread_mm = 0.f; ///< pire palier
    float residual_mm = 0.f; ///< pire écart à la droite
    float hysteresis_mm = NAN; ///< retour − aller, moyenne sur 170 et 215 °C
};

/// Les paliers mesurés (dans l'ordre de temperatures) → pente, et si elle peut servir
Outcome evaluate(const std::array<Stage, stages> &s);

} // namespace od3d::nozzle_expansion
