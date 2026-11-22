/// @file
/// OneDrop3D (F10 réduite) : suivi de la pointe du filament par emplacement, sans lecture de l'imprimante (testé dans
/// tests/unit/feature/od3d).
///
/// À chaque changement d'outil en impression, Prusa mesure le recul de l'extrudeur entre la fin du ramming et le moment
/// où son capteur ne voit plus le filament (MMU2::unloadEPosOnFSOff, 14 mm attendus). Plus court : du filament est resté
/// dans la zone de fusion ; plus long : pointe filandreuse. Prusa s'en sert pour compenser le chargement suivant.
/// On garde par emplacement une moyenne glissante sur ~50 changements, une moyenne courte sur ~5 et l'écart σ.
///
/// Alerte, comme une carte de contrôle EWMA (Roberts ; λ = 0,2, L = 3) : la moyenne courte s'écarte de la moyenne
/// longue de plus de 3 σ √(λ / (2 − λ)) = σ, et d'au moins 1 mm (hystérésis du capteur du filament). σ vient de l'étendue
/// mobile (écart moyen d'un changement au suivant / 1,128, carte des individus) : une dérive ne l'élargit pas, elle
/// reste visible. Une pointe plus courte de 2 mm (écart de ±0,5 mm) est signalée en 5 changements au plus ; l'alerte
/// retombe quand la nouvelle valeur devient l'habitude. Pas avant 10 changements (σ trop incertain).
#pragma once

#include <cmath>
#include <cstdint>

namespace od3d::tip {

inline constexpr uint16_t window = 50; ///< moyenne longue et écart sur ~50 changements
inline constexpr uint16_t recent_window = 5; ///< moyenne courte : λ = 1/5
inline constexpr uint16_t min_count = 10; ///< pas d'alerte avant
inline constexpr float floor_mm = 1.f;
inline constexpr float d2 = 1.128f; ///< étendue moyenne de 2 mesures / σ (loi normale)
/// MMU2_RETRY_UNLOAD_FINISH_LENGTH : au-delà (ou à 0), le capteur n'a pas vu partir le filament, rien de mesuré
inline constexpr float max_mm = 80.f;

/// Gardé par emplacement (config_store « OD3D Tip Stats »). Taille fixe : pour la changer, créer un nouvel élément sous
/// un autre nom.
struct Stats {
    float mean = 0.f; ///< mm, ~50 derniers changements
    float recent = 0.f; ///< mm, ~5 derniers changements
    float range = 0.f; ///< écart moyen d'un changement au suivant [mm]
    float last = 0.f; ///< dernière mesure [mm]
    uint16_t count = 0; ///< changements comptés (plafonné)

    constexpr bool operator==(const Stats &) const = default;
};

constexpr bool valid(float mm) {
    return mm > 0.f && mm <= max_mm; // NaN : faux
}

/// Moyennes glissantes exponentielles ; tant qu'il y a moins de changements que la fenêtre, moyennes exactes
constexpr Stats add(Stats s, float mm) {
    if (!valid(mm)) {
        return s;
    }
    const auto weight = [](uint16_t n, uint16_t w) {
        return 1.f / float(n < w ? n : w);
    };
    if (s.count) {
        const float jump = mm > s.last ? mm - s.last : s.last - mm;
        s.range += weight(s.count, window) * (jump - s.range); // count - 1 écarts avant celui-ci
    }
    if (s.count < UINT16_MAX) {
        ++s.count;
    }
    s.mean += weight(s.count, window) * (mm - s.mean);
    s.recent += weight(s.count, recent_window) * (mm - s.recent);
    s.last = mm;
    return s;
}

constexpr float sigma(const Stats &s) {
    return s.range / d2;
}

inline bool drifting(const Stats &s) {
    return s.count >= min_count && std::fabs(s.recent - s.mean) > std::fmax(floor_mm, sigma(s));
}

} // namespace od3d::tip
