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

inline constexpr uint8_t window = 50; ///< moyenne longue et écart sur ~50 changements
inline constexpr uint8_t recent_window = 5; ///< moyenne courte : λ = 1/5
inline constexpr uint8_t min_count = 10; ///< pas d'alerte avant
inline constexpr float floor_mm = 1.f;
inline constexpr float d2 = 1.128f; ///< étendue moyenne de 2 mesures / σ (loi normale)
/// MMU2_RETRY_UNLOAD_FINISH_LENGTH : au-delà (ou à 0), le capteur n'a pas vu partir le filament, rien de mesuré
inline constexpr float max_mm = 80.f;
inline constexpr float unit_mm = 0.002f; ///< résolution gardée (0 à 131 mm sur 16 bits)

/// Gardé par emplacement (config_store « OD3D Tip Stats », 8 octets : la banque du config_store est presque pleine).
/// Taille fixe : pour la changer, créer un nouvel élément sous un autre nom.
struct Stats {
    uint16_t mean = 0; ///< × unit_mm, ~50 derniers changements
    uint16_t recent = 0; ///< × unit_mm, ~5 derniers changements
    uint16_t range = 0; ///< × unit_mm, écart moyen d'un changement au suivant
    uint8_t count = 0; ///< changements comptés (plafonné à 255)

    constexpr bool operator==(const Stats &) const = default;
};
static_assert(sizeof(Stats) == 8);

constexpr float mm_of(uint16_t units) {
    return float(units) * unit_mm;
}

constexpr uint16_t units_of(float mm) {
    const float u = mm / unit_mm + 0.5f;
    return u <= 0.f ? 0 : (u >= 65535.f ? 65535 : uint16_t(u));
}

constexpr bool valid(float mm) {
    return mm > 0.f && mm <= max_mm; // NaN : faux
}

/// Moyennes glissantes exponentielles ; tant qu'il y a moins de changements que la fenêtre, moyennes exactes. last :
/// mesure d'avant (pas encore depuis le démarrage, ou non valable : l'écart n'est pas mis à jour)
constexpr Stats add(Stats s, float mm, float last) {
    if (!valid(mm)) {
        return s;
    }
    const auto weight = [](uint8_t n, uint8_t w) {
        return 1.f / float(n < w ? n : w);
    };
    if (s.count && valid(last)) {
        const float jump = mm > last ? mm - last : last - mm;
        s.range = units_of(mm_of(s.range) + weight(s.count, window) * (jump - mm_of(s.range)));
    }
    if (s.count < UINT8_MAX) {
        ++s.count;
    }
    s.mean = units_of(mm_of(s.mean) + weight(s.count, window) * (mm - mm_of(s.mean)));
    s.recent = units_of(mm_of(s.recent) + weight(s.count, recent_window) * (mm - mm_of(s.recent)));
    return s;
}

constexpr float sigma(const Stats &s) {
    return mm_of(s.range) / d2;
}

inline bool drifting(const Stats &s) {
    return s.count >= min_count && std::fabs(mm_of(s.recent) - mm_of(s.mean)) > std::fmax(floor_mm, sigma(s));
}

} // namespace od3d::tip
