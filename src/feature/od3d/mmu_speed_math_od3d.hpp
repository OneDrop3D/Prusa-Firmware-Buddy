/// @file
/// OneDrop3D (N4) : vitesse de la poulie du MMU choisie d'après des cycles chargement + déchargement à vitesse
/// croissante, sans lecture de l'imprimante (testé dans tests/unit/feature/od3d).
///
/// Dans le MMU (Prusa-Firmware-MMU, logic/feed_to_bondtech.cpp et unload_to_finda.cpp), la poulie avance à
/// Pulley_Load_Feedrate (registre 0x11) sur la longueur du tube, puis à 20 mm/s (0x14) jusqu'au capteur de
/// l'extrudeur (progression FeedingToFSensor) ; elle recule à Pulley_Unload_Feedrate (0x0d) jusqu'au FINDA.
/// L'accélération (0x0e, 800 mm/s²) reste celle de Prusa. Ces registres ne sont pas gardés par le MMU : l'imprimante
/// les renvoie à chaque démarrage (ProtocolLogic, registres d'initialisation).
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

namespace od3d::mmu_speed {

inline constexpr uint8_t prusa_speed = 95; ///< config::pulleyLoadFeedrate et pulleyUnloadFeedrate du MMU [mm/s]
/// Paliers essayés, de la vitesse de Prusa à 190 mm/s (80 % : 152 mm/s au plus ; le MMU pas à pas jusqu'à 40 kHz)
inline constexpr std::array<uint8_t, 6> levels { prusa_speed, 115, 135, 155, 175, 190 };
inline constexpr uint8_t keep_percent = 80; ///< marge sous le dernier palier passé
/// Avance lente plus longue qu'au palier de Prusa : la poulie a patiné (ou le moteur décroché) pendant l'avance rapide.
/// 1 s = 20 mm à 20 mm/s ; la progression est relue ~5 fois par seconde (±0,2 s)
inline constexpr uint32_t slip_ms = 1000;
/// Cycle plus lent que le palier d'avant : réessai caché (déchargement et rechargement) ou poulie qui décroche au
/// déchargement. Un palier gagne ~0,5 s ; l'écart d'un cycle à l'autre reste sous ~0,5 s (interrogations du MMU)
inline constexpr uint32_t slower_ms = 1000;

/// Un cycle chargement + déchargement (test de chargement de Prusa, M1704)
struct Cycle {
    bool error = false; ///< erreur du MMU ou de l'imprimante, réessai compris
    uint32_t total_ms = 0;
    uint32_t slow_ms = 0; ///< temps passé dans l'avance lente (FeedingToFSensor)
};

/// Palier passé : sans erreur ni réessai, sans patinage par rapport au palier de Prusa, pas plus lent que le palier
/// d'avant. prusa et previous : nullptr pour le palier de Prusa lui-même
constexpr bool passed(const Cycle &c, const Cycle *prusa, const Cycle *previous) {
    return !c.error
        && (!prusa || c.slow_ms <= prusa->slow_ms + slip_ms)
        && (!previous || c.total_ms <= previous->total_ms + slower_ms);
}

struct Choice {
    uint8_t passed = 0; ///< paliers passés d'affilée depuis celui de Prusa
    uint8_t speed = 0; ///< vitesse à garder ; 0 : le palier de Prusa n'est pas passé (rien ne change)
};

/// cycles[i] : cycle au palier levels[i]. On garde 80 % du dernier palier passé avant le premier échec, jamais moins
/// que la vitesse de Prusa
constexpr Choice choose(std::span<const Cycle> cycles) {
    Choice c;
    const size_t n = std::min(cycles.size(), levels.size());
    while (c.passed < n && passed(cycles[c.passed], c.passed ? &cycles[0] : nullptr, c.passed ? &cycles[c.passed - 1] : nullptr)) {
        ++c.passed;
    }
    if (c.passed) {
        c.speed = uint8_t(std::max<unsigned>(prusa_speed, unsigned(levels[c.passed - 1]) * keep_percent / 100));
    }
    return c;
}

/// Gardé dans le config_store (« OD3D MMU Speeds ») : vitesse choisie et durées mesurées d'un cycle. Taille fixe :
/// pour la changer, créer un nouvel élément sous un autre nom.
struct Saved {
    uint8_t speed = 0; ///< mm/s ; 0 : celle de Prusa
    uint16_t prusa_ds : 15 = 0; ///< cycle à la vitesse de Prusa [0,1 s], 0 sans mesure
    /// Bit 15 : choix de l'assistance F11, retirée. Ignoré : il peut valoir 1 là où elle a été activée, jusqu'à la
    /// prochaine écriture (M2002 ou remise à zéro)
    uint16_t unused : 1 = 0;
    uint16_t cycle_ds = 0; ///< cycle à la vitesse choisie [0,1 s]

    constexpr bool operator==(const Saved &) const = default;
};
static_assert(sizeof(Saved) == 6, "même place dans le config_store qu'avant F11");

/// Vitesse envoyée au MMU
constexpr uint8_t speed_of(const Saved &s) {
    return s.speed ? s.speed : prusa_speed;
}

} // namespace od3d::mmu_speed
