/// @file
/// OneDrop3D (F41) : longueur du tube PTFE avant (registre 0x22 du MMU, « Tube PTFE avant ») calculée d'après une
/// mesure, sans lecture de l'imprimante (testé dans tests/unit/feature/od3d).
///
/// Dans le MMU (Prusa-Firmware-MMU, logic/feed_to_bondtech.cpp), le chargement part du FINDA : la poulie est remise à
/// zéro, avance vite sur la longueur du registre 0x22, puis lentement (20 mm/s) jusqu'à ce que le capteur de
/// l'extrudeur voie le filament. S'il le voit pendant l'avance rapide, c'est l'erreur « capteur trop tôt » : la
/// longueur doit rester un peu sous la distance réelle FINDA → capteur. La mesure se fait avec la longueur minimale
/// (tout le trajet au-delà se fait lentement) ; la position de la poulie (registre 0x1a) relevée juste avant que le
/// capteur voie le filament est au plus la distance réelle (relevé jusqu'à ~200 ms en retard, ~4 mm).
#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>

namespace od3d::ptfe {

inline constexpr uint16_t min_mm = 341; ///< config::minimumBowdenLength du MMU (le registre refuse moins)
inline constexpr uint16_t max_mm = 1000; ///< config::maximumBowdenLength : au-delà, le MMU abandonne le chargement
inline constexpr uint16_t late_sample_mm = 10; ///< relevé en retard, capteur vu tout au début de l'avance lente

/// Marge sous la distance mesurée : 20 mm, ou 5 % si c'est plus (la poulie glisse plus ou moins d'une fois à l'autre)
constexpr uint16_t margin_mm(uint16_t distance_mm) {
    return std::max<uint16_t>(20, distance_mm / 20);
}

/// Longueur à écrire dans le registre 0x22. pulley_mm : position de la poulie relevée juste avant que le capteur de
/// l'extrudeur voie le filament, mesure faite avec fast_mm d'avance rapide. nullopt si le relevé est invraisemblable :
/// capteur vu pendant l'avance rapide (tube plus court que fast_mm), ou au-delà de max_mm.
constexpr std::optional<uint16_t> bowden_length(uint16_t pulley_mm, uint16_t fast_mm = min_mm) {
    if (pulley_mm + late_sample_mm < fast_mm || pulley_mm > max_mm) {
        return std::nullopt;
    }
    const int length = int(pulley_mm) - int(margin_mm(pulley_mm));
    return uint16_t(std::clamp(length, int(min_mm), int(max_mm)));
}

} // namespace od3d::ptfe
