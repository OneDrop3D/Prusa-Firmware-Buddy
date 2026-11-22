/// @file
/// OneDrop3D (F41) : longueur du tube PTFE avant (registre 0x22 du MMU, « Tube PTFE avant ») calculée d'après une
/// mesure, sans lecture de l'imprimante (testé dans tests/unit/feature/od3d).
///
/// Dans le MMU (Prusa-Firmware-MMU, logic/feed_to_bondtech.cpp), le chargement part du FINDA : la poulie avance vite sur
/// la longueur du registre 0x22, puis lentement (20 mm/s, progression FeedingToFSensor) jusqu'à ce que le capteur de
/// l'extrudeur voie le filament. S'il le voit pendant l'avance rapide, c'est l'erreur « capteur trop tôt » : la
/// longueur doit rester un peu sous la distance réelle FINDA → capteur. La mesure se fait avec la longueur minimale
/// (tout le trajet au-delà se fait lentement).
///
/// La position de la poulie (registre 0x1a) n'est PAS remise à zéro à chaque chargement : Pulley::InitAxis ne fait que
/// réinitialiser le pilote, la position s'additionne depuis l'allumage du MMU (chargements, déchargements,
/// préchargements). La 1re version la prenait pour la distance : valeur au hasard, d'où « échec » ou une longueur
/// fausse. On mesure donc le chemin de la poulie pendant l'avance lente, qui commence exactement à fast_mm du FINDA.
#pragma once

#include "ptfe_od3d.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>

namespace od3d::ptfe {

inline constexpr uint16_t min_mm = 341; ///< config::minimumBowdenLength du MMU (le registre refuse moins)
inline constexpr uint16_t max_mm = 1000; ///< config::maximumBowdenLength : au-delà, le MMU abandonne le chargement

/// Marge sous la distance mesurée : 20 mm, ou 5 % si c'est plus (la poulie glisse plus ou moins d'une fois à l'autre)
constexpr uint16_t margin_mm(uint16_t distance_mm) {
    return std::max<uint16_t>(20, distance_mm / 20);
}

/// Longueur à écrire dans le registre 0x22 pour une distance FINDA → capteur de l'extrudeur ; nullopt hors de ce que le
/// MMU accepte (sous l'avance rapide minimale, au-delà de 1000 mm)
constexpr std::optional<uint16_t> bowden_length(uint16_t distance_mm) {
    if (distance_mm < min_mm || distance_mm > max_mm) {
        return std::nullopt;
    }
    return uint16_t(std::max<int>(int(distance_mm) - int(margin_mm(distance_mm)), min_mm));
}

/// Suivi d'une mesure : sample() à chaque tour, jusqu'au capteur. La position de la poulie est relue après la
/// progression, à chaque interrogation (~5 fois par seconde) : la 1re position relue dans l'avance lente peut dater
/// d'avant elle, elle est écartée ; la suivante est prise comme départ. distance = fast_mm + chemin entre ce départ et
/// la dernière position relue avant le capteur : sous-estimée d'un ou deux relevés (~4 mm chacun), jamais trop longue.
class Tracker {
public:
    struct Outcome {
        Result result;
        uint16_t measured_mm; ///< distance FINDA → capteur (0 sans)
        uint16_t saved_mm; ///< longueur à écrire (0 sans)
    };

    /// slow : le MMU est dans son avance lente (FeedingToFSensor) ; pulley : registre 0x1a ; sensor : le capteur de
    /// l'extrudeur voit le filament
    void sample(bool slow, uint16_t pulley, bool sensor) {
        if (triggered) {
            return;
        }
        if (slow && !slow_now) { // entrée dans l'avance lente, aussi après une reprise du MMU
            stale = pulley;
            fresh = false;
            slow_seen = true;
        }
        slow_now = slow;
        if (sensor) {
            triggered = true;
            return;
        }
        if (slow && !fresh && pulley != stale) {
            fresh = true;
            start = pulley;
        }
        if (slow && fresh) {
            last = pulley;
        }
    }

    Outcome outcome(uint16_t fast_mm = min_mm) const {
        if (!triggered) {
            return { slow_seen ? Result::no_trigger : Result::interrupted, 0, 0 };
        }
        if (!slow_now) {
            return { Result::too_short, 0, 0 }; // capteur vu pendant l'avance rapide
        }
        // uint16_t : la position relue est un int32 du MMU tronqué, l'écart reste juste même s'il passe par 0
        const uint32_t distance = uint32_t(fast_mm) + (fresh ? uint16_t(last - start) : 0u);
        const auto length = bowden_length(uint16_t(std::min<uint32_t>(distance, UINT16_MAX)));
        if (!length) {
            return { distance > max_mm ? Result::too_long : Result::too_short, uint16_t(std::min<uint32_t>(distance, UINT16_MAX)), 0 };
        }
        return { Result::saved, uint16_t(distance), *length };
    }

private:
    bool triggered = false;
    bool slow_now = false; ///< avance lente au dernier relevé
    bool slow_seen = false;
    bool fresh = false; ///< départ relevé dans l'avance lente
    uint16_t stale = 0; ///< position relue à l'entrée dans l'avance lente (peut-être d'avant)
    uint16_t start = 0;
    uint16_t last = 0;
};

} // namespace od3d::ptfe
