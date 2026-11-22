/// @file
/// OneDrop3D (F1, interface G, UI-10) : ce que montre la pastille « Dynamic Island » de l'en-tête, sans affichage ni
/// lecture de l'imprimante (testé dans tests/unit/feature/od3d). Une seule pastille à la fois, par priorité :
/// 1. le chargement ou déchargement réduit par la flèche de son écran ;
/// 2. l'impression en cours, hors de l'écran Impression ;
/// 3. la buse qui chauffe hors impression et hors machine à états (préchauffage), hors de l'aperçu des températures.
#pragma once

#include <cstdint>

namespace od3d::island {

enum class Kind : uint8_t {
    none, ///< pas de pastille
    filament, ///< chargement ou déchargement réduit
    print, ///< impression en cours
    preheat, ///< buse qui chauffe
};

/// État de l'impression, en couleur : orange (en cours), jaune (pause), gris (attente : reprise, chauffe, question),
/// rouge (erreur)
enum class Tone : uint8_t {
    normal,
    paused,
    waiting,
    error,
};

struct Face {
    Kind kind = Kind::none;
    Tone tone = Tone::normal;
    uint8_t percent = 0; ///< chargement, impression
    int16_t current = 0; ///< préchauffage : buse [°C]
    int16_t target = 0;

    constexpr bool operator==(const Face &) const = default;
};

/// Ce qu'on sait de l'imprimante et de l'écran ouvert
struct Inputs {
    bool filament_minimized = false;
    uint8_t filament_percent = 0;

    bool print_fsm = false; ///< machine Printing ouverte (du départ à « Accueil » après la fin)
    bool print_ended = false; ///< terminée ou arrêtée : plus de pastille
    bool print_hidden_by_fsm = false; ///< une autre machine à états s'affiche par-dessus (son écran est la priorité)
    bool on_print_screen = false;
    Tone print_tone = Tone::normal;
    uint8_t print_percent = 0;

    bool other_fsm = false; ///< une machine à états autre que l'impression : sa chauffe est la sienne
    bool on_temperatures_screen = false;
    bool nozzle_heating = false; ///< PreheatLatch
    int16_t nozzle = 0;
    int16_t nozzle_target = 0;
};

constexpr Face choose(const Inputs &in) {
    if (in.filament_minimized) {
        return Face { .kind = Kind::filament, .percent = in.filament_percent };
    }
    if (in.print_fsm) {
        if (in.print_ended || in.print_hidden_by_fsm || in.on_print_screen) {
            return {};
        }
        return Face { .kind = Kind::print, .tone = in.print_tone, .percent = in.print_percent > 100 ? uint8_t(100) : in.print_percent };
    }
    if (in.nozzle_heating && !in.other_fsm && !in.on_temperatures_screen) {
        return Face { .kind = Kind::preheat, .current = in.nozzle, .target = in.nozzle_target };
    }
    return {};
}

/// La buse chauffe-t-elle ? Avec une hystérésis, pour que la pastille n'apparaisse pas à chaque oscillation de la
/// régulation : elle apparaît quand une nouvelle cible est loin au-dessus de la température, et disparaît une fois la
/// cible atteinte, jusqu'à la cible suivante (une buse qui refroidit un peu sous le ventilateur ne la rallume pas).
class PreheatLatch {
public:
    static constexpr int16_t start_below = 5; ///< [°C] sous la cible : elle chauffe vraiment
    static constexpr int16_t reached_below = 2; ///< [°C] sous la cible : atteinte

    constexpr bool update(int16_t current, int16_t target) {
        if (target <= 0) {
            latched_target = 0;
            heating = false;
            return false;
        }
        if (target != latched_target) {
            latched_target = target;
            heating = current < target - start_below;
        } else if (heating && current >= target - reached_below) {
            heating = false;
        }
        return heating;
    }

private:
    int16_t latched_target = 0;
    bool heating = false;
};

} // namespace od3d::island
