/// @file
/// OneDrop3D (F39) : mode veille, logique pure (tests/unit/feature/od3d/sleep_od3d_test.cpp).
///
/// Quand l'imprimante peut-elle s'endormir, qu'est-ce qui la réveille, quelles coupures sont dues. Les branchements
/// (écran, lumières, chauffes, moteurs, ventilateurs) sont dans sleep_od3d.* (fil de Marlin) et
/// gui/od3d/sleep_gui_od3d.* (interface).
///
/// - Endormissement : mode actif et délai écoulé depuis la dernière activité, ou « Mettre en veille maintenant ».
///   Jamais quand l'imprimante est occupée (impression ou pause, G-code en cours, machine à états ouverte autre que
///   l'écran de fin d'impression) : c'est une activité, le délai repart de la fin.
/// - Réveil : toucher, molette, clé USB insérée, changement d'état de l'impression (travail reçu par PrusaLink ou
///   Connect), nouvelle machine à états (erreurs, alertes, chargement…), événement du MMU, imprimante occupée.
/// - L'entrée qui réveille un écran éteint n'agit pas : le toucher, le cran de molette (et le reste du geste pendant
///   grace_ms), l'appui sur le bouton jusqu'à son relâchement.
/// - Coupures, une seule fois par mise en veille : moteurs dès l'endormissement, chauffes après leur propre délai.
#pragma once

#include <cstdint>

namespace od3d::sleep {

/// Délais proposés [min], pour la mise en veille et pour les chauffes (qui ont aussi 0 : dès l'endormissement)
inline constexpr uint8_t delays_min[] = { 1, 2, 5, 10, 15, 30, 60 };

/// Délais réglables à la minute près (retour du 24/11 : « une durée personnalisée ») : veille de 1 à 240 min, chauffes
/// de 0 (dès l'endormissement) à 240 min après l'endormissement
inline constexpr uint8_t delay_max_min = 240;

/// Maintien du moteur Z en veille, en % de son courant de maintien normal (config_store « OD3D Sleep Z Hold »).
/// Prusa garde Z alimenté à plein courant, sans jamais le couper : à l'arrêt, son hacheur siffle (le « léger bruit »
/// entendu en veille, retour du 24/11). 100 : comme Prusa ; 0 : Z coupé (le plateau peut descendre, l'origine Z est
/// perdue). Rétabli au réveil et avant tout travail.
inline constexpr uint8_t z_hold_levels[] = { 100, 50, 25, 0 };
inline constexpr uint8_t default_z_hold = 50;

/// Courant de maintien réduit au pourcentage pct (au moins 1 pas de 32 si pct > 0 : un IHOLD nul serait une coupure)
constexpr uint8_t reduced_ihold(uint8_t ihold, uint8_t pct) {
    if (pct >= 100) {
        return ihold;
    }
    const unsigned v = (unsigned(ihold) * pct + 50) / 100;
    return uint8_t(pct && v == 0 ? 1 : v);
}

/// Réglages (config_store, « OD3D Sleep Settings »). Pour changer la structure, prendre un autre nom d'élément.
struct Settings {
    bool enabled = true; ///< mise en veille automatique
    uint8_t delay_min = 10; ///< sans activité
    bool screen = true; ///< écran éteint (rétroéclairage coupé, écran noir)
    bool light = true; ///< lumières de la chambre, barre d'état et plateau éteintes
    bool heaters = true; ///< buse et plateau coupés
    uint8_t heaters_delay_min = 15; ///< après l'endormissement (0 : tout de suite)
    bool motors = true; ///< moteurs coupés (comme la coupure d'inactivité de Prusa : X, Y, extrudeur)
    bool fans = false; ///< ventilateurs de la chambre et filtration coupés (les secours de surchauffe restent)

    bool operator==(const Settings &) const = default;
};
static_assert(sizeof(Settings) == 8);

/// Ce que l'interface voit de l'imprimante, relevé à chaque tour
struct Watch {
    bool busy = false; ///< impression ou pause, G-code en cours, machine à états ouverte (hors fin d'impression)
    uint8_t print_state = 0; ///< marlin_server::State
    uint8_t fsm = 0; ///< machine à états du dessus + 1, 0 : aucune
    bool usb = false; ///< clé insérée
    uint16_t mmu = 0; ///< état du MMU et FINDA
};

enum class Input : uint8_t {
    touch,
    encoder,
    button, ///< appui (ou appui maintenu)
    button_release, ///< relâchement
};

/// Coupures à faire maintenant
struct Due {
    bool motors = false;
    bool heaters = false;
};

class Idle {
public:
    /// Après un réveil par une entrée, le reste du geste (molette, second toucher) n'agit pas non plus
    static constexpr uint32_t grace_ms = 500;

    /// À chaque relevé de l'interface. @return l'imprimante vient de s'endormir ou de se réveiller
    bool step(uint32_t now_ms, const Settings &s, const Watch &w);

    /// Entrée de l'utilisateur (une activité). @param screen_off l'écran est éteint
    /// @return l'entrée est avalée : elle a réveillé l'écran, ou elle termine le geste du réveil
    bool input(uint32_t now_ms, Input in, bool screen_off);

    /// « Mettre en veille maintenant ». @return false : imprimante occupée, rien n'est fait
    bool sleep_now(uint32_t now_ms);

    bool asleep() const { return asleep_; }

    /// Coupures dues (chacune une fois par mise en veille)
    Due take_due(uint32_t now_ms, const Settings &s);

private:
    void fall_asleep(uint32_t now_ms);

    Watch last {};
    uint32_t activity_ms = 0;
    uint32_t asleep_since_ms = 0;
    uint32_t woke_ms = 0;
    bool started = false;
    bool asleep_ = false;
    bool grace = false; ///< réveil par une entrée, écran éteint : grace_ms à partir de woke_ms
    bool swallow_button = false; ///< réveil par le bouton : avaler jusqu'au relâchement
    bool motors_done = false;
    bool heaters_done = false;
};

} // namespace od3d::sleep
