/// @file
/// OneDrop3D (F8, F26) : lecture des mouvements à venir dans la file du planificateur. Partie pure (sans Marlin,
/// testée en unitaire) ; la lecture de la vraie file est dans lookahead_od3d.cpp.
///
/// La file est un tampon circulaire [tail, head) de taille puissance de deux. Chaque bloc est résumé par sa durée
/// (longueur / vitesse nominale : les accélérations l'allongent un peu, c'est une estimation), le filament qu'il
/// pousse et la vitesse du ventilateur pièce qu'il porte (M106 est appliqué par le planificateur, bloc par bloc).
///
/// - F8 : vitesse moyenne du filament sur les ~500 ms à venir (pondérée par la durée des blocs).
/// - F26 : ventilateur pièce le plus fort demandé par les blocs qui commencent dans l'avance choisie (seules les
///   hausses sont avancées ; une baisse reste à l'heure de Prusa).
#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>

namespace od3d::lookahead {

/// Un bloc de la file, résumé
struct BlockInfo {
    float duration_s; ///< durée estimée [s] (0 : inconnue, ne compte pas)
    float e_mm; ///< filament poussé [mm] (négatif : recul ; 0 pour un mouvement du seul extrudeur)
    uint8_t fan; ///< ventilateur pièce porté par le bloc, 0 à 255
    bool is_move; ///< false : bloc de synchronisation (sans mouvement, ventilateur à 0) : ignoré
};

/// Durée estimée d'un bloc : longueur / vitesse nominale [s], 0 si les valeurs ne sont pas exploitables
float block_duration(float millimeters, float nominal_speed);

/// Fenêtre de F8 : débit des ~500 ms à venir
inline constexpr float flow_window_s = 0.5f;

/// Avance maximale du ventilateur (F26) [s]
inline constexpr float max_fan_lead_s = 1.0f;

/// F8 : moyenne de la vitesse du filament sur la fenêtre, pondérée par la durée des blocs. Une vitesse négative
/// (recul) ou au-delà de max_e_speed (chargement, saut) compte pour 0. Si la file couvre moins que la fenêtre, la
/// moyenne porte sur ce qu'elle contient.
class FlowScan {
public:
    FlowScan(float window_s, float max_e_speed)
        : window_s_(window_s)
        , max_e_speed_(max_e_speed) {}

    /// Ajoute le bloc suivant. @returns false quand la fenêtre est pleine (inutile de lire plus loin)
    bool add(const BlockInfo &b);

    /// Vitesse moyenne du filament [mm/s], 0 si la file est vide
    float mean_e_speed() const;

    /// Durée lue [s]
    float scanned_s() const { return time_s_; }

private:
    float window_s_;
    float max_e_speed_;
    float time_s_ = 0;
    float e_sum_ = 0; ///< Σ vitesse × durée retenue
};

/// F26 : ventilateur du premier bloc de mouvement, relevé par celui des blocs qui commencent moins de lead_s après
/// lui (le plus fort l'emporte)
class FanScan {
public:
    explicit FanScan(float lead_s)
        : lead_s_(lead_s) {}

    /// Ajoute le bloc suivant. @returns false quand les blocs suivants commencent au-delà de l'avance
    bool add(const BlockInfo &b);

    /// nullopt : aucun bloc de mouvement
    std::optional<uint8_t> fan() const { return has_first_ ? std::optional<uint8_t>(fan_) : std::nullopt; }

private:
    float lead_s_;
    float elapsed_s_ = 0; ///< début du bloc suivant, depuis le début du premier bloc de mouvement
    uint8_t fan_ = 0;
    bool has_first_ = false;
};

/// Parcourt la file circulaire [tail, head) dans l'ordre d'exécution. get(index) rend le BlockInfo d'une case ;
/// scan.add() arrête le parcours. size : taille du tampon (puissance de deux). Au plus size cases lues, même avec
/// des index incohérents.
template <class Scan, class Get>
void scan_ring(uint8_t tail, uint8_t head, uint8_t size, Scan &scan, Get &&get) {
    const uint8_t mask = uint8_t(size - 1);
    uint8_t i = uint8_t(tail & mask);
    const uint8_t end = uint8_t(head & mask);
    for (uint16_t n = 0; i != end && n < size; ++n, i = uint8_t((i + 1) & mask)) {
        if (!scan.add(get(i))) {
            break;
        }
    }
}

} // namespace od3d::lookahead
