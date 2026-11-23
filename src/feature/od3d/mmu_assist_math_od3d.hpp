/// @file
/// OneDrop3D (F11) : assistance du MMU pendant l'impression. Partie pure (testée dans tests/unit/feature/od3d) ;
/// l'envoi est dans mmu_assist_od3d.cpp, entre deux interrogations du MMU (ProtocolLogic::IdleWait).
///
/// Poussée à force limitée : l'imprimante écrit dans le registre 0x25 du MMU (firmware OneDrop3D du MMU,
/// logic/pulley_service.h) la vitesse voulue de la poulie, un peu plus rapide que le filament (ratio) ; le MMU serre
/// le galet et fait tourner la poulie à courant réduit : en avance, elle ne pousse qu'avec une petite force et décroche.
/// Pas de synchronisation pas à pas : une vitesse moyenne, au plus toutes les period_ms.
/// - Vitesse du filament : moyenne des ~500 ms à venir dans la file du planificateur (lookahead::planner_e_speed_ahead :
///   reculs, retours et mouvements du seul extrudeur comptent 0).
/// - Coupée (0 : poulie arrêtée, courant et galet comme chez Prusa) hors impression en cours (pause, fin, erreur du MMU
///   qui met en pause) ou assistance non choisie. Pendant un changement d'outil, le MMU est occupé : rien n'est envoyé,
///   et le MMU arrête lui-même l'assistance à la commande.
/// - Débit sous min_e_speed pendant l'impression (reculs, déplacements) : la poulie ralentit à creep_value et le galet
///   reste serré tant que ça dure moins de hold_ms (pas d'aller-retour du galet à chaque déplacement) ; au-delà
///   (attente de chauffe, longue pause du G-code), coupée. Un recul dure quelques dizaines de ms : un registre lu au
///   plus deux fois par seconde ne peut pas le suivre ; à force limitée, la poulie qui pousse doucement pendant ce
///   temps ne fait que comprimer un peu le filament dans le tube.
/// - Chaque écriture est relue : sans le bit 15, le MMU ne connaît pas le registre (Prusa ou ancien OneDrop3D lisent
///   0) ; vitesse relue 0 alors qu'on en demande une : refusée (filament pas jusqu'à la buse…), renvoyée ensuite.
#pragma once

#include <algorithm>
#include <cstdint>

namespace od3d::mmu_assist {

/// Rapport vitesse de la poulie / vitesse du filament : un peu plus vite (1,05 à 1,10)
inline constexpr float ratio = 1.07f;
/// Sous ce débit [mm/s de filament], l'assistance est coupée
inline constexpr float min_e_speed = 0.2f;
/// Vitesse du filament au-delà de laquelle un bloc ne compte pas (chargement, saut) [mm/s], ~60 mm³/s en 1,75 mm
inline constexpr float max_e_speed = 25.0f;
/// Vitesse maximale acceptée par le MMU [0,01 mm/s]
inline constexpr uint16_t max_value = 3000;
/// Bit 15 de la valeur relue : le MMU connaît le registre
inline constexpr uint16_t present_bit = 0x8000;
/// Au plus un échange par période [ms] ; le MMU garde un ordre 4 s puis s'arrête seul
inline constexpr uint32_t period_ms = 500;

/// Débit nul plus court que ça pendant l'impression : galet gardé serré, poulie à creep_value [ms]
inline constexpr uint32_t hold_ms = 3000;
/// Vitesse de la poulie pendant ce temps [0,01 mm/s] : la plus lente du MMU (32 pas/s, calc_timer)
inline constexpr uint16_t creep_value = 40;

/// Valeur à écrire pour une vitesse moyenne du filament [mm/s] : vitesse de la poulie [0,01 mm/s], 0 = coupée
constexpr uint16_t pulley_value(bool run, float e_speed) {
    if (!run || !(e_speed >= min_e_speed)) { // NaN compris
        return 0;
    }
    return uint16_t(std::min(e_speed * ratio * 100.0f + 0.5f, float(max_value)));
}

/// Vitesse voulue d'une période à l'autre, avec le maintien des débits nuls courts
class Speed {
public:
    /// run : impression en cours et assistance choisie ; e_speed : vitesse moyenne du filament à venir [mm/s]
    constexpr uint16_t next(uint32_t now_ms, bool run, float e_speed) {
        if (const uint16_t v = pulley_value(run, e_speed)) {
            flowing_ = true;
            flow_ms_ = now_ms;
            return v;
        }
        if (run && flowing_ && now_ms - flow_ms_ < hold_ms) {
            return creep_value;
        }
        flowing_ = false;
        return 0;
    }

private:
    uint32_t flow_ms_ = 0;
    bool flowing_ = false;
};

enum class Support : uint8_t {
    unknown, ///< pas encore relu (MMU qui démarre)
    yes,
    no, ///< firmware du MMU sans le registre
};

/// Échanges avec le registre : reconnaissance du MMU, puis écriture (et relecture) au plus toutes les period_ms tant
/// qu'une vitesse est voulue, et une fois l'arrêt.
class Link {
public:
    /// Un échange peut être dû maintenant (sinon inutile de calculer la vitesse voulue)
    constexpr bool due(uint32_t now_ms) const {
        return support_ != Support::no && !(started_ && now_ms - last_ms_ < period_ms);
    }

    /// true : un échange est dû maintenant (lecture seule si detecting(), sinon écrire value())
    constexpr bool next(uint32_t now_ms, uint16_t wanted) {
        if (!due(now_ms)) {
            return false;
        }
        if (support_ == Support::yes) {
            if (wanted == 0 && sent_ == 0) {
                return false; // arrêt déjà envoyé
            }
            sent_ = wanted;
        }
        started_ = true;
        last_ms_ = now_ms;
        return true;
    }

    constexpr bool detecting() const { return support_ == Support::unknown; }
    constexpr uint16_t value() const { return sent_; }

    /// Valeur relue après l'échange
    constexpr void read_back(uint16_t raw) {
        support_ = (raw & present_bit) ? Support::yes : Support::no;
        applied_ = raw & ~present_bit;
    }

    constexpr Support support() const { return support_; }
    /// Vitesse appliquée par le MMU [0,01 mm/s] ; 0 : arrêtée ou refusée
    constexpr uint16_t applied() const { return applied_; }

private:
    uint32_t last_ms_ = 0;
    uint16_t sent_ = 0;
    uint16_t applied_ = 0;
    Support support_ = Support::unknown;
    bool started_ = false;
};

} // namespace od3d::mmu_assist
