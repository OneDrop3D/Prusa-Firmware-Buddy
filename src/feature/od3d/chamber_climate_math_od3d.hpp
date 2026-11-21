/// @file
/// OneDrop3D (F29) : climat de la chambre, partie sans matériel (testée en unitaire).
///
/// - Recettes par matière : deux bits par matière, « plafond de chambre » et « descente douce ».
/// - Plafond : pendant l'impression, si le trancheur n'a donné aucune cible de chambre, la cible devient la température
///   maximale de la matière (préréglage de Prusa) ; les ventilateurs de la chambre de Prusa, en automatique, la tiennent.
/// - Descente douce : après une impression réussie, la consigne du plateau descend par paliers (step_c toutes les
///   interval_min minutes) jusqu'au seuil floor_c, puis le plateau est coupé.
///
/// Règle n° 1 (sécurité thermique) : la descente ne fait que **baisser** une consigne. Chaque palier vaut au plus la
/// température mesurée du plateau (on ne réchauffe jamais), au plus la consigne de l'impression, et la durée totale est
/// bornée (max_duration_ms). Toute consigne posée par quelqu'un d'autre (M140, « Refroidir », préchauffage, minuteur de
/// sécurité, erreur thermique) arrête la descente sans rien toucher.
#pragma once

#include <cstdint>
#include <optional>

namespace od3d::chamber_climate {

// ---- Recettes ----

/// Comportement d'une matière (valeurs enregistrées : ne pas renuméroter)
enum class Recipe : uint8_t {
    none = 0,
    ceiling = 1, ///< chambre plafonnée au maximum de la matière (PLA, PETG : heat creep)
    soft_cooldown = 2, ///< descente douce du plateau en fin d'impression (ABS, ASA, PC : retrait, gauchissement)
    both = 3,
};

inline constexpr bool has_ceiling(Recipe r) {
    return (static_cast<uint8_t>(r) & 1) != 0;
}
inline constexpr bool has_soft_cooldown(Recipe r) {
    return (static_cast<uint8_t>(r) & 2) != 0;
}

/// Recette de la matière n dans un mot : bit n = plafond, bit n + half = descente douce
template <typename Word>
constexpr Recipe recipe_get(Word word, uint8_t index) {
    constexpr uint8_t half = sizeof(Word) * 4;
    if (index >= half) {
        return Recipe::none;
    }
    const uint8_t ceiling = (word >> index) & 1;
    const uint8_t soft = (word >> (index + half)) & 1;
    return static_cast<Recipe>(ceiling | (soft << 1));
}

template <typename Word>
constexpr Word recipe_set(Word word, uint8_t index, Recipe r) {
    constexpr uint8_t half = sizeof(Word) * 4;
    if (index >= half) {
        return word;
    }
    const Word ceiling_bit = Word(Word(1) << index);
    const Word soft_bit = Word(Word(1) << (index + half));
    word = Word(word & ~ceiling_bit & ~soft_bit);
    if (has_ceiling(r)) {
        word = Word(word | ceiling_bit);
    }
    if (has_soft_cooldown(r)) {
        word = Word(word | soft_bit);
    }
    return word;
}

// ---- Plafond de chambre ----

/// Nouvelle cible de la chambre pendant l'impression, ou nullopt pour ne rien changer.
/// On n'agit que si la recette a le plafond, que la matière a un maximum, que la chambre peut refroidir maintenant
/// (ventilateurs en automatique) et que personne (trancheur, M141, utilisateur) n'a posé de cible : une cible donnée
/// prime toujours, et M191 n'attend jamais une cible que l'on aurait changée.
std::optional<uint8_t> ceiling_target(Recipe recipe, std::optional<uint8_t> material_max_c, bool chamber_has_target, bool can_cool);

// ---- Descente douce ----

struct CooldownParams {
    uint8_t step_c = 5; ///< baisse de chaque palier
    uint8_t interval_min = 5; ///< durée d'un palier
    uint8_t floor_c = 50; ///< en dessous : plateau coupé, fin

    static constexpr uint8_t step_min = 1, step_max = 10;
    static constexpr uint8_t interval_min_min = 1, interval_min_max = 20; ///< sous les 30 min du minuteur de sécurité
    static constexpr uint8_t floor_min = 35, floor_max = 90;

    /// Valeurs ramenées dans leurs plages
    CooldownParams clamped() const;
};

/// Durée maximale d'une descente, quels que soient les réglages
inline constexpr uint32_t max_duration_ms = 3ul * 60 * 60 * 1000;

/// Machine de la descente douce (thread de Marlin). Pas d'accès au matériel : step() dit quoi faire.
class Cooldown {
public:
    enum class Action : uint8_t {
        none, ///< rien à faire
        set_target, ///< poser la consigne target du plateau
        finish, ///< fin normale : couper le plateau (consigne 0)
        abort, ///< arrêt sans toucher au plateau (quelqu'un d'autre a la main)
    };

    struct Step {
        Action action = Action::none;
        int16_t target = 0;
    };

    struct Inputs {
        uint32_t now_ms;
        float bed_current; ///< température mesurée
        int16_t bed_target; ///< consigne actuelle (lue juste avant)
        bool nozzle_heating; ///< une buse a une consigne : l'imprimante sert à autre chose
    };

    /// Démarre la descente : premier palier = min(consigne de l'impression, mesure arrondie en dessous).
    /// \returns la consigne à poser, ou nullopt si le plateau est déjà sous le seuil (rien à faire)
    std::optional<int16_t> start(const CooldownParams &params, int16_t print_bed_target, float bed_current, uint32_t now_ms);

    /// À appeler régulièrement ; la consigne posée par set_target doit être confirmée par confirm()
    Step step(const Inputs &in);

    /// La consigne demandée a été posée (setTargetBed peut la rogner : on retient celle qui est vraiment en place)
    void confirm(int16_t target_set) {
        target_ = target_set;
    }

    /// Arrêt demandé (interface, M1998 S0, nouvelle impression)
    void stop() {
        active_ = false;
    }

    bool active() const {
        return active_;
    }

    int16_t target() const {
        return target_;
    }

    /// Temps restant avant le prochain palier
    uint32_t ms_to_next_step(uint32_t now_ms) const;

    /// Nombre de paliers restants (fin comprise), pour l'affichage
    uint16_t steps_left() const;

private:
    CooldownParams params_;
    bool active_ = false;
    int16_t target_ = 0;
    uint32_t start_ms_ = 0;
    uint32_t last_step_ms_ = 0;
};

} // namespace od3d::chamber_climate
