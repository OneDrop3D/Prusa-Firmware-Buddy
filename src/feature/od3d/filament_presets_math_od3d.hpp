/// @file
/// OneDrop3D (UI-13) : températures des préréglages de Prusa (PLA, PETG…) modifiables par l'utilisateur.
/// Une modification par champ et par préréglage, enregistrée à part (config_store « OD3D Preset Overrides ») ; la
/// table de Prusa ne change pas. Sans modification, la valeur de Prusa s'applique, y compris après une mise à jour qui
/// la changerait. Partie pure, sans dépendance (testée dans tests/unit/feature/od3d/filament_presets_test.cpp).
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

namespace od3d::filament_presets {

/// Champs modifiables, dans l'ordre de l'écran
enum Field : uint8_t {
    nozzle,
    nozzle_preheat,
    bed,
    chamber_target,
    chamber_min,
    chamber_max,
    field_count,
};

/// Valeur absente : dans Values, pas de valeur (chambre « Aucune ») ; dans Override, la valeur de Prusa
inline constexpr int16_t none = -1;

/// Les températures d'un filament, en °C
using Values = std::array<int16_t, field_count>;

/// Modifications d'un préréglage (none : valeur de Prusa ; pour la chambre, 0 = « Aucune »)
struct Override {
    Values values { none, none, none, none, none, none };

    constexpr bool modified(Field f) const {
        return values[f] != none;
    }
    constexpr bool any() const {
        return std::ranges::any_of(values, [](int16_t v) { return v != none; });
    }
    constexpr bool operator==(const Override &) const = default;
};

/// Valeurs de Prusa avec les modifications ; la chambre reste dans l'ordre min ≤ cible ≤ max (une mise à jour de
/// Prusa pourrait déplacer une borne non modifiée)
constexpr Values apply(Values v, const Override &o) {
    for (uint8_t f = 0; f < field_count; ++f) {
        if (o.values[f] != none) {
            v[f] = (f >= chamber_target && o.values[f] == 0) ? none : o.values[f];
        }
    }
    int16_t &target = v[chamber_target], &lo = v[chamber_min], &hi = v[chamber_max];
    if (lo != none && hi != none && lo > hi) {
        hi = lo;
    }
    if (target != none) {
        if (lo != none && target < lo) {
            target = lo;
        }
        if (hi != none && target > hi) {
            target = hi;
        }
    }
    return v;
}

/// Modifications après « f = value » (value en °C, chambre 0 = « Aucune ») : une valeur égale à celle de Prusa est
/// retirée, le champ suit alors Prusa
constexpr Override with(Override o, Field f, int16_t value, const Values &prusa) {
    const int16_t prusa_value = (f >= chamber_target && prusa[f] == none) ? 0 : prusa[f];
    o.values[f] = value == prusa_value ? none : value;
    return o;
}

/// Override sans la modification de f (« Rétablir la valeur de Prusa »)
constexpr Override without(Override o, Field f) {
    o.values[f] = none;
    return o;
}

} // namespace od3d::filament_presets
