/// @file
/// OneDrop3D (UI-13) : préréglages de Prusa modifiés par l'utilisateur, appliqués à FilamentTypeParameters.
/// FilamentType::parameters() (filament.cpp) est le seul accès aux préréglages : préchauffage, chargement, chambre,
/// recettes F29, températures de M1994… lisent tous les valeurs modifiées. preset_filament_parameters() reste la table
/// de Prusa (« Rétablir la valeur de Prusa »).
#pragma once

#include "filament_presets_math_od3d.hpp"

#include <filament.hpp>
#include <utility>

namespace od3d::filament_presets {

/// Températures de p
inline Values values(const FilamentTypeParameters &p) {
    Values v { p.nozzle_temperature, p.nozzle_preheat_temperature, p.heatbed_temperature, none, none, none };
#if HAS_CHAMBER_API()
    const auto c = [](std::optional<uint8_t> t) { return t ? int16_t(*t) : none; };
    v[chamber_target] = c(p.chamber_target_temperature);
    v[chamber_min] = c(p.chamber_min_temperature);
    v[chamber_max] = c(p.chamber_max_temperature);
#endif
    return v;
}

/// p avec les modifications o
inline FilamentTypeParameters apply(FilamentTypeParameters p, const Override &o) {
    if (!o.any()) {
        return p;
    }
    const Values v = apply(values(p), o);
    p.nozzle_temperature = v[nozzle];
    p.nozzle_preheat_temperature = v[nozzle_preheat];
    p.heatbed_temperature = v[bed];
#if HAS_CHAMBER_API()
    const auto c = [](int16_t t) { return t == none ? std::nullopt : std::optional<uint8_t>(uint8_t(t)); };
    p.chamber_target_temperature = c(v[chamber_target]);
    p.chamber_min_temperature = c(v[chamber_min]);
    p.chamber_max_temperature = c(v[chamber_max]);
#endif
    return p;
}

} // namespace od3d::filament_presets
