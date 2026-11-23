/// @file
/// OneDrop3D : plafond de M201 X/Y accepté par l'utilisateur après l'accélération max utile (M2004, « Appliquer » de
/// l'assistant). Gardé dans « OD3D Calib Values », case accel_limit, [2] X et [3] Y (mm/s² ; 0 : « Conserver » ou
/// retiré, no_value : jamais mesuré, sans plafond) : pas de nouvel élément, la config_store est pleine. Appliqué à chaque
/// M201 (Planner::set_max_acceleration, G-code de départ de PrusaSlicer compris) et aux valeurs de Prusa remises au
/// démarrage (MarlinSettings::reset_motion) ; un M201 plus bas passe tel quel (vibration::capped()).
#pragma once

#include "calib_registry_od3d.hpp"
#include "vibration_math_od3d.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

#include <config_store/store_instance.hpp>

namespace od3d::accel_cap {

inline constexpr size_t values_index = size_t(calib_registry::Test::accel_limit);

/// Plafonds X, Y tels que gardés (≤ 0 : aucun)
inline std::array<int16_t, 2> get() {
    const calib_registry::Values v = config_store().od3d_calib_values.get(values_index);
    return { v.v[2], v.v[3] };
}

/// « Appliquer » : les valeurs mesurées ; « Conserver » ou « Retirer » : 0, 0
inline void set(uint16_t x, uint16_t y) {
    calib_registry::Values v = config_store().od3d_calib_values.get(values_index);
    v.v[2] = int16_t(std::min<uint16_t>(x, INT16_MAX));
    v.v[3] = int16_t(std::min<uint16_t>(y, INT16_MAX));
    config_store().od3d_calib_values.set(values_index, v);
}

/// Accélération max de l'axe (0 : X, 1 : Y, autres : inchangée) avec le plafond
inline float apply(uint8_t axis, float requested) {
    return axis < 2 ? vibration::capped(requested, get()[axis]) : requested;
}

} // namespace od3d::accel_cap
