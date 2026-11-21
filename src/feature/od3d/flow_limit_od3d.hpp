/// @file
/// OneDrop3D (F5) : limite de débit appliquée par le planificateur de Marlin. Écrite par spool_calib_od3d
/// (refresh_flow_limit), suspendue pendant les mesures (measure_session_od3d), lue par Planner::_populate_block.
/// Elle ne fait que réduire la vitesse d'un mouvement d'impression (E en avant avec XY) dont le filament irait plus
/// vite qu'elle ; jamais l'augmenter.
#pragma once

#include <atomic>

namespace od3d::flow_limit {

/// Vitesse maximale du filament [mm/s] ; 0 : pas de limite
inline std::atomic<float> e_speed_mm_s = 0.f;

/// Une mesure par la cellule de charge est en cours : pas de limite
inline std::atomic<bool> suspended = false;

/// Limite en vigueur [mm/s de filament] ; 0 : aucune
inline float planner_limit() {
    return suspended.load(std::memory_order_relaxed) ? 0.f : e_speed_mm_s.load(std::memory_order_relaxed);
}

} // namespace od3d::flow_limit
