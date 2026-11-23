/// @file
/// OneDrop3D (F11) : assistance du MMU pendant l'impression (Atelier › Assistance), calcul et règles dans
/// mmu_assist_math_od3d.hpp. Le choix est gardé dans « OD3D MMU Speeds » (mmu_speed::Saved::assist, aucune nouvelle
/// entrée : config_store plein) ; les échanges avec le MMU sont faits par ProtocolLogic (fil de Marlin).
#pragma once

#include "mmu_assist_math_od3d.hpp"

namespace od3d::mmu_assist {

/// Assistance choisie (gardée)
bool enabled();
void set_enabled(bool on);

/// Ce que le MMU a dit à la dernière relecture (lisible depuis tous les fils)
Support support();
/// Vitesse de la poulie appliquée par le MMU [0,01 mm/s], 0 : arrêtée ou refusée
uint16_t applied();

} // namespace od3d::mmu_assist
