/// @file
/// OneDrop3D (F10 réduite) : pointe du filament suivie par emplacement à chaque changement d'outil en impression
/// (tip_math_od3d.hpp), affichée dans Atelier › Infos du MMU et sur la fiche de la bobine. Rien ne change dans le MMU.
///
/// Les mesures s'ajoutent en mémoire vive ; elles sont écrites (config_store « OD3D Tip Stats ») une fois, à la fin de
/// l'impression. Une coupure de courant pendant l'impression perd seulement les changements de celle-ci.
#pragma once

#include "tip_math_od3d.hpp"

#include <cstdint>

namespace od3d::tip {

inline constexpr uint8_t slots = 5; ///< MMU3 : emplacements 0 à 4

/// Ce qui est enregistré pour l'emplacement (tout fil) : mis à jour à la fin de chaque impression
Stats stats(uint8_t slot);

/// Fil de Marlin, à chaque tour de idle() (marlin_server cycle)
void step();

} // namespace od3d::tip
