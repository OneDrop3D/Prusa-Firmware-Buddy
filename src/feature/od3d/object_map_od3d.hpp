/// @file
/// OneDrop3D : place des objets de l'impression sur le plateau, pour le plan de « Annuler des objets »
/// (screen_objects_od3d.hpp). Suivie dans le fil de Marlin (G1, G2/G3, M486), lue par l'interface. La partie pure est
/// dans object_map_math_od3d.hpp.
#pragma once

#include "object_map_math_od3d.hpp"

#include <cstddef>
#include <cstdint>

namespace od3d::object_map {

/// Objets suivis : les 16 premiers, comme leurs noms (marlin_vars_t::CANCEL_OBJECTS_NAME_COUNT) ; 64 octets
inline constexpr size_t max_objects = 16;

/// Fil de Marlin : nouvelle impression (M486 T, départ) ou reprise après une coupure (CancelObject::set_state). Aucun
/// objet en cours jusqu'au prochain M486 S : l'état de départ de Prusa dit « objet 0 », et la ligne de purge du G-code
/// de départ serait comptée dans sa place.
void reset();

/// Fil de Marlin : M486 S (CancelObject::set_current_object)
void set_current(int16_t object);

/// Fil de Marlin : un tracé de l'objet en cours qui pousse du filament, de (xa, ya) à (xb, yb) en mm
void on_extrusion(float xa, float ya, float xb, float yb);

/// Place de l'objet (pack), unknown s'il n'est pas encore imprimé ou au-delà des 16 premiers
uint32_t box(size_t object);

/// Change dès qu'une place grandit ou que tout est remis à zéro : le plan se redessine
uint32_t revision();

} // namespace od3d::object_map
