/// @file
/// OneDrop3D (interface G) : anneau d'attente (SPEC_G.md §3.4) : bord #3A3D42 dont un quart orange tourne, un tour en
/// 1,6 s, anticrénelé. D'une image à la suivante, seuls les deux bouts de l'arc changent : on ne repeint qu'eux.
/// Partie pure (couleur d'un pixel, zones qui changent), testée seule ; le dessin est dans widgets_od3d.cpp.
#pragma once

#include <array>
#include <cstdint>
#include "style_od3d.hpp"

namespace od3d::spinner {

inline constexpr uint32_t turn_ms = 1600; ///< un tour
inline constexpr uint8_t steps = 40; ///< positions par tour : une par image à 25 images/s

/// Position de l'arc à l'instant ms
constexpr uint8_t step_at(uint32_t ms) {
    return uint8_t(ms % turn_ms * steps / turn_ms);
}

/// Pixel (x, y) d'un anneau de diamètre d et de bord w (carré de d px), l'arc commençant à la position step (0 = en haut,
/// sens horaire) ; bg hors de l'anneau
Color pixel(int x, int y, uint8_t d, uint8_t w, uint8_t step, Color bg);

/// Ce qui change dans le carré quand l'arc passe de from à to : la queue et la tête, chacune dans un rect (relatif au
/// carré, vide si rien) ; tout pixel qui change de couleur est dans l'un des deux
std::array<Rect16, 2> changed(uint8_t d, uint8_t w, uint8_t from, uint8_t to);

} // namespace od3d::spinner
