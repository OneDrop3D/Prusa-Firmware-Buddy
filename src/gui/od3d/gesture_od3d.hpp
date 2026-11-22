/// @file
/// OneDrop3D (interface G) : ce qu'est un contact sur l'écran tactile (toucher, défilement, balayage), d'après le
/// déplacement du doigt entre sa pose et son lever et la durée du contact. Fonction pure, testée seule.
#pragma once

#include <cstdint>
#include <window_event.hpp>

namespace od3d::gesture {

/// Défilement vertical d'une liste : comme Prusa, dès 10 px et surtout vertical
inline constexpr int16_t scroll_min = 10;
/// Balayage horizontal (retour) : au moins 60 px, surtout horizontal, en moins de 600 ms. Un toucher un peu glissé, un
/// défilement de travers ou un curseur tiré lentement n'en sont jamais un (chez Prusa : 10 px, sans durée).
inline constexpr int16_t swipe_min = 60;
inline constexpr uint32_t swipe_max_ms = 600;

/// dx, dy : du point de pose au dernier point touché ; ms : durée du contact ; click_max : écart encore pris pour un
/// toucher. Rend TOUCH_CLICK, TOUCH_SWIPE_*, ou GUI_event_t::_count si ce n'est rien.
constexpr GUI_event_t classify(int16_t dx, int16_t dy, uint32_t ms, int16_t click_max) {
    const int ax = dx < 0 ? -dx : dx;
    const int ay = dy < 0 ? -dy : dy;
    if (ax <= click_max && ay <= click_max) {
        return GUI_event_t::TOUCH_CLICK;
    }
    // « Surtout » : l'autre axe fait au plus la moitié (angle de 26,6° au plus, comme Prusa)
    if (ay >= scroll_min && 2 * ax <= ay) {
        return dy < 0 ? GUI_event_t::TOUCH_SWIPE_UP : GUI_event_t::TOUCH_SWIPE_DOWN;
    }
    if (ax >= swipe_min && 2 * ay <= ax && ms < swipe_max_ms) {
        return dx < 0 ? GUI_event_t::TOUCH_SWIPE_LEFT : GUI_event_t::TOUCH_SWIPE_RIGHT;
    }
    return GUI_event_t::_count;
}

} // namespace od3d::gesture
