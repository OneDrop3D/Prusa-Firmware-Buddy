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

/// Fiche de bobine tirée vers le bas par sa poignée ou son haut (retour du 22/11 : « prendre la petite tige et la
/// baisser pour fermer le menu »)
namespace pull {

    /// Le doigt saisit la fiche dès 6 px vers le bas, surtout vertical ; avant, c'est encore un toucher ou un balayage
    inline constexpr int16_t grab_min = 6;
    constexpr bool grabs(int16_t dx, int16_t dy) {
        return dy >= grab_min && dy > (dx < 0 ? -dx : dx);
    }

    /// Décalage de la fiche sous un doigt descendu de dy : jamais au-dessus de sa place, au plus max
    constexpr int16_t offset(int16_t dy, int16_t max) {
        return dy < 0 ? 0 : (dy > max ? max : dy);
    }

    /// Lâchée : fermée à partir du tiers de sa hauteur, sinon elle remonte
    constexpr bool closes(int16_t offset, int16_t height) {
        return 3 * offset >= height;
    }

    /// Une image toutes les 40 ms au plus (25 images/s)
    inline constexpr uint32_t frame_ms = 40;
    /// Remontée : le décalage de l'image suivante (÷ 3, 4 images depuis le tiers de la fiche : 160 ms)
    constexpr int16_t spring(int16_t offset) {
        return offset / 3;
    }

} // namespace pull

} // namespace od3d::gesture
