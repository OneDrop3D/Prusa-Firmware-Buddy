#include "screen_menu.hpp"

#include <ScreenHandler.hpp>

void ScreenMenuBase_::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    // OneDrop3D (interface G) : appui long = retour, sauf sous un dialogue ; les écrans qui s'en servent déjà
    // (Contrôle, Matériel) le traitent avant d'arriver ici
    if (event == GUI_event_t::HELD_RELEASED && !GetFirstDialog()) {
        Screens::Access()->close_screen();
        return;
    }
    screen_t::windowEvent(sender, event, param);
}

void ScreenMenuBase_::unconditionalDraw() {
    od3d::fill_background(*this, getFirstNormal(), getLastNormal());
}
