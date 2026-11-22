#include "screen_menu.hpp"

#include <ScreenHandler.hpp>

void ScreenMenuBase_::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    // OneDrop3D (interface G) : appui long = retour, sauf sous un dialogue ; les écrans qui s'en servent déjà
    // (Contrôle, Matériel) le traitent avant d'arriver ici
    if (event == GUI_event_t::HELD_RELEASED && !GetFirstDialog()) {
        Screens::Access()->close_screen();
        return;
    }
    // OneDrop3D (interface G) : sans capture, molette, clic et toucher passent par la fenêtre qui a le focus. Un dialogue
    // qui l'avait pris (clavier numérique d'une valeur touchée, liste de choix, couleur, texte) le laisse à personne en se
    // fermant : plus rien ne répondait, jusqu'au redémarrage. Le focus revient alors à la liste.
    if ((event == GUI_event_t::KNOB || event == GUI_event_t::CLICK || event == GUI_event_t::TOUCH_CLICK) && !get_focused_child_window()) {
        i_menu->SetFocus();
    }
    screen_t::windowEvent(sender, event, param);
}

void ScreenMenuBase_::unconditionalDraw() {
    od3d::fill_background(*this, getFirstNormal(), getLastNormal());
}
