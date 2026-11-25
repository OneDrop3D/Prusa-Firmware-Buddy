#include "screen_menu.hpp"

#include <ScreenHandler.hpp>
#include <i_window_menu_item.hpp>
#include <sound.hpp>

void ScreenMenuBase_::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    // OneDrop3D (interface G) : appui long = retour, sauf sous un dialogue ou sans retour (1er démarrage : langue,
    // configuration de l'imprimante) ; les écrans qui s'en servent déjà (Contrôle, Matériel) le traitent avant d'arriver ici
    if (event == GUI_event_t::HELD_RELEASED && !GetFirstDialog() && header.back_shown()) {
        Screens::Access()->close_screen();
        return;
    }
    // Balayage = retour, comme l'appui long, dans toute liste qui a un retour, impression comprise : la règle générale de
    // ScreenHandler ne ferme que ce que Prusa ferme au délai des menus, et Ajuster ne l'est pas (ClrMenuTimeoutClose)
    if ((event == GUI_event_t::TOUCH_SWIPE_LEFT || event == GUI_event_t::TOUCH_SWIPE_RIGHT) && !GetFirstDialog() && header.back_shown()
        && !IWindowMenuItem::edited_item()) {
        sound::play(SoundType::button_echo);
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

void ScreenMenuBase_::draw() {
    if (!od3d::draw_whole(*this, getFirstNormal(), getLastNormal())) {
        screen_t::draw();
    }
}

void ScreenMenuBase_::unconditionalDraw() {
    od3d::fill_background(*this, getFirstNormal(), getLastNormal());
}
