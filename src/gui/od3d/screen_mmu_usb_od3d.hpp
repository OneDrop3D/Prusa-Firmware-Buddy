/// @file
/// OneDrop3D : écran « Mise à jour USB du MMU » (feature/od3d/mmu_usb_od3d.hpp). « Préparer la mise à jour USB du MMU »
/// (Réglages › Fonctions OneDrop3D, Atelier › MMU › Réglages) envoie M709 S2 et l'ouvre : le MMU reste alimenté et
/// l'imprimante ne lui parle plus. « Terminer » envoie M709 X2 (coupure puis reprise, comme « Réinitialiser le MMU »
/// de Prusa) et affiche la version relue. Quitter l'écran garde le mode : la pastille de l'en-tête le rouvre.
#pragma once

#include "atelier_od3d.hpp"
#include "widgets_od3d.hpp"

#include <array>
#include <WindowMenuItems.hpp>

namespace od3d {

/// « Préparer la mise à jour USB du MMU » : refusé pendant une impression ou MMU désactivé
class MI_MMU_USB final : public IWindowMenuItem {
public:
    MI_MMU_USB();
    void click(IWindowMenu &) override;
};

class ScreenMmuUsb final : public Screen {
public:
    ScreenMmuUsb();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();

    Header header;
    window_text_t info;
    window_text_t state;
    Button action;
    TabBar tabs;

    /// G-code envoyé, pas encore exécuté par Marlin : l'état affiché l'attend (3 s au plus)
    enum class Pending : uint8_t {
        none,
        start, ///< M709 S2
        finish, ///< M709 X2
    };
    Pending pending = Pending::none;
    uint32_t pending_ms = 0;
    std::array<char, 48> state_text {};
    bool shown = false; ///< premier update() : texte et bouton posés
    bool shown_waiting = true;
};

} // namespace od3d
