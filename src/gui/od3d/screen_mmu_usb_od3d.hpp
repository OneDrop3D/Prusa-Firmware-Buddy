/// @file
/// OneDrop3D : écran « Mise à jour USB du MMU » (feature/od3d/mmu_usb_od3d.hpp). « Préparer la mise à jour USB du MMU »
/// (Réglages › Fonctions OneDrop3D, Atelier › MMU › Réglages) envoie M709 S2 et l'ouvre : le MMU reste alimenté et
/// l'imprimante ne lui parle plus. « Terminer » envoie M709 X2 (coupure puis reprise, comme « Réinitialiser le MMU »
/// de Prusa) et affiche la version relue. Quitter l'écran garde le mode : la pastille de l'en-tête le rouvre.
/// « Mettre à jour le MMU par le port USB avant » (Atelier › MMU › Réglages) ouvre le même écran, guidé pour brancher le
/// micro-USB du MMU sur le port avant : l'imprimante y écrit et relit le firmware qu'elle contient
/// (feature/od3d/mmu_flash_od3d.hpp), après confirmation. Un MMU que l'on ne voit pas s'installe aussi : « Installer »,
/// puis son bouton reset quand l'écran le demande (mode de secours).
#pragma once

#include "atelier_od3d.hpp"
#include "widgets_od3d.hpp"

#include <array>
#include <window_progress.hpp>
#include <WindowMenuItems.hpp>

namespace od3d {

/// « Préparer la mise à jour USB du MMU » : refusé pendant une impression ou MMU désactivé
class MI_MMU_USB final : public IWindowMenuItem {
public:
    MI_MMU_USB();
    void click(IWindowMenu &) override;
};

/// « Mettre à jour le MMU par le port USB avant » : le même mode, l'écran guide le branchement sur le port avant
class MI_MMU_USB_FRONT final : public IWindowMenuItem {
public:
    MI_MMU_USB_FRONT();
    void click(IWindowMenu &) override;
};

class ScreenMmuUsb final : public Screen {
public:
    ScreenMmuUsb();

    /// Posé par MI_MMU_USB_FRONT juste avant d'ouvrir l'écran
    static inline bool open_front = false;

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();
    /// Le texte d'état du port avant : branchement, écriture, résultat
    void front_state(std::array<char, 160> &text, bool &busy);
    void install();

    const bool front;
    Header header;
    window_text_t info;
    window_text_t state;
    WindowRoundedProgressBar bar;
    Button install_button;
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
    std::array<char, 160> state_text {};
    std::array<char, 16> version_text {}; ///< firmware contenu dans l'imprimante, « 3.0.4+940 »
    bool shown = false; ///< premier update() : texte et bouton posés
    bool shown_waiting = true;
    bool shown_bar = false;
    bool shown_retry = false;
};

} // namespace od3d
