/// @file
/// OneDrop3D (F1, interface G) : l'accueil. Étape 1 : en-tête, trois cartes et la barre d'onglets.
#pragma once

#include <screen.hpp>
#include <window_header.hpp>
#include <window_text.hpp>

namespace od3d {

class ScreenHome final : public screen_t {
public:
    ScreenHome();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    window_header_t header;
    WindowButton cards[3]; ///< Fichiers, Températures, Contrôle
    WindowButton tabs[4]; ///< Accueil, Imprimer, Atelier, Réglages
};

} // namespace od3d
