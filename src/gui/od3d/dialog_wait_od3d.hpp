/// @file
/// OneDrop3D (interface G, UI-10) : le « Patientez » de la machine à états Wait de Prusa (origine G28, nettoyage de la
/// buse et palpage G29), qui couvrait tout l'écran sous l'en-tête (retour de l'utilisateur, 22/11). « Continuer en
/// arrière-plan » ou la flèche en haut à gauche le réduisent dans la pastille de l'en-tête (island_od3d.hpp) : la
/// machine continue, toucher la pastille le rouvre. Pendant nos processus (mesure du PA...), il ne s'ouvre pas du tout
/// (DialogHandler.cpp) : la pastille montre le processus, son écran montre l'étape.
#pragma once

#include "atelier_od3d.hpp" // Button

#include <window_dlg_wait.hpp>

namespace od3d {

class DialogWait final : public window_dlg_wait_t {
public:
    explicit DialogWait(fsm::BaseData data);

    /// Tout l'écran : la flèche de l'en-tête est au-dessus du cadre
    Rect16 get_rect_for_touch() const override;

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    Button background; ///< « Continuer en arrière-plan »
};

} // namespace od3d
