/// @file
/// OneDrop3D : écran « Assistant de calibration » (demande du 22/11), ouvert depuis Réglages › Fonctions OneDrop3D
/// (jamais proposé au démarrage, demande du 23/11). Un seul écran dont le contenu suit les étapes :
/// proposition (calibrations à faire, durée) → « Avez-vous un accéléromètre ? » → buse sous 40 °C (température en
/// direct, ventilateur) → « Fixez l'accéléromètre » (vérifié) → tests à l'accéléromètre en série → « Retirez
/// l'accéléromètre » → « Plateau vide et propre ? » → MPC et mesures par bobine en série → bilan (toucher un échec le
/// refait). Les séries tournent dans Marlin (M1989, calib_assistant_od3d.hpp) : l'écran peut être quitté, la pastille
/// de l'en-tête les montre et y ramène. L'état de l'assistant survit à l'écran.
#pragma once

#include "atelier_od3d.hpp"

#include <array>
#include <optional>
#include <feature/od3d/calib_registry_od3d.hpp>
#include <i_window_menu_item.hpp>
#include <string_view_utf8.hpp>
#include <WindowMenuSwitch.hpp>

namespace od3d {

/// L'assistant attend l'utilisateur ou fait une série (pas la proposition ni le bilan) : la pastille le montre
bool calibration_assistant_active();

/// Nom court de ce que fait l'assistant (test en cours, sinon « Calibration »), pour la pastille
string_view_utf8 calibration_assistant_name();

/// Une série de l'assistant tourne : les assistants de Prusa qu'elle lance (input shaper, phase stepping, sans question)
/// ne prennent pas l'écran (DialogHandler) ; l'écran de l'assistant et la pastille les montrent
bool calibration_series_running();

/// À chaque tour de nos écrans : si l'utilisateur suit la série (l'assistant, ou l'écran de l'étape d'avant, est à
/// l'écran), l'écran de la mesure qui commence s'ouvre (courbes en direct) ; retour à l'assistant pour les étapes sans
/// écran à nous et à la fin de la série
void calibration_follow();

/// Avant d'arrêter une mesure depuis son écran : pendant une série, demande confirmation (l'assistant passe à la
/// suivante) ; vrai hors série
bool confirm_stop_measure();

/// Un test du registre : nom à gauche, pastille d'état à droite (orange à faire, vert réussi ou fait à la main,
/// rouge échec avec le contour d'alerte, jaune passé ou à faire à la main, gris à jour ; sans pastille : sans objet) ;
/// le détail (mot et chiffres) s'ouvre au toucher dans le bilan
class CalibTile final : public Button {
public:
    CalibTile(window_t *parent, Rect16 body, uint8_t tag);

    void show(const string_view_utf8 &name, std::optional<Color> dot, Look look, bool active);

protected:
    void unconditionalDraw() override;

private:
    std::optional<Color> dot;
};

class ScreenCalibAssistant final : public Screen {
public:
    ScreenCalibAssistant();
    ~ScreenCalibAssistant();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update(); ///< passe d'une étape à l'autre, puis redessine ce qui a changé
    void show(); ///< texte, tuiles et boutons de l'étape
    void clicked(uint8_t tag);

    // Ordre de construction = ordre de la molette
    Header header;
    window_text_t info;
    std::array<CalibTile, calib_registry::entries.size()> tiles;
    Button button_a;
    Button button_b;
    Button button_c;
    TabBar tabs;

    StringViewUtf8Parameters<64> info_params;
    uint32_t shown_key = 0xffffffff; ///< ce qui est affiché (étape, températures, avancement) : redessiné s'il change
    uint8_t buttons_stage = 0xff; ///< étape dont les boutons portent les libellés
    uint8_t focus_stage = 0xff; ///< étape dont le focus a été posé
    bool motors_held = false; ///< od3d::sleep::hold_motors : de la buse qui refroidit à la fin des tests à l'accéléromètre
};

/// Réglages › Fonctions OneDrop3D › « Assistant de calibration »
class MI_CALIB_ASSISTANT final : public IWindowMenuItem {
public:
    MI_CALIB_ASSISTANT();
    void click(IWindowMenu &) override;
};

/// Réglages › Fonctions OneDrop3D › « Accéléromètre disponible » : Non / Oui (jamais répondu : Non, l'assistant demande)
class MI_ACCELEROMETER final : public MenuItemSwitch {
public:
    MI_ACCELEROMETER();

protected:
    bool on_item_selected(const OnItemSelectedArgs &args) override;
};

} // namespace od3d
