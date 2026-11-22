/// @file
/// OneDrop3D : écran « Assistant de calibration » (demande du 22/11). Proposé au démarrage après l'installation de
/// OneDrop3D ou une mise à jour qui change une calibration (calib_registry_od3d.hpp), et ouvert depuis Réglages ›
/// Fonctions OneDrop3D. Un seul écran dont le contenu suit les étapes :
/// proposition (calibrations à faire, durée) → « Avez-vous un accéléromètre ? » → buse sous 40 °C (température en
/// direct, ventilateur) → « Fixez l'accéléromètre » (vérifié) → tests à l'accéléromètre en série → « Retirez
/// l'accéléromètre » → « Plateau vide et propre ? » → MPC et mesures par bobine en série → bilan (toucher un échec le
/// refait). Les séries tournent dans Marlin (M1989, calib_assistant_od3d.hpp) : l'écran peut être quitté, la pastille
/// de l'en-tête les montre et y ramène. L'état de l'assistant survit à l'écran.
#pragma once

#include "atelier_od3d.hpp"

#include <array>
#include <feature/od3d/calib_registry_od3d.hpp>
#include <i_window_menu_item.hpp>
#include <string_view_utf8.hpp>
#include <WindowMenuSwitch.hpp>

namespace od3d {

/// Au démarrage (guimain) : vrai s'il faut proposer l'assistant (il est alors prêt à s'ouvrir en proposition)
bool calibration_prompt_at_boot();

/// L'assistant attend l'utilisateur ou fait une série (pas la proposition ni le bilan) : la pastille le montre
bool calibration_assistant_active();

/// Nom court de ce que fait l'assistant (test en cours, sinon « Calibration »), pour la pastille
string_view_utf8 calibration_assistant_name();

/// Un test du registre : nom à gauche, état à droite (couleur selon le résultat)
class CalibTile final : public Button {
public:
    CalibTile(window_t *parent, Rect16 body, uint8_t tag);

    void show(const string_view_utf8 &name, const string_view_utf8 &status, Color status_color, bool alert, bool active);

protected:
    void unconditionalDraw() override;

private:
    string_view_utf8 status;
    Color status_color;
};

class ScreenCalibAssistant final : public Screen {
public:
    ScreenCalibAssistant();

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

    StringViewUtf8Parameters<40> info_params;
    uint32_t shown_key = 0xffffffff; ///< ce qui est affiché (étape, températures, avancement) : redessiné s'il change
    uint8_t buttons_stage = 0xff; ///< étape dont les boutons portent les libellés
    uint8_t focus_stage = 0xff; ///< étape dont le focus a été posé
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
