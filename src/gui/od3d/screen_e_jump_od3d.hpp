/// @file
/// OneDrop3D (M2000) : écran « Emplacement n · Accél. fil. », ouvert depuis Réglages › Fonctions OneDrop3D (sur
/// l'emplacement chargé), par l'assistant de calibration et par la pastille. Saut sûr gardé, accélération max du
/// filament qu'il donne avec le K de l'emplacement, accélération de ligne permise (estimée) ; barres des paliers en
/// direct (vert : sûr, jaune : douteux, rouge : calé), « Démarrer » / « Annuler », « Effacer ».
#pragma once

#include "atelier_od3d.hpp"

#include <array>
#include <feature/od3d/e_jump_od3d.hpp>

namespace od3d {

/// Carte de gauche : saut sûr, F, ligne permise
class EJumpCard final : public window_t {
public:
    EJumpCard(window_t *parent, Rect16 rect, uint8_t slot);

protected:
    void unconditionalDraw() override;

private:
    uint8_t slot;
};

/// Barres des paliers de la dernière mesure de l'emplacement (pendant la mesure : les paliers faits)
class EJumpSteps final : public window_t {
public:
    EJumpSteps(window_t *parent, Rect16 rect, uint8_t slot);

protected:
    void unconditionalDraw() override;

private:
    uint8_t slot;
};

class ScreenEJump final : public Screen {
public:
    explicit ScreenEJump(uint8_t slot);

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();
    void show_note();
    void start();

    uint8_t slot;
    StringViewUtf8Parameters<4> title_params; ///< avant l'en-tête, qui garde le titre formaté
    Header header;
    EJumpCard card;
    EJumpSteps steps;
    window_text_t note;
    Button action;
    Button clear;
    TabBar tabs;

    std::array<char, 160> note_text {};
    bool pending = false; ///< M2000 envoyé, pas encore commencé
    uint16_t runs_at_request = 0;
    bool shown_running = false;
    bool shown_pending = false;
    bool action_cancels = false;
    e_jump::Status shown {};
    uint16_t shown_runs = 0;
    uint16_t shown_revision = 0xffff;
};

} // namespace od3d
