/// @file
/// OneDrop3D (F4) : écran « Emplacement n · PA », ouvert par la puce « PA » de la fiche d'une bobine. Valeur mesurée
/// et confiance, courbe de force de la dernière mesure (forme moyenne d'un créneau au K le plus bas, au meilleur K et
/// au K le plus haut, avec la commande), « Mesurer » (M1992, feature/od3d/pa_od3d.hpp) et « Annuler », « Effacer »,
/// réglage général « le PA de l'imprimante prime » et export des courbes sur la clé (CSV).
#pragma once

#include "atelier_od3d.hpp"

#include <array>
#include <feature/od3d/pa_od3d.hpp>

namespace od3d {

/// Ouvre l'écran PA de l'emplacement (0 à 4)
void open_pa(uint8_t slot);

/// Carte de gauche : K, confiance, matière et température
class PaValueCard : public window_t {
public:
    PaValueCard(window_t *parent, Rect16 rect, uint8_t slot);

protected:
    void unconditionalDraw() override;

private:
    uint8_t slot;
};

/// Courbe de force de la dernière mesure de l'emplacement
class PaGraph : public window_t {
public:
    PaGraph(window_t *parent, Rect16 rect, uint8_t slot);

protected:
    void unconditionalDraw() override;

private:
    uint8_t slot;
};

class ScreenPa final : public Screen {
public:
    explicit ScreenPa(uint8_t slot);

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update(); ///< état de la mesure, boutons actifs, texte
    void show_toggles();
    void show_note();
    void measure();

    uint8_t slot;
    StringViewUtf8Parameters<4> title_params; ///< avant l'en-tête, qui garde le titre formaté
    Header header;
    PaValueCard card;
    PaGraph graph;
    window_text_t note;
    Button wins;
    Button csv;
    Button measure_button;
    Button second; ///< « Annuler » pendant la mesure, « Effacer » sinon
    TabBar tabs;

    std::array<char, 64> note_text {};
    bool pending = false; ///< M1992 envoyé, pas encore commencé
    uint16_t runs_at_request = 0;
    bool shown_running = false;
    bool shown_pending = false;
    bool second_cancels = false; ///< « second » montre « Annuler »
    pa::Status shown {};
    uint16_t shown_runs = 0;
};

} // namespace od3d
