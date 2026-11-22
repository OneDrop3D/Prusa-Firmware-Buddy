/// @file
/// OneDrop3D (F4) : écran « Emplacement n · PA », ouvert par la puce « PA » de la fiche d'une bobine. Valeur mesurée
/// (ou choisie) et confiance, courbe de force de la dernière mesure (forme moyenne d'un créneau au K le plus bas, au
/// meilleur K et au K le plus haut, avec la commande), « Mesurer » (M1994 : motif imprimé à l'avant du plateau,
/// feature/od3d/pa_od3d.hpp) et « Annuler », « Effacer », réglage général « le PA de l'imprimante prime » et
/// « Meilleure ligne » : repli visuel, l'utilisateur choisit lui-même la ligne la plus régulière du motif (écran
/// ScreenPaLines). « Débit, temp. » ouvre l'écran frère des autres calibrations de la bobine (F5, F6 :
/// screen_spool_calib_od3d.hpp), où se règle aussi l'export des courbes sur la clé (commun).
#pragma once

#include "atelier_od3d.hpp"

#include <array>
#include <feature/od3d/pa_od3d.hpp>

namespace od3d {

/// Ouvre l'écran PA de l'emplacement (0 à 4)
void open_pa(uint8_t slot);

/// Courbes des mesures exportées sur la clé (F4, F5, F6) : choix commun, gardé jusqu'au redémarrage
bool usb_curves();
void set_usb_curves(bool set);

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
    Button lines; ///< « Meilleure ligne » : choix à l'œil sur le dernier motif imprimé
    Button measure_button;
    Button second; ///< « Annuler » pendant la mesure, « Effacer » sinon
    Button calib; ///< « Débit, temp. » : écran des autres calibrations de la bobine (F5, F6)
    TabBar tabs;

    std::array<char, 64> note_text {};
    bool pending = false; ///< M1994 envoyé, pas encore commencé
    uint16_t runs_at_request = 0;
    bool shown_running = false;
    bool shown_pending = false;
    bool second_cancels = false; ///< « second » montre « Annuler »
    pa::Status shown {};
    uint16_t shown_runs = 0;
};

/// « Meilleure ligne » : les lignes du dernier motif de l'emplacement dans leur ordre sur le plateau (1 à l'avant), avec
/// leur K ; toucher (ou cliquer) une ligne enregistre son K pour l'emplacement après confirmation, comme le test PA
/// d'Orca. La ligne du K enregistré est en surbrillance.
class ScreenPaLines final : public Screen {
public:
    explicit ScreenPaLines(uint8_t slot);

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    /// Bouton d'une ligne, placé d'après son numéro (colonnes de 5) ; garde son libellé
    class LineButton : public Button {
    public:
        LineButton(window_t *parent, uint8_t index);
        std::array<char, 20> text {};
    };

    void choose(uint8_t index);

    uint8_t slot;
    StringViewUtf8Parameters<4> title_params;
    Header header;
    window_text_t help;
    LineButton buttons[pa::max_lines];
    TabBar tabs;
};

} // namespace od3d
