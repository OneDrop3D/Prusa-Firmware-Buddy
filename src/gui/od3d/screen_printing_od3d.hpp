/// @file
/// OneDrop3D (F1, interface G) : écran « Impression » (SPEC_G.md §3.12). DialogHandler l'ouvre à la place de
/// screen_printing_data_t (FSM Printing) ; il en reprend la logique : états de l'impression, pause, reprise, saut,
/// arrêt avec la confirmation de Prusa, fin (réimprimer, accueil), « Set Ready » de Connect, messages d'état,
/// compteur. Les erreurs, la reprise après coupure et les collisions restent les écrans et dialogues de Prusa.
#pragma once

#include "thumbnail_od3d.hpp"
#include "widgets_od3d.hpp"

#include <array>
#include <optional>

namespace od3d {

/// Une impression est ouverte, de son départ jusqu'à « Accueil » après la fin (FSM Printing)
bool print_active();

/// Onglet Imprimer : l'écran Impression pendant une impression, sinon les fichiers (la pile redevient [accueil, écran])
void open_print_tab();

class ScreenPrinting;

/// Bouton de la rangée du bas (88 × 62) : légende et valeur (Vitesse, Live Z), ou icône / symbole et libellé.
/// Inactif, il reste ciblable (comme les boutons grisés de Prusa) mais son clic ne fait rien.
class PrintingButton : public Card {
public:
    using Handler = void (ScreenPrinting::*)();

    PrintingButton(ScreenPrinting &screen, Rect16 body, Handler handler);

    /// Icône ou symbole en haut, libellé en bas ; danger : contour rouge et carré d'« Arrêter » sans icône
    void show(const img::Resource *icon, const char *glyph, const string_view_utf8 &label, bool active, bool danger = false);

    /// Légende grisée en haut, valeur en gras en bas
    void show_value(const string_view_utf8 &caption, const char *value, bool active);

    bool active() const { return face.active; }

protected:
    void unconditionalDraw() override;
    void draw_content(Rect16 body) override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    struct Face {
        const img::Resource *icon = nullptr;
        const char *glyph = nullptr;
        string_view_utf8 text; ///< libellé, ou légende au-dessus de value
        std::array<char, 12> value {}; ///< vide : pas de valeur
        bool active = true;
        bool danger = false;

        bool same(const Face &o) const;
    };
    void set(const Face &f);

    ScreenPrinting &screen;
    Handler handler;
    Face face;
};

/// Aperçu de l'impression : vignette grisée au-dessus de la hauteur imprimée, trait orange à la limite
class PrintThumb : public window_t {
public:
    PrintThumb(ScreenPrinting &screen, Rect16 rect);

    void set_progress(uint8_t percent);

protected:
    void unconditionalDraw() override;

private:
    ScreenPrinting &screen;
    uint16_t grey_rows;
    int16_t line_y = 0; ///< trait orange dessiné (0 : aucun), effacé quand il descend
};

/// Colonne de droite, en deux fenêtres pour que chacune ne se redessine que si sa partie change :
/// progress = « 47 % » et l'état (« fin 21:48 ») ; spool = bobine en cours et message d'état ou suite.
/// La ligne « Reste 1 h 12 · Z 12.4 mm », qui change à chaque couche, est un window_text_t entre les deux.
class PrintInfo : public window_t {
public:
    enum class Part : uint8_t {
        progress,
        spool,
    };

    struct Data {
        uint8_t percent = 0;
        std::array<char, 24> state {}; ///< « fin 21:48 », « en pause »...
        uint8_t slot = 0xff; ///< emplacement chargé, 0xff : aucun
        spool::Look look;
        std::array<char, 8> material {};
        std::array<char, 72> bottom {}; ///< message d'état (bottom_strong), sinon prochain changement ou nom du fichier
        bool bottom_strong = false;
    };

    PrintInfo(window_t *parent, Rect16 rect, Part part);

    void set(const Data &d);

protected:
    void unconditionalDraw() override;

private:
    Data data;
    Part part;
};

/// Frise de progression : piste, partie imprimée en orange, curseur blanc
class PrintStrip : public window_t {
public:
    PrintStrip(window_t *parent, Rect16 rect);

    void set_progress(uint8_t percent);

protected:
    void unconditionalDraw() override;

private:
    uint8_t percent = 0;
};

class ScreenPrinting final : public Screen {
public:
    ScreenPrinting();

    void on_speed();
    void on_live_z();
    void on_tune();
    void on_pause();
    void on_stop();

    std::optional<ScaledQoiReader> scaler; ///< dans l'écran plutôt que sur la pile de l'interface (6 Ko)

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    /// États de screen_printing_data_t (printing_state_t)
    enum class PrintState : uint8_t {
        initial,
        printing,
        skippable,
        pausing,
        paused,
        resuming,
        aborting,
        reheating,
        stopped,
        printed,
    };

    void update();
    void update_state();
    void update_buttons();
    void update_info();
    void reprint();

    Header header;
    PrintThumb thumb;
    PrintInfo progress;
    window_text_t line;
    PrintInfo slot_info;
    PrintStrip strip;
    PrintingButton speed;
    PrintingButton live_z;
    PrintingButton tune;
    PrintingButton pause;
    PrintingButton stop;
    TabBar tabs;

    std::array<char, 40> line_text {};
    PrintState state = PrintState::initial;
    bool stop_pressed = false;
    bool waiting_for_abort = false; ///< arrêt demandé pendant le palpage (G29) : renvoyé quand il se termine
};

} // namespace od3d
