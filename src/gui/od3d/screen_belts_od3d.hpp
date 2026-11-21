/// @file
/// OneDrop3D (F14) : écran « Tension des courroies », ouvert depuis Réglages › Fonctions OneDrop3D.
/// Deux jauges (courroie A = moteur A / pilote X, courroie B = moteur B / pilote Y) avec la fréquence mesurée et la
/// plage de Prusa, le spectre de la dernière mesure (courbe en direct pendant la mesure), un conseil de réglage
/// (formule de l'assistant de Prusa) et « Démarrer » / « Annuler ». La mesure est faite par M1992
/// (feature/od3d/belt_tension_od3d.hpp) ; l'écran la lance, l'annule et montre ses résultats.
#pragma once

#include "atelier_od3d.hpp"

#include <array>
#include <feature/od3d/belt_tension_od3d.hpp>
#include <string_view_utf8.hpp>

namespace od3d {

/// Jauge d'une courroie : fond de panneau, nom, état, fréquence en grand, barre avec la plage de Prusa et un repère
class BeltGauge final : public window_t {
public:
    BeltGauge(window_t *parent, Rect16 rect, const string_view_utf8 &title);

    /// Fréquence mesurée (NAN : pas de mesure) ; found = pic net. Redessine si cela change.
    void set(float frequency, bool found);

protected:
    void unconditionalDraw() override;

private:
    string_view_utf8 title;
    float frequency = NAN;
    bool found = false;
};

/// Panneau du spectre : fond, titre, légende et bornes ; la courbe est une fenêtre à part (redessinée seule)
class SpectrumPanel final : public window_t {
public:
    SpectrumPanel(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

/// Courbes A et B du spectre, en une image (sans clignotement), bande de la plage de Prusa en fond
class SpectrumCurve final : public window_t {
public:
    SpectrumCurve(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

class ScreenBelts final : public Screen {
public:
    ScreenBelts();
    ~ScreenBelts();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update(); ///< état de la mesure, jauges, courbe, texte, bouton
    void show_info();
    void start();

    // Ordre de construction = ordre de la molette (seul le bouton est actif, avec le retour et les onglets)
    Header header;
    BeltGauge gauge_a;
    BeltGauge gauge_b;
    SpectrumPanel spectrum_panel;
    SpectrumCurve curve;
    window_text_t info;
    Button action;
    TabBar tabs;

    StringViewUtf8Parameters<40> info_params;
    std::array<char, 24> word {}; ///< « serrer » / « desserrer » traduit, paramètre du conseil
    std::array<char, 12> turns {}; ///< « 1 3/8 »

    bool pending = false; ///< M1992 envoyé, pas encore commencé
    uint16_t runs_at_request = 0;
    uint16_t shown_revision = 0xffff;
    bool shown_running = false;
    bool shown_pending = false;
    belts::Status shown_status = belts::Status::idle;
    belts::Stage shown_stage = belts::Stage::preparing;
    uint8_t shown_progress = 0xff;
    bool warned_accelerometer = false; ///< message de Prusa déjà montré pour cette mesure
};

} // namespace od3d
