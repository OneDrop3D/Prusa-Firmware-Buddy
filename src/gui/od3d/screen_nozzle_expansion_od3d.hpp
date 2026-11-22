/// @file
/// OneDrop3D (M2003) : écran « Dilatation de la buse », ouvert depuis Réglages › Fonctions OneDrop3D › Température de la
/// buse, et par l'assistant de calibration pendant son étape. En direct : z du centre du plateau à chaque palier (aller
/// en orange, retour en bleu), la droite qui passe au mieux et celle de Prusa ; à gauche la pente, celle de Prusa et
/// l'écart à 250 °C. « Modèle Prusa » remet le modèle de Prusa pour la buse réglée (M2003 R).
#pragma once

#include "atelier_od3d.hpp"

#include <feature/od3d/nozzle_expansion_od3d.hpp>
#include <string_view_utf8.hpp>

namespace od3d {

/// Résultat d'une mesure (N_, à traduire) ; idle : ce que fait la mesure. Écran de la mesure et bilan de l'assistant.
const char *nozzle_expansion_result_text(nozzle_expansion::Status status);

/// Pente (mesurée en direct, ou celle utilisée), celle de Prusa, écart à 250 °C, retour − aller
class ExpansionPanel final : public window_t {
public:
    ExpansionPanel(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

/// Fond du graphique : titre, légende, températures (ne change pas)
class ExpansionChartBack final : public window_t {
public:
    ExpansionChartBack(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

/// Points des paliers et droites, redessinés d'un bloc
class ExpansionChart final : public window_t {
public:
    ExpansionChart(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

class ScreenNozzleExpansion final : public Screen {
public:
    ScreenNozzleExpansion();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();
    void show_info();
    void start();

    // Ordre de construction = ordre de la molette
    Header header;
    ExpansionPanel panel;
    ExpansionChartBack chart_back;
    ExpansionChart chart;
    window_text_t info;
    Button action;
    Button prusa_button;
    TabBar tabs;

    StringViewUtf8Parameters<32> info_params;

    bool pending = false;
    uint16_t runs_at_request = 0;
    uint16_t shown_revision = 0xffff;
    bool shown_running = false;
    bool shown_pending = false;
    nozzle_expansion::Status shown_status = nozzle_expansion::Status::idle;
    uint8_t shown_progress = 0xff;
    float shown_slope = 0.f; ///< pente utilisée : le panneau change après « Modèle Prusa »
};

} // namespace od3d
