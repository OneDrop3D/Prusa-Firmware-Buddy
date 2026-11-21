/// @file
/// OneDrop3D (F5, F6) : écran « Emplacement n · Débit, temp. », frère de l'écran PA (ouvert par son bouton
/// « Débit, temp. »). Débit maximal et température minimale mesurés, courbe de force de la dernière mesure
/// (force/débit avec la loi de référence et le coude, ou force/température avec le seuil), « Débit max » (M1995),
/// « Temp. min » (M1996), « Annuler » / « Effacer », réglage « limiter au débit mesuré » et courbes sur la clé.
#pragma once

#include "atelier_od3d.hpp"

#include <array>
#include <feature/od3d/spool_calib_od3d.hpp>

namespace od3d {

/// Ouvre l'écran des calibrations de la bobine de l'emplacement (0 à 4)
void open_spool_calib(uint8_t slot);

/// Carte de gauche : débit maximal, température minimale, limite
class SpoolCalibCard : public window_t {
public:
    SpoolCalibCard(window_t *parent, Rect16 rect, uint8_t slot);

protected:
    void unconditionalDraw() override;

private:
    uint8_t slot;
};

/// Courbe de force de la dernière mesure de l'emplacement
class SpoolCalibGraph : public window_t {
public:
    SpoolCalibGraph(window_t *parent, Rect16 rect, uint8_t slot);

protected:
    void unconditionalDraw() override;

private:
    uint8_t slot;
};

class ScreenSpoolCalib final : public Screen {
public:
    explicit ScreenSpoolCalib(uint8_t slot);

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update(); ///< état de la mesure, boutons actifs, texte
    void show_toggles();
    void show_note();
    void measure(calib::Kind kind);

    uint8_t slot;
    StringViewUtf8Parameters<4> title_params; ///< avant l'en-tête, qui garde le titre formaté
    Header header;
    SpoolCalibCard card;
    SpoolCalibGraph graph;
    window_text_t note;
    Button limit;
    Button csv;
    Button flow_button;
    Button temp_button;
    Button second; ///< « Annuler » pendant une mesure, « Effacer » sinon
    TabBar tabs;

    std::array<char, 64> note_text {};
    bool pending = false; ///< M1995 / M1996 envoyé, pas encore commencé
    uint16_t runs_at_request = 0;
    bool shown_running = false;
    bool shown_pending = false;
    bool second_cancels = false; ///< « second » montre « Annuler »
    bool shown_pa_running = false; ///< une mesure du PA occupe l'imprimante
    calib::Status shown {};
    uint16_t shown_runs = 0;
};

} // namespace od3d
