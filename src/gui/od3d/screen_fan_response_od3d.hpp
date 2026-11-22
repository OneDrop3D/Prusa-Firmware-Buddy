/// @file
/// OneDrop3D (N3) : écran « Réponse du ventilateur », ouvert depuis Réglages › Fonctions OneDrop3D (et par
/// l'assistant de calibration). Régime du ventilateur pièce en direct pendant M2001 (créneaux de 50 ms), temps de
/// montée mesuré et avance du ventilateur (F26) appliquée (feature/od3d/fan_response_od3d.hpp).
#pragma once

#include "atelier_od3d.hpp"

#include <feature/od3d/fan_response_od3d.hpp>
#include <string_view_utf8.hpp>

namespace od3d {

/// Temps de montée en grand, avance appliquée et régime final
class FanResponsePanel final : public window_t {
public:
    FanResponsePanel(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

/// Fond du panneau de la courbe, titre et durée (ne change pas)
class FanResponseCurveBack final : public window_t {
public:
    FanResponseCurveBack(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

/// Régime par créneau, la commande et le seuil de 90 %, redessinés d'un bloc
class FanResponseCurve final : public window_t {
public:
    FanResponseCurve(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

class ScreenFanResponse final : public Screen {
public:
    ScreenFanResponse();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();
    void show_info();
    void start();

    // Ordre de construction = ordre de la molette
    Header header;
    FanResponsePanel panel;
    FanResponseCurveBack curve_back;
    FanResponseCurve curve;
    window_text_t info;
    Button action;
    TabBar tabs;

    StringViewUtf8Parameters<24> info_params;

    bool pending = false;
    uint16_t runs_at_request = 0;
    uint8_t shown_bins = 0xff;
    bool shown_running = false;
    bool shown_pending = false;
    fan_response::Status shown_status = fan_response::Status::idle;
    uint8_t shown_progress = 0xff;
};

} // namespace od3d
