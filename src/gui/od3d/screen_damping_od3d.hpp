/// @file
/// OneDrop3D (F33) : écran « Amortissement des axes », ouvert depuis Réglages › Fonctions OneDrop3D.
/// Pour X et Y : fréquence du pic, amortissement ζ (« zeta ») et facteur Q mesurés par M1997
/// (feature/od3d/damping_od3d.hpp), courbes de résonance en direct, et l'amortissement actuel de l'input shaper.
/// « Appliquer » écrit ζ mesuré dans la configuration de l'input shaper de Prusa (comme son menu : config_store puis
/// M9200) ; « Valeur Prusa » remet sa valeur d'origine (0,1). Rien n'est changé sans l'un de ces deux boutons.
#pragma once

#include "accelerometer_guide_od3d.hpp"
#include "atelier_od3d.hpp"

#include <array>
#include <feature/od3d/damping_od3d.hpp>
#include <string_view_utf8.hpp>

namespace od3d {

/// Résultat d'un axe : nom, fréquence du pic, ζ en grand, Q et largeur à −3 dB
class DampingPanel final : public window_t {
public:
    DampingPanel(window_t *parent, Rect16 rect, uint8_t axis);

protected:
    void unconditionalDraw() override;

private:
    uint8_t axis;
};

/// Fond du panneau des courbes, titre, légende et plage balayée (ne change pas)
class DampingCurvesBack final : public window_t {
public:
    DampingCurvesBack(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

/// Courbes de résonance de X et Y, redessinées d'un bloc (sans clignotement) sur le panneau
class DampingCurves final : public window_t {
public:
    DampingCurves(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

class ScreenDamping final : public Screen {
public:
    ScreenDamping();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();
    void show_info();
    void start();
    void apply(bool measured); ///< ζ mesurés (axes valables) ou valeur de Prusa sur X et Y

    // Ordre de construction = ordre de la molette
    Header header;
    DampingPanel panel_x;
    DampingPanel panel_y;
    DampingCurvesBack curves_back;
    DampingCurves curves;
    window_text_t info;
    Button action;
    Button apply_button;
    Button prusa_button;
    TabBar tabs;

    StringViewUtf8Parameters<32> info_params;

    bool pending = false;
    uint16_t runs_at_request = 0;
    uint16_t shown_revision = 0xffff;
    bool shown_running = false;
    bool shown_pending = false;
    damping::Status shown_status = damping::Status::idle;
    uint8_t shown_axis = 0;
    uint8_t shown_progress = 0xff;
    bool shown_calibrating = false;
    bool warned_accelerometer = true; ///< ACC-1 : guide déjà montré, ou rien demandé depuis cet écran
    const char *shown_homing = nullptr; ///< accelerometer_homing_text() affiché
    AccelerometerMotorHold motor_hold; ///< moteurs tenus tant que l'écran est ouvert (jamais d'origine accéléromètre fixé)
};

} // namespace od3d
