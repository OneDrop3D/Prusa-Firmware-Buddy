/// @file
/// OneDrop3D (F22) : écran « Vitesses critiques », ouvert depuis Réglages › Fonctions OneDrop3D.
/// Pour X et Y, une barre par vitesse (20 à 80 mm/s) : énergie vibratoire mesurée à vitesse constante, en orange les
/// vitesses à éviter, listées au-dessus ; Démarrer / Annuler. La mesure est faite par M1993
/// (feature/od3d/speed_map_od3d.hpp) ; l'écran la lance, l'annule et montre ses résultats.
#pragma once

#include "accelerometer_guide_od3d.hpp"
#include "atelier_od3d.hpp"

#include <array>
#include <feature/od3d/speed_map_od3d.hpp>
#include <string_view_utf8.hpp>

namespace od3d {

/// Panneau d'un axe : fond et nom ; la liste et les barres sont des fenêtres à part (redessinées seules)
class SpeedPanel final : public window_t {
public:
    SpeedPanel(window_t *parent, Rect16 rect, const string_view_utf8 &title);

protected:
    void unconditionalDraw() override;

private:
    string_view_utf8 title;
};

/// Barres d'un axe, en une image : grises, orange pour les vitesses critiques
class SpeedBars final : public window_t {
public:
    SpeedBars(window_t *parent, Rect16 rect, uint8_t axis);

protected:
    void unconditionalDraw() override;

private:
    uint8_t axis;
};

class ScreenSpeedMap final : public Screen {
public:
    ScreenSpeedMap();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();
    void show_info();
    void show_notes(); ///< vitesses à éviter, au-dessus des barres
    void start();

    Header header;
    SpeedPanel panel_x;
    window_text_t note_x;
    SpeedBars bars_x;
    SpeedPanel panel_y;
    window_text_t note_y;
    SpeedBars bars_y;
    window_text_t info;
    Button action;
    TabBar tabs;

    StringViewUtf8Parameters<12> info_params;
    std::array<StringViewUtf8Parameters<40>, speed_map::axis_count> note_params;

    bool pending = false;
    uint16_t runs_at_request = 0;
    uint16_t shown_revision = 0xffff;
    bool shown_running = false;
    bool shown_pending = false;
    speed_map::Status shown_status = speed_map::Status::idle;
    uint8_t shown_axis = 0;
    uint8_t shown_progress = 0xff;
    bool warned_accelerometer = true; ///< ACC-1 : guide déjà montré, ou rien demandé depuis cet écran
    const char *shown_homing = nullptr; ///< accelerometer_homing_text() affiché
    AccelerometerMotorHold motor_hold; ///< moteurs tenus tant que l'écran est ouvert (jamais d'origine accéléromètre fixé)
};

} // namespace od3d
