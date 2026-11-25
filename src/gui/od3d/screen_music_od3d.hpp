/// @file
/// OneDrop3D (F18) : Contrôle › « Musique des moteurs ». Fichier .mid de la clé (racine), moteurs (X et Y, X seul,
/// Z), Lire / Arrêter, et « jouer après chaque impression ». La lecture est faite par M1991
/// (feature/od3d/music_player_od3d.hpp) ; l'écran ne fait que la lancer, l'arrêter et montrer son état.
#pragma once

#include "atelier_od3d.hpp"
#include "widgets_od3d.hpp" // Panel

#include <array>
#include <feature/od3d/music_player_od3d.hpp>

namespace od3d {

class ScreenMusic final : public Screen {
public:
    ScreenMusic();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void scan(); ///< relit la liste des .mid de la clé et retrouve le fichier choisi
    void select(int index); ///< montre et retient le fichier n° index
    void update(); ///< état de la lecture, boutons actifs
    void show_status();
    void show_motors();
    void show_after_print();
    void play();

    // Ordre de construction = ordre de la molette
    Header header;
    Panel file_panel;
    window_text_t file_note;
    Button previous;
    window_text_t file_name;
    Button next;
    Panel motors_panel;
    Button motors[music::motors_count];
    Panel play_panel;
    window_text_t status_note;
    Button play_button;
    Button stop_button;
    Button after_print;
    TabBar tabs;

    std::array<char, 40> name {}; ///< nom long affiché
    std::array<char, 13> short_name {}; ///< nom court 8.3, retenu dans le config_store
    std::array<char, 12> count_text {};
    int16_t index = -1; ///< fichier montré, -1 aucun
    int16_t count = 0; ///< fichiers .mid à la racine de la clé
    bool media = false; ///< clé présente au dernier passage
    bool pending = false; ///< M1991 envoyé, pas encore commencé
    uint16_t runs_at_request = 0;
    bool shown_playing = false;
    music::Status shown_status = music::Status::idle;
    bool shown_pending = false;
};

} // namespace od3d
