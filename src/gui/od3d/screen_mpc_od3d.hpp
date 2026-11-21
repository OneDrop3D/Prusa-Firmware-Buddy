/// @file
/// OneDrop3D (F7) : régulation de la buse dans « Fonctions OneDrop3D › Température de la buse » (SPEC_G.md §3.23)
/// — choix PID / MPC (PID par défaut, MPC grisé sans calibration valable, bascule refusée pendant une impression) —
/// et l'écran de progression de la calibration du modèle (§1.9, `mpc` : Refroidissement, Chauffe libre, Mesure des
/// pertes (ventilateur), Calcul du modèle), avec « Annuler ». La calibration est faite par M306 T
/// (feature/od3d/mpc_control_od3d.hpp) ; l'écran ne fait que la lancer, l'annuler et montrer son état.
#pragma once

#include "atelier_od3d.hpp"
#include "screen_mmu_od3d.hpp" // Panel

#include <array>
#include <cstdint>
#include <feature/od3d/mpc_control_od3d.hpp>
#include <gui/menu_item/menu_item_select_menu.hpp>
#include <i_window_menu_item.hpp>

namespace od3d {

/// « Régulation » : PID (Prusa) ou MPC (exp.). Un clic passe à l'autre valeur.
class MI_NOZZLE_CONTROL final : public MenuItemSelectMenu {
public:
    MI_NOZZLE_CONTROL();
    int item_count() const override;
    string_view_utf8 build_item_text(int index, StringViewUtf8ParamBase &params) const override;

protected:
    bool on_item_selected(const OnItemSelectedArgs &args) override;
    void Loop() override;

private:
    void refresh(); ///< valeur et état (grisé) d'après le mode, la calibration et l'impression
};

/// « Calibrer le modèle MPC » : ouvre l'écran de la calibration
class MI_MPC_CALIBRATE final : public IWindowMenuItem {
public:
    MI_MPC_CALIBRATE();
    void click(IWindowMenu &) override;

protected:
    void Loop() override;
};

/// Calibration du modèle thermique : étapes, température de la buse, barre, « Lancer » et « Annuler »
class ScreenMpcCalibration final : public Screen {
public:
    ScreenMpcCalibration();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    /// Les quatre étapes de la maquette, avec leurs repères ✓ ● ○
    class Steps : public window_t {
    public:
        Steps(window_t *parent, Rect16 rect);
        void set_phase(mpc::Phase phase, bool failed);

    protected:
        void unconditionalDraw() override;

    private:
        int8_t current = -1; ///< étape en cours, -1 : aucune ; 4 : toutes faites
        bool failed = false;
    };

    class Bar : public window_t {
    public:
        Bar(window_t *parent, Rect16 rect);
        void set_progress(uint8_t set);

    protected:
        void unconditionalDraw() override;

    private:
        uint8_t progress = 0;
    };

    void update();
    void start();
    void show_status();

    // Ordre de construction = ordre de la molette
    Header header;
    Panel panel;
    window_text_t temp_note;
    Steps steps;
    window_text_t status;
    Bar bar;
    Button start_button;
    Button cancel_button;
    TabBar tabs;

    std::array<char, 24> temp_text {};
    std::array<char, 64> status_text {};
    bool pending = false; ///< M306 T envoyé, pas encore commencé
    uint16_t runs_at_request = 0;
    mpc::TuneStatus shown {};
    bool shown_pending = false;
    int16_t shown_temp = INT16_MIN;
};

} // namespace od3d
