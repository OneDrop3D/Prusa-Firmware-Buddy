/// @file
/// OneDrop3D (F7) : régulation de la buse dans « Fonctions OneDrop3D › Température de la buse » (SPEC_G.md §3.23)
/// — choix PID / MPC (PID par défaut, MPC grisé sans calibration valable, bascule refusée pendant une impression) —
/// et l'écran de progression de la calibration du modèle (§1.9, `mpc` : Refroidissement, Chauffe libre, Mesure des
/// pertes (ventilateur), Calcul du modèle), avec « Annuler ». La calibration est faite par M306 T
/// (feature/od3d/mpc_control_od3d.hpp) ; l'écran (ScreenMpcCalibration, dans le .cpp) ne fait que la lancer, l'annuler
/// et montrer son état. Ce fichier est inclus par screen_menu_od3d.hpp : rien des écrans de l'atelier ici.
#pragma once

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

} // namespace od3d
