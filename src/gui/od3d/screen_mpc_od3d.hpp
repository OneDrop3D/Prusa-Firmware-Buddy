/// @file
/// OneDrop3D (F7) : régulation de la buse dans « Fonctions OneDrop3D › Température de la buse » (SPEC_G.md §3.23)
/// — choix PID / MPC (PID par défaut, MPC grisé sans calibration valable, bascule refusée pendant une impression) —
/// et l'écran de progression de la calibration du modèle (§1.9, `mpc` : Refroidissement, Chauffe libre, Mesure des
/// pertes (ventilateur), Calcul du modèle), avec « Annuler ». La calibration est faite par M306 T
/// (feature/od3d/mpc_control_od3d.hpp) ; l'écran (ScreenMpcCalibration, dans le .cpp) ne fait que la lancer, l'annuler
/// et montrer son état. Ce fichier est inclus par screen_menu_od3d.hpp : rien des écrans de l'atelier ici.
/// F8 (température selon le débit, seulement avec le MPC) : marche / arrêt (grisé hors MPC), seuil, hausse maximale
/// et hausse en cours, dans le même menu.
#pragma once

#include <gui/menu_item/menu_item_select_menu.hpp>
#include <i_window_menu_item.hpp>
#include <numeric_input_config.hpp>
#include <WindowItemFormatableLabel.hpp>
#include <WindowMenuItems.hpp>

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

/// Réglage entier : la valeur et son unité à droite ; un clic ouvre l'éditeur de valeur (interface G)
class MI_OD3D_NUMBER : public WI_LAMBDA_LABEL_t {
public:
    using Get = uint8_t (*)();
    using Set = void (*)(uint8_t);

    /// @param editor_label titre de l'éditeur (une ligne), marqué N_()
    MI_OD3D_NUMBER(const string_view_utf8 &label, const img::Resource *icon, const char *editor_label, const NumericInputConfig &config, const char *unit, Get get, Set set);
    void click(IWindowMenu &) override;

protected:
    void Loop() override; ///< valeur changée ailleurs (G-code, autre écran)

private:
    const char *editor_label;
    const NumericInputConfig &config;
    const char *unit;
    Get get;
    Set set;
    uint8_t shown;
};

/// F8 : « Température selon le débit », marche / arrêt (coupé par défaut) ; grisé tant que la régulation n'est pas
/// le MPC (le réglage n'a d'effet qu'avec le MPC)
class MI_FLOW_TEMP final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_FLOW_TEMP();
    void OnChange(size_t) override;

protected:
    void Loop() override;
};

/// F8 : seuil de débit [mm³/s]
class MI_FLOW_TEMP_THRESHOLD final : public MI_OD3D_NUMBER {
public:
    MI_FLOW_TEMP_THRESHOLD();
};

/// F8 : hausse maximale [°C]
class MI_FLOW_TEMP_MAX_BOOST final : public MI_OD3D_NUMBER {
public:
    MI_FLOW_TEMP_MAX_BOOST();
};

/// F8 : hausse appliquée maintenant, en lecture seule
class MI_FLOW_TEMP_BOOST final : public MenuItemAutoUpdatingLabel<int> {
public:
    MI_FLOW_TEMP_BOOST();
};

} // namespace od3d
