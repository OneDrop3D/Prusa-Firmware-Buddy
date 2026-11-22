/// @file
/// OneDrop3D (F29, UI-13 en partie) : Réglages › Fonctions OneDrop3D › « Climat de la chambre ».
/// - « Recettes par matière » : pour chaque préréglage de Prusa et chaque filament de l'utilisateur, la cible et le
///   maximum de chambre et la filtration (ceux de Prusa, modifiables dans « Filaments »), et notre recette : aucune,
///   plafond, descente douce, les deux (feature/od3d/chamber_climate_od3d.hpp).
/// - « Descente douce » : état (palier en cours, suivant), « Arrêter », réglages des paliers, « Démarrer maintenant ».
/// Menus de Prusa dessinés en style G (menu_od3d) : doigt et molette ; aucun windowEvent redéfini.
#pragma once

#include <basic_screen_menu.hpp>
#include <filament.hpp>
#include <gui/menu_item/menu_item_select_menu.hpp>
#include <WindowItemFormatableLabel.hpp>
#include <WindowMenuItems.hpp>
#include <WindowMenuSpin.hpp>

#include <array>
#include <cstdint>

namespace od3d {

/// Fonctions OneDrop3D › « Climat de la chambre »
class MI_CHAMBER_CLIMATE final : public IWindowMenuItem {
public:
    MI_CHAMBER_CLIMATE();
    void click(IWindowMenu &) override;
};

/// Ouvre « Recettes par matière »
class MI_CLIMATE_RECIPES final : public IWindowMenuItem {
public:
    MI_CLIMATE_RECIPES();
    void click(IWindowMenu &) override;
};

/// État de la descente douce (« arrêtée », « 80 °C, 3 min ») ; le clic propose de l'arrêter
class MI_SOFT_COOLDOWN_STATE final : public MenuItemAutoUpdatingLabel<uint32_t> {
public:
    MI_SOFT_COOLDOWN_STATE();
    void click(IWindowMenu &) override;

protected:
    void Loop() override; ///< grisé (« Arrêt ») quand il n'y a rien à arrêter : le toucher ne faisait rien
};

class MI_COOLDOWN_STEP final : public WiSpin {
public:
    MI_COOLDOWN_STEP();
    void OnClick() override;
};

class MI_COOLDOWN_INTERVAL final : public WiSpin {
public:
    MI_COOLDOWN_INTERVAL();
    void OnClick() override;
};

class MI_COOLDOWN_FLOOR final : public WiSpin {
public:
    MI_COOLDOWN_FLOOR();
    void OnClick() override;
};

/// « Démarrer maintenant » : M1998 S1 (hors impression), pour essayer sans imprimer
class MI_COOLDOWN_START final : public IWindowMenuItem {
public:
    MI_COOLDOWN_START();
    void click(IWindowMenu &) override;
};

/// Les filaments de Prusa (cible, minimum, maximum de chambre, filtration) : on ne les refait pas
class MI_CLIMATE_FILAMENTS final : public IWindowMenuItem {
public:
    MI_CLIMATE_FILAMENTS();
    void click(IWindowMenu &) override;
};

/// Recette d'une matière : nom, puis « cible 20 °C, max 38 °C, filtre » en gris ; valeur choisie dans une liste
class MI_CLIMATE_RECIPE : public MenuItemSelectMenu {
public:
    int item_count() const override;
    string_view_utf8 build_item_text(int index, StringViewUtf8ParamBase &params) const override;

protected:
    /// user = false : préréglage n° index dans l'ordre d'affichage de Prusa ; true : filament de l'utilisateur
    MI_CLIMATE_RECIPE(bool user, uint8_t index);
    bool on_item_selected(const OnItemSelectedArgs &args) override;

private:
    bool user;
    uint8_t index; ///< valeur de PresetFilamentType, ou indice du filament de l'utilisateur
    std::array<char, 40> label;
};

template <uint8_t i>
class MI_CLIMATE_PRESET final : public MI_CLIMATE_RECIPE {
public:
    MI_CLIMATE_PRESET()
        : MI_CLIMATE_RECIPE(false, i) {}
};

template <uint8_t i>
class MI_CLIMATE_USER final : public MI_CLIMATE_RECIPE {
public:
    MI_CLIMATE_USER()
        : MI_CLIMATE_RECIPE(true, i) {}
};

static_assert(preset_filament_type_count <= 14 && user_filament_type_count <= 8, "Une ligne par matière");

class ScreenChamberClimate final : public BasicScreenMenu<MI_CLIMATE_RECIPES, MI_SOFT_COOLDOWN_STATE, MI_COOLDOWN_STEP, MI_COOLDOWN_INTERVAL,
                                       MI_COOLDOWN_FLOOR, MI_COOLDOWN_START, MI_CLIMATE_FILAMENTS> {
public:
    ScreenChamberClimate();
};

class ScreenClimateRecipes final : public BasicScreenMenu<
                                       MI_CLIMATE_PRESET<0>, MI_CLIMATE_PRESET<1>, MI_CLIMATE_PRESET<2>, MI_CLIMATE_PRESET<3>, MI_CLIMATE_PRESET<4>,
                                       MI_CLIMATE_PRESET<5>, MI_CLIMATE_PRESET<6>, MI_CLIMATE_PRESET<7>, MI_CLIMATE_PRESET<8>, MI_CLIMATE_PRESET<9>,
                                       MI_CLIMATE_PRESET<10>, MI_CLIMATE_PRESET<11>, MI_CLIMATE_PRESET<12>, MI_CLIMATE_PRESET<13>,
                                       MI_CLIMATE_USER<0>, MI_CLIMATE_USER<1>, MI_CLIMATE_USER<2>, MI_CLIMATE_USER<3>,
                                       MI_CLIMATE_USER<4>, MI_CLIMATE_USER<5>, MI_CLIMATE_USER<6>, MI_CLIMATE_USER<7>> {
public:
    ScreenClimateRecipes();
};

} // namespace od3d
