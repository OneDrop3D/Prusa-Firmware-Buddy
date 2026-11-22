/// @file
/// OneDrop3D (UI-13) : Réglages › « Filaments » : les préréglages de Prusa (PLA, PETG…) modifiables.
/// Filaments › un préréglage › buse, buse en attente, plateau, chambre (cible, minimum, maximum), chacun par l'éditeur
/// de valeur ; « Rétablir la valeur de Prusa » par champ ou pour tout le préréglage. Les modifications sont dans
/// config_store (od3d_preset_overrides) et passent par FilamentType::parameters() (feature/od3d/filament_presets_od3d.hpp).
/// Menus de Prusa dessinés en style G (menu_od3d) : doigt et molette ; aucun windowEvent redéfini.
#pragma once

#include <basic_screen_menu.hpp>
#include <filament.hpp>
#include <feature/od3d/filament_presets_math_od3d.hpp>
#include <WindowItemFormatableLabel.hpp>
#include <WindowMenuItems.hpp>

#include <array>
#include <cstdint>

namespace od3d {

namespace fp = filament_presets;

/// Réglages › « Filaments »
class MI_FILAMENT_PRESETS final : public IWindowMenuItem {
public:
    MI_FILAMENT_PRESETS();
    void click(IWindowMenu &) override;
};

/// Un préréglage : « PLA » (« PLA · modifié »), puis « 215/60 °C · chambre 20 °C » en gris ; ouvre ses champs
class MI_PRESET : public IWindowMenuItem {
public:
    void click(IWindowMenu &) override;

protected:
    explicit MI_PRESET(uint8_t list_index); ///< rang dans preset_filament_types (ordre d'affichage de Prusa)

private:
    uint8_t list_index;
    std::array<char, 64> label; ///< UTF-8 : « PLA · modifié\n215/60°C · chambre 20°C » fait jusqu'à 52 octets
};

template <uint8_t i>
class MI_PRESET_N final : public MI_PRESET {
public:
    MI_PRESET_N()
        : MI_PRESET(i) {}
};

static_assert(preset_filament_type_count <= 14, "Une ligne par préréglage");

/// « Mes filaments » : les filaments de l'utilisateur, dans la gestion des filaments de Prusa
class MI_USER_FILAMENTS final : public IWindowMenuItem {
public:
    MI_USER_FILAMENTS();
    void click(IWindowMenu &) override;
};

class ScreenFilamentPresets final : public BasicScreenMenu<MI_PRESET_N<0>, MI_PRESET_N<1>, MI_PRESET_N<2>, MI_PRESET_N<3>, MI_PRESET_N<4>,
                                        MI_PRESET_N<5>, MI_PRESET_N<6>, MI_PRESET_N<7>, MI_PRESET_N<8>, MI_PRESET_N<9>, MI_PRESET_N<10>,
                                        MI_PRESET_N<11>, MI_PRESET_N<12>, MI_PRESET_N<13>, MI_USER_FILAMENTS> {
public:
    ScreenFilamentPresets();
};

/// Un champ du préréglage ouvert : valeur à droite ; en gris, « Valeur Prusa », ou « Modifié · Prusa 20 °C »
class MI_PRESET_FIELD : public WI_LAMBDA_LABEL_t {
public:
    void click(IWindowMenu &) override;

protected:
    explicit MI_PRESET_FIELD(fp::Field field);

private:
    void update_label();

    fp::Field field;
    std::array<char, 48> label;
};

template <fp::Field f>
class MI_PRESET_FIELD_N final : public MI_PRESET_FIELD {
public:
    MI_PRESET_FIELD_N()
        : MI_PRESET_FIELD(f) {}
};

/// Ouvre « Rétablir la valeur de Prusa »
class MI_PRESET_RESTORE final : public IWindowMenuItem {
public:
    MI_PRESET_RESTORE();
    void click(IWindowMenu &) override;
};

class ScreenFilamentPreset final : public BasicScreenMenu<MI_PRESET_FIELD_N<fp::nozzle>, MI_PRESET_FIELD_N<fp::nozzle_preheat>, MI_PRESET_FIELD_N<fp::bed>,
                                       MI_PRESET_FIELD_N<fp::chamber_target>, MI_PRESET_FIELD_N<fp::chamber_min>, MI_PRESET_FIELD_N<fp::chamber_max>,
                                       MI_PRESET_RESTORE> {
public:
    ScreenFilamentPreset();
};

/// Rétablit la valeur de Prusa d'un champ (fp::field_count : tout le préréglage) ; grisé sans modification
class MI_PRESET_RESET : public IWindowMenuItem {
public:
    void click(IWindowMenu &) override;

protected:
    explicit MI_PRESET_RESET(uint8_t field);
    void Loop() override; ///< grisé dès que le champ suit Prusa (après « Tout rétablir » aussi)

private:
    bool modified() const;

    uint8_t field;
    std::array<char, 48> label;
};

template <uint8_t f>
class MI_PRESET_RESET_N final : public MI_PRESET_RESET {
public:
    MI_PRESET_RESET_N()
        : MI_PRESET_RESET(f) {}
};

class ScreenPresetRestore final : public BasicScreenMenu<MI_PRESET_RESET_N<fp::field_count>, MI_PRESET_RESET_N<fp::nozzle>, MI_PRESET_RESET_N<fp::nozzle_preheat>,
                                      MI_PRESET_RESET_N<fp::bed>, MI_PRESET_RESET_N<fp::chamber_target>, MI_PRESET_RESET_N<fp::chamber_min>,
                                      MI_PRESET_RESET_N<fp::chamber_max>> {
public:
    ScreenPresetRestore();
};

} // namespace od3d
