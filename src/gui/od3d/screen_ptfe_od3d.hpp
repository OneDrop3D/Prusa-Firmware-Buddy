/// @file
/// OneDrop3D (F41) : Réglages › Matériel › MMU › « Mesurer automatiquement » (feature/od3d/ptfe_od3d.hpp) : longueur
/// actuelle, emplacement à utiliser, « Lancer la mesure » (M1999 B1 P<n>), résultat de la dernière mesure. Menu de
/// Prusa dessiné en style G (menu_od3d) : doigt et molette ; aucun windowEvent redéfini.
#pragma once

#include <array>
#include <cstdint>
#include <basic_screen_menu.hpp>
#include <WindowItemFormatableLabel.hpp>
#include <WindowMenuItems.hpp>
#include <gui/menu_item/menu_item_select_menu.hpp>
#include <feature/od3d/ptfe_od3d.hpp>

namespace od3d {

/// Raison d'un résultat (N_, à traduire), nullptr pour « enregistré » (chiffres) : écran de la mesure et bilan de
/// l'assistant de calibration
const char *ptfe_result_text(ptfe::Result result);

/// Réglages › Matériel › MMU : ouvre l'écran de la mesure
class MI_PTFE_AUTO final : public IWindowMenuItem {
public:
    MI_PTFE_AUTO();
    void click(IWindowMenu &) override;
};

/// Longueur enregistrée dans le MMU (registre 0x22, relu à chaque interrogation)
class MI_PTFE_LENGTH_NOW final : public MenuItemAutoUpdatingLabel<int> {
public:
    MI_PTFE_LENGTH_NOW();
};

/// Emplacement dont la bobine préchargée sert à la mesure : « Emplacement n », chaque appui passe au suivant. La liste de
/// choix G n'a que 5 lignes, « Retour » compris : l'emplacement 5 y était caché sous le bas (retour du 22/11)
class MI_PTFE_SLOT final : public MenuItemSelectMenu {
public:
    MI_PTFE_SLOT();
    int item_count() const override;
    string_view_utf8 build_item_text(int index, StringViewUtf8ParamBase &params) const override;

protected:
    bool on_item_selected(const OnItemSelectedArgs &args) override;
};

/// Confirmation, puis M1999 B1 P<emplacement>
class MI_PTFE_START final : public IWindowMenuItem {
public:
    MI_PTFE_START();
    void click(IWindowMenu &) override;
};

/// « Dernière mesure » et son résultat en 2e ligne, mis à jour à chaque changement
class MI_PTFE_RESULT final : public IWindowMenuItem {
public:
    MI_PTFE_RESULT();

protected:
    void Loop() override;

private:
    void show();
    uint32_t shown_key = UINT32_MAX;
    std::array<char, 112> text {};
};

class ScreenPtfe final : public BasicScreenMenu<MI_PTFE_LENGTH_NOW, MI_PTFE_SLOT, MI_PTFE_START, MI_PTFE_RESULT> {
public:
    ScreenPtfe();
};

} // namespace od3d
