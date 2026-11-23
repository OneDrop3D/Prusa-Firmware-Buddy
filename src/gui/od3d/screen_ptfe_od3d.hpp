/// @file
/// OneDrop3D (F41) : Réglages › Matériel › MMU › « Mesurer automatiquement » (feature/od3d/ptfe_od3d.hpp) : longueur
/// actuelle, « Lancer la mesure » (choix de la bobine, confirmation, M1999 B1 P<n>), résultat de la dernière mesure.
/// Menu de Prusa dessiné en style G (menu_od3d) : doigt et molette ; aucun windowEvent redéfini.
#pragma once

#include <array>
#include <cstdint>
#include <basic_screen_menu.hpp>
#include <WindowItemFormatableLabel.hpp>
#include <WindowMenuItems.hpp>
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

/// Choix de la bobine préchargée (cartes de l'étagère, le dernier choix mis en avant), confirmation, puis
/// M1999 B1 P<emplacement>. Avant, « Emplacement n » à faire défiler appui par appui (retour du 23/11 : pas agréable)
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

class ScreenPtfe final : public BasicScreenMenu<MI_PTFE_LENGTH_NOW, MI_PTFE_START, MI_PTFE_RESULT> {
public:
    ScreenPtfe();
};

} // namespace od3d
