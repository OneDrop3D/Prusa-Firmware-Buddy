/// @file
/// OneDrop3D (F1, interface G) : l'étagère du MMU (SPEC_G.md §3.2), ses moteurs (§3.8), ses réglages et ses infos
/// (§4 : éléments des menus Prusa, dessinés en liste G par screen_menu.hpp).
#pragma once

#include "atelier_od3d.hpp"

#include <basic_screen_menu.hpp>
#include <gui/menu_item/specific/menu_items_hw_mmu.hpp>
#include <gui/od3d/screen_menu_od3d.hpp>
#include <MItem_mmu.hpp>

namespace od3d {

/// Bobine de l'étagère : grande carte avec numéro, bobine Ø64, matière, état ; remontée et orange si chargée
class ShelfSpool : public Button {
public:
    ShelfSpool(window_t *parent, uint8_t index);

    /// Relit l'emplacement et le redessine s'il a changé
    void update(bool mmu_ready);

    bool loaded() const { return slot.loaded; }

protected:
    void unconditionalDraw() override;

private:
    Slot slot;
};

/// Raccourci de l'étagère : tuile d'icône, titre, et une 2e ligne (« Ouvrir > », « bientôt »…)
class Shortcut : public Button {
public:
    Shortcut(window_t *parent, Rect16 body, uint8_t tag, const img::Resource &icon, Color tile, const string_view_utf8 &title, const string_view_utf8 &subtitle);

protected:
    void unconditionalDraw() override;

private:
    Color tile;
    string_view_utf8 subtitle;
};

class ScreenMMU final : public Screen {
public:
    ScreenMMU();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();

    Header header;
    ShelfSpool spools[5];
    Shortcut preload_all;
    Shortcut assist;
    Shortcut motors;
    Shortcut pressure_advance;
    Shortcut settings;
    Shortcut info;
    TabBar tabs;
};

/// Bandeau d'un groupe des moteurs : fond, titre ; la note de droite est une fenêtre à part (redessinée seule)
class Panel : public window_t {
public:
    Panel(window_t *parent, Rect16 rect, const string_view_utf8 &title);

protected:
    void unconditionalDraw() override;

private:
    string_view_utf8 title;
};

/// Choix d'une position (sélecteur ou galet) : petite bobine facultative et numéro
class PositionButton : public Button {
public:
    PositionButton(window_t *parent, Rect16 body, uint8_t tag, int8_t spool, const string_view_utf8 &label);

protected:
    void unconditionalDraw() override;

private:
    int8_t spool; ///< emplacement dont on dessine la bobine, -1 sans bobine
};

class ScreenMMUMotors final : public Screen {
public:
    ScreenMMUMotors();
    /// Relâche le galet serré depuis cet écran (BUG-1 : serré, son moteur reste à 530 mA)
    ~ScreenMMUMotors();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();
    void show();

    Header header;
    Panel pulley_panel;
    window_text_t pulley_note;
    Button pulley_back;
    Button pulley_forward;
    Button pulley_clean;
    Panel selector_panel;
    window_text_t selector_note;
    PositionButton selector[6];
    Panel idler_panel;
    window_text_t idler_note;
    PositionButton idler[6];
    TabBar tabs;

    int8_t selector_at = -1; ///< position lue dans le MMU (0 à 4, 5 = parking), -1 inconnue
    int8_t idler_at = -1; ///< position lue dans le MMU (0 à 4 = serré, 5 = libre), -1 inconnue
    bool engaged_here = false; ///< galet serré par cet écran : à relâcher en partant ou après le délai
    uint32_t engaged_ms = 0; ///< moment de la dernière commande « serrer »
    int16_t seconds_left = -1; ///< compte à rebours affiché, -1 sans
    StringViewUtf8Parameters<4> selector_params;
    StringViewUtf8Parameters<8> idler_params; ///< emplacement et secondes restantes
};

/// Réglages du MMU (éléments Prusa, liste G) : MMU, coupe, mode furtif, longueur du tube, relais de bobine
using ScreenMenuMMUSettingsBase = BasicScreenMenu<MI_MMU_ENABLE, MI_MMU_CUTTER, MI_MMU_STEALTH_MODE, MI_MMU_FRONT_PTFE_LENGTH, MI_SPOOL_JOIN_SLOTS>;

class ScreenMenuMMUSettings final : public ScreenMenuMMUSettingsBase {
public:
    ScreenMenuMMUSettings();
};

/// Infos du MMU (éléments Prusa, liste G) : FINDA et échecs de chargement
using ScreenMenuMMUInfoBase = BasicScreenMenu<MI_INFO_FINDA, MI_MMU_LOAD_FAILS, MI_MMU_TOTAL_LOAD_FAILS, MI_MMU_GENERAL_FAILS, MI_MMU_TOTAL_GENERAL_FAILS>;

class ScreenMenuMMUInfo final : public ScreenMenuMMUInfoBase {
public:
    ScreenMenuMMUInfo();
};

} // namespace od3d
