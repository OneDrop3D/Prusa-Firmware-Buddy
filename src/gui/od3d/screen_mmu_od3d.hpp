/// @file
/// OneDrop3D (F1, interface G) : l'étagère du MMU (SPEC_G.md §3.2), ses moteurs (§3.8), ses réglages et ses infos
/// (§4 : éléments des menus Prusa, dessinés en liste G par screen_menu.hpp).
#pragma once

#include "atelier_od3d.hpp"

#include <basic_screen_menu.hpp>
#include <gui/menu_item/specific/menu_items_hw_mmu.hpp>
#include <gui/od3d/screen_menu_od3d.hpp>
#include <gui/od3d/screen_mmu_speed_od3d.hpp>
#include <gui/od3d/screen_ptfe_od3d.hpp>
#include <MItem_mmu.hpp>
#include <tool_index.hpp>
#include <utility>

namespace od3d {

/// Bobine de l'étagère : grande carte avec numéro, bobine Ø64, matière, état ; remontée et orange si chargée. Sur deux
/// rangées (INDX 8 têtes), carte basse : bobine Ø32 à gauche, « n MATIÈRE » et l'état à droite.
class ShelfSpool : public Button {
public:
    /// top : haut de l'étagère (sous l'en-tête dans l'Atelier, sous l'aide dans le choix d'une bobine)
    ShelfSpool(window_t *parent, uint8_t index, uint8_t tools, int16_t top = style::top);

    /// Relit l'emplacement et le redessine s'il a changé
    void update(bool mmu_ready);

    bool loaded() const { return slot.loaded; }

protected:
    void unconditionalDraw() override;

private:
    Slot slot;
};

/// Raccourci de l'étagère : tuile d'icône, titre, et une 2e ligne (« Ouvrir > »…)
class Shortcut : public Button {
public:
    Shortcut(window_t *parent, Rect16 body, uint8_t tag, const img::Resource &icon, Color tile, const string_view_utf8 &title, const string_view_utf8 &subtitle);

protected:
    void unconditionalDraw() override;

private:
    Color tile;
    string_view_utf8 subtitle;
};

/// Étagère de l'onglet Atelier : une bobine par outil lu à l'exécution (MMU3 : 5 ; sans MMU : 1 ; INDX : 3 à 8, sur deux
/// rangées au-delà de 5), puis les raccourcis
class ScreenMMU final : public Screen {
public:
    ScreenMMU();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    template <size_t... I>
    explicit ScreenMMU(std::index_sequence<I...>);

    void update();

    Header header;
    ShelfSpool spools[VirtualToolIndex::count];
    Shortcut preload_all;
    Shortcut motors;
    Shortcut pressure_advance;
    Shortcut settings;
    Shortcut info;
    TabBar tabs;
};

/// Rappel du choix d'une bobine : true, le choix se referme ; false, il reste ouvert (question refusée)
using SlotPick = bool (*)(uint8_t slot);

/// Ouvre le choix d'une bobine : titre, aide d'une ligne, une carte de l'étagère par outil (MMU3 : 5, sans MMU : 1,
/// INDX : ses têtes) ; molette et toucher, le clic choisit et appelle pick. focus : carte mise en avant à l'ouverture.
void open_slot_pick(const string_view_utf8 &title, const string_view_utf8 &help, uint8_t focus, SlotPick pick);

/// Écran du choix d'une bobine (open_slot_pick), comme l'aperçu des températures : des cartes, pas une liste
class ScreenSlotPick final : public Screen {
public:
    ScreenSlotPick();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    template <size_t... I>
    explicit ScreenSlotPick(std::index_sequence<I...>);

    Header header;
    window_text_t help;
    ShelfSpool spools[VirtualToolIndex::count];
    TabBar tabs;
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
    /// Relâche le galet serré depuis cet écran (BUG-1 : serré, son moteur reste à 530 mA) et arrête le nettoyage
    ~ScreenMMUMotors();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();
    void show();
    /// Emplacement des pas de 10 mm : celui du galet serré, sinon celui du sélecteur ; -1 sans emplacement
    int8_t pulley_slot() const;

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
    bool pulley_supported = false; ///< le firmware du MMU connecté a le registre 0x24 (F12)
    bool cleaning = false; ///< « Nettoyer » envoyé par cet écran, pas encore arrêté
    uint32_t cleaning_ms = 0; ///< moment du lancement du nettoyage
    int16_t clean_left = -1; ///< secondes de nettoyage restantes affichées, -1 sans
    StringViewUtf8Parameters<4> pulley_params;
    StringViewUtf8Parameters<4> selector_params;
    StringViewUtf8Parameters<8> idler_params; ///< emplacement et secondes restantes
};

/// Réglages du MMU (éléments Prusa, liste G) : MMU, coupe, mode furtif, longueur du tube et sa mesure automatique (F41,
/// même écran que Réglages › Matériel › MMU), vitesse de la poulie (N4), relais de bobine, mises à jour du MMU (par un
/// ordinateur, par l'USB avant)
using ScreenMenuMMUSettingsBase = BasicScreenMenu<MI_MMU_ENABLE, MI_MMU_CUTTER, MI_MMU_STEALTH_MODE, MI_MMU_FRONT_PTFE_LENGTH, MI_PTFE_AUTO, MI_MMU_SPEED, MI_SPOOL_JOIN_SLOTS, MI_MMU_USB, MI_MMU_USB_FRONT>;

class ScreenMenuMMUSettings final : public ScreenMenuMMUSettingsBase {
public:
    ScreenMenuMMUSettings();
};

/// Infos du MMU (éléments Prusa, liste G) : FINDA et échecs de chargement, puis la pointe du filament de chaque
/// emplacement (F10)
using ScreenMenuMMUInfoBase = BasicScreenMenu<MI_INFO_FINDA, MI_MMU_LOAD_FAILS, MI_MMU_TOTAL_LOAD_FAILS, MI_MMU_GENERAL_FAILS, MI_MMU_TOTAL_GENERAL_FAILS,
    MI_TIP<0>, MI_TIP<1>, MI_TIP<2>, MI_TIP<3>, MI_TIP<4>>;

class ScreenMenuMMUInfo final : public ScreenMenuMMUInfoBase {
public:
    ScreenMenuMMUInfo();
};

} // namespace od3d
