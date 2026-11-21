/// @file
/// OneDrop3D : le groupe « Fonctions OneDrop3D », en tête des réglages (SPEC_G.md §3.22), et ses écrans
/// « Température de la buse » (§3.23 : régulation PID / MPC et sa calibration (F7, screen_mpc_od3d.*), compensation de
/// dilatation (F2)) et « Relais de bobine » (§3.24) ;
/// « Équerrage XY » (F15) est dans screen_skew_od3d.*, « Tension des courroies » (F14) dans screen_belts_od3d.*,
/// « Vitesses critiques » (F22) dans screen_speed_map_od3d.*.
/// Chaque fonction a une ligne d'explication sous son nom (libellé sur deux lignes, séparées par « \n »).
#pragma once

#include "screen_mpc_od3d.hpp"
#include "screen_skew_od3d.hpp"
#include "menu_items_od3d.hpp"

#include <basic_screen_menu.hpp>
#include <MItem_menus.hpp>
#include <WindowMenuItems.hpp>
#include <WindowItemFormatableLabel.hpp>
#include <gui/menu_item/menu_item_select_menu.hpp>
#include <option/has_mmu2.h>
#include <option/has_nozzle_thermal_compensation.h>
#include <tool_index.hpp>

static_assert(HAS_NOZZLE_THERMAL_COMPENSATION() && HAS_MMU2(), "Fonctions OneDrop3D : CORE One avec MMU");
static_assert(VirtualToolIndex::count == 5, "Un relais par emplacement du MMU");

class ScreenMenuInfo; // menu Info de Prusa

namespace od3d {

/// Ouvre l'écran des fonctions OneDrop3D
class MI_FEATURES final : public IWindowMenuItem {
public:
    MI_FEATURES();
    void click(IWindowMenu &) override;
};

/// « Infos de l'imprimante » : le menu Info de Prusa, deuxième ligne des réglages
using MI_PRINTER_INFO = MI_SCREEN<N_("Printer info"), ::ScreenMenuInfo, &img::info_16x16>;

/// Ouvre « Température de la buse »
class MI_NOZZLE_TEMPERATURE final : public IWindowMenuItem {
public:
    MI_NOZZLE_TEMPERATURE();
    void click(IWindowMenu &) override;
};

/// Ouvre « Relais de bobine »
class MI_SPOOL_JOIN_SLOTS final : public IWindowMenuItem {
public:
    MI_SPOOL_JOIN_SLOTS();
    void click(IWindowMenu &) override;
};

/// F2 : compensation de la dilatation de la buse
class MI_NOZZLE_THERMAL_COMPENSATION final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_NOZZLE_THERMAL_COMPENSATION();
    void OnChange(size_t) override;
};

/// F2 : correction appliquée maintenant (cible de la buse), en lecture seule
class MI_NOZZLE_THERMAL_CORRECTION final : public MenuItemAutoUpdatingLabel<float> {
public:
    MI_NOZZLE_THERMAL_CORRECTION();
};

/// F4 : « le PA de l'imprimante prime » : le K mesuré de l'emplacement remplace le M572 du trancheur
class MI_PA_PRINTER_WINS final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_PA_PRINTER_WINS();
    void OnChange(size_t) override;
};

/// F5 : « limiter au débit mesuré » : le planificateur ralentit ce qui dépasserait le débit maximal de la bobine
class MI_FLOW_LIMIT final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_FLOW_LIMIT();
    void OnChange(size_t) override;
};

/// F3 : relais de bobine permanents, interrupteur général
class MI_SPOOL_JOIN final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_SPOOL_JOIN();
    void OnChange(size_t) override;
};

/// F3 : « Quand n est vide » → emplacement k ; l'élément 0 désactive le relais de cet emplacement.
/// Un clic passe à la valeur suivante (SPEC_G.md §3.24).
class MI_SPOOL_JOIN_TARGET : public MenuItemSelectMenu {
public:
    explicit MI_SPOOL_JOIN_TARGET(uint8_t slot);
    int item_count() const override;
    string_view_utf8 build_item_text(int index, StringViewUtf8ParamBase &params) const override;

protected:
    bool on_item_selected(const OnItemSelectedArgs &args) override;

private:
    uint8_t slot;
    StringViewUtf8Parameters<2> label_params;
};

template <uint8_t slot_>
class MI_SPOOL_JOIN_TARGET_N final : public MI_SPOOL_JOIN_TARGET {
public:
    MI_SPOOL_JOIN_TARGET_N()
        : MI_SPOOL_JOIN_TARGET(slot_) {}
};

/// Association de PrintForge : affiche un code à 6 chiffres, valable tant que le message est ouvert (5 minutes au plus)
class MI_PAIR_PRINTFORGE final : public IWindowMenuItem {
public:
    MI_PAIR_PRINTFORGE();
    void click(IWindowMenu &) override;
};

class ScreenMenuFeatures final : public BasicScreenMenu<MI_NOZZLE_TEMPERATURE, MI_SPOOL_JOIN_SLOTS, MI_PA_PRINTER_WINS, MI_FLOW_LIMIT, MI_SKEW_XY, MI_BELT_TENSION, MI_SPEED_MAP, MI_PAIR_PRINTFORGE> {
public:
    ScreenMenuFeatures();
};

class ScreenNozzleTemperature final : public BasicScreenMenu<MI_NOZZLE_CONTROL, MI_MPC_CALIBRATE, MI_NOZZLE_THERMAL_COMPENSATION, MI_NOZZLE_THERMAL_CORRECTION> {
public:
    ScreenNozzleTemperature();
};

class ScreenSpoolJoin final : public BasicScreenMenu<MI_SPOOL_JOIN,
                                  MI_SPOOL_JOIN_TARGET_N<0>, MI_SPOOL_JOIN_TARGET_N<1>, MI_SPOOL_JOIN_TARGET_N<2>, MI_SPOOL_JOIN_TARGET_N<3>, MI_SPOOL_JOIN_TARGET_N<4>> {
public:
    ScreenSpoolJoin();
};

} // namespace od3d
