/// @file
/// OneDrop3D : le groupe « Fonctions OneDrop3D », en tête des réglages.
#pragma once

#include <basic_screen_menu.hpp>
#include <WindowMenuItems.hpp>
#include <gui/menu_item/menu_item_select_menu.hpp>
#include <option/has_mmu2.h>
#include <option/has_nozzle_thermal_compensation.h>
#include <tool_index.hpp>

static_assert(HAS_NOZZLE_THERMAL_COMPENSATION() && HAS_MMU2(), "Fonctions OneDrop3D : CORE One avec MMU");
static_assert(VirtualToolIndex::count == 5, "Un relais par emplacement du MMU");

namespace od3d {

/// Ouvre l'écran des fonctions OneDrop3D
class MI_FEATURES final : public IWindowMenuItem {
public:
    MI_FEATURES();
    void click(IWindowMenu &) override;
};

/// F2 : compensation de la dilatation de la buse
class MI_NOZZLE_THERMAL_COMPENSATION final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_NOZZLE_THERMAL_COMPENSATION();
    void OnChange(size_t) override;
};

/// F3 : relais de bobine permanents, interrupteur général
class MI_SPOOL_JOIN final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_SPOOL_JOIN();
    void OnChange(size_t) override;
};

/// F3 : « Emplacement n vide » → emplacement k ; l'élément 0 désactive le relais de cet emplacement
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

using ScreenMenuFeaturesBase = BasicScreenMenu<MI_NOZZLE_THERMAL_COMPENSATION, MI_SPOOL_JOIN,
    MI_SPOOL_JOIN_TARGET_N<0>, MI_SPOOL_JOIN_TARGET_N<1>, MI_SPOOL_JOIN_TARGET_N<2>, MI_SPOOL_JOIN_TARGET_N<3>, MI_SPOOL_JOIN_TARGET_N<4>>;

class ScreenMenuFeatures final : public ScreenMenuFeaturesBase {
public:
    ScreenMenuFeatures();
};

} // namespace od3d
