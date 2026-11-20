/// @file
/// OneDrop3D : le groupe « Fonctions OneDrop3D », en tête des réglages.
#pragma once

#include <basic_screen_menu.hpp>
#include <WindowMenuItems.hpp>
#include <option/has_mmu2.h>
#include <option/has_nozzle_thermal_compensation.h>

static_assert(HAS_NOZZLE_THERMAL_COMPENSATION() && HAS_MMU2(), "Fonctions OneDrop3D : CORE One avec MMU");

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

using ScreenMenuFeaturesBase = BasicScreenMenu<MI_NOZZLE_THERMAL_COMPENSATION>;

class ScreenMenuFeatures final : public ScreenMenuFeaturesBase {
public:
    ScreenMenuFeatures();
};

} // namespace od3d
