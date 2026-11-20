/// @file
#include "screen_menu_od3d.hpp"

#include <feature/nozzle_thermal_compensation/nozzle_thermal_compensation.hpp>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <ScreenFactory.hpp>
#include <ScreenHandler.hpp>

namespace od3d {

MI_FEATURES::MI_FEATURES()
    : IWindowMenuItem(_("OneDrop3D Features"), nullptr, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_FEATURES::click(IWindowMenu &) {
    Screens::Access()->Open(ScreenFactory::Screen<ScreenMenuFeatures>);
}

// Pas pendant une impression : le Z changerait d'un coup (jusqu'à 0,05 mm à 280 °C).
MI_NOZZLE_THERMAL_COMPENSATION::MI_NOZZLE_THERMAL_COMPENSATION()
    : WI_ICON_SWITCH_OFF_ON_t(buddy::nozzle_thermal_compensation::is_enabled(), _("Nozzle Expansion"), nullptr,
        marlin_client::is_printing() ? is_enabled_t::no : is_enabled_t::yes) {}

void MI_NOZZLE_THERMAL_COMPENSATION::OnChange(size_t) {
    buddy::nozzle_thermal_compensation::set_enabled(value());
}

ScreenMenuFeatures::ScreenMenuFeatures()
    : ScreenMenuFeaturesBase(_("ONEDROP3D FEATURES"), &img::settings_16x16) {
    EnableLongHoldScreenAction();
}

} // namespace od3d
