/// @file
#include "screen_menu_od3d.hpp"

#include <config_store/store_instance.hpp>
#include <cstdio>
#include <feature/od3d/pairing_od3d.hpp>
#include <window_msgbox.hpp>
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

MI_PAIR_PRINTFORGE::MI_PAIR_PRINTFORGE()
    : IWindowMenuItem(_("Pair PrintForge"), nullptr, is_enabled_t::yes, is_hidden_t::no) {}

void MI_PAIR_PRINTFORGE::click(IWindowMenu &) {
    // PrintForge reçoit le mot de passe PrusaLink : sans lui, il n'y a rien à transmettre.
    if (!config_store().prusalink_enabled.get() || !config_store().prusalink_password.get()[0]) {
        MsgBoxWarning(_("Turn on PrusaLink first: Settings > Network > PrusaLink."), Responses_Ok);
        return;
    }
    char code[8];
    snprintf(code, sizeof(code), "%06lu", static_cast<unsigned long>(od3d::pairing::start()));
    StringViewUtf8Parameters<8> params;
    MsgBoxInfo(_("Type this code in PrintForge:\n\n%s\n\nValid for 5 minutes.").formatted(params, code), Responses_Ok);
    od3d::pairing::stop();
}

// Pas pendant une impression : le Z changerait d'un coup (jusqu'à 0,05 mm à 280 °C).
MI_NOZZLE_THERMAL_COMPENSATION::MI_NOZZLE_THERMAL_COMPENSATION()
    : WI_ICON_SWITCH_OFF_ON_t(buddy::nozzle_thermal_compensation::is_enabled(), _("Nozzle Expansion"), nullptr,
        marlin_client::is_printing() ? is_enabled_t::no : is_enabled_t::yes) {}

void MI_NOZZLE_THERMAL_COMPENSATION::OnChange(size_t) {
    buddy::nozzle_thermal_compensation::set_enabled(value());
}

// Les relais sont lus au départ de chaque impression : un changement vaut pour la suivante.
MI_SPOOL_JOIN::MI_SPOOL_JOIN()
    : WI_ICON_SWITCH_OFF_ON_t(config_store().od3d_spool_join.get(), _("Spool Join")) {}

void MI_SPOOL_JOIN::OnChange(size_t) {
    config_store().od3d_spool_join.set(value());
}

MI_SPOOL_JOIN_TARGET::MI_SPOOL_JOIN_TARGET(uint8_t slot)
    : MenuItemSelectMenu({})
    , slot(slot) {
    SetLabel(_("Slot %i empty").formatted(label_params, slot + 1));
    set_is_hidden(!VirtualToolIndex::from_raw(slot).is_enabled());
    const uint8_t target = config_store().od3d_spool_join_target.get(slot);
    set_current_item(target < VirtualToolIndex::count ? target + 1 : 0);
}

int MI_SPOOL_JOIN_TARGET::item_count() const {
    return VirtualToolIndex::count + 1;
}

string_view_utf8 MI_SPOOL_JOIN_TARGET::build_item_text(int index, StringViewUtf8ParamBase &params) const {
    return index == 0 ? _("Off") : _("Slot %i").formatted(params, index);
}

bool MI_SPOOL_JOIN_TARGET::on_item_selected(const OnItemSelectedArgs &args) {
    if (args.new_index == slot + 1) {
        return false; // un emplacement ne peut pas prendre le relais de lui-même
    }
    config_store().od3d_spool_join_target.set(slot, args.new_index == 0 ? uint8_t(0xff) : uint8_t(args.new_index - 1));
    return true;
}

ScreenMenuFeatures::ScreenMenuFeatures()
    : ScreenMenuFeaturesBase(_("ONEDROP3D FEATURES"), &img::settings_16x16) {
    EnableLongHoldScreenAction();
}

} // namespace od3d
