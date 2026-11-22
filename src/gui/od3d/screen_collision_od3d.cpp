/// @file
#include "screen_collision_od3d.hpp"

#include "widgets_od3d.hpp"

#include <common/marlin_vars.hpp>
#include <feature/od3d/collision_od3d.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <window_msgbox.hpp>

namespace od3d {

namespace co = collision;

namespace {

    constexpr const char *sensitivity_items[] = {
        N_("Low"),
        N_("Normal"),
        N_("High"),
    };
    static_assert(uint8_t(co::Sensitivity::low) == 0 && uint8_t(co::Sensitivity::high) == 2, "ordre des éléments");

    constexpr const char *action_items[] = {
        N_("Pause the print"),
        N_("Warn only"),
    };
    static_assert(uint8_t(co::Action::pause) == 0 && uint8_t(co::Action::warn) == 1, "ordre des éléments");

    constexpr float max_test_nozzle_c = 50;

    template <typename F>
    void modify(F &&f) {
        co::Settings s = co::settings();
        f(s);
        co::set_settings(s);
    }

} // namespace

MI_COLLISION::MI_COLLISION()
    : IWindowMenuItem(_("Collision guard\nPause when the nozzle hits the part"), &img::nozzle_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_COLLISION::click(IWindowMenu &) {
    open_screen<ScreenCollision>();
}

// Vaut dès la prochaine phase d'impression armée
MI_COLLISION_ENABLE::MI_COLLISION_ENABLE()
    : WI_ICON_SWITCH_OFF_ON_t(co::settings().enabled, _("Collision guard\nLoad cell, during prints only"), &img::nozzle_16x16) {}

void MI_COLLISION_ENABLE::OnChange(size_t) {
    modify([this](co::Settings &s) { s.enabled = value(); });
}

MI_COLLISION_SENSITIVITY::MI_COLLISION_SENSITIVITY()
    : MenuItemSwitch(_("Sensitivity\nThresholds 600 / 400 / 250 g"), sensitivity_items, size_t(co::settings().sensitivity)) {
    SetIconId(&img::metrics_16x16);
}

bool MI_COLLISION_SENSITIVITY::on_item_selected(const OnItemSelectedArgs &args) {
    modify([&args](co::Settings &s) { s.sensitivity = co::Sensitivity(args.new_index); });
    return MenuItemSwitch::on_item_selected(args);
}

MI_COLLISION_ACTION::MI_COLLISION_ACTION()
    : MenuItemSwitch(_("Action\nOn repeated hits"), action_items, size_t(co::settings().action)) {
    SetIconId(&img::warning_16x16);
}

bool MI_COLLISION_ACTION::on_item_selected(const OnItemSelectedArgs &args) {
    modify([&args](co::Settings &s) { s.action = args.new_index == 1 ? co::Action::warn : co::Action::pause; });
    return MenuItemSwitch::on_item_selected(args);
}

MI_COLLISION_LAST::MI_COLLISION_LAST()
    : MenuItemAutoUpdatingLabel(_("Last hit\nPeak above the normal force"), "%d g", [](auto *) -> int {
        return co::last_peak_g();
    }) {
    SetIconId(&img::metrics_16x16);
}

MI_COLLISION_TEST::MI_COLLISION_TEST()
    : IWindowMenuItem(_("Test with a cold nozzle\nTap under the nozzle 3 times"), &img::selftest_16x16, is_enabled_t::yes, is_hidden_t::no) {}

void MI_COLLISION_TEST::click(IWindowMenu &) {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    auto &hotend = marlin_vars().active_hotend();
    if (hotend.temp_nozzle.get() > max_test_nozzle_c || hotend.target_nozzle.get() > max_test_nozzle_c) {
        MsgBoxWarning(_("Let the nozzle cool below 50°C, then start again."), Responses_Ok);
        return;
    }
    co::start_test();
    MsgBoxInfo(_("Bed lowered, nozzle cold: for 60 s, tap the nozzle tip upward 3 times, sharply, with a wooden stick. \"Last hit\" shows each force; the collision message then appears on the home screen."), Responses_Ok);
}

ScreenCollision::ScreenCollision()
    : BasicScreenMenu(_("Collision guard")) {}

} // namespace od3d
