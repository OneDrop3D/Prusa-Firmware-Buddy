/// @file
#include "screen_sleep_od3d.hpp"

#include "sleep_gui_od3d.hpp"
#include "widgets_od3d.hpp"

#include <algorithm>
#include <iterator>
#include <feature/od3d/sleep_od3d.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    namespace sl = sleep;

    // Délais à la minute près (retour du 24/11 : « une durée personnalisée »), dans l'éditeur de valeur
    constexpr NumericInputConfig delay_config { .min_value = 1, .max_value = sl::delay_max_min };
    constexpr NumericInputConfig heaters_delay_config { .min_value = 0, .max_value = sl::delay_max_min };

    uint8_t get_delay() {
        return sl::settings().delay_min;
    }
    void set_delay(uint8_t v) {
        sl::Settings s = sl::settings();
        s.delay_min = std::clamp<uint8_t>(v, 1, sl::delay_max_min);
        sl::set_settings(s);
    }
    uint8_t get_heaters_delay() {
        return sl::settings().heaters_delay_min;
    }
    void set_heaters_delay(uint8_t v) {
        sl::Settings s = sl::settings();
        s.heaters_delay_min = std::min(v, sl::delay_max_min);
        sl::set_settings(s);
    }

    // Les pourcentages s'écrivent pareil en français
    constexpr const char *z_hold_items[] = { "100 %", "50 %", "25 %", N_("Off") };
    static_assert(std::size(z_hold_items) == std::size(sl::z_hold_levels));

    size_t z_hold_index() {
        const auto it = std::ranges::find(sl::z_hold_levels, sl::z_hold());
        return it == std::end(sl::z_hold_levels) ? 1 : size_t(it - std::begin(sl::z_hold_levels));
    }

} // namespace

MI_SLEEP::MI_SLEEP()
    : IWindowMenuItem(_("Sleep mode\nScreen, lights, heaters, motors"), &img::settings_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_SLEEP::click(IWindowMenu &) {
    open_screen<ScreenSleep>();
}

MI_SLEEP_NOW::MI_SLEEP_NOW()
    : IWindowMenuItem(_("Sleep now"), &img::home_shape_16x16, is_enabled_t::yes, is_hidden_t::no) {}

void MI_SLEEP_NOW::click(IWindowMenu &) {
    if (!sl::sleep_now()) {
        MsgBoxWarning(_("Not while printing or during an operation."), Responses_Ok);
    }
}

MI_SLEEP_SWITCH::MI_SLEEP_SWITCH(bool sl::Settings::*field, const char *label, const img::Resource *icon)
    : WI_ICON_SWITCH_OFF_ON_t(sl::settings().*field, _(label), icon)
    , field(field) {}

void MI_SLEEP_SWITCH::OnChange(size_t) {
    sl::Settings s = sl::settings();
    s.*field = value();
    sl::set_settings(s);
}

// Chaque réglage vaut dès la prochaine mise en veille
MI_SLEEP_ENABLE::MI_SLEEP_ENABLE()
    : MI_SLEEP_SWITCH(&sl::Settings::enabled, N_("Automatic sleep\nAfter the delay without use"), &img::settings_16x16) {}

MI_SLEEP_DELAY::MI_SLEEP_DELAY()
    : MI_OD3D_NUMBER(_("Delay\nWithout touch, knob or job, 1 to 240 min"), &img::speed_16x16, N_("Sleep delay"), delay_config, "min",
        get_delay, set_delay) {}

MI_SLEEP_SCREEN::MI_SLEEP_SCREEN()
    : MI_SLEEP_SWITCH(&sl::Settings::screen, N_("Screen off\nBacklight off, black screen"), &img::info_16x16) {}

MI_SLEEP_LIGHT::MI_SLEEP_LIGHT()
    : MI_SLEEP_SWITCH(&sl::Settings::light, N_("Lights off\nChamber, status bar and bed"), &img::enclosure_16x16) {}

MI_SLEEP_HEATERS::MI_SLEEP_HEATERS()
    : MI_SLEEP_SWITCH(&sl::Settings::heaters, N_("Heaters off\nNozzle and bed, never while printing"), &img::temperature_16x16) {}

MI_SLEEP_HEATERS_DELAY::MI_SLEEP_HEATERS_DELAY()
    : MI_OD3D_NUMBER(_("Heaters delay\nAfter falling asleep, 0: immediately"), &img::heatbed_16x16, N_("Heaters delay"), heaters_delay_config, "min",
        get_heaters_delay, set_heaters_delay) {}

MI_SLEEP_MOTORS::MI_SLEEP_MOTORS()
    : MI_SLEEP_SWITCH(&sl::Settings::motors, N_("Motors off\nX, Y and extruder: homing is lost"), &img::move_16x16) {}

MI_SLEEP_Z_HOLD::MI_SLEEP_Z_HOLD()
    : MenuItemSwitch(_("Z motor hold\nLower is quieter; Off: the bed may drop"), z_hold_items, z_hold_index()) {
    SetIconId(&img::move_16x16);
}

bool MI_SLEEP_Z_HOLD::on_item_selected(const OnItemSelectedArgs &args) {
    sl::set_z_hold(sl::z_hold_levels[args.new_index]);
    return MenuItemSwitch::on_item_selected(args);
}

MI_SLEEP_FANS::MI_SLEEP_FANS()
    : MI_SLEEP_SWITCH(&sl::Settings::fans, N_("Fans off\nChamber fans and filtration"), &img::turbine_16x16) {}

ScreenSleep::ScreenSleep()
    : BasicScreenMenu(_("Sleep mode")) {}

} // namespace od3d
