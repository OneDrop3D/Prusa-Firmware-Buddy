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

    // Les minutes s'écrivent pareil en français : pas de traduction (_() rend le texte tel quel)
    constexpr const char *delay_items[] = { "1 min", "2 min", "5 min", "10 min", "15 min", "30 min", "60 min" };
    constexpr const char *heaters_delay_items[] = { N_("Immediately"), "1 min", "2 min", "5 min", "10 min", "15 min", "30 min", "60 min" };
    static_assert(std::size(delay_items) == std::size(sl::delays_min) && std::size(heaters_delay_items) == std::size(sl::delays_min) + 1);

    size_t index_of(uint8_t minutes, bool with_zero) {
        if (with_zero && minutes == 0) {
            return 0;
        }
        const auto it = std::ranges::find(sl::delays_min, minutes);
        const size_t i = it == std::end(sl::delays_min) ? 3 : size_t(it - std::begin(sl::delays_min)); // inconnu : 10 min
        return i + with_zero;
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

MI_SLEEP_MINUTES::MI_SLEEP_MINUTES(uint8_t sl::Settings::*field, const char *label, const img::Resource *icon, bool with_zero)
    : MenuItemSwitch(_(label), with_zero ? std::span<const char *const>(heaters_delay_items) : std::span<const char *const>(delay_items),
        index_of(sl::settings().*field, with_zero))
    , field(field)
    , with_zero(with_zero) {
    SetIconId(icon);
}

bool MI_SLEEP_MINUTES::on_item_selected(const OnItemSelectedArgs &args) {
    sl::Settings s = sl::settings();
    s.*field = (with_zero && args.new_index == 0) ? 0 : sl::delays_min[args.new_index - with_zero];
    sl::set_settings(s);
    return MenuItemSwitch::on_item_selected(args);
}

// Chaque réglage vaut dès la prochaine mise en veille
MI_SLEEP_ENABLE::MI_SLEEP_ENABLE()
    : MI_SLEEP_SWITCH(&sl::Settings::enabled, N_("Automatic sleep\nAfter the delay without use"), &img::settings_16x16) {}

MI_SLEEP_DELAY::MI_SLEEP_DELAY()
    : MI_SLEEP_MINUTES(&sl::Settings::delay_min, N_("Delay\nWithout touch, knob or job"), &img::speed_16x16, false) {}

MI_SLEEP_SCREEN::MI_SLEEP_SCREEN()
    : MI_SLEEP_SWITCH(&sl::Settings::screen, N_("Screen off\nBacklight off, black screen"), &img::info_16x16) {}

MI_SLEEP_LIGHT::MI_SLEEP_LIGHT()
    : MI_SLEEP_SWITCH(&sl::Settings::light, N_("Lights off\nChamber, status bar and bed"), &img::enclosure_16x16) {}

MI_SLEEP_HEATERS::MI_SLEEP_HEATERS()
    : MI_SLEEP_SWITCH(&sl::Settings::heaters, N_("Heaters off\nNozzle and bed, never while printing"), &img::temperature_16x16) {}

MI_SLEEP_HEATERS_DELAY::MI_SLEEP_HEATERS_DELAY()
    : MI_SLEEP_MINUTES(&sl::Settings::heaters_delay_min, N_("Heaters delay\nAfter falling asleep"), &img::heatbed_16x16, true) {}

MI_SLEEP_MOTORS::MI_SLEEP_MOTORS()
    : MI_SLEEP_SWITCH(&sl::Settings::motors, N_("Motors off\nX, Y and extruder: homing is lost"), &img::move_16x16) {}

MI_SLEEP_FANS::MI_SLEEP_FANS()
    : MI_SLEEP_SWITCH(&sl::Settings::fans, N_("Fans off\nChamber fans and filtration"), &img::turbine_16x16) {}

ScreenSleep::ScreenSleep()
    : BasicScreenMenu(_("Sleep mode")) {}

} // namespace od3d
