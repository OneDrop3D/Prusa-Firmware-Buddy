/// @file
#include "screen_menu_settings.hpp"

#include "screen_menu_experimental_settings.hpp"
#include "ScreenHandler.hpp"
#include "knob_event.hpp"
#include "img_resources.hpp"

ScreenMenuSettings::ScreenMenuSettings()
    : ScreenMenuSettingsBase {
        _("SETTINGS"),
        &img::settings_16x16,
    }
    , old_action(gui::knob::GetLongPressScreenAction()) { // backup hold action
    gui::knob::RegisterLongPressScreenAction([]() { Screens::Access()->Open(ScreenFactory::Screen<ScreenMenuExperimentalSettings>); }); // new hold action
    EnableLongHoldScreenAction();
#if HAS_NOZZLE_THERMAL_COMPENSATION() && HAS_MMU2()
    // OneDrop3D (interface G) : une ligne d'explication sous chaque sous-menu de Prusa
    Item<od3d::MI_PRINTER_INFO>().SetLabel(_("Printer info\nVersion, statistics, sensors"));
    Item<MI_TOOLHEAD_SETTINGS>().SetLabel(_("Printhead\nNozzle diameter and type, sock"));
    Item<MI_HARDWARE>().SetLabel(_("Hardware\nDoor, belts, input shaper, MMU tube"));
    Item<MI_USER_INTERFACE>().SetLabel(_("Interface\nSounds, file order, touch, footer"));
    Item<MI_NETWORK>().SetLabel(_("Network\nWi-Fi, PrusaLink, Prusa Connect"));
    Item<MI_LANG_AND_TIME>().SetLabel(_("Language and time\nLanguage, time zone, clock"));
    Item<MI_SYSTEM>().SetLabel(_("System\nFactory reset, settings file, diagnostics"));
#endif
}

ScreenMenuSettings::~ScreenMenuSettings() {
    gui::knob::RegisterLongPressScreenAction(old_action); // restore hold action
}
