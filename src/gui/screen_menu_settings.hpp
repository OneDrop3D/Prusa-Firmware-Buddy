/// @file
#pragma once

#include <basic_screen_menu.hpp>
#include "MItem_menus.hpp"
#include "MItem_tools.hpp"
#include "knob_event.hpp"
#include "MItem_crash.hpp"
#include "Configuration_adv.h"
#include <option/has_crash_detection.h>
#include <option/has_mmu2.h>
#include <option/xbuddy_extension_variant.h>
#include <option/has_indx.h>
#include <option/has_lights_menu.h>
#include <option/has_wastebin_fill_tracking.h>
#include <option/development_items.h>

#if HAS_MMU2()
    #include "MItem_mmu.hpp"
#endif

#if XBUDDY_EXTENSION_VARIANT_IS_STANDARD()
    #include <gui/menu_item/specific/menu_items_xbuddy_extension.hpp>
#endif

#include <option/has_chamber_filtration_api.h>
#if HAS_CHAMBER_FILTRATION_API()
    #include <gui/menu_item/specific/menu_items_chamber_filtration.hpp>
#endif

#include <option/has_switchable_auto_retract.h>
#if HAS_SWITCHABLE_AUTO_RETRACT()
    #include <gui/menu_item/specific/menu_items_auto_retract.hpp>
#endif

#include <option/has_anfc.h>
#if HAS_ANFC()
    #include <screen/openprinttag/screen_opt_settings.hpp>
#endif

#include <option/has_nozzle_thermal_compensation.h>
#if HAS_NOZZLE_THERMAL_COMPENSATION() && HAS_MMU2()
    #include <gui/od3d/screen_menu_od3d.hpp>
    #include <gui/od3d/screen_sleep_od3d.hpp>
    #include <gui/od3d/screen_filament_presets_od3d.hpp>
    #include <gui/od3d/menu_items_od3d.hpp>
#endif

#if HAS_NOZZLE_THERMAL_COMPENSATION() && HAS_MMU2()
// OneDrop3D (interface G, retour du 22/11 « clarifier les menus de Prusa ») : 13 lignes au lieu de 23, chacune avec une
// ligne d'explication ; les interrupteurs de Prusa sont rangés dans « Impression », « Filament et MMU » et « Chambre »
// (gui/od3d/screen_settings_od3d.hpp), rien n'est retiré. L'ordre de Prusa reste ci-dessous pour les autres imprimantes.
using ScreenMenuSettingsBase = BasicScreenMenu<
    od3d::MI_FEATURES,
    od3d::MI_PRINTER_INFO, // « Infos de l'imprimante » en deuxième, comme la maquette G
    od3d::MI_FILAMENT_PRESETS, // UI-13 : températures des préréglages de Prusa
    od3d::MI_SET_PRINTING,
    od3d::MI_SET_FILAMENT,
    MI_TOOLHEAD_SETTINGS,
    od3d::MI_SET_CHAMBER,
    MI_HARDWARE,
    MI_USER_INTERFACE,
    od3d::MI_SLEEP, // F39 : mode veille (remplace l'atténuation des lumières de Prusa)
    MI_NETWORK,
    MI_LANG_AND_TIME,
#if DEVELOPMENT_ITEMS()
    MI_DEVELOPMENT,
#endif
    // MI_SYSTEM needs to be last to ensure we can safely hit factory reset even in presence of unknown languages
    MI_SYSTEM>;
#else
using ScreenMenuSettingsBase = BasicScreenMenu<
    MI_USER_INTERFACE,
    MI_TOOLHEAD_SETTINGS,
#if HAS_FILAMENT_SENSORS_MENU()
    MI_FILAMENT_SENSORS,
#else
    MI_FILAMENT_SENSOR,
#endif
#if HAS_LIGHTS_MENU()
    MI_LIGHTS,
#endif
    MI_NETWORK,
#if HAS_LOADCELL() && !HAS_INDX()
    MI_STUCK_FILAMENT_DETECTION,
#endif
#if HAS_SWITCHABLE_AUTO_RETRACT()
    MI_AUTO_RETRACT_ENABLE,
#endif
#if HAS_ANFC()
    buddy::openprinttag::MI_OPT_SETTINGS,
#endif
#if HAS_MMU2()
    MI_MMU_ENABLE,
    MI_MMU_BOOTLOADER_RESULT,
    MI_MMU_CUTTER,
#endif
#if XBUDDY_EXTENSION_VARIANT_IS_STANDARD()
    MI_CAM_USB_PWR,
#endif
    MI_STEALTH_MODE,
    MI_FAN_CHECK,
    MI_GCODE_VERIFY,
#if HAS_WASTEBIN_FILL_TRACKING()
    MI_WASTEBIN,
#endif
#if HAS_CHAMBER_FILTRATION_API()
    MI_CHAMBER_FILTRATION,
#endif
#if HAS_CRASH_DETECTION()
    MI_CRASH_DETECTION,
    MI_GCODE_INTERRUPTS,
#endif
    MI_LANG_AND_TIME,
    MI_HARDWARE,
#if DEVELOPMENT_ITEMS()
    MI_DEVELOPMENT,
#endif
    // MI_SYSTEM needs to be last to ensure we can safely hit factory reset even in presence of unknown languages
    MI_SYSTEM>;
#endif

class ScreenMenuSettings final : public ScreenMenuSettingsBase {
    gui::knob::screen_action_cb old_action;

public:
    ScreenMenuSettings();
    ~ScreenMenuSettings();
};
