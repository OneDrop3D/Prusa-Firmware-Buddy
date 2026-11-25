/// @file
#pragma once

#include "MItem_menus.hpp"
#include "MItem_tools.hpp"
#include <MItem_filament.hpp>
#include <basic_screen_menu.hpp>
#include <screen_menu_footer_settings.hpp>

#include <option/has_tool_mapping.h>
#include <option/has_mmu2.h>
#include <option/has_nozzle_thermal_compensation.h>
#include <option/has_od3d_ui.h>

#include <option/has_touch.h>
#if HAS_TOUCH()
    #include "MItem_touch.hpp"
#endif

using ScreenMenuUserInterface__ = BasicScreenMenu<
    MI_FOOTER_SETTINGS,
    MI_SOUND_MODE,
    MI_SORT_FILES,
    MI_PRINT_PROGRESS_TIME,
    MI_TIMEOUT,
    MI_FILAMENT_CHANGE_PREHEAT_ALL,
#if HAS_TOOL_MAPPING()
    MI_TOOL_MAPPING_PRESET,
#endif
    MI_ASK_COLOR_ON_LOAD,
#if HAS_ST7789_DISPLAY()
    // We could potentionally have MINI display without buzzer.
    // So we only allow sound control for ST7789
    MI_SOUND_VOLUME,
#endif
#if HAS_ILI9488_DISPLAY() && !(HAS_OD3D_UI()) // OneDrop3D : dans Système › Diagnostic
    MI_DISPLAY_BAUDRATE,
#endif
#if HAS_TOUCH()
    MI_ENABLE_TOUCH,
    #if !(HAS_OD3D_UI()) // OneDrop3D : dans Système › Diagnostic
    TOUCH_SIG_WORKAROUND,
    #endif
#endif
    MI_ALWAYS_HIDDEN>;

class ScreenMenuUserInterface final : public ScreenMenuUserInterface__ {
public:
    ScreenMenuUserInterface();
};
