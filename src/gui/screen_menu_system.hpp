/// @file
#pragma once

#include "MItem_menus.hpp"
#include "MItem_tools.hpp"
#include <basic_screen_menu.hpp>
#include <option/has_e2ee_support.h>
#include <option/has_internal_storage_flash.h>
#include <option/has_mmu2.h>
#include <option/has_nozzle_thermal_compensation.h>
#include <option/has_od3d_ui.h>
#if HAS_OD3D_UI()
    #include <gui/od3d/menu_items_od3d.hpp>
#endif

using ScreenMenuSystemBase = BasicScreenMenu<
#if HAS_OD3D_UI()
    MI_LOAD_SETTINGS,
    od3d::MI_DIAGNOSTICS, // OneDrop3D : vidage, journal, empreinte du QR, métriques (outils de support)
#else
    MI_SAVE_DUMP,
    MI_LOG_TO_TXT,
    MI_DEVHASH_IN_QR,
    MI_LOAD_SETTINGS,
#endif
#if HAS_E2EE_SUPPORT()
    MI_E2EE,
#endif
#if HAS_INTERNAL_STORAGE_FLASH()
    MI_INTERNAL_STORAGE,
#endif
    MI_OPEN_FACTORY_RESET>;

class ScreenMenuSystem final : public ScreenMenuSystemBase {
public:
    ScreenMenuSystem();
};
