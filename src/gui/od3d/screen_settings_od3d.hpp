/// @file
/// OneDrop3D (interface G) : groupes des réglages de Prusa (retour du 22/11, « clarifier les menus de Prusa »).
/// Les éléments restent ceux de Prusa (MI_*), avec leur logique ; seul leur rangement change :
/// - « Impression » : détection de filament coincé, fin de filament, ventilateurs, G-code, rétraction, mode furtif ;
/// - « Filament et MMU » : capteurs de filament, MMU, coupe-filament, OpenPrintTag ;
/// - « Chambre » : filtration, lumières, caméra ;
/// - Système › « Diagnostic » : vidage mémoire, journal, empreinte dans le QR, métriques, vitesse et correctif de
///   l'écran (outils de support, sortis de Système, Réseau et Interface).
/// Menus de Prusa dessinés en style G (menu_od3d) : doigt et molette ; aucun windowEvent redéfini.
#pragma once

#include "menu_items_od3d.hpp"

#include <basic_screen_menu.hpp>
#include <MItem_crash.hpp>
#include <MItem_menus.hpp>
#include <MItem_mmu.hpp>
#include <MItem_tools.hpp>
#include <gui/menu_item/specific/menu_items_auto_retract.hpp>
#include <gui/menu_item/specific/menu_items_xbuddy_extension.hpp>
#include <screen/openprinttag/screen_opt_settings.hpp>
#include <screen/toolhead/screen_toolhead_settings.hpp>
#include <option/has_touch.h>
#if HAS_TOUCH()
    #include <MItem_touch.hpp>
#endif

namespace od3d {

class ScreenSetPrinting final : public BasicScreenMenu<MI_STUCK_FILAMENT_DETECTION, MI_GCODE_INTERRUPTS, MI_CRASH_DETECTION, MI_FAN_CHECK, MI_GCODE_VERIFY,
                                    MI_AUTO_RETRACT_ENABLE, MI_STEALTH_MODE> {
public:
    ScreenSetPrinting();
};

class ScreenSetFilament final : public BasicScreenMenu<MI_FILAMENT_SENSORS, MI_MMU_ENABLE, MI_MMU_BOOTLOADER_RESULT, MI_MMU_CUTTER, buddy::openprinttag::MI_OPT_SETTINGS> {
public:
    ScreenSetFilament();
};

class ScreenSetChamber final : public BasicScreenMenu<MI_CHAMBER_FILTRATION, MI_LIGHTS, MI_CAM_USB_PWR> {
public:
    ScreenSetChamber();
};

class ScreenDiagnostics final : public BasicScreenMenu<MI_SAVE_DUMP, MI_LOG_TO_TXT, MI_DEVHASH_IN_QR, MI_METRICS_SETTINGS, MI_DISPLAY_BAUDRATE
#if HAS_TOUCH()
                                    ,
                                    TOUCH_SIG_WORKAROUND
#endif
                                    > {
public:
    ScreenDiagnostics();
};

} // namespace od3d
