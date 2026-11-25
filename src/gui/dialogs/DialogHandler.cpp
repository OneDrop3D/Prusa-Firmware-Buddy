#include "DialogHandler.hpp"

#include "IScreenPrinting.hpp"
#include "ScreenHandler.hpp"
#include "screen_printing.hpp"
#include "config_features.h"
#include <screen/screen_print_preview.hpp>
#include "window_dlg_quickpause.hpp"
#include "window_dlg_wait.hpp"
#include "window_dlg_warning.hpp"
#include <common/marlin_vars.hpp>
#include <utility>
#include <option/has_crash_detection.h>
#include <option/has_gearbox_alignment.h>
#include <option/has_phase_stepping_calibration.h>
#include <option/has_input_shaper_calibration.h>
#include <option/has_coldpull.h>
#include <option/has_door_sensor_calibration.h>
#include <option/has_manual_belt_tuning.h>
#include <option/has_loadcell.h>
#include <option/has_indx.h>
#include <option/has_heaters_selftest_gcode.h>
#include <option/has_tool_offset_sensor.h>
#include <option/has_wastebin_fill_tracking.h>
#include <gui/screen/screen_preheat.hpp>
#include <gui/screen/dialog_safety_timer.hpp>
#include <option/has_mmu2.h>
#include <option/has_nozzle_thermal_compensation.h>
#include <option/has_od3d_ui.h>
#if HAS_OD3D_UI()
    #include <gui/od3d/screen_printing_od3d.hpp> // OneDrop3D : interface G
    #include <marlin_client.hpp>
    #include <gui/od3d/screen_progress_od3d.hpp>
    #include <gui/od3d/island_od3d.hpp>
    #include <gui/od3d/dialog_wait_od3d.hpp>
    #include <gui/od3d/screen_calib_assistant_od3d.hpp>
    #include <gui/od3d/widgets_od3d.hpp>
    #include <i18n.h>
#else
    #include <gui/screen/screen_filament_change.hpp>
#endif

#if HAS_LOADCELL()
    #include <gui/screen/screen_nozzle_cleaning_failed.hpp>
#endif

#if HAS_INDX()
    #include <gui/screen/screen_nozzle_mismatch.hpp>
    #include <feature/indx_dock_calibration/screen_dock_calibration.hpp>
    #include <feature/indx_nozzle_cleaner_calibration/screen_nozzle_cleaner_calibration.hpp>
#endif
#if HAS_GANTRY_SQUARENESS_CHECK()
    #include <feature/indx_gantry_squareness/screen_gantry_squareness.hpp>
#endif

#if HAS_WASTEBIN_FILL_TRACKING()
    #include <feature/wastebin_watcher/screen_wastebin_emptying.hpp>
#endif

#if HAS_TOOL_OFFSET_SENSOR()
    #include <feature/tool_offset_wizard/screen_tool_offset_wizard.hpp>
#endif

#if HAS_MANUAL_BELT_TUNING()
    #include <screen/selftest/screen_manual_belt_tuning.hpp>
#endif

#if HAS_COLDPULL()
    #include "screen_cold_pull.hpp"
#endif

#if HAS_SELFTEST()
    #include <screen_fan_selftest.hpp>
    #if HAS_HEATERS_SELFTEST_GCODE()
        #include <screen_heaters_selftest.hpp>
    #endif
    #include "ScreenSelftest.hpp"
    #include <gui/screen/selftest/screen_selftest_fsensors.hpp>
#endif

#if HAS_CRASH_DETECTION()
    #include "screen_crash_recovery.hpp"
#endif

#include <option/has_serial_print.h>
#if HAS_SERIAL_PRINT()
    #include "screen_printing_serial.hpp"
#endif

#if HAS_PHASE_STEPPING_CALIBRATION()
    #include "screen_phase_stepping_calibration.hpp"
#endif

#if HAS_INPUT_SHAPER_CALIBRATION()
    #include "screen_input_shaper_calibration.hpp"
#endif

#if HAS_GEARBOX_ALIGNMENT()
    #include "feature/gearbox_alignment/screen_gearbox_alignment.hpp"
#endif

#if HAS_DOOR_SENSOR_CALIBRATION()
    #include <feature/door_sensor_calibration/screen_door_sensor_calibration.hpp>
#endif

#include <option/has_esp.h>
#include <bsod/bsod.h>
#if HAS_ESP()
    #include <screen_network_setup.hpp>
#endif

alignas(std::max_align_t) static std::array<uint8_t, 1300> mem_space;

// safer than make_static_unique_ptr, checks storage size
template <class T, class... Args>
static static_unique_ptr<IDialogMarlin> make_dialog_ptr(Args &&...args) {
    return make_static_unique_ptr<T>(mem_space, std::forward<Args>(args)...);
}

struct FSMScreenDefBase {
    [[nodiscard]] static bool open_base(ScreenFactory::Creator screen, [[maybe_unused]] fsm::BaseData data) {
        // We need to get out of the GUI loop to open the screen, windows are on the stack
        if (gui_get_nesting()) {
            return false;
        }

        if (Screens::Access()->IsScreenOpened(screen)) {
            // Already opened, no need to do anything
            return true;
        }

        Screens::Access()->Open(screen);

        // Do loop immediately for the window open to take effect
        // Otherwise, the change() that follows right after open() would not do anything
        Screens::Access()->Loop();

        return true;
    }
};

template <ClientFSM fsm_, typename Screen>
struct FSMScreenDef : public FSMScreenDefBase {
    static constexpr ClientFSM fsm = fsm_;
    static constexpr auto screen = ScreenFactory::Screen<Screen>;

    [[nodiscard]] static bool open(fsm::BaseData data) {
        return open_base(screen, data);
    }

    static void close() {
        debug_assert(Screens::Access()->IsScreenOnStack<Screen>());
        Screens::Access()->close_screen<Screen>();
    }

    [[nodiscard]] static bool change(fsm::BaseData data) {
        // DO NOT change screens while in the nested loop
        // We would be potentially changing phases while the GUI widgets are on the stack
        if (gui_get_nesting()) {
            return false;
        }

        if (auto s = Screens::Access()->get<Screen>()) {
            s->Change(data);
            return true;

        } else {
            // The screen is not on the top of the stack, we cannot notify it about the change
            // TODO: This should be okay as the screen should read the current state at the creation, but it's not doing that currently
            return false;
        }
    }

#if HAS_OD3D_UI()
    /// OneDrop3D : nos onglets (Screens::CloseAll) peuvent fermer l'écran de la machine à états depuis un écran ouvert
    /// au-dessus (« Changer les filaments » de l'association des outils, par exemple) alors qu'elle attend une réponse :
    /// plus rien ne l'affichait et l'imprimante restait bloquée. Il est rouvert, avec ses dernières données.
    static void reopen_if_lost(fsm::BaseData data) {
        if (gui_get_nesting() || Screens::Access()->IsScreenOnStack<Screen>()) {
            return;
        }
        if (open_base(screen, data)) {
            (void)change(data);
        }
    }
#endif
};

struct FSMDialogDefBase {

    static void close() {
        // Ptr is a static_unique_ptr, it will call the destructor
        DialogHandler::Access().ptr = nullptr;
    }

    [[nodiscard]] static bool change(fsm::BaseData data) {
        // We CANNOT check for gui nesting for dialogs - dialogs can be shown blockingly over screens
        // One just has to hope that noone would call DialogHandler::Loop() inside a FSM dialog function - there's no nice way to check it
        // debug_assert(!gui_get_nesting())

        if (auto &ptr = DialogHandler::Access().ptr) {
            ptr->Change(data);
            return true;

        } else {
            // The dialog is not on the stact - should not happen
            debug_assert(false);
            return false;
        }
    }
};

template <ClientFSM fsm_, typename Dialog_>
struct FSMDialogDef : public FSMDialogDefBase {
    using Dialog = Dialog_;
    static constexpr ClientFSM fsm = fsm_;

    [[nodiscard]] static bool open(fsm::BaseData data) {
        auto &ptr = DialogHandler::Access().ptr;
        if (ptr) {
            // This should never happen - appropriate close() should always be called before
            bsod("Opening 2nd FSM dialog");
        }

        // If we're about to close some screen, do it now, so that we can spawn the dialog directly over the new top
        // This reduces redraws
        if (!gui_get_nesting()) {
            Screens::Access()->Loop();
        }

        ptr = make_dialog_ptr<Dialog>(data);
        return true;
    }
};

template <ClientFSM fsm_>
struct FSMPrintDef {
    static constexpr ClientFSM fsm = fsm_;

    [[nodiscard]] static bool open([[maybe_unused]] fsm::BaseData data) {
        // We need to get out of the GUI loop to open the screen, windows are on the stack
        if (gui_get_nesting()) {
            return false;
        }

        ScreenFactory::Creator screen;

#if HAS_SERIAL_PRINT()
        if constexpr (fsm == ClientFSM::Serial_printing) {
            screen = ScreenFactory::Screen<screen_printing_serial_data_t>;
            if (Screens::Access()->IsScreenOpened(screen)) {
                // Already opened, no need to do anything
                return true;
            }

            Screens::Access()->ClosePrinting();

        } else
#endif
            if constexpr (fsm == ClientFSM::Printing) {
#if HAS_OD3D_UI()
            // OneDrop3D : l'impression redevient la machine du dessus après un changement d'outil, une alerte... :
            // l'utilisateur reste où il est (la pastille de l'en-tête ramène à l'écran Impression)
            if (shown) {
                return true;
            }
            shown = true;
            screen = ScreenFactory::Screen<od3d::ScreenPrinting>; // OneDrop3D : notre écran Impression remplace celui de Prusa
#else
            screen = ScreenFactory::Screen<screen_printing_data_t>;
#endif
            if (Screens::Access()->IsScreenOpened(screen)) {
                // Already opened, no need to do anything
                return true;
            }

            Screens::Access()->CloseAll();

        } else {
            static_assert(0);
        }

        Screens::Access()->Open(screen);

        // Do loop immediately for the window open to take effect
        // Otherwise, the change() that follows right after open() would not do anything
        Screens::Access()->Loop();
        return true;
    }

    static void close() {
#if HAS_OD3D_UI()
        if constexpr (fsm == ClientFSM::Printing) {
            // OneDrop3D : seulement recouverte par une autre machine (changement d'outil du MMU, alerte...) : Prusa
            // fermait tous les écrans à chaque fois, l'utilisateur perdait sa place pendant toute l'impression
            if (marlin_vars().peek_fsm_states([](const auto &states) { return states.is_active(ClientFSM::Printing); })) {
                return;
            }
            shown = false;
            // Écran Impression déjà quitté (fin laissée par un onglet, voir update_screen) : on reste où on est
            if (!Screens::Access()->IsScreenOnStack<od3d::ScreenPrinting>()) {
                return;
            }
        }
#endif
        Screens::Access()->CloseAll();
    }

    [[nodiscard]] static bool change([[maybe_unused]] fsm::BaseData data) {
        // Do nothing
        return true;
    }

#if HAS_OD3D_UI()
    static inline bool shown = false; ///< OneDrop3D : écran Impression déjà ouvert pour cette impression
#endif
};

#if HAS_OD3D_UI()
/// OneDrop3D : l'écran de progression peut être réduit dans l'en-tête (island_od3d.hpp). Réduit, il n'est plus sur la
/// pile : les nouvelles données vont à la pastille, et l'écran revient dès que Prusa attend l'utilisateur.
struct FSMLoadUnloadDef : public FSMScreenDef<ClientFSM::Load_unload, od3d::ScreenLoadUnload> {
    using Base = FSMScreenDef<ClientFSM::Load_unload, od3d::ScreenLoadUnload>;

    [[nodiscard]] static bool open(fsm::BaseData data) {
        if (gui_get_nesting()) {
            return false; // comme open_base() : change() suit open() et ne doit pas trouver l'interface imbriquée
        }
        if (od3d::island::open_minimized(data)) {
            return true; // en arrière-plan d'emblée (changement d'outil pendant une impression...) : change() rouvre si besoin
        }
        od3d::island::end(); // nouvelle opération : à l'écran
        return Base::open(data);
    }

    static void reopen_if_lost(fsm::BaseData data) {
        if (!od3d::island::minimized()) { // réduit dans l'en-tête, il n'est pas sur la pile exprès
            Base::reopen_if_lost(data);
        }
    }

    static void close() {
        if (od3d::island::minimized()) {
            od3d::island::end(); // l'écran n'est pas sur la pile : il n'y a que la pastille à retirer
            return;
        }
        Base::close();
    }

    [[nodiscard]] static bool change(fsm::BaseData data) {
        if (od3d::island::minimized()) {
            if (!od3d::ScreenLoadUnload::needs_user(data)) {
                od3d::island::update(data);
                return true;
            }
            if (gui_get_nesting()) {
                return false; // rouvert au prochain tour, hors de la boucle de l'interface
            }
            od3d::island::prepare_reopen();
            if (!open_base(screen, data)) {
                return false;
            }
        }
        return Base::change(data);
    }
};
#endif

#if HAS_OD3D_UI()
/// OneDrop3D : le « Patientez » (origine, palpage) peut être réduit dans l'en-tête (dialog_wait_od3d.hpp) ; pendant nos
/// processus, il y reste d'emblée. Réduit, le dialogue n'existe pas : la machine n'attend jamais l'utilisateur, ses
/// alertes sont d'autres machines, qui s'ouvrent par-dessus. Ouvert ou fermé ici seulement, hors de ses événements.
struct FSMWaitDef : public FSMDialogDef<ClientFSM::Wait, od3d::DialogWait> {
    using Base = FSMDialogDef<ClientFSM::Wait, od3d::DialogWait>;

    [[nodiscard]] static bool open(fsm::BaseData data) {
        return od3d::island::wait_opens_minimized() || Base::open(data);
    }

    static void close() {
        od3d::island::wait_end();
        Base::close();
    }

    [[nodiscard]] static bool change(fsm::BaseData data) {
        return !DialogHandler::Access().ptr || Base::change(data);
    }

    /// À chaque tour : réduit depuis son bouton (ou sa flèche), ou rouvert depuis la pastille
    static void reopen_if_lost(fsm::BaseData data) {
        if (gui_get_nesting()) {
            return;
        }
        auto &ptr = DialogHandler::Access().ptr;
        if (od3d::island::wait_minimized() && ptr) {
            Base::close();
        } else if (!od3d::island::wait_minimized() && !ptr) {
            (void)Base::open(data);
        }
    }
};
#endif

#if HAS_OD3D_UI()
/// OneDrop3D : pendant une série de l'assistant de calibration, les assistants de Prusa qu'elle lance sans question
/// (M1959 A, M1977 U A) ne prennent pas l'écran : on garde la main (retour, onglets), l'écran de l'assistant les suit en
/// direct, la pastille de l'en-tête montre l'étape. Son « Annuler » leur envoie « Abandonner ».
template <ClientFSM fsm_, typename Screen_>
struct FSMAssistantDef : public FSMScreenDef<fsm_, Screen_> {
    using Base = FSMScreenDef<fsm_, Screen_>;
    static inline bool hidden = false;

    [[nodiscard]] static bool open(fsm::BaseData data) {
        hidden = od3d::calibration_series_running();
        return hidden || Base::open(data);
    }

    static void close() {
        if (std::exchange(hidden, false)) {
            return;
        }
        Base::close();
    }

    [[nodiscard]] static bool change(fsm::BaseData data) {
        return hidden || Base::change(data);
    }

    static void reopen_if_lost(fsm::BaseData data) {
        if (!hidden) {
            Base::reopen_if_lost(data);
        }
    }
};
#endif

// Just so that we have something at the end of the list and don't have to care about commas
struct FSMEndDef {
    static constexpr ClientFSM fsm = ClientFSM::_count;

    [[nodiscard]] static bool open(fsm::BaseData) {
        return true;
    }

    static void close() {}

    [[nodiscard]] static bool change(fsm::BaseData) {
        return true;
    }
};

template <class... T>
struct FSMDisplayConfigDef {
};

using FSMDisplayConfig = FSMDisplayConfigDef<
#if HAS_OD3D_UI()
    FSMWaitDef, // OneDrop3D : réductible dans l'en-tête
#else
    FSMDialogDef<ClientFSM::Wait, window_dlg_wait_t>,
#endif
    FSMDialogDef<ClientFSM::SafetyTimer, DialogSafetyTimer>,
#if HAS_SERIAL_PRINT()
    FSMPrintDef<ClientFSM::Serial_printing>,
#endif
#if HAS_OD3D_UI()
    FSMLoadUnloadDef, // OneDrop3D : même machine, apparence G, réductible dans l'en-tête
#else
    FSMScreenDef<ClientFSM::Load_unload, ScreenFilamentChange>,
#endif
    FSMScreenDef<ClientFSM::Preheat, ScreenPreheat>,
#if HAS_SELFTEST()
    FSMScreenDef<ClientFSM::Selftest, ScreenSelftest>,
    FSMScreenDef<ClientFSM::FansSelftest, ScreenFanSelftest>,
    FSMScreenDef<ClientFSM::SelftestFSensors, ScreenSelftestFSensors>,
#endif
#if HAS_HEATERS_SELFTEST_GCODE()
    FSMScreenDef<ClientFSM::HeatersSelftest, ScreenHeatersSelftest>,
#endif
#if HAS_ESP()
    FSMScreenDef<ClientFSM::NetworkSetup, ScreenNetworkSetup>,
#endif
    FSMPrintDef<ClientFSM::Printing>,
#if HAS_CRASH_DETECTION()
    FSMScreenDef<ClientFSM::CrashRecovery, ScreenCrashRecovery>,
#endif
    FSMDialogDef<ClientFSM::QuickPause, DialogQuickPause>,
    FSMDialogDef<ClientFSM::Warning, DialogWarning>,
    FSMScreenDef<ClientFSM::PrintPreview, ScreenPrintPreview>,
#if HAS_COLDPULL()
    FSMScreenDef<ClientFSM::ColdPull, ScreenColdPull>,
#endif
#if HAS_PHASE_STEPPING_CALIBRATION()
    #if HAS_OD3D_UI()
    FSMAssistantDef<ClientFSM::PhaseSteppingCalibration, ScreenPhaseSteppingCalibration>, // OneDrop3D : pastille pendant l'assistant
    #else
    FSMScreenDef<ClientFSM::PhaseSteppingCalibration, ScreenPhaseSteppingCalibration>,
    #endif
#endif
#if HAS_INPUT_SHAPER_CALIBRATION()
    #if HAS_OD3D_UI()
    FSMAssistantDef<ClientFSM::InputShaperCalibration, ScreenInputShaperCalibration>, // OneDrop3D : pastille pendant l'assistant
    #else
    FSMScreenDef<ClientFSM::InputShaperCalibration, ScreenInputShaperCalibration>,
    #endif
#endif
#if HAS_GEARBOX_ALIGNMENT()
    FSMScreenDef<ClientFSM::GearboxAlignment, ScreenGearboxAlignment>,
#endif
#if HAS_DOOR_SENSOR_CALIBRATION()
    FSMScreenDef<ClientFSM::DoorSensorCalibration, ScreenDoorSensorCalibration>,
#endif
#if HAS_MANUAL_BELT_TUNING()
    FSMScreenDef<ClientFSM::ManualBeltTuning, ScreenManualBeltTuning>,
#endif
#if HAS_LOADCELL()
    FSMScreenDef<ClientFSM::NozzleCleaningFailed, ScreenNozzleCleaningFailed>,
#endif
#if HAS_INDX()
    FSMScreenDef<ClientFSM::NozzleMismatch, ScreenNozzleMismatch>,
    FSMScreenDef<ClientFSM::DockCalibration, ScreenDockCalibration>,
    FSMScreenDef<ClientFSM::NozzleCleanerCalibration, ScreenNozzleCleanerCalibration>,
#endif
#if HAS_GANTRY_SQUARENESS_CHECK()
    FSMScreenDef<ClientFSM::GantrySquareness, ScreenGantrySquareness>,
#endif
#if HAS_WASTEBIN_FILL_TRACKING()
    FSMScreenDef<ClientFSM::WastebinEmptying, ScreenWastebinEmptying>,
#endif
#if HAS_TOOL_OFFSET_SENSOR()
    FSMScreenDef<ClientFSM::ToolOffsetsCalibration, ScreenToolOffsetWizard>,
#endif
    // This is here so that we can worry-free write commas at the end of each argument
    FSMEndDef>;

void visit_display_config(ClientFSM fsm, auto f) {
    [&]<class... T>(FSMDisplayConfigDef<T...>) {
        ((fsm == T::fsm ? f(T()) : void()), ...);
    }(FSMDisplayConfig());
};

static constexpr size_t fsm_display_config_size = []<class... T>(FSMDisplayConfigDef<T...>) { return sizeof...(T); }(FSMDisplayConfig());
static_assert(fsm_display_config_size == std::to_underlying(ClientFSM::_count) + 1);

static constexpr auto dlg_size = []<typename T> {
    if constexpr (std::is_base_of_v<FSMDialogDefBase, T>) {
        return sizeof(typename T::Dialog);
    } else {
        return 0;
    }
};
static constexpr size_t required_dlg_mem_space = []<class... T>(FSMDisplayConfigDef<T...>) { return std::max(std::initializer_list<size_t> { dlg_size.operator()<T>()... }); }(FSMDisplayConfig {});
static_assert(sizeof(mem_space) < required_dlg_mem_space + 100, "DialogHandler::mem_space is unnecessarily big");

//*****************************************************************************
// method definitions
bool DialogHandler::open(ClientFSM fsm_type, fsm::BaseData data) {
    bool result = false;
    visit_display_config(fsm_type, [&]<typename Config>(Config) {
        result = Config::open(data);
    });
    return result;
}

void DialogHandler::close(ClientFSM fsm_type) {
    visit_display_config(fsm_type, []<typename Config>(Config) {
        Config::close();
    });
}

bool DialogHandler::change(ClientFSM fsm_type, fsm::BaseData data) {
    bool result = false;

    visit_display_config(fsm_type, [&]<typename Config>(Config) {
        result = Config::change(data);
    });

    return result;
}

bool DialogHandler::IsOpen() const {
    return ptr != nullptr;
}

DialogHandler &DialogHandler::Access() {
    static DialogHandler instance;
    return instance;
}

void DialogHandler::Loop() {
    update_screen();

    // If a Screen gets destroyed underneath a FSM Dialog (IDialogMarlin), we have to re-register it to the new screen
    // Do this after update_screen to reduce GUI redraws
    if (ptr && !ptr->GetParent()) {
        auto current_screen = Screens::Access()->Get();
        ptr->SetParent(current_screen);
        current_screen->RegisterSubWin(*ptr);
    }
}

void DialogHandler::update_screen() {
    const auto new_top = marlin_vars().peek_fsm_states([](const auto &states) { return states.get_top(); });

#if HAS_OD3D_UI()
    // OneDrop3D : impression finie pendant qu'une autre machine la recouvrait (son close() n'a pas été appelé) : la
    // suivante doit rouvrir l'écran Impression
    if (FSMPrintDef<ClientFSM::Printing>::shown && !marlin_vars().peek_fsm_states([](const auto &states) { return states.is_active(ClientFSM::Printing); })) {
        FSMPrintDef<ClientFSM::Printing>::shown = false;
    }

    // OneDrop3D : impression terminée ou arrêtée dont l'écran n'est plus sur la pile (onglet, retour, fin pendant qu'on
    // était ailleurs) : fermée comme par son « Accueil », plus de pastille ni d'écran de fin qui revient (retour du
    // 24/11 : « aprés un aret ou la fin d'une impression si on a pas cliquer sur accueil on a toujour le pop up »)
    // I-04 : un bandeau le dit, une fois (sinon elle disparaissait sans trace). Pas en Exit : on a déjà quitté l'écran de
    // fin par son « Accueil ».
    static bool end_noticed = false;
    if (FSMPrintDef<ClientFSM::Printing>::shown && od3d::print_ended() && !Screens::Access()->IsScreenOnStack<od3d::ScreenPrinting>()) {
        const marlin_server::State state = marlin_vars().print_state;
        if (!end_noticed && state != marlin_server::State::Exit) {
            od3d::show_notice(state == marlin_server::State::Finished ? _("Print\nfinished") : _("Print\nstopped"));
        }
        end_noticed = true;
        marlin_client::print_exit(); // sans effet hors de Finished / Aborted
    } else if (!od3d::print_ended()) {
        end_noticed = false;
    }

    // OneDrop3D : écran de la machine à états fermé par nos onglets alors qu'elle continue (voir reopen_if_lost)
    if (new_top && current_fsm_top && new_top->fsm_type == current_fsm_top->fsm_type) {
        visit_display_config(new_top->fsm_type, [&]<typename Config>(Config) {
            if constexpr (requires { Config::reopen_if_lost(new_top->data); }) {
                Config::reopen_if_lost(new_top->data);
            }
        });
    }
#endif

    if (new_top == current_fsm_top) {
        return;
    }

    // Shortcut - we're just changing the data
    if (new_top && current_fsm_top && new_top->fsm_type == current_fsm_top->fsm_type) {
        if (!change(new_top->fsm_type, new_top->data)) {
            // Failed to change state - try again later
            return;
        }
        current_fsm_top = new_top;
        return;
    }

    // Close the previous screen/dialog
    if (current_fsm_top) {
        // Closing should never fail
        // Dialog - destroys the dialog (we're praying that noone got the idea to call DialogHandler::Loop() inside a FSM dialog function - there's no nice way to check it)
        // Screen - just marks the screen to be destroyed on the next Screens::Access()->Loop
        close(current_fsm_top->fsm_type);
        current_fsm_top = std::nullopt;
    }

    // Do NOT do Screens::Access()->Loop(); here.
    // The ScreenHandler works faster if it can do both close and open in the same step

    // Open the new one
    if (new_top) {
        if (!open(new_top->fsm_type, new_top->data)) {
            return;
        }

        if (!change(new_top->fsm_type, new_top->data)) {
            bsod_unreachable(); // Should never fail, we should just have opened the screen
        }

        current_fsm_top = new_top;
    }

    // Make sure all changes are carried over now to reduce redraws
    if (!gui_get_nesting()) {
        Screens::Access()->Loop();
    }
}
