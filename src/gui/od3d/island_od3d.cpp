/// @file
#include "island_od3d.hpp"

#include "atelier_od3d.hpp"
#include "screen_printing_od3d.hpp"
#include "screen_progress_od3d.hpp"
#include "screen_temperatures_od3d.hpp"
#include "widgets_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <common/marlin_vars.hpp>
#include <fsm/filament_change_phases.hpp>
#include <fsm_loadunload_type.hpp>
#include <option/has_tool_mapping.h>
#include <ScreenFactory.hpp>
#include <ScreenHandler.hpp>

namespace od3d::island {

namespace {

    bool is_minimized = false;
    fsm::BaseData last_data;
    std::optional<Origin> last_origin;
    uint8_t last_progress = 0;
    uint16_t rev = 0;

    void take(fsm::BaseData data) {
        last_data = data;
        // MMU_ERRWaitingForUser porte un pointeur vers l'erreur, pas d'avancement (et rouvre l'écran de toute façon)
        if (GetEnumFromPhaseIndex<PhasesLoadUnload>(data.GetPhase()) != PhasesLoadUnload::MMU_ERRWaitingForUser) {
            last_progress = fsm::deserialize_data<FSMLoadUnloadData>(data.GetData()).progress;
        }
        ++rev;
    }

    PreheatLatch preheat_latch;

    /// Couleur de l'état de l'impression ; nullopt : pas de pastille (pas commencée, terminée ou arrêtée).
    /// Même découpage que ScreenPrinting::update_state().
    std::optional<Tone> print_tone(marlin_server::State state) {
        using marlin_server::State;
        switch (state) {
        case State::Idle:
        case State::PrintPreviewInit:
        case State::PrintPreviewImage:
        case State::PrintPreviewConfirmed:
        case State::PrintPreviewQuestions:
#if HAS_TOOL_MAPPING()
        case State::PrintPreviewToolsMapping:
#endif
        case State::Aborted:
        case State::Finished:
        case State::Exit:
            return std::nullopt;
        case State::Pausing_Begin:
        case State::Pausing_WaitIdle:
        case State::Pausing_ParkHead:
        case State::Paused:
            return Tone::paused;
        case State::MediaErrorRecovery_BufferData:
        case State::CrashRecovery_HOMEFAIL:
        case State::CrashRecovery_Axis_NOK:
        case State::CrashRecovery_Repeated_Crash:
            return Tone::error;
        case State::PrintInit:
        case State::PowerPanic_AwaitingResume:
        case State::Resuming_Reheating:
        case State::Resuming_BufferData:
        case State::Resuming_Begin:
        case State::Resuming_ExecutingGCodeInterrupt:
        case State::Resuming_UnparkHead_XY:
        case State::Resuming_UnparkHead_ZE:
        case State::CrashRecovery_Begin:
        case State::CrashRecovery_Retracting:
        case State::CrashRecovery_Lifting:
        case State::CrashRecovery_ToolchangePowerPanic:
        case State::CrashRecovery_XY_Measure:
        case State::CrashRecovery_XY_HOME:
        case State::PowerPanic_Resume:
        case State::Aborting_Begin:
        case State::Aborting_WaitIdle:
        case State::Aborting_UnloadFilament:
        case State::Aborting_ParkHead:
        case State::Aborting_Preview:
            return Tone::waiting;
        default: // en cours, fin en cours (Finishing_*), états propres à d'autres imprimantes
            return Tone::normal;
        }
    }

} // namespace

bool minimized() {
    return is_minimized;
}

void minimize(fsm::BaseData data, std::optional<Origin> origin) {
    is_minimized = true;
    last_origin = origin;
    take(data);
}

void update(fsm::BaseData data) {
    if (is_minimized) {
        take(data);
    }
}

void end() {
    is_minimized = false;
    last_origin.reset();
    ++rev;
}

void prepare_reopen() {
    is_minimized = false;
    if (last_origin) {
        atelier::restore_request({ atelier::Op(last_origin->op), last_origin->slot });
    }
    ++rev;
}

void restore() {
    if (!is_minimized) {
        return;
    }
    prepare_reopen();
    screen_init_variant init;
    init.SetFsmBaseData(last_data);
    Screens::Access()->Open(screen_node(ScreenFactory::Screen<ScreenLoadUnload>, init));
}

uint8_t progress() {
    return last_progress;
}

std::optional<uint8_t> slot() {
    return last_origin ? std::optional<uint8_t>(last_origin->slot) : std::nullopt;
}

uint16_t revision() {
    return rev;
}

Face face() {
    Inputs in;
    in.filament_minimized = is_minimized;
    in.filament_percent = std::min<uint8_t>(last_progress, 100);

    auto &vars = marlin_vars();
    struct Fsm {
        bool print;
        std::optional<ClientFSM> top;
    };
    const Fsm fsm = vars.peek_fsm_states([](const auto &states) {
        const auto top = states.get_top();
        return Fsm { states.is_active(ClientFSM::Printing), top ? std::optional<ClientFSM>(top->fsm_type) : std::nullopt };
    });
    in.print_fsm = fsm.print;
    in.other_fsm = fsm.top && *fsm.top != ClientFSM::Printing;

    if (fsm.print) {
        const auto tone = print_tone(vars.print_state.get());
        in.print_ended = !tone;
        in.print_tone = tone.value_or(Tone::normal);
        // Par-dessus l'impression : un dialogue (l'en-tête reste visible au-dessus) le dit en couleur ; l'écran d'une
        // autre machine (chargement, collision...) s'affiche lui-même, c'est lui qu'il faut regarder
        if (in.other_fsm) {
            switch (*fsm.top) {
            case ClientFSM::Warning:
                in.print_tone = Tone::error;
                break;
            case ClientFSM::Wait:
            case ClientFSM::SafetyTimer:
            case ClientFSM::QuickPause:
                if (in.print_tone != Tone::error) {
                    in.print_tone = Tone::waiting;
                }
                break;
            default:
                in.print_hidden_by_fsm = true;
                break;
            }
        }
        in.on_print_screen = Screens::Access()->IsScreenOpened<ScreenPrinting>();
        in.print_percent = std::min<uint8_t>(vars.sd_percent_done, 100);
    }

    // Toujours suivie (même pendant une impression), pour que la cible déjà atteinte ne rallume pas la pastille après
    auto &hotend = vars.active_hotend();
    in.nozzle = static_cast<int16_t>(std::lround(hotend.temp_nozzle.get()));
    in.nozzle_target = hotend.target_nozzle.get();
    in.nozzle_heating = preheat_latch.update(in.nozzle, in.nozzle_target);
    in.on_temperatures_screen = Screens::Access()->IsScreenOpened<ScreenTemperatures>();

    return choose(in);
}

void activate() {
    switch (face().kind) {
    case Kind::filament:
        restore();
        break;
    case Kind::print:
        open_print_tab(); // la pile redevient [accueil, Impression], comme l'onglet Imprimer
        break;
    case Kind::preheat:
        open_temperatures();
        break;
    case Kind::none:
        break;
    }
}

} // namespace od3d::island
