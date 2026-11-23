/// @file
#include "island_od3d.hpp"

#include "atelier_od3d.hpp"
#include "screen_belts_od3d.hpp"
#include "screen_calib_assistant_od3d.hpp"
#include "screen_collision_od3d.hpp"
#include "screen_damping_od3d.hpp"
#include "screen_fan_response_od3d.hpp"
#include "screen_e_jump_od3d.hpp"
#include "screen_mpc_od3d.hpp"
#include "screen_mmu_speed_od3d.hpp"
#include "screen_mmu_usb_od3d.hpp"
#include "screen_music_od3d.hpp"
#include "screen_nozzle_expansion_od3d.hpp"
#include "screen_pa_od3d.hpp"
#include "screen_printing_od3d.hpp"
#include "screen_progress_od3d.hpp"
#include "screen_speed_map_od3d.hpp"
#include "screen_spool_calib_od3d.hpp"
#include "screen_temperatures_od3d.hpp"
#include "widgets_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <common/marlin_server_shared.h>
#include <common/marlin_vars.hpp>
#include <feature/od3d/belt_tension_od3d.hpp>
#include <feature/od3d/calib_assistant_od3d.hpp>
#include <feature/od3d/collision_od3d.hpp>
#include <feature/od3d/damping_od3d.hpp>
#include <feature/od3d/fan_response_od3d.hpp>
#include <feature/od3d/e_jump_od3d.hpp>
#include <feature/od3d/mmu_flash_od3d.hpp>
#include <feature/od3d/mmu_speed_od3d.hpp>
#include <feature/od3d/mmu_usb_od3d.hpp>
#include <feature/od3d/mpc_control_od3d.hpp>
#include <feature/od3d/music_player_od3d.hpp>
#include <feature/od3d/nozzle_expansion_od3d.hpp>
#include <feature/od3d/pa_od3d.hpp>
#include <feature/od3d/speed_map_od3d.hpp>
#include <feature/od3d/spool_calib_od3d.hpp>
#include <fsm/filament_change_phases.hpp>
#include <fsm_loadunload_type.hpp>
#include <i18n.h>
#include <option/has_tool_mapping.h>
#include <ScreenFactory.hpp>
#include <ScreenHandler.hpp>
#include <timing.h>

namespace od3d::island {

namespace {

    bool is_minimized = false;
    fsm::BaseData last_data;
    std::optional<Origin> last_origin;
    uint8_t last_progress = 0;
    std::optional<uint32_t> chain_start_ms; ///< fin d'une opération réduite : la suivante, si elle suit de près, reste réduite
    std::optional<uint16_t> kept_job; ///< impression pendant laquelle l'utilisateur a choisi l'arrière-plan
    constexpr uint32_t chain_ms = 3'000; ///< comme l'atelier : Prusa ferme et rouvre la machine entre deux commandes du MMU

    void take(fsm::BaseData data) {
        last_data = data;
        // MMU_ERRWaitingForUser porte un pointeur vers l'erreur, pas d'avancement (et rouvre l'écran de toute façon)
        if (GetEnumFromPhaseIndex<PhasesLoadUnload>(data.GetPhase()) != PhasesLoadUnload::MMU_ERRWaitingForUser) {
            last_progress = fsm::deserialize_data<FSMLoadUnloadData>(data.GetData()).progress;
        }
    }

    PreheatLatch preheat_latch;

    bool wait_is_minimized = false;
    std::optional<uint32_t> wait_chain_start_ms; ///< fin d'une attente réduite : G29 enchaîne ses commandes

    /// Étapes communes de la mesure du PA (F4) et des mesures F5 / F6 : chauffe, palpage, purge, lignes
    template <class Phase>
    Stage stage_of(Phase phase, uint8_t steps) {
        switch (phase) {
        case Phase::bed:
        case Phase::heating:
            return Stage::heating;
        case Phase::leveling:
            return Stage::probing;
        case Phase::priming:
            return Stage::priming;
        case Phase::measuring:
            return steps ? Stage::step : Stage::busy;
        default:
            return Stage::busy;
        }
    }

    /// Le processus en cours (un seul à la fois : ce sont des G-codes, que Marlin exécute l'un après l'autre)
    std::optional<Face> process_face() {
        Face f { .kind = Kind::process };
        const auto percent = [&](Process process, uint8_t value) {
            f.process = process;
            f.stage = Stage::percent;
            f.percent = std::min<uint8_t>(value, 100);
            return f;
        };
        // L'écriture du firmware du MMU par le port avant passe avant l'assistant, qu'on peut lancer pendant ce temps : on
        // suit son avancement, puis la pastille rappelle de « Terminer » (ou de réessayer) tant que le mode dure
        if (const mmu_flash::Status s = mmu_flash::status(); s.phase != mmu_flash::Phase::idle && mmu_usb::active()) {
            if (s.phase == mmu_flash::Phase::connecting || s.phase == mmu_flash::Phase::writing) {
                return percent(Process::mmu_usb, s.percent);
            }
            f.process = Process::mmu_usb;
            return f;
        }
        // L'assistant de calibration d'abord : ses étapes (PA, courroies...) sont montrées comme les siennes
        if (calibration_assistant_active()) {
            f.process = Process::calibration;
            if (const calib_assistant::Progress p = calib_assistant::progress(); p.running) {
                f.stage = Stage::step;
                f.step = uint8_t(p.index + 1);
                f.steps = p.count;
            }
            return f;
        }
        if (pa::is_running()) {
            const pa::Status s = pa::status();
            f.process = Process::pa;
            f.slot = s.slot;
            f.stage = s.phase == pa::Phase::loading ? Stage::loading : stage_of(s.phase, s.steps);
            f.step = s.step;
            f.steps = s.steps;
            return f;
        }
        if (calib::is_running()) {
            const calib::Status s = calib::status();
            f.process = s.kind == calib::Kind::flow ? Process::flow : Process::min_temp;
            f.slot = s.slot;
            f.stage = s.phase == calib::Phase::settling ? Stage::step : stage_of(s.phase, s.steps);
            f.step = s.step;
            f.steps = s.steps;
            return f;
        }
        if (e_jump::is_running()) {
            const e_jump::Status s = e_jump::status();
            f.process = Process::e_jump;
            f.slot = s.slot;
            f.stage = s.phase == e_jump::Phase::heating ? Stage::heating : (s.phase == e_jump::Phase::measuring ? Stage::step : Stage::busy);
            f.step = s.step;
            f.steps = s.steps;
            return f;
        }
        if (belts::is_running()) {
            return percent(Process::belts, belts::progress_percent());
        }
        if (speed_map::is_running()) {
            return percent(Process::speed_map, speed_map::progress_percent());
        }
        if (damping::is_running()) {
            return percent(Process::damping, damping::progress_percent());
        }
        if (nozzle_expansion::is_running()) {
            return percent(Process::nozzle_expansion, nozzle_expansion::progress_percent());
        }
        if (mmu_speed::is_running()) {
            const mmu_speed::Measurement &m = mmu_speed::last();
            f.process = Process::mmu_speed;
            f.slot = m.slot;
            f.stage = Stage::step;
            f.step = uint8_t(m.step + 1);
            f.steps = mmu_speed::cycles;
            return f;
        }
        if (const mpc::TuneStatus s = mpc::tune_status(); s.running) {
            return percent(Process::mpc, s.progress);
        }
        if (fan_response::is_running()) {
            return percent(Process::fan_response, fan_response::progress_percent());
        }
        if (music::is_playing()) {
            f.process = Process::music;
            return f;
        }
        if (collision::test_running()) {
            f.process = Process::hit_test;
            return f;
        }
        if (mmu_usb::active()) {
            f.process = Process::mmu_usb;
            return f;
        }
        return std::nullopt;
    }

    /// L'écran du processus (celui qui l'a lancé)
    ScreenFactory::Creator process_screen(const Face &f) {
        switch (f.process) {
        case Process::pa:
            return ScreenFactory::ScreenWithArg<ScreenPa>(f.slot);
        case Process::flow:
        case Process::min_temp:
            return ScreenFactory::ScreenWithArg<ScreenSpoolCalib>(f.slot);
        case Process::belts:
            return ScreenFactory::Screen<ScreenBelts>;
        case Process::speed_map:
            return ScreenFactory::Screen<ScreenSpeedMap>;
        case Process::damping:
            return ScreenFactory::Screen<ScreenDamping>;
        case Process::mpc:
            return mpc_calibration_screen();
        case Process::music:
            return ScreenFactory::Screen<ScreenMusic>;
        case Process::hit_test:
            return ScreenFactory::Screen<ScreenCollision>;
        case Process::mmu_usb:
            return ScreenFactory::Screen<ScreenMmuUsb>;
        case Process::calibration:
            return ScreenFactory::Screen<ScreenCalibAssistant>;
        case Process::fan_response:
            return ScreenFactory::Screen<ScreenFanResponse>;
        case Process::nozzle_expansion:
            return ScreenFactory::Screen<ScreenNozzleExpansion>;
        case Process::e_jump:
            return ScreenFactory::ScreenWithArg<ScreenEJump>(f.slot);
        case Process::mmu_speed:
            return ScreenFactory::Screen<ScreenMmuSpeed>;
        }
        return {};
    }

    string_view_utf8 process_name(Process process) {
        switch (process) {
        case Process::pa:
            return string_view_utf8::MakeCPUFLASH("PA");
        case Process::flow:
            return _("Flow");
        case Process::min_temp:
            return _("Min temp");
        case Process::belts:
            return _("Belts");
        case Process::speed_map:
            return _("Speeds");
        case Process::damping:
            return _("Damping");
        case Process::mpc:
            return string_view_utf8::MakeCPUFLASH("MPC");
        case Process::music:
            return _("Music");
        case Process::hit_test:
            return _("Hit test");
        case Process::mmu_usb:
            return string_view_utf8::MakeCPUFLASH("MMU USB");
        case Process::calibration:
            return calibration_assistant_name();
        case Process::fan_response:
            return _("Part fan");
        case Process::nozzle_expansion:
            return _("Expansion");
        case Process::e_jump:
            return _("Accel");
        case Process::mmu_speed:
            return _("MMU speed");
        }
        return {};
    }

    const char *stage_word(Stage stage) {
        switch (stage) {
        case Stage::heating:
            return N_("heating");
        case Stage::probing:
            return N_("probing");
        case Stage::loading:
            return N_("loading");
        case Stage::priming:
            return N_("priming");
        default:
            return nullptr;
        }
    }

    /// Couleur de l'état de l'impression ; nullopt : pas encore commencée (pas de pastille). Terminée ou arrêtée : grise,
    /// pour retrouver l'écran de fin si on était ailleurs (Prusa n'y ramène pas). Même découpage que
    /// ScreenPrinting::update_state().
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
            return std::nullopt;
        case State::Aborted:
        case State::Finished:
        case State::Exit:
            return Tone::ended;
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
    if (print_active() && !in_process()) { // le chargement de la mesure du PA automatique n'est pas un choix
        kept_job = marlin_vars().job_id.get(); // les changements d'outil suivants de cette impression aussi
    }
}

bool open_minimized(fsm::BaseData data) {
    const bool printing = print_active();
    const bool chained = chain_start_ms && ticks_ms() - *chain_start_ms < chain_ms; // une seule fois : l'opération qui suit
    chain_start_ms.reset();
    const BackgroundInputs in {
        .process = in_process(),
        .chained = chained,
        .printing = printing,
        .kept_for_print = printing && kept_job == marlin_vars().job_id.get(),
        .on_print_screen = Screens::Access()->IsScreenOpened<ScreenPrinting>(),
    };
    if (!open_in_background(in)) {
        return false;
    }
    // Pas d'écran pour prendre la demande de l'atelier : la pastille la garde pour le rouvrir avec son nom
    std::optional<Origin> origin;
    if (const auto request = atelier::take_request()) {
        origin = Origin { uint8_t(request->op), request->slot };
        atelier::release_request();
    }
    minimize(data, origin);
    return true;
}

void update(fsm::BaseData data) {
    if (is_minimized) {
        take(data);
    }
}

void end() {
    if (is_minimized) {
        chain_start_ms = ticks_ms();
    }
    is_minimized = false;
    last_origin.reset();
}

void prepare_reopen() {
    is_minimized = false;
    if (last_origin) {
        atelier::restore_request({ atelier::Op(last_origin->op), last_origin->slot });
    }
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

bool in_process() {
    return process_face().has_value();
}

bool wait_opens_minimized() {
    const bool chained = wait_chain_start_ms && ticks_ms() - *wait_chain_start_ms < chain_ms;
    wait_chain_start_ms.reset();
    wait_is_minimized = chained || in_process();
    return wait_is_minimized;
}

bool wait_minimized() {
    return wait_is_minimized;
}

void minimize_wait() {
    wait_is_minimized = true;
}

void wait_end() {
    if (wait_is_minimized) {
        wait_chain_start_ms = ticks_ms();
    }
    wait_is_minimized = false;
}

void text(const Face &f, char *buf, size_t size) {
    char name[24] = "";
    char word[24] = "";
    if (f.kind == Kind::process) {
        process_name(f.process).copyToRAM(name, sizeof(name));
        if (const char *w = stage_word(f.stage)) {
            _(w).copyToRAM(word, sizeof(word));
        }
    } else if (f.kind == Kind::wait) {
        _("Wait").copyToRAM(name, sizeof(name));
    }
    format_text(f, buf, size, name, word);
}

Face face() {
    Inputs in;
    if (const auto process = process_face()) {
        in.process = *process;
        in.on_process_screen = Screens::Access()->IsScreenOpened(process_screen(*process));
    }
    in.wait_minimized = wait_is_minimized;
    in.filament_minimized = is_minimized;
    in.filament_percent = std::min<uint8_t>(last_progress, 100);
    in.filament_slot = last_origin ? last_origin->slot : no_slot;

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
        in.print_not_started = !tone;
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
        if (const uint32_t left = vars.time_to_end; left != marlin_server::TIME_TO_END_INVALID) {
            in.print_minutes = static_cast<uint16_t>(std::min<uint32_t>((left + 59) / 60, no_time - 1));
        }
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
    const Face f = face();
    switch (f.kind) {
    case Kind::process:
        Screens::Access()->CloseAll(); // la pile redevient [accueil, écran du processus], comme l'onglet Imprimer
        Screens::Access()->Open(process_screen(f));
        break;
    case Kind::wait:
        wait_is_minimized = false; // DialogHandler rouvre le dialogue au tour suivant
        break;
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
