/// @file
#include "calib_assistant_od3d.hpp"

#include "accelerometer_od3d.hpp"
#include "belt_tension_od3d.hpp"
#include "damping_od3d.hpp"
#include "mpc_control_od3d.hpp"
#include "mpc_od3d.hpp"
#include "pa_od3d.hpp"
#include "ptfe_od3d.hpp"
#include "speed_map_od3d.hpp"
#include "spool_calib_od3d.hpp"

#include <atomic>
#include <cstdio>
#include <ctime>
#include <optional>

#include <common/marlin_server.hpp>
#include <config_store/store_instance.hpp>
#include <filament.hpp>
#include <Marlin/src/gcode/gcode.h>
#include <Marlin/src/gcode/queue.h>
#include <Marlin/src/module/motion.h>
#include <Marlin/src/module/planner.h>
#include <Marlin/src/module/temperature.h>
#include <test_result.hpp>
#include <tool_index.hpp>
#include <utils/string_builder.hpp>

namespace od3d::calib_assistant {

using namespace calib_registry;

namespace {

    constexpr size_t max_steps = 32;
    std::array<Step, max_steps> steps;
    std::atomic<uint8_t> step_count = 0;
    std::atomic<uint8_t> current = 0;
    std::atomic<bool> running = false;
    std::atomic<bool> cancel_requested = false;
    std::atomic<uint16_t> runs = 0;
    std::array<std::atomic<Result>, max_tests> results {};
    uint32_t run_mask = 0;

    std::atomic<Check> last_check = Check::none;
    std::atomic<uint16_t> checks = 0;

    /// Résultat de Prusa avant son étape (input shaper, phase stepping) : remis si l'étape est annulée
    TestResult prusa_before = TestResult::unknown;

    /// Chargement de la bobine avant F5 / F6, ou déchargement avant le tube PTFE (mis en file d'injection, lu après
    /// le retour de M1989)
    char load_sequence[96];

    bool cold() {
        return nozzle_cold(Temperature::degHotend(0), Temperature::degTargetHotend(0));
    }

    uint32_t now() {
        const time_t t = time(nullptr);
        return t > 1'600'000'000 ? uint32_t(t) : 0; // heure pas encore reçue : inconnue
    }

    std::optional<TestResult> prusa_result(Test test) {
        switch (test) {
        case Test::input_shaper:
            return config_store().selftest_result_input_shaper_calibration.get();
        case Test::phase_stepping:
            return config_store().selftest_result_phase_stepping.get();
        default:
            return std::nullopt;
        }
    }

    void set_prusa_result(Test test, TestResult r) {
        if (test == Test::input_shaper) {
            config_store().selftest_result_input_shaper_calibration.set(r);
        } else if (test == Test::phase_stepping) {
            config_store().selftest_result_phase_stepping.set(r);
        }
    }

    bool succeeded(const Step &s) {
        switch (s.test) {
        case Test::input_shaper:
        case Test::phase_stepping: {
            const TestResult r = *prusa_result(s.test);
            if (r == TestResult::unknown) {
                set_prusa_result(s.test, prusa_before); // annulée : l'état de Prusa d'avant
            }
            return r == TestResult::passed;
        }
        case Test::belts:
            return belts::status() == belts::Status::done;
        case Test::speed_map:
            return speed_map::status() == speed_map::Status::done;
        case Test::damping:
            return damping::status() == damping::Status::done;
        case Test::mpc:
            return mpc::tune_status().phase == mpc::Phase::done;
        case Test::pa: {
            const pa::Status st = pa::status();
            return st.phase == pa::Phase::done && st.slot == s.slot;
        }
        case Test::mmu_ptfe:
            return ptfe::status().result == ptfe::Result::saved;
        case Test::max_flow:
        case Test::min_temp: {
            const calib::Status st = calib::status();
            const calib::Kind kind = s.test == Test::max_flow ? calib::Kind::flow : calib::Kind::temperature;
            return st.kind == kind && st.phase == calib::Phase::done && st.slot == s.slot;
        }
        case Test::skew:
        case Test::count:
            break;
        }
        return false;
    }

    void add_result(Test test, Result r) {
        auto &slot = results[size_t(test)];
        slot = fold(slot.load(), r);
    }

    void finish() {
        const uint32_t date = now();
        for (const Entry &e : entries) {
            if (run_mask & test_bit(e.test)) {
                const size_t i = size_t(e.test);
                config_store().od3d_calib_done.set(i, after_run(config_store().od3d_calib_done.get(i), e, results[i].load(), date));
            }
        }
        running = false;
        ++runs;
    }

    /// Met en file l'étape i et son repère ; false si elle ne peut pas partir
    bool enqueue(uint8_t i) {
        const Step &s = steps[i];
        char gcode[24];
        char marker[16];
        if (!step_gcode(s, gcode) || snprintf(marker, sizeof(marker), "M1989 N%u", unsigned(i)) <= 0) {
            return false;
        }
        if (BUFSIZE - GCodeQueue::length < 2) {
            return false; // la file a été remplie d'ailleurs : l'étape échoue plutôt que de perdre le repère
        }
        // F5, F6 : bobine pas dans la buse (pas de PA juste avant) : « Charger dans la buse » de Prusa d'abord, comme
        // M1994 en un clic (origine si besoin, position de chargement levée), par la file d'injection qui passe avant
        // Tube PTFE : le test de chargement de Prusa veut l'extrudeur et le sélecteur vides : déchargement d'abord
        const auto loaded = VirtualToolIndex::currently_selected_opt();
        const bool needs_load = (s.test == Test::max_flow || s.test == Test::min_temp) && (!loaded || loaded->to_raw() != s.slot);
        const bool needs_unload = s.test == Test::mmu_ptfe && loaded;
        if (needs_load || needs_unload) {
            char filament[16];
            StringBuilder sb(filament);
            FilamentType::for_tool(VirtualToolIndex::from_raw(s.slot)).build_encoded_gcode_param(sb);
            const int n = needs_unload ? snprintf(load_sequence, sizeof(load_sequence), "G28 O I\nG27 W2\nM702 W2")
                                       : snprintf(load_sequence, sizeof(load_sequence), "G28 O I\nG27 W2\nM701 S\"%s\" W2 P%u", sb.str(), unsigned(s.slot));
            if (n <= 0 || size_t(n) >= sizeof(load_sequence)
                || !marlin_server::inject(GCodeLiteral { .gcode = ConstexprString::from_str_unsafe(load_sequence) })) {
                return false;
            }
        }
        if (!marlin_server::enqueue_gcode_try(gcode) || !marlin_server::enqueue_gcode_try(marker)) {
            return false;
        }
        // Rien ne tourne avant le retour de M1989 : le résultat lu à la fin sera celui de cette étape, pas d'avant
        if (const auto r = prusa_result(s.test)) {
            prusa_before = *r;
            set_prusa_result(s.test, TestResult::unknown);
        }
        return true;
    }

    /// Lance l'étape i, ou la suivante qui peut partir ; sinon fin de la série
    void start(uint8_t i) {
        for (; i < step_count; ++i) {
            current = i;
            if (cancel_requested || marlin_server::is_printing()) {
                for (; i < step_count; ++i) {
                    add_result(steps[i].test, Result::skipped);
                }
                break;
            }
            // Buse froide exigée (accéléromètre sur la buse) : refusée sans rien bouger
            if ((entry(steps[i].test).needs & cold_nozzle) && !cold()) {
                add_result(steps[i].test, Result::failed);
                continue;
            }
            if (enqueue(i)) {
                return;
            }
            add_result(steps[i].test, Result::failed);
        }
        finish();
    }

} // namespace

// ---- Mémoire et réglages ----

Context context() {
    Context ctx;
    ctx.accelerometer = accelerometer();
    ctx.mmu = config_store().mmu2_enabled.get();
    ctx.flow_limit = calib::flow_limit_enabled();
    for (uint8_t i = 0; i < VirtualToolIndex::count; ++i) {
        const VirtualToolIndex tool = VirtualToolIndex::from_raw(i);
        if (tool.is_enabled() && FilamentType::for_tool(tool)) {
            ctx.spools |= uint8_t(1u << i);
        }
    }
    return ctx;
}

Record record(Test test) {
    return config_store().od3d_calib_done.get(size_t(test));
}

std::array<Record, max_tests> records() {
    std::array<Record, max_tests> out;
    for (size_t i = 0; i < max_tests; ++i) {
        out[i] = config_store().od3d_calib_done.get(i);
    }
    return out;
}

Accelerometer accelerometer() {
    return config_store().od3d_accelerometer.get();
}

void set_accelerometer(Accelerometer value) {
    config_store().od3d_accelerometer.set(value);
}

void dismiss() {
    config_store().od3d_calib_dismissed.set(signature());
}

uint32_t dismissed() {
    return config_store().od3d_calib_dismissed.get();
}

void record_done(Test test) {
    const size_t i = size_t(test);
    config_store().od3d_calib_done.set(i, after_run(config_store().od3d_calib_done.get(i), entry(test), Result::ok, now()));
}

// ---- Série en cours ----

Progress progress() {
    const uint8_t i = current;
    return Progress { .running = running, .index = i, .count = step_count, .step = steps[i % max_steps], .runs = runs };
}

Result result(Test test) {
    return results[size_t(test)].load();
}

void request_cancel() {
    if (!running) {
        return;
    }
    cancel_requested = true;
    // Seulement la mesure qui tourne : un arrêt demandé avant son début vaudrait aussi pour sa prochaine mesure à la main
    switch (steps[current % max_steps].test) {
    case Test::belts:
        if (belts::is_running()) {
            belts::request_stop();
        }
        break;
    case Test::speed_map:
        if (speed_map::is_running()) {
            speed_map::request_stop();
        }
        break;
    case Test::damping:
        if (damping::is_running()) {
            damping::request_stop();
        }
        break;
    case Test::mpc:
        if (mpc::tune_status().running) {
            mpc::request_cancel();
        }
        break;
    case Test::pa:
        if (pa::is_running()) {
            pa::request_stop();
        }
        break;
    case Test::max_flow:
    case Test::min_temp:
        if (calib::is_running()) {
            calib::request_stop();
        }
        break;
    default:
        break; // assistants de Prusa (input shaper, phase stepping) : leur écran a son « Annuler »
    }
}

Check check_result() {
    return last_check;
}

uint16_t check_count() {
    return checks;
}

// ---- M1989 ----

void prepare() {
    if (marlin_server::is_printing()) {
        return;
    }
    // Comme le début de M1959 : chauffe coupée et ventilateur à fond dès maintenant, origine, buse au centre
    Temperature::disable_hotend();
    if (Temperature::degHotend(0) >= cold_nozzle_max) {
        Temperature::set_print_fan_speed(255);
    }
    GcodeSuite::G28_no_parser(true, true, true, { .only_if_needed = true, .precise = false });
    if (!axes_need_homing() && !planner.draining()) {
        do_blocking_move_to(X_BED_SIZE / 2, Y_BED_SIZE / 2, Z_SIZE / 2);
    }
}

void check() {
    Check c = Check::ok;
    if (marlin_server::is_printing()) {
        c = Check::printing;
    } else if (!cold()) {
        c = Check::hot;
    } else if (axes_need_homing()) {
        c = Check::not_homed; // l'origine se refait sans l'accéléromètre (Prusa ne palpe jamais avec lui)
    } else if (!accel::ready()) {
        c = Check::no_accelerometer;
    }
    last_check = c;
    ++checks;
}

void run(uint32_t mask) {
    if (running) {
        return;
    }
    const Context ctx = context();
    step_count = uint8_t(build_steps(mask & automatic_mask(), ctx, steps));
    run_mask = 0;
    for (uint8_t i = 0; i < step_count; ++i) {
        run_mask |= test_bit(steps[i].test);
    }
    for (const Entry &e : entries) {
        if (run_mask & test_bit(e.test)) {
            results[size_t(e.test)] = Result::none;
        }
    }
    cancel_requested = false;
    running = true;
    start(0);
}

void step_end(uint8_t index) {
    if (!running || index != current) {
        return; // repère d'une autre série
    }
    const Step &s = steps[index];
    add_result(s.test, succeeded(s) ? Result::ok : (cancel_requested ? Result::skipped : Result::failed));
    start(uint8_t(index + 1));
}

} // namespace od3d::calib_assistant
