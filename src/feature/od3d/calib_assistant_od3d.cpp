/// @file
#include "calib_assistant_od3d.hpp"

#include "accelerometer_od3d.hpp"
#include "belt_tension_od3d.hpp"
#include "damping_od3d.hpp"
#include "fan_response_od3d.hpp"
#include "lookahead_od3d.hpp"
#include "e_jump_od3d.hpp"
#include "filament_memory_od3d.hpp"
#include "mpc_control_od3d.hpp"
#include "mmu_speed_od3d.hpp"
#include "mpc_od3d.hpp"
#include "nozzle_expansion_od3d.hpp"
#include "pa_od3d.hpp"
#include "ptfe_od3d.hpp"
#include "speed_map_od3d.hpp"
#include "spool_calib_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <optional>

#include <common/marlin_server.hpp>
#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <feature/nozzle_thermal_compensation/nozzle_thermal_compensation.hpp>
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

    std::array<Numbers, max_tests> before_numbers;
    std::array<Numbers, max_tests> after_numbers;

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
        case Test::fan_response:
            return fan_response::status() == fan_response::Status::done;
        case Test::nozzle_expansion:
            return nozzle_expansion::status() == nozzle_expansion::Status::saved;
        case Test::e_jump: {
            const e_jump::Status st = e_jump::status();
            return st.phase == e_jump::Phase::done && st.slot == s.slot;
        }
        case Test::mmu_speed:
            return mmu_speed::last().result == mmu_speed::Result::saved;
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

    int16_t scaled(float v, float k) {
        return std::isfinite(v) ? int16_t(std::lround(std::clamp(v * k, -32767.f, 32767.f))) : no_value;
    }

    /// Chiffres du test (voir numbers_before()) : avant son étape, ou après son succès
    void capture(const Step &s, Numbers &n, bool after) {
        const Values stored = config_store().od3d_calib_values.get(size_t(s.test));
        switch (s.test) {
        case Test::input_shaper:
            break; // M1959 A, qui a le spectre
        case Test::phase_stepping:
            if (!after) {
                std::copy(stored.v.begin(), stored.v.end(), n.begin());
            } // après : M1977
            break;
        case Test::fan_response:
            if (!after) {
                std::copy(stored.v.begin(), stored.v.end(), n.begin()); // temps de montée de la fois d'avant
                n[1] = int16_t(lookahead::fan_lead_ds());
            } else {
                const fan_response::Result &r = fan_response::result();
                n[0] = scaled(r.rise.rise_s, 1000.f);
                n[1] = int16_t(r.lead_ds);
                n[2] = scaled(r.rise.rpm, 1.f);
            }
            break;
        case Test::belts:
        case Test::damping:
        case Test::speed_map:
            if (!after) {
                std::copy(stored.v.begin(), stored.v.end(), n.begin());
            } else if (s.test == Test::belts) {
                for (uint8_t b = 0; b < belts::belt_count; ++b) {
                    const auto &peak = belts::last().peaks[b];
                    n[b] = peak.found ? scaled(peak.frequency, 10.f) : no_value;
                }
            } else if (s.test == Test::damping) {
                for (uint8_t a = 0; a < damping::axis_count; ++a) {
                    const auto &res = damping::last().results[a];
                    n[a] = res.found ? scaled(res.zeta, 1000.f) : no_value;
                }
            } else {
                std::array<uint8_t, 16> found;
                n[1] = int16_t(speed_map::critical(speed_map::last(), 0, found));
                n[2] = int16_t(speed_map::critical(speed_map::last(), 1, found));
                n[0] = int16_t(n[1] + n[2]);
            }
            break;
        case Test::mpc:
            if (after || mpc::has_valid_calibration()) {
                const mpc::Params &p = mpc::params();
                n[0] = scaled(p.heater_power, 10.f);
                n[1] = scaled(p.block_heat_capacity, 10.f);
                n[2] = scaled(p.ambient_xfer_coeff, 1000.f);
                n[3] = scaled(p.fan_xfer_coeff, 1000.f);
            }
            break;
        case Test::mmu_ptfe:
            if (after) {
                n[0] = int16_t(ptfe::status().saved_mm);
                n[1] = int16_t(ptfe::status().measured_mm);
            } else if (const uint16_t length = marlin_vars().mmu2_bowden_length.get(); length > 0) {
                n[0] = int16_t(length);
            }
            break;
        case Test::nozzle_expansion: {
            namespace comp = buddy::nozzle_thermal_compensation;
            const int16_t prusa_nm = int16_t(comp::slope_to_nm(comp::prusa_slope_mm_per_c));
            if (!after) {
                if (const uint16_t nm = comp::measured_nm_per_c(comp::current_nozzle_key()); nm > 0) {
                    n[0] = int16_t(std::min<uint16_t>(nm, INT16_MAX)); // sinon : modèle de Prusa
                }
                break;
            }
            const nozzle_expansion::Outcome &o = nozzle_expansion::last().outcome;
            n[0] = scaled(o.slope_mm_per_c, 1e6f);
            n[1] = prusa_nm;
            // Écart avec Prusa à 250 °C, la buse palpée à la référence (170 °C) [0,1 µm]
            n[2] = scaled((o.slope_mm_per_c - comp::prusa_slope_mm_per_c) * (250.f - comp::reference_temperature_c), 1e4f);
            n[3] = scaled(o.hysteresis_mm, 1e4f);
            break;
        }
        case Test::mmu_speed:
            if (after) {
                const mmu_speed::Measurement &m = mmu_speed::last();
                const mmu_speed::Saved saved = mmu_speed::saved();
                n[0] = m.speed;
                n[1] = int16_t(saved.prusa_ds);
                n[2] = int16_t(saved.cycle_ds);
                const auto passed = std::find(m.passed.begin(), m.passed.begin() + mmu_speed::levels.size(), false) - m.passed.begin();
                n[3] = passed ? mmu_speed::levels[passed - 1] : no_value;
            } else {
                n[0] = mmu_speed::speed_of(mmu_speed::saved());
            }
            break;
        case Test::pa:
            if (const pa::Record r = pa::record(s.slot); r.has_k()) {
                n[s.slot % n.size()] = int16_t(r.k_e4);
            }
            break;
        case Test::max_flow:
        case Test::min_temp:
            if (const calib::Record r = calib::record(s.slot); s.test == Test::max_flow ? r.has_flow() : r.has_min_temperature()) {
                n[s.slot % n.size()] = s.test == Test::max_flow ? int16_t(r.max_flow_e1) : r.min_temperature;
            }
            break;
        case Test::e_jump: // F de l'emplacement avec son K : la mesure d'avant (ou 120 de Prusa), puis la nouvelle
            n[s.slot % n.size()] = scaled(e_jump::filament_accel_for(s.slot), 1.f);
            break;
        case Test::skew:
        case Test::count:
            break;
        }
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
                if (results[i].load() == Result::ok && !(e.needs & filament)) {
                    Values kept;
                    std::copy_n(after_numbers[i].begin(), kept.v.size(), kept.v.begin());
                    config_store().od3d_calib_values.set(i, kept); // le « avant » de la fois suivante
                }
            }
        }
        running = false;
        ++runs;
    }

    /// Met en file l'étape i et son repère ; false si elle ne peut pas partir
    bool enqueue(uint8_t i) {
        const Step &s = steps[i];
        if (s.test == Test::mmu_ptfe) {
            ptfe::forget(); // le bilan lit le résultat de cette mesure, jamais celui d'une mesure d'avant
        } else if (s.test == Test::mmu_speed) {
            mmu_speed::forget();
        }
        char gcode[24];
        char marker[16];
        if (!step_gcode(s, gcode) || snprintf(marker, sizeof(marker), "M1989 N%u", unsigned(i)) <= 0) {
            return false;
        }
        if (BUFSIZE - GCodeQueue::length < 2) {
            return false; // la file a été remplie d'ailleurs : l'étape échoue plutôt que de perdre le repère
        }
        // F5, F6, M2000 : bobine pas dans la buse (pas de PA juste avant) : « Charger dans la buse » de Prusa d'abord, comme
        // M1994 en un clic (origine si besoin, position de chargement levée), par la file d'injection qui passe avant
        // Tube PTFE, vitesse de la poulie : le test de chargement de Prusa veut l'extrudeur et le sélecteur vides :
        // déchargement d'abord ; de même pour la dilatation de la buse (M2003), si le capteur de l'extrudeur voit du filament
        const auto loaded = VirtualToolIndex::currently_selected_opt();
        const bool needs_load = (s.test == Test::max_flow || s.test == Test::min_temp || s.test == Test::e_jump) && filament_memory::loaded_slot() != s.slot; // F43
        const bool needs_unload = ((s.test == Test::mmu_ptfe || s.test == Test::mmu_speed) && loaded) || (s.test == Test::nozzle_expansion && nozzle_expansion::filament_loaded()); // M2003 : buse vide
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
        capture(s, before_numbers[size_t(s.test)], false);
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
    case Test::nozzle_expansion:
        if (nozzle_expansion::is_running()) {
            nozzle_expansion::request_stop();
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
    case Test::fan_response:
        if (fan_response::is_running()) {
            fan_response::request_stop();
        }
        break;
    case Test::e_jump:
        if (e_jump::is_running()) {
            e_jump::request_stop();
        }
        break;
    case Test::mmu_speed:
        mmu_speed::request_stop(); // entre deux cycles
        break;
    default:
        break; // assistants de Prusa (input shaper, phase stepping) : leur écran a son « Annuler »
    }
}

const Numbers &numbers_before(Test test) {
    return before_numbers[size_t(test) % max_tests];
}

const Numbers &numbers_after(Test test) {
    return after_numbers[size_t(test) % max_tests];
}

void set_number(Test test, bool after, uint8_t index, int16_t value) {
    if (size_t(test) < max_tests && index < Numbers().size()) {
        (after ? after_numbers : before_numbers)[size_t(test)][index] = value;
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
            before_numbers[size_t(e.test)] = no_numbers;
            after_numbers[size_t(e.test)] = no_numbers;
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
    const bool ok = succeeded(s);
    if (ok) {
        capture(s, after_numbers[size_t(s.test)], true);
    }
    add_result(s.test, ok ? Result::ok : (cancel_requested ? Result::skipped : Result::failed));
    start(uint8_t(index + 1));
}

} // namespace od3d::calib_assistant
