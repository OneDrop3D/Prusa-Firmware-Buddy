/// @file
#include "belt_tension_od3d.hpp"
#include "accelerometer_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#include <common/marlin_server.hpp>
#include <option/has_indx.h> // utilisé par manual_belt_tuning_config.hpp
#include <feature/manual_belt_tuning/manual_belt_tuning_config.hpp>
#include <feature/motordriver_util.h>
#include <feature/phase_stepping/phase_stepping.hpp>
#include <Marlin/src/gcode/calibrate/M958.hpp>
#include <Marlin/src/gcode/gcode.h>
#include <Marlin/src/Marlin.h>
#include <Marlin/src/module/motion.h>
#include <Marlin/src/module/planner.h>
#include <Marlin/src/module/temperature.h>
#include <raii/scope_guard.hpp>

namespace od3d::belts {

namespace {

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<Status> last_status = Status::idle;
    std::atomic<Stage> current_stage = Stage::preparing;
    std::atomic<uint8_t> progress = 0;
    std::atomic<uint16_t> runs = 0;
    std::atomic<uint16_t> revisions = 0;

    Measurement measurement; ///< ~420 octets en mémoire statique, lue par l'interface

    /// Fenêtre de recherche du pic : la plage de Prusa élargie de 10 Hz de chaque côté
    constexpr float window_margin_hz = 10.f;

    /// Excitation : comme le réglage manuel des courroies de Prusa (M961), 50 périodes par fréquence comme M1959
    constexpr uint32_t excitation_cycles = 50;

    constexpr float min_z = 10.f; ///< buse au moins à 10 mm du plateau
    constexpr float above_print = 5.f; ///< et 5 mm au-dessus de ce qui a été imprimé
    constexpr float travel_speed_xy = 50.f; ///< mm/s, comme M961
    constexpr float travel_speed_z = 10.f;

    bool must_stop() {
        return stop_requested.load() || planner.draining() || !marlin_server::printer_idle();
    }

    /// Prise d'origine si besoin, hauteur sûre, position de mesure de Prusa (M961) bornée par les butées logicielles
    bool prepare() {
        if (axes_need_homing()) {
            // Comme l'assistant input shaper de Prusa (M1959)
            GcodeSuite::G28_no_parser(true, true, true, { .only_if_needed = true, .precise = false });
        }
        if (must_stop() || axes_need_homing()) {
            return false;
        }
        const float z_floor = std::min(std::max(min_z, planner.max_printed_z + above_print), soft_endstop.max.z - 1.f);
        if (current_position.z < z_floor) {
            do_blocking_move_to_z(z_floor, travel_speed_z);
        }
        const float x = std::clamp<float>(manual_belt_tuning::calib_position_x, soft_endstop.min.x, soft_endstop.max.x);
        const float y = std::clamp<float>(manual_belt_tuning::calib_position_y, soft_endstop.min.y, soft_endstop.max.y);
        do_blocking_move_to_xy(x, y, travel_speed_xy);
        planner.synchronize();
        return !must_stop();
    }

    void set_progress(uint8_t belt_index, float point_progress) {
        const float done = float(belt_index) * sweep_points + point_progress;
        progress = uint8_t(std::clamp(done * 100.f / float(belt_count * sweep_points), 0.f, 100.f));
    }

    /// Balaye une courroie ; Status::done si tout est mesuré
    Status sweep(Belt belt, vibrate_measure::MeasureParams &args, const vibrate_measure::MicrostepRestorer &restorer) {
        using namespace vibrate_measure;
        const uint8_t index = uint8_t(belt);
        // Un seul moteur : A = pilote X, B = pilote Y (l'enchaînement de setup_steppers() de Prusa mettrait les deux)
        args.axis_flag = belt == Belt::a ? StepEventFlag::STEP_EVENT_FLAG_STEP_X : StepEventFlag::STEP_EVENT_FLAG_STEP_Y;
        if (!args.setup(restorer)) {
            return Status::failed;
        }

        const auto progress_hook = [&](const ProgressHookParams &params) -> Result<void> {
            if (must_stop()) {
                return std::unexpected(Error::aborted);
            }
            if (params.phase == ProgressHookParams::Phase::calibrating) {
                current_stage = Stage::calibrating;
                progress = uint8_t(std::clamp(params.progress * 100.f, 0.f, 100.f));
            }
            idle(true);
            return {};
        };

        for (size_t i = 0; i < sweep_points; ++i) {
            if (must_stop()) {
                return Status::stopped;
            }
            const float frequency = sweep_start_hz + float(i) * sweep_step_hz;
            // Accélération de Prusa pour cette fréquence (7 à 10,5 m/s² entre 60 et 130 Hz : quelques dizaines de µm)
            args.excitation_acceleration = std::fabs(manual_belt_tuning::calc_accel(frequency) * 0.001f);
            const auto result = measure_repeat(args, frequency, progress_hook);
            args.calibrate_accelerometer = false;
            if (!result.has_value()) {
                return result.error() == Error::aborted ? Status::stopped : Status::failed;
            }
            current_stage = belt == Belt::a ? Stage::belt_a : Stage::belt_b;
            measurement.spectra[index][i] = result->gain_square();
            measurement.points[index] = uint8_t(i + 1);
            set_progress(index, float(i + 1));
            ++revisions;
        }
        const Targets t = targets();
        measurement.peaks[index] = find_peak(measurement.spectra[index], sweep_start_hz, sweep_step_hz,
            t.result_min - window_margin_hz, t.result_max + window_margin_hz);
        ++revisions;
        return Status::done;
    }

    Status measure_belts(uint8_t harmonic) {
        if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
            return Status::printing;
        }
        if (stop_requested) {
            return Status::stopped;
        }
        if (Temperature::degHotend(0) > max_nozzle_temperature || Temperature::degTargetHotend(0) > max_nozzle_temperature) {
            return Status::hot_nozzle;
        }
        if (!accel::ready()) { // ACC-1 : avant tout mouvement
            return Status::no_accelerometer;
        }

        current_stage = Stage::preparing;
        if (!prepare()) {
            return must_stop() ? Status::stopped : Status::not_homed;
        }

        // Le phase stepping doit être coupé avant de lire les micro-pas (M958, M961) ; il revient après eux
        phase_stepping::EnsureDisabled phase_stepping_disabler;
        vibrate_measure::MicrostepRestorer microstep_restorer;
        enable_all_steppers(); // même état que pendant une impression
        stepper_microsteps(X_AXIS, 128);
        stepper_microsteps(Y_AXIS, 128);

        vibrate_measure::MeasureParams args {
            .excitation_acceleration = std::fabs(manual_belt_tuning::calc_accel(sweep_start_hz) * 0.001f),
            .excitation_cycles = excitation_cycles,
            .klipper_mode = harmonic == 1, // la 2e harmonique n'existe qu'en mode « Prusa »
            .calibrate_accelerometer = true, // une fois, au premier point de la courroie A (~15 s)
            .axis_flag = StepEventFlag::STEP_EVENT_FLAG_STEP_X,
            .measured_harmonic = harmonic,
        };

        for (const Belt belt : { Belt::a, Belt::b }) {
            if (const Status s = sweep(belt, args, microstep_restorer); s != Status::done) {
                return s;
            }
        }
        return Status::done;
    }

} // namespace

void run(uint8_t harmonic) {
    ScopeGuard count_run = [] { ++runs; }; // après last_status : l'interface lit l'état une fois runs changé
    if (running.exchange(true)) {
        return; // déjà en cours (ne devrait pas arriver : Marlin traite un G-code à la fois)
    }
    // Un arrêt demandé avant le début (écran fermé, M1992 encore en file) vaut pour cette mesure ; effacé à la fin
    ScopeGuard not_running = [] {
        stop_requested = false;
        running = false;
    };

    measurement = {};
    measurement.harmonic = harmonic == 2 ? 2 : 1;
    progress = 0;
    current_stage = Stage::preparing;
    last_status = Status::running;
    ++revisions;

    const Status s = measure_belts(measurement.harmonic);
    last_status = s;
    ++revisions;
}

void request_stop() {
    stop_requested = true;
}

bool is_running() {
    return running.load();
}

Status status() {
    return last_status.load();
}

Stage stage() {
    return current_stage.load();
}

uint8_t progress_percent() {
    return progress.load();
}

uint16_t run_count() {
    return runs.load();
}

uint16_t revision() {
    return revisions.load();
}

const Measurement &last() {
    return measurement;
}

Targets targets() {
    namespace mbt = manual_belt_tuning;
    return {
        .higher_optimal = mbt::higher_freq_belt_optimal,
        .lower_optimal = mbt::lower_freq_belt_optimal,
        .result_min = float(mbt::freq_result_min),
        .result_max = float(mbt::freq_result_max),
        .diff_limit = float(mbt::freq_diff_limit),
        .hz_per_rev = float(mbt::belt_hz_per_rev),
        .hz_per_rev2 = float(mbt::belt_hz_per_rev2),
    };
}

} // namespace od3d::belts
