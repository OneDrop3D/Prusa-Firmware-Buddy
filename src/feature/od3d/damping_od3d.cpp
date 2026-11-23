/// @file
#include "damping_od3d.hpp"
#include "accelerometer_od3d.hpp"
#include "live_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#include <common/marlin_server.hpp>
#include <feature/phase_stepping/phase_stepping.hpp>
#include <i18n.h>
#include <Marlin/src/gcode/calibrate/M958.hpp>
#include <Marlin/src/gcode/gcode.h>
#include <Marlin/src/Marlin.h>
#include <Marlin/src/module/motion.h>
#include <Marlin/src/module/planner.h>
#include <Marlin/src/module/temperature.h>
#include <raii/scope_guard.hpp>

namespace od3d::damping {

namespace {

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<Status> last_status = Status::idle;
    std::atomic<uint8_t> axis_now = 0;
    std::atomic<bool> calibrating_now = false;
    std::atomic<uint8_t> progress = 0;
    std::atomic<uint16_t> runs = 0;
    std::atomic<uint16_t> revisions = 0;

    Measurement measurement; ///< ~1 Ko en mémoire statique, lue par l'interface

    // Excitation de l'assistant input shaper de Prusa (M1959.cpp)
    constexpr float excitation_acceleration = 2.5f; ///< m/s²
    constexpr uint32_t excitation_cycles = 50;

    constexpr float min_z = 10.f;
    constexpr float above_print = 5.f;
    constexpr float travel_speed_xy = 50.f; ///< mm/s, comme M961
    constexpr float travel_speed_z = 10.f;

    bool must_stop() {
        return stop_requested.load() || planner.draining() || !marlin_server::printer_idle();
    }

    /// La position de M1959 (centre du plateau, mi-hauteur), sans descendre sous ce qui a été imprimé ; bornée par les
    /// butées logicielles ; false sans origine
    bool prepare() {
        // Jamais d'origine ici (retour du 23/11 : le plateau montait palper, accéléromètre fixé) : l'écran ou
        // l'assistant la prend avant de demander de le fixer (M1989 H) ; perdue depuis : refusé, sans bouger
        if (must_stop() || axes_need_homing()) {
            return false;
        }
        const float z = std::min(std::max({ min_z, float(Z_SIZE) / 2.f, planner.max_printed_z + above_print }), soft_endstop.max.z - 1.f);
        if (current_position.z < z) {
            do_blocking_move_to_z(z, travel_speed_z);
        }
        const float x = std::clamp(X_BED_SIZE / 2.f, soft_endstop.min.x, soft_endstop.max.x);
        const float y = std::clamp(Y_BED_SIZE / 2.f, soft_endstop.min.y, soft_endstop.max.y);
        do_blocking_move_to_xy(x, y, travel_speed_xy);
        planner.synchronize();
        return !must_stop();
    }

    Status sweep(uint8_t axis_index, vibrate_measure::MeasureParams &args, const vibrate_measure::MicrostepRestorer &restorer) {
        using namespace vibrate_measure;
        const AxisEnum axis = axis_index == 0 ? X_AXIS : Y_AXIS;
        // CoreXY : les deux moteurs, comme M1959
        args.axis_flag = setup_steppers(axis_index == 0 ? StepEventFlag::STEP_EVENT_FLAG_STEP_X : StepEventFlag::STEP_EVENT_FLAG_STEP_Y);
        if (!args.setup(restorer)) {
            return Status::failed;
        }

        const auto progress_hook = [&](const ProgressHookParams &params) -> Result<void> {
            if (must_stop()) {
                return std::unexpected(Error::aborted);
            }
            if (params.phase == ProgressHookParams::Phase::calibrating) {
                calibrating_now = true;
                progress = uint8_t(std::clamp(params.progress * 100.f, 0.f, 100.f)); // avancement de l'étalonnage
            }
            idle(true);
            return {};
        };

        for (size_t i = 0; i < sweep_points; ++i) {
            if (must_stop()) {
                return Status::stopped;
            }
            const auto result = measure_repeat(args, sweep_start_hz + float(i) * sweep_step_hz, progress_hook);
            args.calibrate_accelerometer = false;
            calibrating_now = false;
            if (!result.has_value()) {
                return result.error() == Error::aborted ? Status::stopped : Status::failed;
            }
            // Gain de l'axe excité seul : la courbe de résonance de l'axe (M1959 y ajoute les deux autres axes)
            measurement.psd[axis_index][i] = sq(result->gain[axis]);
            measurement.points[axis_index] = uint8_t(i + 1);
            progress = uint8_t((axis_index * sweep_points + i + 1) * 100 / (axis_count * sweep_points));
            ++revisions;
        }
        measurement.results[axis_index] = analyse(measurement.psd[axis_index], sweep_start_hz, sweep_step_hz);
        ++revisions;
        return Status::done;
    }

    Status measure() {
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
        if (!prepare()) {
            return must_stop() ? Status::stopped : Status::not_homed;
        }

        // Comme M1959 : phase stepping coupé avant de lire les micro-pas, tout est rétabli à la sortie
        phase_stepping::EnsureDisabled phase_stepping_disabler;
        vibrate_measure::MicrostepRestorer microstep_restorer;
        enable_all_steppers();

        vibrate_measure::MeasureParams args {
            .excitation_acceleration = excitation_acceleration,
            .excitation_cycles = excitation_cycles,
            .klipper_mode = true,
            .calibrate_accelerometer = true, // une fois, au premier point de X (~15 s)
            .axis_flag = StepEventFlag::STEP_EVENT_FLAG_STEP_X,
        };
        for (uint8_t a = 0; a < axis_count; ++a) {
            axis_now = a;
            live::set(N_("Axis %c: sweep around the resonance"), a == 0 ? 'X' : 'Y');
            if (const Status s = sweep(a, args, microstep_restorer); s != Status::done) {
                return s;
            }
        }
        return Status::done;
    }

} // namespace

void run() {
    ScopeGuard count_run = [] { ++runs; };
    if (running.exchange(true)) {
        return;
    }
    ScopeGuard not_running = [] {
        stop_requested = false;
        calibrating_now = false;
        running = false;
    };

    measurement = {};
    progress = 0;
    axis_now = 0;
    last_status = Status::running;
    ++revisions;

    last_status = measure();
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

uint8_t current_axis() {
    return axis_now.load();
}

bool calibrating() {
    return calibrating_now.load();
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

} // namespace od3d::damping
