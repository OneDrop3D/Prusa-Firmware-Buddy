/// @file
#include "nozzle_expansion_od3d.hpp"
#include "measure_session_od3d.hpp"
#include "mesh_memory_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>

#include <common/marlin_server.hpp>
#include <feature/bedlevel/bedlevel.h>
#include <feature/filament_sensor/filament_sensors_handler.hpp>
#include <feature/nozzle_thermal_compensation/nozzle_thermal_compensation.hpp>
#include <gcode/gcode.h>
#include <module/motion.h>
#include <module/planner.h>
#include <module/probe.h>
#include <module/temperature.h>
#include <raii/scope_guard.hpp>

namespace od3d::nozzle_expansion {

namespace comp = buddy::nozzle_thermal_compensation;

namespace {

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<Status> last_status = Status::idle;
    std::atomic<Phase> phase_now = Phase::bed;
    std::atomic<uint8_t> stage_now = 0;
    std::atomic<uint8_t> taps_now = 0;
    std::atomic<uint16_t> runs = 0;
    std::atomic<uint16_t> revisions = 0;

    Measurement measurement; ///< mémoire statique, lue par l'interface

    Status from(session::Result r) {
        switch (r) {
        case session::Result::stopped:
            return Status::stopped;
        case session::Result::homing:
            return Status::homing;
        default:
            return Status::heating;
        }
    }

    Status measure() {
        if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
            return Status::printing;
        }
        if (stop_requested) {
            return Status::stopped;
        }
        if (filament_loaded()) {
            return Status::filament;
        }

        session::Session s(stop_requested); // consignes, maillage actif et le reste remis à la fin, buse levée
        Temperature::set_print_fan_speed(0); // le même air autour de la buse à chaque palier
        if (const session::Result r = s.heat_bed(bed_temperature, temperatures[0]); r != session::Result::ok) {
            return from(r);
        }
        if (!session::wait_ms(bed_soak_ms)) {
            return Status::stopped;
        }
        if (const session::Result r = s.home(temperatures[0]); r != session::Result::ok) {
            return from(r);
        }
        // Comme le G-code de départ de la CORE One avant de palper : un reste de plastique au bout de la buse, mou à
        // 260 °C, fausserait le contact
        GcodeSuite::process_subcommands_now_P("G29 P9 X208 Y-2.5 W32 H4");
        if (session::must_stop()) {
            return Status::stopped;
        }
        set_bed_leveling_enabled(false); // même point, z natif ; ~Session remet le maillage

        const xy_pos_t point { X_BED_SIZE / 2.f, Y_BED_SIZE / 2.f };
        for (uint8_t i = 0; i < stages; ++i) {
            stage_now = i;
            taps_now = 0;
            phase_now = Phase::heating;
            ++revisions;
            do_blocking_move_to_z(hover_z, session::travel_speed_z);
            do_blocking_move_to_xy(point, session::travel_speed_xy);
            if (const session::Result r = s.heat(temperatures[i], 1.5f, settle_ms, session::heat_timeout_ms, false); r != session::Result::ok) {
                return from(r);
            }

            phase_now = Phase::probing;
            ++revisions;
            std::array<float, taps> z;
            for (uint8_t k = 0; k < taps; ++k) {
                // Sans correction : ni la dilatation de Prusa, ni décalage d'outil
                z[k] = probe_at_point(point, PROBE_PT_RAISE, 0, true, 1, ApplyToolCorrections::no);
                if (session::must_stop()) {
                    return Status::stopped;
                }
                if (std::isnan(z[k])) {
                    return Status::probing;
                }
                taps_now = uint8_t(k + 1);
                ++revisions;
            }
            measurement.stage[i] = stage_of(z);
            measurement.done = uint8_t(i + 1);
            ++revisions;
        }

        measurement.outcome = evaluate(measurement.stage);
        switch (measurement.outcome.verdict) {
        case Verdict::ok:
            comp::set_measured_nm_per_c(measurement.key, comp::slope_to_nm(measurement.outcome.slope_mm_per_c));
            mesh_memory::forget(); // palpé avec l'ancienne pente
            return Status::saved;
        case Verdict::spread:
            return Status::spread;
        case Verdict::residual:
            return Status::residual;
        case Verdict::slope:
            break;
        }
        return Status::slope;
    }

} // namespace

void run() {
    ScopeGuard count_run = [] { ++runs; };
    if (running.exchange(true)) {
        return;
    }
    ScopeGuard not_running = [] {
        stop_requested = false;
        running = false;
    };

    measurement = {};
    measurement.key = comp::current_nozzle_key();
    measurement.before_nm = comp::measured_nm_per_c(measurement.key);
    stage_now = 0;
    taps_now = 0;
    phase_now = Phase::bed;
    last_status = Status::running;
    ++revisions;

    last_status = measure();
    ++revisions;
}

void forget() {
    if (marlin_server::is_printing() || running) {
        return;
    }
    comp::set_measured_nm_per_c(comp::current_nozzle_key(), 0);
    mesh_memory::forget(); // palpé avec la pente mesurée
    ++revisions;
}

bool filament_loaded() {
    return FSensors_instance().sensor_state(LogicalFilamentSensor::extruder) == FilamentSensorState::HasFilament;
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

Phase phase() {
    return phase_now.load();
}

uint8_t current_stage() {
    return stage_now.load();
}

uint8_t progress_percent() {
    constexpr unsigned per_stage = taps + 1; // chauffe, puis chaque palpage
    const unsigned done = unsigned(stage_now) * per_stage + (phase_now == Phase::probing ? 1u + taps_now : 0u);
    return uint8_t(std::min(100u, done * 100u / (unsigned(stages) * per_stage)));
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

} // namespace od3d::nozzle_expansion
