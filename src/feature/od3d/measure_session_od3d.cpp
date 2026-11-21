/// @file
#include "measure_session_od3d.hpp"
#include "flow_limit_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include <common/marlin_server.hpp>
#include <config_store/store_instance.hpp>
#include <Marlin.h>
#include <feature/pressure_advance/pressure_advance_config.hpp>
#include <feature/prusa/e-stall_detector.h>
#include <gcode/gcode.h>
#include <module/motion.h>
#include <module/planner.h>
#include <module/temperature.h>
#include <timing.h>
#include <tool/hotend/hotend.hpp>

namespace od3d::session {

namespace {

    constexpr float retract_mm = 0.8f; ///< à la fin, contre le suintement
    constexpr float default_smooth_time = 0.04f;

    /// Drapeau d'arrêt de la session en cours (nullptr sans session)
    const std::atomic<bool> *active_stop = nullptr;

    // État d'avant la session (une seule à la fois, fil de Marlin)
    pressure_advance::Config saved_pa;
    user_planner_settings_t saved_settings;
    bool saved_stealth = false;
    Hotend::TargetTemperature saved_target = 0;
    std::optional<BlockEStallDetection> block_estall; ///< la détection de filament coincé lirait nos paliers comme des à-coups

    Hotend &hotend() {
        return Hotend::for_tool(PhysicalToolIndex::from_raw(0));
    }

} // namespace

bool must_stop() {
    return (active_stop && active_stop->load()) || planner.draining() || marlin_server::is_printing() || !marlin_server::printer_idle();
}

bool wait_ms(uint32_t ms) {
    const uint32_t start = ticks_ms();
    while (ticks_ms() - start < ms) {
        if (must_stop()) {
            return false;
        }
        idle(true);
    }
    return true;
}

bool wait_moves() {
    while (planner.processing()) {
        if (must_stop()) {
            return false;
        }
        idle(true);
    }
    return !must_stop();
}

bool wait_queue(uint8_t depth) {
    while (planner.movesplanned() >= depth) {
        if (must_stop()) {
            return false;
        }
        idle(true);
    }
    return !must_stop();
}

Session::Session(const std::atomic<bool> &stop_requested) {
    active_stop = &stop_requested;
    planner.synchronize();
    saved_pa = pressure_advance::get_axis_e_config();
    saved_settings = planner.user_settings;
    saved_stealth = config_store().stealth_mode.get();
    saved_target = hotend().nozzle_target_temp();
    block_estall.emplace();
    flow_limit::suspended = true;
}

Session::~Session() {
    if (!planner.draining()) {
        // Un peu de filament en arrière contre le suintement, puis les réglages d'avant
        if (!thermalManager.tooColdToExtrude(PhysicalToolIndex::from_raw(0))) {
            current_position.e -= retract_mm;
            line_to_current_position(35.f);
        }
        planner.synchronize();
    }
    if (!planner.draining()) {
        pressure_advance::set_axis_e_config(saved_pa);
    }
    planner.apply_settings(saved_settings);
    planner.set_stealth_mode(saved_stealth);
    hotend().set_nozzle_target_temp(saved_target);
    block_estall.reset();
    flow_limit::suspended = false;
    active_stop = nullptr;
}

float Session::smooth_time() const {
    return saved_pa.smooth_time > 0.f ? saved_pa.smooth_time : default_smooth_time;
}

Result Session::go_to(float x_lo, float x_hi, float x_start, float &y, float y_span, float z) {
    // Origine si nécessaire (avant de chauffer)
    if (axes_need_homing()) {
        GcodeSuite::G28_no_parser(true, true, true, { .only_if_needed = true, .precise = false });
    }
    if (must_stop()) {
        return Result::stopped;
    }
    if (axes_need_homing()) {
        return Result::homing;
    }
    y = std::clamp(y, soft_endstop.min.y + edge_margin, soft_endstop.max.y - edge_margin - std::max(y_span, 0.f));
    z = std::min(std::max(z, planner.max_printed_z + 10.f), soft_endstop.max.z - edge_margin);
    if (x_lo < soft_endstop.min.x + edge_margin || x_hi > soft_endstop.max.x - edge_margin || z < 5.f
        || y + y_span > soft_endstop.max.y - edge_margin) {
        return Result::no_room;
    }
    if (current_position.z < z) {
        do_blocking_move_to_z(z, travel_speed_z);
    }
    do_blocking_move_to_xy(x_start, y, travel_speed_xy);
    return must_stop() ? Result::stopped : Result::ok;
}

Result Session::heat(int16_t temperature, float tolerance, uint32_t hold_ms, uint32_t timeout_ms) {
    Hotend &h = hotend();
    h.set_nozzle_target_temp(temperature);
    const uint32_t start = ticks_ms();
    std::optional<uint32_t> inside_since;
    while (true) {
        if (must_stop()) {
            return Result::stopped;
        }
        // La consigne réelle (Prusa peut la borner) ; une consigne retirée entre-temps (refroidissement) : échec
        const Hotend::TargetTemperature target = h.nozzle_target_temp();
        if (target < EXTRUDE_MINTEMP) {
            return Result::heating;
        }
        const bool inside = std::fabs(h.nozzle_temp().value_or(0.f) - float(target)) <= tolerance;
        const uint32_t now = ticks_ms();
        if (!inside) {
            inside_since.reset();
        } else if (!inside_since) {
            inside_since = now;
        } else if (now - *inside_since >= hold_ms) {
            break;
        }
        if (now - start > timeout_ms) {
            return Result::heating;
        }
        idle(true);
    }
    return thermalManager.tooColdToExtrude(PhysicalToolIndex::from_raw(0)) ? Result::heating : Result::ok;
}

bool Session::prime(float mm, float speed) {
    current_position.e += mm;
    line_to_current_position(speed);
    return wait_moves() && wait_ms(500);
}

} // namespace od3d::session
