/// @file
#include "fan_response_od3d.hpp"
#include "live_od3d.hpp"
#include "lookahead_od3d.hpp"

#include <atomic>
#include <cmath>
#include <span>

#include <common/marlin_server.hpp>
#include <i18n.h>
#include <Marlin/src/Marlin.h>
#include <Marlin/src/module/planner.h>
#include <Marlin/src/module/temperature.h>
#include <raii/scope_guard.hpp>
#include <timing.h>

namespace od3d::fan_response {

namespace {

    constexpr uint32_t total_ms = spin_down_ms + baseline_ms + high_ms;
    constexpr uint16_t bin_ticks = uint16_t(lookahead::fan_bin_s * 1000 + 0.5f); ///< 50 ticks de 1 ms

    // Interruption (écrits par tick(), remis par start_sampling() avant le comptage)
    std::atomic<bool> sampling_on = false;
    std::array<uint8_t, max_bins> bin_edges {};
    std::atomic<uint8_t> bin_count = 0;
    std::atomic<uint16_t> ticks = 0; ///< depuis le début du comptage [ms]
    uint8_t edges_now = 0;
    bool last_level = false;

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<Status> last_status = Status::idle;
    std::atomic<uint8_t> progress = 0;
    std::atomic<uint16_t> runs = 0;
    Result last;

    bool must_stop() {
        return stop_requested.load() || planner.draining() || !marlin_server::printer_idle();
    }

    /// Attend ms ; done_ms : déjà fait de la mesure (avancement)
    bool wait(uint32_t ms, uint32_t done_ms) {
        const uint32_t start = ticks_ms();
        for (uint32_t t; (t = ticks_ms() - start) < ms;) {
            if (must_stop()) {
                return false;
            }
            progress = uint8_t((done_ms + t) * 100 / total_ms);
            idle(true);
        }
        return true;
    }

    void start_sampling() {
        bin_count = 0;
        ticks = 0;
        edges_now = 0;
        sampling_on = true; // en dernier : l'interruption ne lit rien avant
    }

    Status measure() {
        if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
            return Status::printing;
        }
        if (Temperature::degHotend(0) > max_nozzle_temperature || Temperature::degTargetHotend(0) > max_nozzle_temperature) {
            return Status::hot_nozzle; // V-09 : comme vibration::refused()
        }
        const uint8_t saved = Temperature::get_print_fan_speed();
        ScopeGuard restore = [saved] {
            sampling_on = false;
            Temperature::set_print_fan_speed(saved);
        };
        Temperature::set_print_fan_speed(0);
        live::set(N_("Part fan off, then 100%%: spin-up timed"));
        if (!wait(spin_down_ms, 0)) {
            return Status::stopped;
        }
        start_sampling();
        if (!wait(baseline_ms, spin_down_ms)) {
            return Status::stopped;
        }
        // Comme M106 : appliqué par Planner::check_axes_activity() (10 Hz), le retard compte dans la montée comme en
        // impression
        last.command_s = float(ticks.load()) / 1000.f;
        Temperature::set_print_fan_speed(255);
        if (!wait(high_ms, spin_down_ms + baseline_ms)) {
            return Status::stopped;
        }
        sampling_on = false;
        last.rise = lookahead::fan_rise(std::span<const uint8_t>(bin_edges.data(), bin_count.load()), last.command_s, min_rpm);
        if (!last.rise.found) {
            return Status::no_signal;
        }
        last.lead_ds = lookahead::fan_lead_for(last.rise.rise_s);
        live::set_value(N_("Measured: rise time %d ms"), int(std::lround(last.rise.rise_s * 1000.f)));
        lookahead::set_fan_lead_ds(last.lead_ds);
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
        progress = 100;
        running = false;
    };
    sampling_on = false;
    bin_count = 0;
    last = Result { .lead_before_ds = lookahead::fan_lead_ds() };
    progress = 0;
    last_status = Status::running;
    last_status = measure();
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

uint8_t progress_percent() {
    return progress.load();
}

uint16_t run_count() {
    return runs.load();
}

const std::array<uint8_t, max_bins> &edges() {
    return bin_edges;
}

uint8_t bins() {
    return bin_count.load();
}

const Result &result() {
    return last;
}

bool sampling() {
    return sampling_on.load(std::memory_order_relaxed);
}

void tick(bool level) {
    edges_now += level != last_level;
    last_level = level;
    const uint16_t t = uint16_t(ticks.load(std::memory_order_relaxed) + 1);
    ticks.store(t, std::memory_order_relaxed);
    if (t % bin_ticks == 0) {
        if (const uint8_t n = bin_count.load(std::memory_order_relaxed); n < max_bins) {
            bin_edges[n] = edges_now;
            bin_count.store(uint8_t(n + 1), std::memory_order_release);
        }
        edges_now = 0;
    }
}

} // namespace od3d::fan_response
