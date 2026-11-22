/// @file
#include "collision_od3d.hpp"

#include "force_probe_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>

#include <common/marlin_server.hpp>
#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <feature/prusa/e-stall_detector.h>
#include <option/has_local_accelerometer.h>
#include <timing.h>
#include <utils/atomic_circular_queue.hpp>
#if HAS_LOCAL_ACCELEROMETER()
    #include <module/prusa/accelerometer.h>
    #include <module/prusa/accelerometer_local.hpp>
#endif

namespace od3d::collision {

namespace {

    constexpr const char *csv_path = "/usb/OD3D_COLL.CSV";
    constexpr uint32_t test_duration_ms = 60'000;

    // Réglages (lus par le fil de Marlin, écrits par l'interface)
    std::atomic<bool> enabled { false };
    std::atomic<uint8_t> sensitivity { uint8_t(Sensitivity::normal) };
    std::atomic<uint8_t> action { uint8_t(Action::pause) };

    // Fil de Marlin → interruption
    std::atomic<bool> armed { false };
    std::atomic<bool> test_mode { false };
    std::atomic<float> log_threshold_g { 200.f };

    // Interruption seulement
    Detector detector;
    bool isr_on = false;

    // Interruption → fil de Marlin
    AtomicCircularQueue<Pulse, uint8_t, 4> pulses;
    std::atomic<uint16_t> lost { 0 }; ///< impulsions perdues (file ou journal pleins)

    // Interface → fil de Marlin
    std::atomic<bool> test_requested { false };

    // Fil de Marlin → interface
    std::atomic<bool> test_on { false };
    std::atomic<int16_t> last_peak { 0 };
    /// Alerte en un mot : x + 512 (bits 0-9), y + 512 (10-19), chocs (20-25), pause (26), essai (27), présente (31)
    std::atomic<uint32_t> alert_word { 0 };

    // Fil de Marlin seulement
    HitMap hit_map;
    uint32_t test_end_ms = 0;
    bool was_in_print = false;

    struct Row {
        uint32_t time_ms;
        Pulse pulse;
        float threshold_g;
        uint8_t near;
        uint8_t sensitivity;
        bool hit;
        bool alert;
        bool test;
    };
    std::array<Row, 12> rows;
    uint8_t row_count = 0;

    /// Garde la ligne ; journal plein : elle remplace la plus faible si elle est plus forte (les plus fortes servent
    /// à régler les seuils)
    void keep_row(const Row &row) {
        if (row_count < rows.size()) {
            rows[row_count++] = row;
            return;
        }
        lost.fetch_add(1, std::memory_order_relaxed);
        auto weakest = std::min_element(rows.begin(), rows.end(), [](const Row &a, const Row &b) { return a.pulse.peak_g < b.pulse.peak_g; });
        if (row.pulse.peak_g > weakest->pulse.peak_g) {
            *weakest = row;
        }
    }

    /// Ajoute les lignes en attente au CSV de la clé (imprimante à l'arrêt) ; sans clé, elles sont perdues
    void write_csv() {
        FILE *probe = fopen(csv_path, "r");
        const bool fresh = !probe;
        if (probe) {
            fclose(probe);
        }
        if (FILE *f = fopen(csv_path, "a")) {
            if (fresh) {
                force_probe::csv_printf(f, "t_ms,x_mm,y_mm,z_mm,peak_g,noise_g,duration_ms,accel_g,threshold_g,hit,near,alert,sensitivity,test,lost\n");
            }
            std::sort(rows.begin(), rows.begin() + row_count, [](const Row &a, const Row &b) { return int32_t(a.time_ms - b.time_ms) < 0; });
            for (uint8_t i = 0; i < row_count; ++i) {
                const Row &r = rows[i];
                force_probe::csv_printf(f, "%lu,%.1f,%.1f,%.2f,%.0f,%.1f,%u,%.2f,%.0f,%d,%u,%d,%u,%d,%u\n",
                    static_cast<unsigned long>(r.time_ms), double(r.pulse.x), double(r.pulse.y), double(r.pulse.z),
                    double(r.pulse.peak_g), double(r.pulse.noise_g), unsigned(r.pulse.duration_ms), double(r.pulse.accel_g),
                    double(r.threshold_g), int(r.hit), unsigned(r.near), int(r.alert), unsigned(r.sensitivity), int(r.test),
                    unsigned(lost.load(std::memory_order_relaxed)));
            }
            fclose(f);
        }
        row_count = 0;
    }

    int16_t clamp_mm(float v) {
        return std::isfinite(v) ? int16_t(std::clamp(std::lround(v), -512L, 511L)) : 0;
    }

    void handle(const Pulse &p, uint32_t now, const Settings &s, bool printing, bool test) {
        last_peak.store(int16_t(std::min(p.peak_g, 32767.f)), std::memory_order_relaxed);
        const Params prm = params_for(s.sensitivity);
        const bool hit = counts(p, prm.threshold_g);
        HitMap::Result r;
        if (hit) {
            r = hit_map.add(now, p.x, p.y, confirmed(p), prm.hits);
        }
        keep_row({ .time_ms = now, .pulse = p, .threshold_g = prm.threshold_g, .near = r.near, .sensitivity = uint8_t(s.sensitivity), .hit = hit, .alert = r.alert, .test = test });
        if (!r.alert) {
            return;
        }
        const bool pause = s.action == Action::pause && printing && !test;
        if (pause) {
            marlin_server::print_pause(); // la pause normale de Prusa, comme le bouton Pause
        }
        const uint32_t word = uint32_t(clamp_mm(p.x) + 512) | (uint32_t(clamp_mm(p.y) + 512) << 10)
            | (uint32_t(std::min<uint8_t>(r.near, 63)) << 20) | (uint32_t(pause) << 26) | (uint32_t(test) << 27) | (1u << 31);
        alert_word.store(word, std::memory_order_release);
    }

} // namespace

// ---- Réglages ----

Settings settings() {
    return {
        .enabled = enabled.load(),
        .sensitivity = Sensitivity(std::min<uint8_t>(sensitivity.load(), uint8_t(Sensitivity::high))),
        .action = action.load() == uint8_t(Action::warn) ? Action::warn : Action::pause,
    };
}

void set_settings(const Settings &s) {
    enabled = s.enabled;
    sensitivity = uint8_t(s.sensitivity);
    action = uint8_t(s.action);
    config_store().od3d_collision_enabled.set(s.enabled);
    config_store().od3d_collision_sensitivity.set(uint8_t(s.sensitivity));
    config_store().od3d_collision_action.set(uint8_t(s.action));
}

void load() {
    enabled = config_store().od3d_collision_enabled.get();
    sensitivity = config_store().od3d_collision_sensitivity.get();
    action = config_store().od3d_collision_action.get();
}

// ---- Interruption de la cellule de charge ----

void on_loadcell_sample(uint32_t time_us, float up_g, float x, float y, float z, float e, bool probing) {
    if (!armed.load(std::memory_order_acquire) || probing) {
        if (isr_on) {
            detector.sample(time_us, up_g, x, y, z, e, false, 0); // la moyenne repartira au réarmement
            isr_on = false;
        }
        return; // désarmé : rien d'autre
    }
    isr_on = true;
#if HAS_LOCAL_ACCELEROMETER()
    // Accéléromètre de la buse, seulement s'il répond et que personne ne s'en sert (sinon rien)
    accelerometer::RawAcceleration raw;
    while (prusa_accelerometer_local_try_get_sample(raw)) {
        detector.accel(accelerometer::raw_to_accel(PrusaAccelerometer::to_printer_coords(raw).val[2]) / 9.80665f);
    }
#endif
    if (test_mode.load(std::memory_order_relaxed)) {
        // Essai à l'arrêt : buse peut-être sans origine, plateau n'importe où
        z = INFINITY;
        x = std::isfinite(x) ? x : 0;
        y = std::isfinite(y) ? y : 0;
    }
    if (detector.sample(time_us, up_g, x, y, z, e, true, log_threshold_g.load(std::memory_order_relaxed))) {
        if (!pulses.enqueue(detector.pulse())) {
            lost.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

// ---- Fil de Marlin ----

void step() {
    const uint32_t now = ticks_ms();
    const bool in_print = marlin_server::is_printing(); // pauses comprises
    const bool printing = marlin_vars().print_state == marlin_server::State::Printing;

    if (test_requested.exchange(false)) {
        if (!in_print) {
            test_end_ms = now + test_duration_ms;
            test_on = true;
            hit_map.clear();
        }
    }
    bool test = test_on.load();
    if (test && (in_print || ticks_diff(now, test_end_ms) >= 0)) {
        test_on = test = false;
    }
    if (in_print && !was_in_print) {
        hit_map.clear(); // nouvelle impression : les chocs d'avant ne comptent plus
    }
    was_in_print = in_print;

    const Settings s = settings();
    const bool arm = ((s.enabled && printing) || test) && !EMotorStallDetector::Instance().Blocked();
    log_threshold_g.store(params_for(s.sensitivity).threshold_g * log_fraction, std::memory_order_relaxed);
    test_mode.store(test, std::memory_order_relaxed);
    armed.store(arm, std::memory_order_release);

    Pulse p;
    while (pulses.dequeue(p)) {
        if (arm) { // désarmé entre l'interruption et ici (début d'un changement d'outil…) : ignorée
            handle(p, now, s, printing, test);
        }
    }

    // La clé seulement à l'arrêt (pause, fin, essai) : jamais pendant un mouvement de l'impression
    if (row_count && marlin_server::printer_idle()) {
        write_csv();
    }
}

// ---- Interface ----

std::optional<Alert> take_alert() {
    const uint32_t w = alert_word.exchange(0, std::memory_order_acquire);
    if (!(w & (1u << 31))) {
        return std::nullopt;
    }
    return Alert {
        .hits = uint8_t((w >> 20) & 63),
        .x_mm = int16_t(int32_t(w & 1023) - 512),
        .y_mm = int16_t(int32_t((w >> 10) & 1023) - 512),
        .paused = bool(w & (1u << 26)),
        .test = bool(w & (1u << 27)),
    };
}

int last_peak_g() {
    return last_peak.load(std::memory_order_relaxed);
}

void start_test() {
    test_requested = true;
}

bool test_running() {
    return test_on.load() || test_requested.load();
}

} // namespace od3d::collision
