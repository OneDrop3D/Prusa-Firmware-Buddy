/// @file
#include "vibration_od3d.hpp"
#include "accelerometer_od3d.hpp"
#include "calib_registry_od3d.hpp"
#include "force_probe_od3d.hpp"
#include "live_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>

#include <common/marlin_server.hpp>
#include <config_store/store_instance.hpp>
#include <fanctl.hpp>
#include <i18n.h>
#include <selftest_fans.hpp>
#include <selftest_fans_config.hpp>
#include <Marlin/src/gcode/calibrate/M958.hpp>
#include <Marlin/src/Marlin.h>
#include <Marlin/src/module/motion.h>
#include <Marlin/src/module/planner.h>
#include <Marlin/src/module/prusa/fourier_series.h>
#include <Marlin/src/module/temperature.h>
#include <option/has_chamber_api.h>
#include <option/xbuddy_extension_variant.h>
#include <raii/scope_guard.hpp>
#include <timing.h>
#if HAS_CHAMBER_API()
    #include <feature/chamber/chamber.hpp>
#endif
#if XBUDDY_EXTENSION_VARIANT_IS_STANDARD()
    #include <feature/xbuddy_extension/xbuddy_extension.hpp>
#endif

namespace od3d::vibration {

namespace {

    /// Refus communs, avant tout mouvement (ACC-1 : l'accéléromètre vérifié en dernier, il synchronise le planificateur)
    std::optional<Status> refused(bool stop) {
        if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
            return Status::printing;
        }
        if (stop) {
            return Status::stopped;
        }
        if (Temperature::degHotend(0) > max_nozzle_temperature || Temperature::degTargetHotend(0) > max_nozzle_temperature) {
            return Status::hot_nozzle;
        }
        if (!accel::ready()) {
            return Status::no_accelerometer;
        }
        return std::nullopt;
    }

    /// Période d'échantillonnage étalonnée par la dernière mesure de Prusa (M1959, M1992...), sinon la nominale
    float sample_period(PrusaAccelerometer &accelerometer) {
        const auto p = vibrate_measure::maybe_calibrate_and_get_accelerometer_sample_period(accelerometer, false, [](float) -> vibrate_measure::Result<void> { return {}; });
        return p.has_value() && p.value() > 0.f ? p.value() : 1.f / 1344.f;
    }

    enum class Read : uint8_t {
        sample,
        empty,
        lost, ///< file pleine : échantillons perdus
        failed, ///< l'accéléromètre ne répond plus
    };

    /// Un échantillon (m/s², axes de l'imprimante), comme M1993
    Read read(PrusaAccelerometer &accelerometer, accelerometer::Acceleration &out) {
        accelerometer::RawAcceleration raw;
        switch (accelerometer.get_sample_printer_coords(raw)) {
        case PrusaAccelerometer::GetSampleResult::ok:
            out = raw.to_acceleration();
            return Read::sample;
        case PrusaAccelerometer::GetSampleResult::buffer_empty:
            return Read::empty;
        case PrusaAccelerometer::GetSampleResult::error:
            break;
        }
        if (accelerometer.get_error() == accelerometer::Error::communication) {
            return Read::failed;
        }
        accelerometer.clear();
        return Read::lost;
    }

} // namespace

} // namespace od3d::vibration

// ---- Accélération max utile (M2004) ----

namespace od3d::accel_limit {

using namespace vibration;

namespace {

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<Status> last_status = Status::idle;
    std::atomic<uint8_t> progress = 0;
    Measurement measurement;

    constexpr const char *csv_path = "/usb/OD3D_ACCEL.CSV";
    constexpr float lag_s = 0.03f; ///< retard possible des échantillons (file de l'accéléromètre), comme M1993
    constexpr float window_s = 0.25f; ///< résidu après l'arrêt (ζ 0,05 à 50 Hz : il en reste 2 % à 250 ms)
    constexpr uint8_t moves = 4; ///< par palier : aller, retour, aller, retour (moyenne)
    constexpr uint32_t window_timeout_ms = 1000;

    constexpr float min_z = 10.f;
    constexpr float above_print = 5.f;
    constexpr float travel_speed_xy = 50.f;
    constexpr float travel_speed_z = 10.f;

    constexpr auto levels = [] {
        std::array<uint16_t, accel_levels> a {};
        for (size_t i = 0; i < a.size(); ++i) {
            a[i] = level_accel(i);
        }
        return a;
    }();

    bool must_stop() {
        return stop_requested.load() || planner.draining() || !marlin_server::printer_idle();
    }

    /// Comme damping_od3d.cpp : la position de M1959 (centre du plateau, mi-hauteur), sans descendre sous ce qui a été
    /// imprimé ; jamais d'origine ici (accéléromètre fixé) ; false sans origine
    bool prepare() {
        if (must_stop() || axes_need_homing()) {
            return false;
        }
        const float z = std::min(std::max({ min_z, float(Z_SIZE) / 2.f, planner.max_printed_z + above_print }), soft_endstop.max.z - 1.f);
        if (current_position.z < z) {
            do_blocking_move_to_z(z, travel_speed_z);
        }
        do_blocking_move_to_xy(std::clamp(X_BED_SIZE / 2.f, soft_endstop.min.x, soft_endstop.max.x), std::clamp(Y_BED_SIZE / 2.f, soft_endstop.min.y, soft_endstop.max.y), travel_speed_xy);
        planner.synchronize();
        return !must_stop();
    }

    /// Résidu (variance, m²/s⁴) dans la fenêtre qui suit la fin du mouvement en cours (au repos : tout de suite)
    Status residual(PrusaAccelerometer &accelerometer, float period, float &out) {
        accelerometer::Acceleration a;
        while (planner.processing()) { // pendant le mouvement : échantillons jetés
            if (read(accelerometer, a) == Read::failed) {
                planner.synchronize();
                return Status::failed;
            }
            idle(true);
        }
        const uint32_t skip = uint32_t(lag_s / period);
        const uint32_t want = uint32_t(window_s / period);
        WindowMeter meter;
        uint32_t seen = 0;
        const uint32_t start = ticks_ms();
        while (meter.count() < want) {
            const Read r = read(accelerometer, a);
            if (r == Read::sample) {
                if (++seen > skip) {
                    meter.add(a.val[0], a.val[1], 0.f); // Z ignoré : le ringing des parois est dans le plan XY
                }
                continue;
            }
            if (r == Read::failed || ticks_ms() - start > window_timeout_ms) {
                return Status::failed;
            }
            idle(true); // vide ou perdus (file pleine) : la fenêtre continue, comme M1993
        }
        out = meter.variance();
        return Status::done;
    }

    void write_csv_header() {
        if (FILE *f = fopen(csv_path, "a")) {
            force_probe::csv_printf(f, "# OneDrop3D M2004 max useful acceleration at X%.1f Y%.1f Z%.1f: ramps %d ms, window %d ms after the stop, %d moves per level\n",
                double(current_position.x), double(current_position.y), double(current_position.z), int(ramp_s * 1000), int(window_s * 1000), int(moves));
            force_probe::csv_printf(f, "# noise at rest: rms %.5f m/s2\naxis,accel_mm_s2,residual_rms_m_s2\n", double(std::sqrt(measurement.noise)));
            fclose(f);
        }
    }

    void write_csv_axis(uint8_t axis) {
        FILE *f = fopen(csv_path, "a");
        if (!f) {
            return; // pas de clé
        }
        for (size_t i = 0; i < accel_levels; ++i) {
            if (std::isfinite(measurement.residual[axis][i])) {
                force_probe::csv_printf(f, "%c,%u,%.5f\n", 'X' + axis, unsigned(levels[i]), double(std::sqrt(measurement.residual[axis][i])));
            }
        }
        const Limit &l = measurement.limits[axis];
        force_probe::csv_printf(f, "# %c: found %d, reference %u, limit %u mm/s2, residual rose %d, top %u\n", 'X' + axis, int(l.found), unsigned(levels[l.reference]),
            unsigned(levels[l.last_ok]), int(l.rose), unsigned(measurement.top[axis]));
        fclose(f);
    }

    Status measure() {
        if (const auto r = refused(stop_requested)) {
            return *r;
        }
        // Réglages de mouvement de la mesure, tous remis à la fin (comme M1993) : mode furtif coupé, accélérations
        planner.synchronize();
        const auto saved_settings = planner.user_settings;
        const bool stealth = config_store().stealth_mode.get();
        ScopeGuard restore = [&] {
            if (!planner.draining()) {
                planner.synchronize();
            }
            planner.apply_settings(saved_settings);
            planner.set_stealth_mode(stealth);
        };
        planner.set_stealth_mode(false);
        if (!prepare()) {
            return must_stop() ? Status::stopped : Status::not_homed;
        }
        {
            auto s = planner.user_settings;
            for (const AxisEnum axis : { X_AXIS, Y_AXIS }) {
                s.max_acceleration_mm_per_s2[axis] = levels.back();
                s.max_feedrate_mm_s[axis] = std::max<float>(s.max_feedrate_mm_s[axis], levels.back() * ramp_s);
            }
            planner.apply_settings(s); // les limites matérielles de Prusa s'appliquent encore : elles bornent les paliers
            for (uint8_t a = 0; a < axis_count; ++a) {
                measurement.top[a] = uint16_t(std::min<uint32_t>(planner.settings.max_acceleration_mm_per_s2[a], levels.back()));
            }
        }

        PrusaAccelerometer accelerometer;
        const float period = sample_period(accelerometer);
        live::set(N_("Head still: accelerometer noise"));
        for (const uint32_t start = ticks_ms(); ticks_ms() - start < 500;) {
            idle(true); // le déplacement jusqu'au centre a fini de vibrer
        }
        accelerometer.clear();
        float sum = 0.f;
        for (uint8_t m = 0; m < moves; ++m) {
            float r;
            if (const Status s = residual(accelerometer, period, r); s != Status::done) {
                return s;
            }
            sum += r;
        }
        measurement.noise = sum / moves;
        write_csv_header();

        for (uint8_t a = 0; a < axis_count; ++a) {
            const AxisEnum axis = a == 0 ? X_AXIS : Y_AXIS;
            const float centre = current_position[axis];
            for (size_t i = 0; i < accel_levels && levels[i] <= measurement.top[a]; ++i) {
                live::set(N_("Axis %c: short moves at %d mm/s2, ringing after the stop"), 'X' + a, levels[i]);
                {
                    auto s = planner.user_settings;
                    s.travel_acceleration = s.acceleration = levels[i];
                    planner.apply_settings(s);
                }
                // Triangle de rampes ramp_s : même forme à chaque palier, mise à l'échelle par l'accélération
                const float speed = levels[i] * ramp_s;
                const float length = speed * ramp_s;
                sum = 0.f;
                for (uint8_t m = 0; m < moves; ++m) {
                    if (must_stop()) {
                        return Status::stopped;
                    }
                    current_position[axis] = centre + (m % 2 ? 0.f : length);
                    line_to_current_position(speed);
                    float r;
                    if (const Status s = residual(accelerometer, period, r); s != Status::done) {
                        return s;
                    }
                    sum += r;
                }
                measurement.residual[a][i] = sum / moves;
                progress = uint8_t((a * accel_levels + i + 1) * 100 / (axis_count * accel_levels));
            }
            measurement.limits[a] = find_limit(measurement.residual[a], levels, measurement.noise);
            write_csv_axis(a);
            if (const auto p = proposal(a)) {
                live::set_value(N_("Measured: axis %c up to %d mm/s2"), 'X' + a, *p);
            }
        }
        return Status::done;
    }

} // namespace

void run() {
    if (running.exchange(true)) {
        return;
    }
    ScopeGuard not_running = [] {
        stop_requested = false;
        running = false;
    };
    measurement = {};
    for (auto &axis : measurement.residual) {
        axis.fill(NAN);
    }
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

const Measurement &last() {
    return measurement;
}

std::optional<uint16_t> proposal(uint8_t axis) {
    if (axis >= axis_count || !measurement.limits[axis].found) {
        return std::nullopt;
    }
    return levels[measurement.limits[axis].last_ok];
}

} // namespace od3d::accel_limit

// ---- Vibrations des ventilateurs (M2001 V) ----

namespace od3d::fan_vibration {

using namespace vibration;

namespace {

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<Status> last_status = Status::idle;
    std::atomic<uint8_t> progress = 0;
    Measurement measurement;

    constexpr const char *csv_path = "/usb/OD3D_FANS.CSV";
    constexpr uint32_t spin_down_ms = 4000; ///< arrêt d'un ventilateur (5015 : ~3 s)
    constexpr uint32_t settle_ms = 4000; ///< régime établi...
    constexpr uint32_t rpm_from_ms = 2000; ///< ...et lu sur les 2 dernières secondes (tachymètre partagé : 1 s sur 2)
    constexpr uint32_t rpm_every_ms = 10;
    constexpr float window_s = 1.f; ///< transformée sur 1 s
    constexpr uint16_t min_rpm = 600; ///< en dessous : pas de régime lu, fréquence inconnue
    constexpr uint8_t window_tries = 3;
    constexpr size_t values_index = size_t(calib_registry::Test::fan_response);

    /// Une transformée en un point par fréquence (celle de M958) ; ~1,2 Ko hors de la pile de Marlin
    std::array<std::optional<FourierSeries3d>, fan_bins> bins;

    constexpr const char *fan_text[fan_count] = {
        N_("Print fan alone at %d%%: vibration at its speed"),
        N_("Hotend fan alone at %d%%: vibration at its speed"),
        N_("Chamber fan 1 alone at %d%%: vibration at its speed"),
        N_("Chamber fan 2 alone at %d%%: vibration at its speed"),
    };

    bool must_stop() {
        return stop_requested.load() || planner.draining() || !marlin_server::printer_idle();
    }

    /// Attend ms en vidant la file de l'accéléromètre ; à partir de from_ms : régime du ventilateur et moyenne des
    /// échantillons (pesanteur, retirée avant la transformée)
    Status settle(fan_selftest::FanHandler *fan, uint32_t ms, uint32_t from_ms, PrusaAccelerometer &accelerometer, std::array<float, 3> &mean) {
        double sum[3] = { 0, 0, 0 };
        uint32_t n = 0;
        uint32_t rpm_at = 0;
        if (fan) {
            fan->reset_samples();
        }
        const uint32_t start = ticks_ms();
        for (uint32_t t; (t = ticks_ms() - start) < ms;) {
            if (must_stop()) {
                return Status::stopped;
            }
            accelerometer::Acceleration a;
            for (Read r; (r = read(accelerometer, a)) != Read::empty;) {
                if (r == Read::failed) {
                    return Status::failed;
                }
                if (r == Read::sample && t >= from_ms) {
                    for (int i = 0; i < 3; ++i) {
                        sum[i] += a.val[i];
                    }
                    ++n;
                }
            }
            if (fan && t >= from_ms && t - rpm_at >= rpm_every_ms) {
                fan->record_sample();
                rpm_at = t;
            }
            idle(true);
        }
        for (int i = 0; i < 3; ++i) {
            mean[i] = n ? float(sum[i] / n) : 0.f;
        }
        return Status::done;
    }

    /// Amplitude à la fréquence de rotation f0 (Hz) et bruit voisin, sur window_s ; un trou dans les échantillons casse
    /// la phase : la fenêtre recommence
    Status measure_tone(float f0, float period, const std::array<float, 3> &mean, PrusaAccelerometer &accelerometer, Tone &out) {
        const auto frequencies = bin_frequencies(f0);
        const uint32_t want = uint32_t(window_s / period);
        for (uint8_t attempt = 0; attempt < window_tries; ++attempt) {
            for (size_t k = 0; k < fan_bins; ++k) {
                bins[k].emplace(frequencies[k]);
            }
            accelerometer.clear();
            uint32_t n = 0;
            bool lost = false;
            const uint32_t start = ticks_ms();
            while (n < want && !lost) {
                if (must_stop()) {
                    return Status::stopped;
                }
                accelerometer::Acceleration a;
                switch (read(accelerometer, a)) {
                case Read::sample: {
                    const accelerometer::Acceleration d(a.val[0] - mean[0], a.val[1] - mean[1], a.val[2] - mean[2]);
                    const float t = float(n) * period;
                    for (auto &b : bins) {
                        b->add_sample(t, d);
                    }
                    ++n;
                    continue;
                }
                case Read::failed:
                    return Status::failed;
                case Read::lost:
                    lost = true;
                    break;
                case Read::empty:
                    if (ticks_ms() - start > 3 * uint32_t(window_s * 1000)) {
                        return Status::failed;
                    }
                    idle(true);
                    break;
                }
            }
            if (lost) {
                continue;
            }
            std::array<float, fan_bins> magnitudes;
            for (size_t k = 0; k < fan_bins; ++k) {
                const xyz_float_t m = bins[k]->get_magnitude();
                magnitudes[k] = std::sqrt(sq(m.x) + sq(m.y) + sq(m.z));
            }
            out = tone(magnitudes);
            return Status::done;
        }
        return Status::failed;
    }

    void write_csv_fan(uint8_t i) {
        FILE *f = fopen(csv_path, "a");
        if (!f) {
            return; // pas de clé
        }
        const FanResult &r = measurement.fans[i];
        for (size_t k = 0; k < fan_pwm.size(); ++k) {
            force_probe::csv_printf(f, "%u,%u,%u,%.2f,%.5f,%.5f,%d\n", unsigned(i + 1), unsigned(fan_pwm[k]), unsigned(r.rpm[k]), double(r.rpm[k] / 60.f),
                double(r.tones[k].amplitude), double(r.tones[k].noise), int(r.tones[k].detected));
        }
        force_probe::csv_printf(f, "# fan %u: level %.5f m/s2, detected %d, reference %.5f, verdict %d\n", unsigned(i + 1), double(r.level.value), int(r.level.detected),
            double(r.reference.value_or(0.f)), int(r.verdict));
        fclose(f);
    }

    Status measure() {
        if (const auto r = refused(stop_requested)) {
            return *r;
        }
        // Les commandes du test des ventilateurs de Prusa (M1978) : chaque ventilateur seul, tous remis comme avant à la fin
        using namespace fan_selftest;
        const PhysicalToolIndex tool = PhysicalToolIndex::from_raw(0);
        CommonFanHandler print_fan(FanType::print, 0, print_fan_range, &Fans::print(tool));
        CommonFanHandler hotend_fan(FanType::heatbreak, 0, heatbreak_fan_range, &Fans::heat_break(tool));
        std::array<FanHandler *, fan_count> fans { &print_fan, &hotend_fan, nullptr, nullptr };
        std::array<FanHandler *, fan_count + 1> all { &print_fan, &hotend_fan, nullptr, nullptr, nullptr };
#if XBUDDY_EXTENSION_VARIANT_IS_STANDARD() && HAS_CHAMBER_API()
        XBEFanHandler cooling_1(FanType::xbe_chamber, 0, chamber_fan_range);
        XBEFanHandler cooling_2(FanType::xbe_chamber, 1, chamber_fan_range);
        XBEFanHandler filtration(FanType::xbe_chamber, 2, filtration_fan_range);
        all = { &print_fan, &hotend_fan, &cooling_1, &cooling_2, &filtration };
        if (buddy::chamber().backend() == buddy::Chamber::Backend::xbuddy_extension) {
            // shortcut: un changement filtration ↔ ventilateurs de la chambre garde la référence de la case, à refaire
            if (buddy::xbuddy_extension().using_filtration_fan_instead_of_cooling_fans()) {
                fans[2] = &filtration;
            } else {
                fans[2] = &cooling_1;
                fans[3] = &cooling_2;
            }
        }
#endif
        for (FanHandler *f : all) {
            if (f) {
                f->set_pwm(0);
            }
        }

        PrusaAccelerometer accelerometer;
        const float period = sample_period(accelerometer);
        std::array<float, 3> mean;
        live::set(N_("All fans off, waiting for them to stop"));
        if (const Status s = settle(nullptr, spin_down_ms, spin_down_ms, accelerometer, mean); s != Status::done) {
            return s;
        }
        if (FILE *f = fopen(csv_path, "a")) {
            force_probe::csv_printf(f, "# OneDrop3D M2001 V fan vibration, each fan alone, motors still; amplitude at the rotation speed (rpm / 60)\n"
                                       "fan,pwm,rpm,freq_hz,amplitude_m_s2,noise_m_s2,detected\n");
            fclose(f);
        }

        const std::array<int16_t, 2> stored = stored_references();
        std::array<uint8_t, fan_count> codes = unpack_references(stored[0], stored[1]);
        const size_t present = size_t(std::count_if(fans.begin(), fans.end(), [](FanHandler *f) { return f != nullptr; }));
        size_t done = 0;
        for (uint8_t i = 0; i < fan_count; ++i) {
            if (!fans[i]) {
                continue;
            }
            FanResult &r = measurement.fans[i];
            r.present = true;
            r.reference = decode(codes[i]);
            for (size_t k = 0; k < fan_pwm.size(); ++k) {
                live::set(fan_text[i], fan_pwm[k] * 100 / 255);
                fans[i]->set_pwm(fan_pwm[k]);
                if (const Status s = settle(fans[i], settle_ms, settle_ms - rpm_from_ms, accelerometer, mean); s != Status::done) {
                    return s;
                }
                r.rpm[k] = fans[i]->calculate_avg_rpm();
                r.tones[k].amplitude = NAN; // pas de régime lu : ce régime ne compte pas
                if (r.rpm[k] >= min_rpm) {
                    if (const Status s = measure_tone(r.rpm[k] / 60.f, period, mean, accelerometer, r.tones[k]); s != Status::done) {
                        return s;
                    }
                }
                progress = uint8_t(std::min<size_t>(100, (done * fan_pwm.size() + k + 1) * 100 / (present * fan_pwm.size())));
            }
            fans[i]->set_pwm(0);
            r.level = fan_level(r.tones);
            r.verdict = compare(r.level, r.reference);
            if (r.verdict == Verdict::reference) {
                codes[i] = encode(r.level.value); // la première mesure captée devient la référence
            }
            write_csv_fan(i);
            if (++done < present) {
                live::set(N_("All fans off, waiting for them to stop"));
                if (const Status s = settle(nullptr, spin_down_ms, spin_down_ms, accelerometer, mean); s != Status::done) {
                    return s;
                }
            }
        }

        // Références gardées dans la case du ventilateur (OD3D Calib Values) : [0], [1] restent ceux du temps de montée
        auto values = config_store().od3d_calib_values.get(values_index);
        values.v[2] = pack(codes[0], codes[1]);
        values.v[3] = pack(codes[2], codes[3]);
        config_store().od3d_calib_values.set(values_index, values);
        return Status::done;
    }

} // namespace

void run() {
    if (running.exchange(true)) {
        return;
    }
    ScopeGuard not_running = [] {
        stop_requested = false;
        running = false;
    };
    measurement = {};
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

const Measurement &last() {
    return measurement;
}

std::array<int16_t, 2> stored_references() {
    const auto values = config_store().od3d_calib_values.get(values_index);
    return { values.v[2], values.v[3] };
}

} // namespace od3d::fan_vibration
