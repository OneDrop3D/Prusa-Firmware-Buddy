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
#include <Marlin/src/feature/input_shaper/input_shaper_config.hpp>
#include <Marlin/src/gcode/calibrate/M958.hpp>
#include <Marlin/src/Marlin.h>
#include <Marlin/src/module/motion.h>
#include <Marlin/src/module/planner.h>
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

    /// Période supposée : la valeur de départ de M958 (LIS2DH12 à 1 344 Hz), tant que rien ne l'a étalonnée
    constexpr float nominal_period = 1.f / 1344.f;

    /// Période d'échantillonnage étalonnée par la dernière mesure de Prusa (M1959, M1992...), sinon la nominale
    float sample_period(PrusaAccelerometer &accelerometer) {
        const auto p = vibrate_measure::maybe_calibrate_and_get_accelerometer_sample_period(accelerometer, false, [](float) -> vibrate_measure::Result<void> { return {}; });
        return p.has_value() && p.value() > 0.f ? p.value() : nominal_period;
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
            force_probe::csv_printf(f, "# OneDrop3D M2004 max useful acceleration at X%.1f Y%.1f Z%.1f: ramps of 1/(2 f), f the input shaper frequency of the axis (%d ms without), window %d ms after the stop, %d moves per level\n",
                double(current_position.x), double(current_position.y), double(current_position.z), int(ramp_s * 1000), int(window_s * 1000), int(moves));
            force_probe::csv_printf(f, "# noise at rest: rms %.5f m/s2\naxis,accel_mm_s2,residual_rms_m_s2\n", double(std::sqrt(measurement.noise)));
            fclose(f);
        }
    }

    void write_csv_axis(uint8_t axis, float ramp) {
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
        force_probe::csv_printf(f, "# %c: ramp %.2f ms, found %d, reference %u, limit %u mm/s2, residual rose %d, top %u\n", 'X' + axis, double(ramp * 1000.f), int(l.found),
            unsigned(levels[l.reference]), unsigned(levels[l.last_ok]), int(l.rose), unsigned(measurement.top[axis]));
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
            const auto shaper = input_shaper::get_axis_config(axis);
            const float ramp = ramp_for(shaper ? shaper->frequency : NAN); // V-05 : demi-période de la résonance de l'axe
            for (size_t i = 0; i < accel_levels && levels[i] <= measurement.top[a]; ++i) {
                live::set(N_("Axis %c: short moves at %d mm/s2, ringing after the stop"), 'X' + a, levels[i]);
                {
                    auto s = planner.user_settings;
                    s.travel_acceleration = s.acceleration = levels[i];
                    planner.apply_settings(s);
                }
                // Triangle de rampes ramp : même forme à chaque palier, mise à l'échelle par l'accélération
                const float speed = levels[i] * ramp;
                const float length = speed * ramp;
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
            write_csv_axis(a, ramp);
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

    /// Une transformée en un point par fréquence (ToneBank, vibration_math_od3d.hpp) ; ~0,8 Ko hors de la pile de Marlin
    ToneBank bank;

    constexpr const char *fan_text[fan_count] = {
        N_("Print fan alone at %d%%: vibration at its speed"),
        N_("Hotend fan alone at %d%%: vibration at its speed"),
        N_("Chamber fan 1 alone at %d%%: vibration at its speed"),
        N_("Chamber fan 2 alone at %d%%: vibration at its speed"),
    };

    bool must_stop() {
        return stop_requested.load() || planner.draining() || !marlin_server::printer_idle();
    }

    /// V-08 : la période étalonnée, jamais la supposée (les bins sont à 0,5 % près : une puce à 1 300 Hz au lieu de
    /// 1 344 décale le pic de 3 %, au bord de sa fenêtre). Sans mesure qui l'ait étalonnée depuis le démarrage (M958 la
    /// garde ; M1959, M1992 l'étalonnent), étalonnée ici, une fois (~15 s) : référence et comparaison ont la vraie
    /// période, d'un démarrage à l'autre
    Status calibrated_period(PrusaAccelerometer &accelerometer, float &period) {
        period = sample_period(accelerometer);
        if (period != nominal_period) {
            return Status::done;
        }
        live::set(N_("Accelerometer calibration"));
        const auto p = vibrate_measure::maybe_calibrate_and_get_accelerometer_sample_period(accelerometer, true, [](float) -> vibrate_measure::Result<void> {
            if (must_stop()) {
                return std::unexpected(vibrate_measure::Error::aborted);
            }
            idle(true);
            return {};
        });
        if (!p.has_value() || !(p.value() > 0.f)) {
            return must_stop() ? Status::stopped : Status::failed;
        }
        period = p.value();
        return Status::done;
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
            bank.start(frequencies, period);
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
                    bank.add(a.val[0] - mean[0], a.val[1] - mean[1], a.val[2] - mean[2]);
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
            out = tone(bank.magnitudes());
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
        float period;
        if (const Status s = calibrated_period(accelerometer, period); s != Status::done) {
            return s;
        }
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

        // Références gardées dans la case du ventilateur (OD3D Calib Values) : [0], [1] restent ceux du temps de montée ;
        // [3] marqué : prises à la période étalonnée (V-08)
        auto values = config_store().od3d_calib_values.get(values_index);
        values.v[2] = pack(codes[0], codes[1]);
        values.v[3] = int16_t(pack(codes[2], codes[3]) | calibrated_mark);
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

// ---- Réglages de mouvement des mesures M2005 et M2006 ----

namespace od3d::vibration {

namespace {

    /// Comme M2004 : mode furtif coupé, accélération de la mesure ; tout est remis à la fin
    class MotionSettings {
    public:
        MotionSettings()
            : saved(planner.user_settings)
            , stealth(config_store().stealth_mode.get()) {
            planner.set_stealth_mode(false);
        }
        ~MotionSettings() {
            if (!planner.draining()) {
                planner.synchronize();
            }
            planner.apply_settings(saved);
            planner.set_stealth_mode(stealth);
        }
        MotionSettings(const MotionSettings &) = delete;
        MotionSettings &operator=(const MotionSettings &) = delete;

        /// Accélération de X et Y pour les déplacements qui suivent, bornée par les limites matérielles de Prusa ;
        /// rend celle qui s'applique vraiment
        uint16_t accelerate(uint16_t accel, float speed) {
            auto s = planner.user_settings;
            for (const AxisEnum axis : { X_AXIS, Y_AXIS }) {
                s.max_acceleration_mm_per_s2[axis] = std::max<uint32_t>(s.max_acceleration_mm_per_s2[axis], accel);
                s.max_feedrate_mm_s[axis] = std::max(s.max_feedrate_mm_s[axis], speed);
            }
            planner.apply_settings(s);
            const uint16_t real = uint16_t(std::min<uint32_t>({ uint32_t(accel), planner.settings.max_acceleration_mm_per_s2[X_AXIS], planner.settings.max_acceleration_mm_per_s2[Y_AXIS] }));
            s = planner.user_settings;
            s.travel_acceleration = s.acceleration = real;
            planner.apply_settings(s);
            return real;
        }

    private:
        user_planner_settings_t saved; ///< une copie : planner.user_settings est une référence aux réglages vivants
        bool stealth;
    };

} // namespace

} // namespace od3d::vibration

// ---- Pose de l'accéléromètre (M2005) ----

namespace od3d::accel_mount {

using namespace vibration;

namespace {

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<Status> last_status = Status::idle;
    std::atomic<uint8_t> progress = 0;
    Measurement measurement;

    constexpr const char *csv_path = "/usb/OD3D_MOUNT.CSV";
    constexpr uint32_t settle_ms = 500; ///< le déplacement jusqu'au centre a fini de vibrer
    constexpr uint32_t rest_ms = 1000; ///< moyenne au repos : pesanteur et bruit
    constexpr uint32_t after_ms = 150; ///< après la fin du mouvement : le retard des échantillons et le freinage
    constexpr uint32_t sample_timeout_ms = 1000;
    constexpr float sweep_mm = 20.f; ///< aller-retour : 20 mm à 100 mm/s, rampes de 50 ms à mount_accel
    constexpr float sweep_speed = 100.f;
    constexpr uint8_t sweeps = 2; ///< allers-retours par axe

    /// Écart moyen |a − pesanteur| de chaque axe au repos : retiré des sommes du mouvement (bruit)
    std::array<float, 3> rest_mad {};

    bool must_stop() {
        return stop_requested.load() || planner.draining() || !marlin_server::printer_idle();
    }

    /// Lit tous les échantillons disponibles ; add(a) pour chacun. false : l'accéléromètre ne répond plus (ou plus
    /// d'échantillon depuis sample_timeout_ms)
    template <class Add>
    bool drain(PrusaAccelerometer &accelerometer, uint32_t &last_sample, Add &&add) {
        accelerometer::Acceleration a;
        for (Read r; (r = read(accelerometer, a)) != Read::empty;) {
            if (r == Read::failed) {
                return false;
            }
            if (r == Read::sample) {
                add(a);
                last_sample = ticks_ms();
            }
        }
        return ticks_ms() - last_sample <= sample_timeout_ms;
    }

    /// Au repos pendant rest_ms : pesanteur (moyenne), bruit (écart type) et écart moyen par axe
    Status rest(PrusaAccelerometer &accelerometer) {
        double sum[3] = { 0, 0, 0 }, sq[3] = { 0, 0, 0 };
        uint32_t n = 0;
        // Deux passes : la moyenne d'abord (pesanteur), puis l'écart moyen autour d'elle
        std::array<float, 3> mean {};
        for (uint8_t pass = 0; pass < 2; ++pass) {
            double mad[3] = { 0, 0, 0 };
            uint32_t m = 0;
            accelerometer.clear();
            uint32_t last_sample = ticks_ms();
            for (const uint32_t start = ticks_ms(); ticks_ms() - start < rest_ms;) {
                if (must_stop()) {
                    return Status::stopped;
                }
                const bool alive = drain(accelerometer, last_sample, [&](const accelerometer::Acceleration &a) {
                    for (int i = 0; i < 3; ++i) {
                        if (pass == 0) {
                            sum[i] += a.val[i];
                            sq[i] += double(a.val[i]) * a.val[i];
                        } else {
                            mad[i] += std::abs(a.val[i] - mean[i]);
                        }
                    }
                    if (pass == 0) {
                        ++n;
                    } else {
                        ++m;
                    }
                });
                if (!alive) {
                    return Status::failed;
                }
                idle(true);
            }
            if (pass == 0) {
                if (n < 100) {
                    return Status::failed; // pas assez d'échantillons en 1 s (1 344 attendus)
                }
                double var = 0;
                for (int i = 0; i < 3; ++i) {
                    mean[i] = float(sum[i] / n);
                    var += sq[i] / n - double(mean[i]) * mean[i];
                }
                measurement.gravity = mean;
                measurement.noise_rms = float(std::sqrt(std::max(var, 0.0)));
            } else if (m > 0) {
                for (int i = 0; i < 3; ++i) {
                    rest_mad[i] = float(mad[i] / m);
                }
            }
        }
        measurement.tilt_deg = tilt_degrees(mean[0], mean[1], mean[2]);
        return Status::done;
    }

    /// Allers-retours sur l'axe a (0 X, 1 Y) : somme des |a − pesanteur| de chaque axe pendant le mouvement, bruit retiré
    /// (gardées dans le CSV : elles montrent comment l'accéléromètre est tourné, sans le juger)
    Status sweep(PrusaAccelerometer &accelerometer, uint8_t a) {
        const AxisEnum axis = a == 0 ? X_AXIS : Y_AXIS;
        const float centre = current_position[axis];
        std::array<double, 3> sums { 0, 0, 0 };
        uint32_t n = 0;
        for (uint8_t s = 0; s < 2 * sweeps; ++s) {
            if (must_stop()) {
                return Status::stopped;
            }
            planner.synchronize();
            accelerometer.clear();
            uint32_t last_sample = ticks_ms();
            current_position[axis] = centre + (s % 2 ? 0.f : sweep_mm);
            line_to_current_position(sweep_speed);
            const auto add = [&](const accelerometer::Acceleration &v) {
                for (int i = 0; i < 3; ++i) {
                    sums[i] += std::abs(v.val[i] - measurement.gravity[i]);
                }
                ++n;
            };
            while (planner.processing()) {
                if (!drain(accelerometer, last_sample, add)) {
                    planner.synchronize();
                    return Status::failed;
                }
                idle(true);
            }
            for (const uint32_t end = ticks_ms(); ticks_ms() - end < after_ms;) {
                if (!drain(accelerometer, last_sample, add)) {
                    return Status::failed;
                }
                idle(true);
            }
        }
        for (int i = 0; i < 3; ++i) {
            measurement.sums[a][i] = float(std::max(sums[i] - double(rest_mad[i]) * n, 0.0));
        }
        measurement.share[a] = plane_share(measurement.sums[a]);
        return Status::done;
    }

    void write_csv() {
        FILE *f = fopen(csv_path, "a");
        if (!f) {
            return; // pas de clé
        }
        const Measurement &m = measurement;
        force_probe::csv_printf(f, "# OneDrop3D M2005 accelerometer mount at X%.1f Y%.1f Z%.1f: %d mm moves at %d mm/s2\n", double(current_position.x),
            double(current_position.y), double(current_position.z), int(sweep_mm), int(mount_accel));
        force_probe::csv_printf(f, "gravity_x,gravity_y,gravity_z,tilt_deg,noise_rms,x_move_x,x_move_y,x_move_z,y_move_x,y_move_y,y_move_z,share_x,share_y,verdict\n");
        force_probe::csv_printf(f, "%.3f,%.3f,%.3f,%.2f,%.4f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%d\n", double(m.gravity[0]), double(m.gravity[1]),
            double(m.gravity[2]), double(m.tilt_deg), double(m.noise_rms), double(m.sums[0][0]), double(m.sums[0][1]), double(m.sums[0][2]),
            double(m.sums[1][0]), double(m.sums[1][1]), double(m.sums[1][2]), double(m.share[0]), double(m.share[1]), int(m.verdict));
        fclose(f);
    }

    Status measure() {
        if (const auto r = refused(stop_requested)) {
            return *r;
        }
        planner.synchronize();
        MotionSettings settings;
        if (!accel_limit::prepare()) {
            return must_stop() ? Status::stopped : Status::not_homed;
        }
        settings.accelerate(mount_accel, sweep_speed);

        PrusaAccelerometer accelerometer;
        live::set(N_("Head still: gravity and noise of the accelerometer"));
        for (const uint32_t start = ticks_ms(); ticks_ms() - start < settle_ms;) {
            idle(true);
        }
        if (const Status s = rest(accelerometer); s != Status::done) {
            return s;
        }
        progress = 30;
        for (uint8_t a = 0; a < 2; ++a) {
            live::set(N_("Axis %c: short moves, which accelerometer axis sees them"), 'X' + a);
            if (const Status s = sweep(accelerometer, a); s != Status::done) {
                return s;
            }
            progress = uint8_t(30 + 35 * (a + 1));
        }
        measurement.verdict = mount_verdict(measurement.tilt_deg, measurement.share[0], measurement.share[1]);
        write_csv();
        const auto whole = [](float v) { // NAN : -1 à l'écran plutôt qu'un nombre au hasard
            return std::isfinite(v) ? int(std::lround(v)) : -1;
        };
        live::set_value(N_("Tilt %d deg, X %d%%, Y %d%%"), whole(measurement.tilt_deg), whole(100.f * measurement.share[0]), whole(100.f * measurement.share[1]));
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
    rest_mad = {};
    progress = 0;
    last_status = Status::running;
    last_status = measure();
    progress = 100;
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

bool passed() {
    return status() == Status::done && measurement.verdict == Mount::ok;
}

} // namespace od3d::accel_mount

// ---- Vérification de l'input shaper (M2006) ----

namespace od3d::shaper_check {

using namespace vibration;

namespace {

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<Status> last_status = Status::idle;
    std::atomic<uint8_t> progress = 0;
    Measurement measurement;

    constexpr const char *csv_path = "/usb/OD3D_SHAPER.CSV";
    constexpr uint8_t moves = 4; ///< aller, retour, aller, retour (moyenne), comme M2004

    bool must_stop() {
        return stop_requested.load() || planner.draining() || !marlin_server::printer_idle();
    }

    /// Résidu moyen (variance) de moves déplacements courts de length mm sur axis autour de centre
    Status ringing(PrusaAccelerometer &accelerometer, float period, AxisEnum axis, float centre, float length, float speed, float &out) {
        float sum = 0.f;
        for (uint8_t m = 0; m < moves; ++m) {
            if (must_stop()) {
                return Status::stopped;
            }
            current_position[axis] = centre + (m % 2 ? 0.f : length);
            line_to_current_position(speed);
            float r;
            if (const Status s = accel_limit::residual(accelerometer, period, r); s != Status::done) {
                return s;
            }
            sum += r;
        }
        out = sum / moves;
        return Status::done;
    }

    /// Remet le filtre d'un axe. Mesure annulée (vidage du planificateur) : set_axis_config n'agit pas pendant
    /// l'annulation, alors la file vidée, la configuration enregistrée de l'input shaper (M1959) est remise telle quelle
    void put_back(AxisEnum axis, const std::optional<input_shaper::AxisConfig> &config) {
        planner.synchronize();
        if (!planner.draining()) {
            input_shaper::set_axis_config(axis, config);
            return;
        }
        while (planner.processing()) {
            idle(true);
        }
        input_shaper::init();
    }

    void write_csv() {
        FILE *f = fopen(csv_path, "a");
        if (!f) {
            return;
        }
        const Measurement &m = measurement;
        force_probe::csv_printf(f, "# OneDrop3D M2006 input shaper check at X%.1f Y%.1f Z%.1f: %d mm/s2, %d moves, noise rms %.5f m/s2\n",
            double(current_position.x), double(current_position.y), double(current_position.z), int(m.accel), int(moves), double(std::sqrt(m.noise)));
        force_probe::csv_printf(f, "axis,filter,ramp_ms,with_rms_m_s2,without_rms_m_s2,reduction,verdict\n");
        for (uint8_t a = 0; a < axis_count; ++a) {
            force_probe::csv_printf(f, "%c,%d,%.2f,%.5f,%.5f,%.3f,%d\n", 'X' + a, int(m.has_filter[a]), double(m.ramp[a] * 1000.f), double(std::sqrt(m.with_filter[a])),
                double(std::sqrt(m.without_filter[a])), double(m.reduction[a]), int(m.verdict[a]));
        }
        fclose(f);
    }

    Status measure() {
        if (const auto r = refused(stop_requested)) {
            return *r;
        }
        planner.synchronize();
        MotionSettings settings;
        if (!accel_limit::prepare()) {
            return must_stop() ? Status::stopped : Status::not_homed;
        }
        measurement.accel = settings.accelerate(check_accel, check_accel * ramp_s); // la vitesse de la rampe la plus longue

        PrusaAccelerometer accelerometer;
        const float period = sample_period(accelerometer);
        live::set(N_("Head still: accelerometer noise"));
        for (const uint32_t start = ticks_ms(); ticks_ms() - start < 500;) {
            idle(true);
        }
        accelerometer.clear();
        float sum = 0.f;
        for (uint8_t m = 0; m < moves; ++m) {
            float r;
            if (const Status s = accel_limit::residual(accelerometer, period, r); s != Status::done) {
                return s;
            }
            sum += r;
        }
        measurement.noise = sum / moves;
        progress = 10;

        for (uint8_t a = 0; a < axis_count; ++a) {
            const AxisEnum axis = a == 0 ? X_AXIS : Y_AXIS;
            const float centre = current_position[axis];
            const std::optional<input_shaper::AxisConfig> config = input_shaper::get_axis_config(axis);
            measurement.has_filter[a] = config.has_value();
            // Triangle de rampes de l'axe, comme un palier de M2004 (V-05) : une demi-période de la résonance que le
            // filtre doit retirer, la même avec et sans lui
            measurement.ramp[a] = ramp_for(config ? config->frequency : NAN);
            const float speed = measurement.accel * measurement.ramp[a];
            const float length = speed * measurement.ramp[a];
            live::set(N_("Axis %c: short moves with its filter"), 'X' + a);
            if (const Status s = ringing(accelerometer, period, axis, centre, length, speed, measurement.with_filter[a]); s != Status::done) {
                return s;
            }
            progress = uint8_t(10 + 45 * a + 20);
            if (config) {
                live::set(N_("Axis %c: same moves without its filter"), 'X' + a);
                planner.synchronize();
                input_shaper::set_axis_config(axis, std::nullopt);
                ScopeGuard restore = [&] { put_back(axis, config); }; // toujours remis, même arrêté ou en échec
                if (const Status s = ringing(accelerometer, period, axis, centre, length, speed, measurement.without_filter[a]); s != Status::done) {
                    return s;
                }
            }
            measurement.reduction[a] = config ? shaper_reduction(measurement.with_filter[a], measurement.without_filter[a], measurement.noise) : NAN;
            measurement.verdict[a] = vibration::shaper_check(measurement.has_filter[a], measurement.reduction[a]);
            if (std::isfinite(measurement.reduction[a])) {
                live::set_value(N_("Axis %c: the filter removes %d%% of the ringing"), 'X' + a, int(std::lround(100.f * measurement.reduction[a])));
            }
            progress = uint8_t(10 + 45 * (a + 1));
        }
        write_csv();
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
    progress = 100;
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

bool passed() {
    return status() == Status::done && shaper_check_passed(measurement.verdict);
}

} // namespace od3d::shaper_check
