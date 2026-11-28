/// @file
#include "speed_map_od3d.hpp"
#include "accelerometer_od3d.hpp"
#include "live_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#include <common/marlin_server.hpp>
#include <config_store/store_instance.hpp>
#include <i18n.h>
#include <Marlin/src/gcode/calibrate/M958.hpp> // maybe_calibrate_and_get_accelerometer_sample_period
#include <Marlin/src/gcode/gcode.h>
#include <Marlin/src/Marlin.h>
#include <Marlin/src/module/motion.h>
#include <Marlin/src/module/planner.h>
#include <Marlin/src/module/temperature.h>
#include <raii/scope_guard.hpp>

namespace od3d::speed_map {

namespace {

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<Status> last_status = Status::idle;
    std::atomic<uint8_t> axis_now = 0;
    std::atomic<uint8_t> progress = 0;
    std::atomic<uint16_t> runs = 0;
    std::atomic<uint16_t> revisions = 0;

    Map map; ///< ~240 octets en mémoire statique, lue par l'interface

    constexpr float accel = 4000.f; ///< mm/s² pendant la mesure (machine : 7 000 en X/Y)
    constexpr float cruise_s = 0.4f; ///< durée mesurée à vitesse constante par passage
    constexpr float margin_mm = 5.f; ///< ignorés après l'accélération et avant le freinage
    constexpr float delay_s = 0.03f; ///< retard possible des échantillons de l'accéléromètre (sa file)
    constexpr float filter_tau = 0.01f; ///< passe-haut à ~16 Hz : pesanteur et mouvement retirés
    constexpr uint32_t min_samples = 100; ///< en dessous, la vitesse n'est pas mesurée (NAN)

    constexpr float bound_xy = 100.f; ///< course de part et d'autre du centre au plus
    constexpr float edge_margin = 10.f; ///< mm gardés aux bords de la course
    constexpr float min_z = 10.f;
    constexpr float above_print = 5.f;
    constexpr float travel_speed_xy = 100.f;
    constexpr float travel_speed_z = 10.f;

    bool must_stop() {
        return stop_requested.load() || planner.draining() || !marlin_server::printer_idle();
    }

    struct Geometry {
        xy_float_t centre;
        std::array<float, axis_count> bound; ///< mm de part et d'autre du centre
    };

    /// Hauteur sûre, centre du plateau borné par les butées logicielles ; not_homed sans origine
    Status prepare(Geometry &g) {
        // Jamais d'origine ici (retour du 23/11 : le plateau montait palper, accéléromètre fixé) : l'écran ou
        // l'assistant la prend avant de demander de le fixer (M1989 H) ; perdue depuis : refusé, sans bouger
        if (must_stop()) {
            return Status::stopped;
        }
        if (axes_need_homing()) {
            return Status::not_homed;
        }
        const float z_floor = std::min(std::max(min_z, planner.max_printed_z + above_print), soft_endstop.max.z - 1.f);
        if (current_position.z < z_floor) {
            do_blocking_move_to_z(z_floor, travel_speed_z);
        }
        if (!accel::take_head()) { // INDX : la tête dont l'accéléromètre mesure, prise Z levé ; rien ailleurs
            return must_stop() ? Status::stopped : Status::failed;
        }
        g.centre.x = std::clamp(X_BED_SIZE / 2.f, soft_endstop.min.x, soft_endstop.max.x);
        g.centre.y = std::clamp(Y_BED_SIZE / 2.f, soft_endstop.min.y, soft_endstop.max.y);
        g.bound[0] = std::min(bound_xy, std::min(g.centre.x - soft_endstop.min.x, soft_endstop.max.x - g.centre.x) - edge_margin);
        g.bound[1] = std::min(bound_xy, std::min(g.centre.y - soft_endstop.min.y, soft_endstop.max.y - g.centre.y) - edge_margin);
        if (g.bound[0] < 20.f || g.bound[1] < 20.f) {
            return Status::no_room;
        }
        // IC-1 : INDX, le chemin du parcage de Prusa
        return accel::move_to(g.centre.x, g.centre.y, travel_speed_xy) && !must_stop() ? Status::done : Status::stopped;
    }

    /// Un passage à la vitesse v jusqu'à target (coordonnée logique de l'axe) ; échantillons comptés dans la partie à
    /// vitesse constante, repérée par la position réelle des moteurs. Status::done, stopped ou failed.
    Status pass(AxisEnum axis, float target, float v, const Pass &p, PrusaAccelerometer &accelerometer, VibrationMeter &meter) {
        planner.synchronize();
        const float start = planner.get_axis_position_mm(axis);
        accelerometer.clear();
        current_position[axis] = target;
        line_to_current_position(v);

        bool stop = false;
        while (true) {
            accelerometer::RawAcceleration raw;
            switch (accelerometer.get_sample_printer_coords(raw)) {
            case PrusaAccelerometer::GetSampleResult::ok: {
                const float d = std::fabs(planner.get_axis_position_mm(axis) - start);
                const accelerometer::Acceleration a = raw.to_acceleration();
                meter.add(a.val[0], a.val[1], a.val[2], in_cruise(p, d));
                continue;
            }
            case PrusaAccelerometer::GetSampleResult::error:
                if (accelerometer.get_error() == accelerometer::Error::communication) {
                    planner.synchronize();
                    return Status::failed;
                }
                accelerometer.clear(); // échantillons perdus (file pleine) : la mesure reste valable
                break;
            case PrusaAccelerometer::GetSampleResult::buffer_empty:
                break;
            }
            if (!planner.processing()) {
                break;
            }
            stop = stop || must_stop(); // le passage en cours finit (1 s au plus), rien d'autre n'est lancé
            idle(true);
        }
        return stop || must_stop() ? Status::stopped : Status::done;
    }

    Status measure_axis(uint8_t axis_index, const Geometry &g, float sample_period, float max_speed, PrusaAccelerometer &accelerometer) {
        const AxisEnum axis = axis_index == 0 ? X_AXIS : Y_AXIS;
        const float centre = axis_index == 0 ? g.centre.x : g.centre.y;
        const float max_length = 2.f * g.bound[axis_index];

        // Tous les passages partent du même bout (retour du 22/11 : chaque passage était centré, donc plus long que le
        // précédent, et la tête s'arrêtait au bout gauche puis repartait un peu plus à gauche vers le départ suivant).
        // Les longueurs croissent avec la vitesse : le passage de max_speed borne tous les autres.
        const float from = sweep_from(centre, pass_for(std::min(max_speed, speed_end), accel, cruise_s, margin_mm, delay_s, max_length).length);
        do_blocking_move_to_xy(axis_index == 0 ? from : g.centre.x, axis_index == 0 ? g.centre.y : from, travel_speed_xy);
        if (must_stop()) {
            return Status::stopped;
        }

        for (size_t k = 0; k < speed_count; ++k) {
            const float v = speed_start + float(k) * speed_step;
            if (v > max_speed + 0.01f) {
                break; // au-delà de la vitesse maximale de la machine : pas mesuré
            }
            const Pass p = pass_for(v, accel, cruise_s, margin_mm, delay_s, max_length);

            VibrationMeter meter(sample_period, filter_tau);
            for (const float target : { from + p.length, from }) {
                if (const Status s = pass(axis, target, v, p, accelerometer, meter); s != Status::done) {
                    return s;
                }
            }
            map.rms[axis_index][k] = meter.count() >= min_samples ? std::sqrt(meter.mean_square()) : NAN;
            map.points[axis_index] = uint8_t(k + 1);
            progress = uint8_t(std::min<size_t>(100, (axis_index * speed_count + k + 1) * 100 / (axis_count * speed_count)));
            ++revisions;
        }
        return Status::done;
    }

    Status measure_map() {
        if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
            return Status::printing;
        }
        if (stop_requested) {
            return Status::stopped;
        }
        if (accel::nozzle_hot(max_nozzle_temperature)) {
            return Status::hot_nozzle;
        }
        if (!accel::head_available()) {
            return Status::no_dock;
        }
        if (!accel::ready()) { // ACC-1 : avant tout mouvement (et avant de changer les réglages de mouvement)
            return Status::no_accelerometer;
        }

        // IC-6 : la tête prise et le trajet au centre avec les réglages de Prusa (INDX : sortie du dock à sa vitesse),
        // avant ceux de la mesure
        Geometry g;
        if (const Status s = prepare(g); s != Status::done) {
            return s;
        }

        // Réglages de mouvement : mode furtif coupé (ses limites fausseraient les vitesses), vitesses et accélérations
        // de la mesure ; tout est remis à la fin, même sur un arrêt de Marlin (comme M1991)
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
        {
            auto s = planner.user_settings;
            for (const AxisEnum axis : { X_AXIS, Y_AXIS }) {
                s.max_feedrate_mm_s[axis] = std::min<float>(s.max_feedrate_mm_s[axis], speed_end);
                s.max_acceleration_mm_per_s2[axis] = std::min<uint32_t>(s.max_acceleration_mm_per_s2[axis], uint32_t(accel));
            }
            s.travel_acceleration = std::min(s.travel_acceleration, accel);
            s.acceleration = std::min(s.acceleration, accel);
            planner.apply_settings(s); // les limites matérielles de Prusa s'appliquent encore
        }

        PrusaAccelerometer accelerometer;
        const auto period = vibrate_measure::maybe_calibrate_and_get_accelerometer_sample_period(accelerometer, false, [](float) -> vibrate_measure::Result<void> { return {}; });
        const float sample_period = period.has_value() ? period.value() : 1.f / 1344.f;

        for (uint8_t a = 0; a < axis_count; ++a) {
            axis_now = a;
            live::set(N_("Axis %c: vibrations measured at each speed"), a == 0 ? 'X' : 'Y');
            const float max_speed = planner.settings.max_feedrate_mm_s[a == 0 ? X_AXIS : Y_AXIS];
            if (const Status s = measure_axis(a, g, sample_period, max_speed, accelerometer); s != Status::done) {
                return s;
            }
        }
        // Retour au centre
        do_blocking_move_to_xy(g.centre.x, g.centre.y, travel_speed_xy);
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
        running = false;
    };

    map = {};
    for (auto &axis : map.rms) {
        axis.fill(NAN);
    }
    progress = 0;
    axis_now = 0;
    last_status = Status::running;
    ++revisions;

    last_status = measure_map();
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

uint8_t progress_percent() {
    return progress.load();
}

uint16_t run_count() {
    return runs.load();
}

uint16_t revision() {
    return revisions.load();
}

const Map &last() {
    return map;
}

size_t critical(const Map &m, uint8_t axis, std::span<uint8_t> out) {
    if (axis >= axis_count) {
        return 0;
    }
    const size_t n = std::min<size_t>(m.points[axis], speed_count);
    return find_critical(std::span<const float>(m.rms[axis].data(), n), critical_factor, 0.f, out);
}

} // namespace od3d::speed_map
