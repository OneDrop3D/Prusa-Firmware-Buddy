/// @file
#include "mpc_control_od3d.hpp"
#include "lookahead_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <numbers>

#include <common/marlin_server.hpp>
#include <config_store/store_instance.hpp>
#include <inc/MarlinConfig.h>
#include <Marlin.h>
#include <module/planner.h>
#include <module/stepper.h>
#include <module/temperature.h>
#include <raii/scope_guard.hpp>
#include <timing.h>
#include <tool/hotend/hotend.hpp>

#if ENABLED(OD3D_MPCTEMP)

static_assert(PID_MAX == 255, "sortie du MPC et du PID sur la même échelle");
// Avec ces options, Prusa compare la sortie à son propre modèle : le MPC ne les a pas, pour ne rien affaiblir
static_assert(DISABLED(MODEL_DETECT_STUCK_THERMISTOR), "MPC OneDrop3D : protection par modèle de Prusa non prise en charge");
static_assert(DISABLED(MODEL_BASED_HOTEND_REGULATOR), "MPC OneDrop3D : seulement à côté du StandardHotendRegulator");
// F8 : la hausse de consigne ne dépasse jamais l'hystérésis de l'emballement thermique de Prusa
static_assert(ENABLED(THERMAL_PROTECTION_HOTENDS) && od3d::mpc::flow_max_boost_range.max <= THERMAL_PROTECTION_HYSTERESIS,
    "F8 : hausse maximale au plus THERMAL_PROTECTION_HYSTERESIS");
static_assert(od3d::mpc::flow_max_boost_range.max <= HEATER_MAXTEMP_SAFETY_MARGIN, "F8 : bornée de toute façon sous la consigne maximale");

namespace od3d::mpc {

namespace {

    /// Ce que la calibration impose à la régulation
    enum class Override : uint8_t {
        none,
        heater_off,
        heater_full,
        force_pid, ///< paliers de la calibration : le PID de Prusa, même si le MPC est choisi
    };

    // Fil de Marlin
    Params active_params {};
    Autotune autotune; ///< ~0,9 Ko en mémoire statique plutôt que sur la pile de Marlin

    // Lus depuis l'interface
    std::atomic<Override> tune_override = Override::none;
    std::atomic<uint32_t> params_version = 1;
    std::atomic<bool> calibration_valid = false;
    std::atomic<bool> mpc_selected = false;
    std::atomic<bool> cancel_requested = false;
    std::atomic<bool> tuning = false;
    std::atomic<Phase> tune_phase = Phase::idle;
    std::atomic<Error> tune_error = Error::none;
    std::atomic<uint8_t> tune_progress = 0;
    std::atomic<uint16_t> tune_runs = 0;

    // F8 : réglages (écrits par l'interface, lus par la régulation) et hausse en cours (pour l'affichage)
    std::atomic<bool> flow_temp_on = false;
    std::atomic<uint8_t> flow_threshold = default_flow_threshold;
    std::atomic<uint8_t> flow_max_boost = default_flow_max_boost;
    std::atomic<uint8_t> current_boost = 0;

    /// Section du filament [mm²] : vitesse du filament → débit volumique (comme PID_EXTRUSION_SCALING de Prusa)
    constexpr float filament_area = std::numbers::pi_v<float> * (DEFAULT_NOMINAL_FILAMENT_DIA / 2) * (DEFAULT_NOMINAL_FILAMENT_DIA / 2);

    uint8_t clamp_setting(uint8_t v, Range r) {
        return static_cast<uint8_t>(std::clamp<float>(v, r.min, r.max));
    }

    bool in_range(float v, Range r) {
        return std::isfinite(v) && v >= r.min && v <= r.max;
    }

    void echo(const char *text) {
        SERIAL_ECHO_START();
        SERIAL_ECHOLN(text);
    }

} // namespace

void report() {
    const Params &p = active_params;
    char buf[160];
    snprintf(buf, sizeof(buf), "M306 P%.2f C%.3f R%.4f A%.5f F%.5f H%.5f ; %s, %s",
        double(p.heater_power), double(p.block_heat_capacity), double(p.sensor_responsiveness),
        double(p.ambient_xfer_coeff), double(p.fan_xfer_coeff), double(p.filament_heat_capacity_permm),
        calibration_valid ? "calibrated" : "not calibrated", mpc_selected ? "MPC" : "PID");
    echo(buf);
    // F8
    snprintf(buf, sizeof(buf), "Flow temperature: %s, +%.1f C per mm3/s above %u mm3/s, max +%u C, now +%u C",
        flow_temp_on ? "on (MPC only)" : "off", double(flow_boost_gain), unsigned(flow_threshold.load()),
        unsigned(flow_max_boost.load()), unsigned(current_boost.load()));
    echo(buf);
}

namespace {

    const char *error_text(Error e) {
        switch (e) {
        case Error::none:
            return "ok";
        case Error::cancelled:
            return "cancelled";
        case Error::interrupted:
            return "interrupted (nozzle target changed)";
        case Error::cooling_timeout:
            return "nozzle did not settle while cooling";
        case Error::heating_timeout:
            return "calibration temperature not reached";
        case Error::overheat:
            return "overheat";
        case Error::hold_timeout:
            return "temperature not stable";
        case Error::bad_result:
            return "inconsistent measurements";
        case Error::refused:
            return "not while printing";
        }
        return "?";
    }

} // namespace

// ---- Régulation ----

HotendHook::Result HotendHook::step(const Args &args) {
    const uint32_t now = ticks_ms();
    const float dt = has_last_ ? float(now - last_ms_) / 1000.0f : 0.0f;

    // Vitesse du filament, comme le PID_EXTRUSION_SCALING de Prusa (position du moteur E) ; les reculs, les
    // chargements et les sauts de position ne comptent pas
    float e_speed = 0;
    const int32_t e_steps = stepper.position(E_AXIS);
    if (has_last_ && dt > 0 && args.is_current_tool) {
        e_speed = float(e_steps - last_e_steps_) * planner.mm_per_step[E_AXIS] / dt;
        if (!(e_speed >= 0 && e_speed <= max_melt_e_speed)) {
            e_speed = 0;
        }
    }
    last_e_steps_ = e_steps;
    last_ms_ = now;
    has_last_ = true;

    const bool valid = calibration_valid.load();
    if (const uint32_t version = params_version.load(); version != params_version_) {
        params_version_ = version;
        model_.set_params(active_params);
        model_.reset(args.current_temp);
    }

    const Override override_now = tune_override.load();
    const bool mpc_regulates = override_now == Override::none && valid && mpc_selected.load();

    // F8 : seulement quand le MPC régule la buse active avec une consigne posée. Le débit à venir sert à la
    // puissance anticipée (réglage mis) ; la hausse suit la demande, puis redescend en rampe (aussi après l'arrêt du
    // réglage). Dans tous les autres cas : aucune hausse, tout de suite. La consigne de Prusa ne change pas.
    float target = float(args.target_temp);
    float ff_e_speed = -1.0f; // comme F7
    if (mpc_regulates && args.is_current_tool && args.target_temp > 0 && (flow_temp_on.load() || boost_.value() > 0)) {
        float request = 0;
        if (flow_temp_on.load()) {
            ff_e_speed = lookahead::planner_e_speed_ahead(max_melt_e_speed);
            request = flow_boost_request(ff_e_speed * filament_area, flow_threshold.load(), flow_max_boost.load());
        }
        boost_.step(request, dt);
        target = boosted_target(target, boost_.value(), float(args.max_target_temp));
    } else {
        boost_.reset();
    }
    current_boost = static_cast<uint8_t>(std::lround(target > args.target_temp ? target - args.target_temp : 0.0f));

    // Le modèle suit la buse même quand le PID régule (sortie appliquée), pour une bascule sans à-coup
    float mpc_pwm = 0;
    if (valid) {
        mpc_pwm = model_.step(StepArgs {
            .dt_s = dt,
            .measured_temp = args.current_temp,
            .target_temp = target,
            .max_target_temp = float(args.max_target_temp),
            .fan = args.fan_speed / 255.0f,
            .e_speed = e_speed,
            .applied_pwm = float(args.applied_pwm),
            .pwm_max = float(PID_MAX),
            .ff_e_speed = ff_e_speed,
        });
    }

    Source source = Source::pid;
    float pwm = 0;
    switch (override_now) {
    case Override::heater_off:
        source = Source::tune;
        break;
    case Override::heater_full:
        source = Source::tune;
        // Pleine puissance seulement avec une consigne posée et sous la consigne maximale
        pwm = (args.target_temp > 0 && args.current_temp < args.max_target_temp) ? float(PID_MAX) : 0.0f;
        break;
    case Override::force_pid:
        break;
    case Override::none:
        if (mpc_regulates) {
            source = Source::mpc;
            pwm = mpc_pwm;
        }
        break;
    }

    const bool reset_pid = source == Source::pid && last_source_ != Source::pid;
    last_source_ = source;
    return Result {
        .handled = source != Source::pid,
        .pwm = std::clamp(pwm, 0.0f, float(PID_MAX)),
        .reset_pid = reset_pid,
    };
}

// ---- Réglages ----

void load() {
    active_params = config_store().od3d_mpc_params.get();
    const bool valid = is_valid(active_params);
    calibration_valid = valid;
    mpc_selected = valid && config_store().od3d_mpc_enabled.get();
    ++params_version;
    flow_temp_on = config_store().od3d_flow_temp_enabled.get();
    flow_threshold = clamp_setting(config_store().od3d_flow_temp_threshold.get(), flow_threshold_range);
    flow_max_boost = clamp_setting(config_store().od3d_flow_temp_max_boost.get(), flow_max_boost_range);
}

// ---- F8 ----

bool flow_temp_enabled() {
    return flow_temp_on.load();
}

void set_flow_temp_enabled(bool enabled) {
    flow_temp_on = enabled;
    config_store().od3d_flow_temp_enabled.set(enabled);
}

uint8_t flow_temp_threshold() {
    return flow_threshold.load();
}

void set_flow_temp_threshold(uint8_t mm3s) {
    const uint8_t v = clamp_setting(mm3s, flow_threshold_range);
    flow_threshold = v;
    config_store().od3d_flow_temp_threshold.set(v);
}

uint8_t flow_temp_max_boost() {
    return flow_max_boost.load();
}

void set_flow_temp_max_boost(uint8_t celsius) {
    const uint8_t v = clamp_setting(celsius, flow_max_boost_range);
    flow_max_boost = v;
    config_store().od3d_flow_temp_max_boost.set(v);
}

uint8_t flow_temp_current_boost() {
    return current_boost.load();
}

bool has_valid_calibration() {
    return calibration_valid.load();
}

bool is_mpc_selected() {
    return mpc_selected.load();
}

ModeResult set_mode(bool mpc, bool printing) {
    if (printing) {
        return ModeResult::printing;
    }
    if (tuning.load()) {
        return ModeResult::tuning;
    }
    if (mpc && !calibration_valid.load()) {
        return ModeResult::not_calibrated;
    }
    mpc_selected = mpc;
    config_store().od3d_mpc_enabled.set(mpc);
    return ModeResult::ok;
}

void set_params(const Params &p) {
    active_params = p;
    const bool valid = is_valid(p);
    calibration_valid = valid;
    if (!valid && mpc_selected.load()) {
        mpc_selected = false;
        config_store().od3d_mpc_enabled.set(false);
    }
    config_store().od3d_mpc_params.set(p);
    ++params_version;
}

const Params &params() {
    return active_params;
}

// ---- Calibration ----

void request_cancel() {
    cancel_requested = true;
}

TuneStatus tune_status() {
    return TuneStatus {
        .phase = tune_phase.load(),
        .error = tune_error.load(),
        .progress = tune_progress.load(),
        .running = tuning.load(),
        .runs = tune_runs.load(),
    };
}

void run_autotune(float tune_temp) {
    if (tuning.load()) {
        return;
    }
    // « Annuler » pressé avant le début compte pour cette calibration ; il est oublié ensuite
    ScopeGuard count_run = [] {
        cancel_requested = false;
        ++tune_runs;
    };
    if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
        echo("?MPC calibration: not while printing");
        tune_phase = Phase::failed;
        tune_error = Error::refused;
        return;
    }

    static_assert(PhysicalToolIndex::count == 1);
    Hotend &hotend = Hotend::for_tool(PhysicalToolIndex::from_raw(0));
    // Sous la consigne maximale, avec la marge du dépassement après la coupure de la pleine puissance
    const float max_target = float(hotend.config().max_nozzle_temp - HEATER_MAXTEMP_SAFETY_MARGIN);
    const Params &p = active_params;
    const TuneConfig config {
        .tune_temp = std::min(std::isfinite(tune_temp) ? tune_temp : 200.0f, max_target - overheat_margin),
        .heater_power = in_range(p.heater_power, heater_power_range) ? p.heater_power : default_heater_power,
        .filament_heat_capacity_permm = in_range(p.filament_heat_capacity_permm, filament_heat_capacity_range) ? p.filament_heat_capacity_permm : default_filament_heat_capacity,
        .pwm_max = float(PID_MAX),
    };

    const uint8_t fan_before = thermalManager.get_print_fan_speed();
    tuning = true;
    tune_error = Error::none;
    tune_progress = 0;

    // Quelle que soit la sortie : régulation normale, chauffe coupée, ventilateur comme avant
    ScopeGuard restore = [&] {
        tune_override = Override::none;
        hotend.set_nozzle_target_temp(0);
        thermalManager.set_print_fan_speed(fan_before);
        tuning = false;
    };

    char buf[96];
    snprintf(buf, sizeof(buf), "MPC calibration at %.0f C: cooling, heating, then holding (fan off, fan 100%%)", double(config.tune_temp));
    echo(buf);

    const uint32_t start_ms = ticks_ms();
    autotune.start(config, 0);
    tune_phase = autotune.phase();
    int16_t commanded_target = 0;
    bool target_owned = false;
    uint32_t last_step_ms = start_ms - 1000;

    while (!autotune.finished()) {
        idle(true);

        if (cancel_requested.load() || planner.draining() || !marlin_server::printer_idle()) {
            autotune.cancel();
        } else if (target_owned && hotend.nozzle_target_temp() != commanded_target) {
            // Quelqu'un d'autre a changé la consigne (minuterie de sécurité, M104…) : on s'arrête
            autotune.fail(Error::interrupted);
        }

        const uint32_t now = ticks_ms();
        if (!autotune.finished() && now - last_step_ms < 100) {
            continue;
        }
        last_step_ms = now;

        const Command c = autotune.step(TuneInput {
            .time_s = float(now - start_ms) / 1000.0f,
            .temp = hotend.nozzle_temp().value_or(NAN),
            .applied_pwm = float(hotend.nozzle_heater_pwm().value),
        });

        if (!autotune.finished()) {
            switch (c.heater) {
            case Heater::off:
                tune_override = Override::heater_off;
                break;
            case Heater::full:
                tune_override = Override::heater_full;
                break;
            case Heater::regulate:
                tune_override = Override::force_pid;
                break;
            }
            const int16_t target = int16_t(std::lround(c.target));
            if (hotend.nozzle_target_temp() != target) {
                hotend.set_nozzle_target_temp(target);
            }
            commanded_target = hotend.nozzle_target_temp();
            target_owned = true;
            thermalManager.set_print_fan_speed(c.fan);
        }

        tune_phase = autotune.phase();
        tune_progress = autotune.progress();
    }

    tune_error = autotune.error();
    tune_phase = autotune.phase();
    tune_progress = autotune.progress();

    if (autotune.phase() == Phase::done) {
        set_params(autotune.result());
        snprintf(buf, sizeof(buf), "MPC calibration done (ambient %.1f C)", double(autotune.ambient_temp()));
        echo(buf);
        report();
    } else {
        snprintf(buf, sizeof(buf), "?MPC calibration failed: %s", error_text(autotune.error()));
        echo(buf);
    }
}

} // namespace od3d::mpc

#endif
