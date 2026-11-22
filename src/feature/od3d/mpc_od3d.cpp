/// @file
#include "mpc_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>

namespace od3d::mpc {

namespace {

    bool within(float v, Range r) {
        return std::isfinite(v) && v >= r.min && v <= r.max;
    }

    bool all_finite(std::initializer_list<float> values) {
        return std::all_of(values.begin(), values.end(), [](float v) { return std::isfinite(v); });
    }

    constexpr float ambient_min = -20.0f; ///< bornes de l'ambiante du modèle (sécurité numérique seulement)
    constexpr float ambient_max = 150.0f;

} // namespace

bool is_valid(const Params &p) {
    return within(p.heater_power, heater_power_range)
        && within(p.block_heat_capacity, block_heat_capacity_range)
        && within(p.sensor_responsiveness, sensor_responsiveness_range)
        && within(p.ambient_xfer_coeff, ambient_xfer_coeff_range)
        && within(p.fan_xfer_coeff, fan_xfer_coeff_range)
        && within(p.filament_heat_capacity_permm, filament_heat_capacity_range);
}

float losses(const Params &p, float fan, float e_speed) {
    return p.ambient_xfer_coeff + std::clamp(fan, 0.0f, 1.0f) * p.fan_xfer_coeff + std::max(e_speed, 0.0f) * p.filament_heat_capacity_permm;
}

// ---- Régulateur ----

void Model::reset(float measured_temp) {
    block_temp_ = measured_temp;
    sensor_temp_ = measured_temp;
    ambient_temp_ = std::min(30.0f, measured_temp);
    initialized_ = std::isfinite(measured_temp);
}

float Model::step(const StepArgs &a) {
    if (!is_valid(params_) || !(a.pwm_max > 0)
        || !all_finite({ a.dt_s, a.measured_temp, a.target_temp, a.max_target_temp, a.fan, a.e_speed, a.applied_pwm, a.pwm_max, a.ff_e_speed })) {
        initialized_ = false;
        return 0;
    }

    // Premier pas, ou trou dans les mesures : on repart de la mesure, sans prédiction
    float dt = a.dt_s;
    if (!initialized_ || !(dt > 0) || dt > max_dt_s) {
        reset(a.measured_temp);
        dt = 0;
    }

    const Params &p = params_;
    const float u = std::clamp(a.applied_pwm / a.pwm_max, 0.0f, 1.0f);
    const float h = losses(p, a.fan, a.e_speed);

    // Prédiction : bloc chauffé par la cartouche, refroidi par l'air (et le filament) ; la sonde suit le bloc
    const float block_delta = (u * p.heater_power - h * (block_temp_ - ambient_temp_)) * dt / p.block_heat_capacity;
    block_temp_ += block_delta;
    sensor_temp_ += (block_temp_ - sensor_temp_) * std::min(1.0f, p.sensor_responsiveness * dt);

    // Recalage sur la mesure : l'écart est soit du bruit (qui se moyenne), soit une erreur lente du modèle
    const float correction = (a.measured_temp - sensor_temp_) * std::min(1.0f, dt / smoothing_time_s);
    block_temp_ += correction;
    sensor_temp_ += correction;

    // L'ambiante absorbe l'erreur lente (c'est l'intégrale du MPC), seulement près de l'équilibre : sortie non
    // saturée, ou bloc presque stable. Pas minimal de min_ambient_change K/s, comme Marlin.
    if (dt > 0 && ((u > 0 && u < 1) || std::fabs(block_delta + correction) < steady_state_rate * dt)) {
        const float min_step = min_ambient_change * dt;
        ambient_temp_ += correction > 0 ? std::max(correction, min_step) : std::min(correction, -min_step);
        ambient_temp_ = std::clamp(ambient_temp_, ambient_min, ambient_max);
    }

    float pwm = 0;
    // Consigne au-delà de la consigne maximale : ramenée à celle-ci ; mesure au-delà (+ marge) : on ne chauffe pas
    if (a.target_temp > 0 && a.measured_temp < a.max_target_temp + max_target_cutoff) {
        const float target = std::min(a.target_temp, a.max_target_temp);
        // F8 : les pertes compensées comptent le filament à venir (puissance anticipée), la prédiction ci-dessus celui
        // qui passe
        const float h_demand = a.ff_e_speed >= 0 ? losses(p, a.fan, a.ff_e_speed) : h;
        // Amener le bloc à la consigne en horizon_s, et compenser les pertes au niveau du bloc
        const float power = (target - block_temp_) * p.block_heat_capacity / horizon_s + (block_temp_ - ambient_temp_) * h_demand;
        pwm = power / p.heater_power * a.pwm_max;
    }

    if (!all_finite({ pwm, block_temp_, sensor_temp_, ambient_temp_ })) {
        initialized_ = false;
        return 0;
    }
    return std::clamp(pwm, 0.0f, a.pwm_max);
}

// ---- F8 : température selon le débit ----

float flow_boost_request(float flow_mm3s, float threshold_mm3s, float max_boost) {
    if (!all_finite({ flow_mm3s, threshold_mm3s, max_boost })) {
        return 0;
    }
    const float cap = std::clamp(max_boost, flow_max_boost_range.min, flow_max_boost_range.max);
    return std::clamp((flow_mm3s - threshold_mm3s) * flow_boost_gain, 0.0f, cap);
}

float boosted_target(float target, float boost, float max_target) {
    if (!(target > 0) || !std::isfinite(target) || !std::isfinite(max_target)) {
        return target;
    }
    const float b = std::isfinite(boost) ? std::clamp(boost, 0.0f, flow_max_boost_range.max) : 0.0f;
    // Jamais au-dessus de la consigne maximale (comme Model::step, qui y ramène toute consigne)
    return std::min(target + b, max_target);
}

float FlowBoost::step(float request, float dt_s) {
    const float r = std::isfinite(request) ? std::clamp(request, 0.0f, flow_max_boost_range.max) : 0.0f;
    const float dt = (std::isfinite(dt_s) && dt_s > 0) ? dt_s : 0.0f;
    value_ = r >= value_ ? r : std::max(r, value_ - flow_boost_fall_rate * dt);
    return value_;
}

// ---- Calibration ----

void Autotune::start(const TuneConfig &config, float time_s) {
    static constexpr Autotune fresh {}; // copiée : pas de temporaire de ~0,9 Ko sur la pile de Marlin pendant M306
    *this = fresh;
    config_ = config;
    config_.tune_temp = std::clamp(config_.tune_temp, min_tune_temp, max_tune_temp);
    enter(Phase::cooling, time_s);
    cooling_check_time_ = time_s;
    cooling_check_temp_ = NAN;
}

void Autotune::enter(Phase p, float time_s) {
    phase_ = p;
    phase_start_ = time_s;
    stable_since_ = -1;
    measure_start_ = -1;
    hold_pwm_integral_ = 0;
    hold_temp_integral_ = 0;
    hold_time_ = 0;
}

void Autotune::cancel() {
    fail(Error::cancelled);
}

void Autotune::fail(Error e) {
    if (finished() || phase_ == Phase::idle) {
        return;
    }
    phase_ = Phase::failed;
    error_ = e;
}

Command Autotune::step(const TuneInput &in) {
    constexpr Command off { Heater::off, 0, 0 };
    if (finished() || phase_ == Phase::idle) {
        return off;
    }
    if (!std::isfinite(in.temp) || !std::isfinite(in.time_s) || !std::isfinite(in.applied_pwm)) {
        fail(Error::interrupted);
        return off;
    }

    const float dt = has_last_ ? std::max(0.0f, in.time_s - last_time_) : 0.0f;
    last_time_ = in.time_s;
    has_last_ = true;
    const float elapsed = in.time_s - phase_start_;
    const float tune = config_.tune_temp;
    const auto fraction = [](float v) { return std::clamp(v, 0.0f, 1.0f); };

    // Au-delà de la température de calibration + marge, on arrête (les protections de Prusa restent en dessous)
    if ((phase_ == Phase::heating || phase_ == Phase::hold || phase_ == Phase::hold_fan) && in.temp > tune + overheat_margin) {
        fail(Error::overheat);
        return off;
    }

    switch (phase_) {

    case Phase::cooling: {
        if (std::isnan(cooling_check_temp_)) {
            cooling_check_temp_ = in.temp;
            cooling_start_temp_ = in.temp;
        }
        if (in.time_s - cooling_check_time_ >= cooling_check_s) {
            if (cooling_check_temp_ - in.temp < cooling_drop_k) {
                // Ne refroidit plus : c'est l'ambiante autour de la buse (moyenne des deux relevés, comme Marlin)
                ambient_ = (cooling_check_temp_ + in.temp) / 2;
                progress_ = 20;
                enter(Phase::spin_down, in.time_s);
                return off;
            }
            cooling_check_time_ = in.time_s;
            cooling_check_temp_ = in.temp;
        }
        if (elapsed > cooling_timeout_s) {
            fail(Error::cooling_timeout);
            return off;
        }
        const float span = cooling_start_temp_ - 40.0f;
        progress_ = uint8_t(20 * (span > 5 ? fraction((cooling_start_temp_ - in.temp) / span) : fraction(elapsed / 20.0f)));
        return { Heater::off, 0, 255 };
    }

    case Phase::spin_down:
        if (elapsed >= spin_down_s) {
            enter(Phase::heating, in.time_s);
            heating_start_temp_ = in.temp;
            samples_[0] = { 0, in.temp };
            sample_count_ = 1;
            sample_interval_ = 0.5f;
            heating_pwm_sum_ = 0;
            heating_time_sum_ = 0;
            return { Heater::full, tune, 0 };
        }
        return off;

    case Phase::heating: {
        heating_pwm_sum_ += in.applied_pwm * dt;
        heating_time_sum_ += dt;

        const auto add_sample = [&](float t) {
            if (sample_count_ == max_samples) {
                // Tampon plein : un échantillon sur deux, intervalle doublé (comme Marlin)
                for (std::size_t i = 0; i < max_samples / 2; ++i) {
                    samples_[i] = samples_[2 * i];
                }
                sample_count_ = max_samples / 2;
                sample_interval_ *= 2;
            }
            samples_[sample_count_++] = { t, in.temp };
        };

        if (in.temp >= tune) {
            add_sample(elapsed);
            const float avg = heating_time_sum_ > 0 ? heating_pwm_sum_ / heating_time_sum_ : config_.pwm_max;
            heating_power_ = config_.heater_power * std::clamp(avg / config_.pwm_max, 0.0f, 1.0f);
            progress_ = 50;
            enter(Phase::hold, in.time_s);
            return { Heater::regulate, tune, 0 };
        }
        if (elapsed >= sample_count_ * sample_interval_) {
            add_sample(elapsed);
        }
        if (elapsed > heating_timeout_s) {
            fail(Error::heating_timeout);
            return off;
        }
        progress_ = uint8_t(20 + 30 * fraction((in.temp - heating_start_temp_) / std::max(1.0f, tune - heating_start_temp_)));
        return { Heater::full, tune, 0 };
    }

    case Phase::hold:
    case Phase::hold_fan: {
        const bool fan = phase_ == Phase::hold_fan;
        float loss = 0;
        if (step_hold(in, dt, loss)) {
            if (!fan) {
                loss_fan0_ = loss;
                progress_ = 70;
                enter(Phase::hold_fan, in.time_s);
                return { Heater::regulate, tune, 255 };
            }
            loss_fan255_ = loss;
            compute();
            return off;
        }
        if (finished()) {
            return off;
        }
        const float f = measure_start_ < 0 ? 0.33f * fraction(stable_since_ < 0 ? 0 : (in.time_s - stable_since_) / hold_settle_s)
                                            : 0.33f + 0.67f * fraction(hold_time_ / hold_measure_s);
        progress_ = uint8_t(fan ? 70 + 25 * f : 50 + 20 * f);
        return { Heater::regulate, tune, uint8_t(fan ? 255 : 0) };
    }

    case Phase::idle:
    case Phase::done:
    case Phase::failed:
        break;
    }
    return off;
}

bool Autotune::step_hold(const TuneInput &in, float dt, float &loss_w_per_k) {
    if (measure_start_ < 0) {
        // Attendre que la température tienne dans la bande pendant hold_settle_s
        if (std::fabs(in.temp - config_.tune_temp) <= hold_band_k) {
            if (stable_since_ < 0) {
                stable_since_ = in.time_s;
            } else if (in.time_s - stable_since_ >= hold_settle_s) {
                measure_start_ = in.time_s;
            }
        } else {
            stable_since_ = -1;
        }
    } else {
        // Puissance et température moyennes (pondérées par le temps)
        hold_pwm_integral_ += in.applied_pwm * dt;
        hold_temp_integral_ += in.temp * dt;
        hold_time_ += dt;
        if (hold_time_ >= hold_measure_s) {
            const float avg_pwm = hold_pwm_integral_ / hold_time_;
            const float avg_temp = hold_temp_integral_ / hold_time_;
            const float delta = avg_temp - ambient_;
            loss_w_per_k = delta > 20 ? config_.heater_power * std::clamp(avg_pwm / config_.pwm_max, 0.0f, 1.0f) / delta : NAN;
            return true;
        }
    }
    if (in.time_s - phase_start_ > hold_timeout_s) {
        fail(Error::hold_timeout);
    }
    return false;
}

float Autotune::fit_error(float c, float r) const {
    // Simule la chauffe à la puissance mesurée, ventilateur arrêté, depuis l'ambiante ; somme des carrés des écarts
    constexpr float sim_dt = 0.1f;
    const float a = loss_fan0_;
    float block = samples_[0].temp;
    float sensor = samples_[0].temp;
    float t = 0;
    float sse = 0;
    for (std::size_t i = 1; i < sample_count_; ++i) {
        while (t < samples_[i].t) {
            const float h = std::min(sim_dt, samples_[i].t - t);
            block += (heating_power_ - a * (block - ambient_)) * h / c;
            sensor += (block - sensor) * (1.0f - std::exp(-r * h));
            t += h;
        }
        const float e = sensor - samples_[i].temp;
        sse += e * e;
    }
    return sse;
}

namespace {

    /// Minimum d'une fonction unimodale sur [lo, hi] (section dorée)
    template <class F>
    float golden_min(F &&f, float lo, float hi, int iterations) {
        constexpr float g = 0.6180340f;
        float x1 = hi - g * (hi - lo);
        float x2 = lo + g * (hi - lo);
        float f1 = f(x1);
        float f2 = f(x2);
        for (int i = 0; i < iterations; ++i) {
            if (f1 < f2) {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - g * (hi - lo);
                f1 = f(x1);
            } else {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + g * (hi - lo);
                f2 = f(x2);
            }
        }
        return (lo + hi) / 2;
    }

} // namespace

void Autotune::compute() {
    progress_ = 95;
    const float a = loss_fan0_;
    const float f = std::max(0.0f, loss_fan255_ - loss_fan0_);
    if (!(a > 0) || !std::isfinite(loss_fan255_) || sample_count_ < 8 || !(heating_power_ > 0)) {
        fail(Error::bad_result);
        return;
    }

    // Première estimation de C par le bilan d'énergie sur la chauffe, à partir du premier quart de la montée
    // (le retard de la sonde y est déjà établi) : C × ΔT = P × Δt − A × ∫(T − T_amb) dt
    const float threshold = ambient_ + 0.25f * (config_.tune_temp - ambient_);
    std::size_t first = 0;
    while (first + 1 < sample_count_ && samples_[first].temp < threshold) {
        ++first;
    }
    const std::size_t last = sample_count_ - 1;
    float integral = 0;
    for (std::size_t i = first; i < last; ++i) {
        integral += ((samples_[i].temp + samples_[i + 1].temp) / 2 - ambient_) * (samples_[i + 1].t - samples_[i].t);
    }
    const float rise = samples_[last].temp - samples_[first].temp;
    const float c0 = rise > 10 ? (heating_power_ * (samples_[last].t - samples_[first].t) - a * integral) / rise : NAN;
    if (!(c0 > 0) || !std::isfinite(c0)) {
        fail(Error::bad_result);
        return;
    }

    // Puis C et R ajustés tour à tour pour que le modèle simulé suive la courbe mesurée
    float c = c0;
    float r = 0.3f;
    for (int pass = 0; pass < 3; ++pass) {
        const float log_r = golden_min([&](float x) { return fit_error(c, std::exp(x)); },
            std::log(sensor_responsiveness_range.min), std::log(sensor_responsiveness_range.max), 30);
        r = std::exp(log_r);
        c = golden_min([&](float x) { return fit_error(x, r); }, 0.5f * c0, 2.0f * c0, 30);
    }

    result_ = Params {
        .heater_power = config_.heater_power,
        .block_heat_capacity = c,
        .sensor_responsiveness = r,
        .ambient_xfer_coeff = a,
        .fan_xfer_coeff = f,
        .filament_heat_capacity_permm = config_.filament_heat_capacity_permm,
    };
    if (!is_valid(result_)) {
        fail(Error::bad_result);
        return;
    }
    progress_ = 100;
    phase_ = Phase::done;
}

} // namespace od3d::mpc
