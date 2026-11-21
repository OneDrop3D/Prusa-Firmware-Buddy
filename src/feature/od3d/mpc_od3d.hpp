/// @file
/// OneDrop3D (F7) : régulation MPC de la buse (expérimentale), d'après le MPC de Marlin 2.1 (MPCTEMP, M306).
///
/// Partie pure (sans Marlin, testée en unitaire) : le modèle thermique et son régulateur, et la calibration
/// automatique (machine à états nourrie par la température mesurée). Le branchement dans le firmware est dans
/// mpc_hotend_od3d.* (LocalHotend) et M306_od3d.cpp.
///
/// Modèle (Marlin, « Model Predictive Temperature Control ») : un bloc chauffant de capacité thermique C, chauffé
/// par la cartouche (P × sortie), qui perd vers l'air ambiant h × (T_bloc − T_amb), avec
/// h = A + F × ventilateur + H × vitesse du filament ; la sonde suit le bloc avec un retard du premier ordre (R).
/// À chaque pas : on prédit bloc et sonde, on recale les deux sur la mesure (lissage), on ajuste lentement la
/// température ambiante du modèle (c'est l'« intégrale » du MPC), puis on demande la puissance qui amène le bloc
/// à la consigne en horizon_s secondes en compensant les pertes. La sortie est bornée à 0…pwm_max.
///
/// Aucune protection thermique n'est ici : elles restent celles de Prusa (BaseHotend::manage, LocalHotend), qui
/// s'appliquent de la même façon quel que soit le régulateur. Ce module refuse seulement, en plus, de chauffer
/// au-delà de la consigne maximale de la buse.
#pragma once

#include <array>
#include <cstdint>

namespace od3d::mpc {

/// Puissance de la cartouche de la buse du Nextruder [W] (40 W sous 24 V). Elle ne fait que fixer l'unité : une
/// erreur sur P se retrouve dans C, A, F et H, calibrés avec la même P, et la sortie (P × sortie) ne change pas.
inline constexpr float default_heater_power = 40.0f;

/// Capacité thermique du filament par mm [J/K/mm] : valeur de Marlin (PLA de 1,75 mm)
inline constexpr float default_filament_heat_capacity = 0.0056f;

/// Paramètres du modèle : mêmes grandeurs et mêmes lettres que M306 de Marlin. Enregistrés dans config_store.
struct Params {
    float heater_power = default_heater_power; ///< P [W] puissance de la cartouche à pwm_max
    float block_heat_capacity = 0; ///< C [J/K] bloc chauffant et buse
    float sensor_responsiveness = 0; ///< R [1/s] vitesse à laquelle la sonde suit le bloc
    float ambient_xfer_coeff = 0; ///< A [W/K] pertes vers l'air, ventilateur arrêté
    float fan_xfer_coeff = 0; ///< F [W/K] pertes en plus, ventilateur pièce à 100 %
    float filament_heat_capacity_permm = default_filament_heat_capacity; ///< H [J/K/mm] chaleur emportée par le filament

    bool operator==(const Params &) const = default;
};
static_assert(sizeof(Params) == 24);

/// Bornes de validité d'un paramètre
struct Range {
    float min;
    float max;
};

inline constexpr Range heater_power_range { 10.0f, 200.0f };
inline constexpr Range block_heat_capacity_range { 1.0f, 100.0f };
inline constexpr Range sensor_responsiveness_range { 0.01f, 5.0f };
inline constexpr Range ambient_xfer_coeff_range { 0.005f, 2.0f };
inline constexpr Range fan_xfer_coeff_range { 0.0f, 2.0f };
inline constexpr Range filament_heat_capacity_range { 0.0f, 0.1f };

/// Calibration valable : toutes les valeurs finies et dans leurs bornes (sans calibration, C = R = A = 0 : non valable)
bool is_valid(const Params &p);

// ---- Régulateur ----

/// Constantes du régulateur (celles de Marlin : MPC_SMOOTHING_FACTOR, MPC_MIN_AMBIENT_CHANGE, MPC_STEADYSTATE)
inline constexpr float horizon_s = 2.0f; ///< la puissance demandée amène le bloc à la consigne en 2 s
inline constexpr float smoothing_time_s = 0.32f; ///< recalage sur la mesure : moitié de l'écart par pas de 0,16 s
inline constexpr float min_ambient_change = 1.0f; ///< [K/s] pas minimal du recalage de l'ambiante
inline constexpr float steady_state_rate = 0.5f; ///< [K/s] en dessous, le bloc est jugé stable
inline constexpr float max_target_cutoff = 5.0f; ///< [K] mesure au-delà de la consigne maximale + 5 K : sortie 0
inline constexpr float max_dt_s = 2.0f; ///< un pas plus long remet le modèle sur la mesure

struct StepArgs {
    float dt_s; ///< temps écoulé depuis le pas précédent [s]
    float measured_temp; ///< température mesurée de la buse [°C]
    float target_temp; ///< consigne [°C], <= 0 : chauffe coupée
    float max_target_temp; ///< consigne maximale de la buse [°C] : une consigne plus haute est ramenée à celle-ci
    float fan; ///< ventilateur pièce, 0 à 1
    float e_speed; ///< vitesse du filament dans la buse [mm/s], 0 hors extrusion
    float applied_pwm; ///< sortie appliquée depuis le pas précédent, 0 à pwm_max
    float pwm_max; ///< sortie à pleine puissance (PID_MAX de Prusa : 255)
};

/// Modèle thermique et régulateur MPC d'une buse
class Model {
public:
    void set_params(const Params &p) { params_ = p; }
    const Params &params() const { return params_; }

    /// Bloc et sonde sur la mesure, ambiante à min(30 °C, mesure) comme Marlin
    void reset(float measured_temp);

    /// Un pas : met le modèle à jour avec la sortie appliquée, puis rend la sortie à appliquer, toujours dans
    /// [0, pwm_max] (0 si la consigne est <= 0, si la mesure dépasse la consigne maximale de max_target_cutoff, ou
    /// si une valeur n'est pas finie)
    float step(const StepArgs &args);

    bool initialized() const { return initialized_; }
    float block_temp() const { return block_temp_; }
    float sensor_temp() const { return sensor_temp_; }
    float ambient_temp() const { return ambient_temp_; }

private:
    Params params_ {};
    float block_temp_ = 0;
    float sensor_temp_ = 0;
    float ambient_temp_ = 0;
    bool initialized_ = false;
};

/// Pertes totales [W/K] pour un ventilateur (0 à 1) et une vitesse du filament [mm/s]
float losses(const Params &p, float fan, float e_speed);

// ---- Calibration (M306 T) ----

/// Étapes de la calibration, dans l'ordre
enum class Phase : uint8_t {
    idle,
    cooling, ///< chauffe coupée, ventilateur à 100 % jusqu'à ce que la buse ne refroidisse plus : température ambiante
    spin_down, ///< ventilateur arrêté quelques secondes avant la chauffe
    heating, ///< pleine puissance, ventilateur arrêté, jusqu'à la température de calibration : courbe de chauffe
    hold, ///< régulée (PID de Prusa) à la température de calibration, ventilateur arrêté : pertes A
    hold_fan, ///< même chose, ventilateur à 100 % : pertes en plus F
    done,
    failed,
};

enum class Error : uint8_t {
    none,
    cancelled, ///< « Annuler », ou arrêt de Marlin
    interrupted, ///< la consigne de la buse a changé pendant la calibration
    cooling_timeout, ///< la buse ne s'est pas stabilisée en refroidissant
    heating_timeout, ///< la température de calibration n'est pas atteinte à pleine puissance
    overheat, ///< la buse a dépassé la température de calibration de plus de overheat_margin
    hold_timeout, ///< la température ne s'est pas stabilisée
    bad_result, ///< mesures incohérentes : paramètres hors des bornes
    refused, ///< pas lancée : impression en cours (M306 T)
};

/// Ce que la calibration demande à la buse
enum class Heater : uint8_t {
    off, ///< chauffe coupée (consigne 0)
    full, ///< pleine puissance (consigne = température de calibration, pour que les protections la surveillent)
    regulate, ///< régulée par le PID de Prusa à la température de calibration
};

struct Command {
    Heater heater;
    float target; ///< consigne à poser [°C] (0 : coupée)
    uint8_t fan; ///< ventilateur pièce, 0 à 255
};

struct TuneConfig {
    float tune_temp = 200.0f; ///< température de calibration [°C] (Marlin : 200 °C)
    float heater_power = default_heater_power; ///< P supposée [W]
    float filament_heat_capacity_permm = default_filament_heat_capacity; ///< H, non mesurée
    float pwm_max = 255.0f;
};

/// Une mesure, à chaque pas de la boucle de M306
struct TuneInput {
    float time_s; ///< temps monotone [s]
    float temp; ///< température mesurée [°C]
    float applied_pwm; ///< sortie appliquée à la buse, 0 à pwm_max
};

/// Durées et seuils de la calibration
inline constexpr float cooling_check_s = 10.0f; ///< la buse ne refroidit plus : moins de cooling_drop_k en 10 s
inline constexpr float cooling_drop_k = 0.3f;
inline constexpr float cooling_timeout_s = 900.0f;
inline constexpr float spin_down_s = 5.0f;
inline constexpr float heating_timeout_s = 300.0f;
inline constexpr float overheat_margin = 30.0f; ///< retard de la sonde : la buse dépasse après la coupure
inline constexpr float hold_band_k = 1.0f; ///< stable : à moins de 1 K de la consigne…
inline constexpr float hold_settle_s = 20.0f; ///< … pendant 20 s
inline constexpr float hold_measure_s = 40.0f; ///< puis puissance moyenne sur 40 s
inline constexpr float hold_timeout_s = 240.0f;
inline constexpr float min_tune_temp = 150.0f;
inline constexpr float max_tune_temp = 250.0f;

/// Calibration automatique : machine à états pure. À chaque pas, step() reçoit la mesure et rend la commande à
/// appliquer ; le résultat est prêt quand phase() == Phase::done.
///
/// Méthode (la nôtre, sur le modèle de Marlin) : température ambiante en fin de refroidissement ; courbe de chauffe
/// à pleine puissance ; pertes A et A + F d'après la puissance moyenne régulée à la température de calibration ;
/// enfin C et R ajustés (moindres carrés, recherche par section dorée) pour que le modèle simulé reproduise la
/// courbe de chauffe mesurée.
class Autotune {
public:
    void start(const TuneConfig &config, float time_s);
    Command step(const TuneInput &in);
    void cancel();
    void fail(Error e);

    Phase phase() const { return phase_; }
    Error error() const { return error_; }
    bool finished() const { return phase_ == Phase::done || phase_ == Phase::failed; }

    /// Avancement estimé, 0 à 100
    uint8_t progress() const { return progress_; }

    /// Résultat (valable si phase() == Phase::done)
    const Params &result() const { return result_; }

    /// Température ambiante mesurée [°C]
    float ambient_temp() const { return ambient_; }

    static constexpr size_t max_samples = 96;

private:
    struct Sample {
        float t; ///< depuis le début de la chauffe [s]
        float temp;
    };

    void enter(Phase p, float time_s);
    bool step_hold(const TuneInput &in, float dt, float &loss_w_per_k);
    void compute();
    float fit_error(float c, float r) const;

    TuneConfig config_ {};
    Phase phase_ = Phase::idle;
    Error error_ = Error::none;
    uint8_t progress_ = 0;
    Params result_ {};

    float phase_start_ = 0;
    float last_time_ = 0;
    bool has_last_ = false;

    // Refroidissement
    float cooling_start_temp_ = 0;
    float cooling_check_time_ = 0;
    float cooling_check_temp_ = 0;
    float ambient_ = 0;

    // Chauffe
    std::array<Sample, max_samples> samples_ {};
    size_t sample_count_ = 0;
    float sample_interval_ = 0.5f;
    float heating_start_temp_ = 0;
    float heating_pwm_sum_ = 0;
    float heating_time_sum_ = 0;
    float heating_power_ = 0; ///< puissance moyenne appliquée pendant la chauffe [W]

    // Paliers
    float stable_since_ = -1;
    float measure_start_ = -1;
    float hold_pwm_integral_ = 0;
    float hold_temp_integral_ = 0;
    float hold_time_ = 0;
    float loss_fan0_ = 0;
    float loss_fan255_ = 0;
};

} // namespace od3d::mpc
