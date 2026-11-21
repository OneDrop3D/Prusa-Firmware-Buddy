// OneDrop3D (F7) : régulation MPC de la buse. Sortie bornée, consigne maximale refusée, montée et palier sur une
// buse simulée, calibration automatique (M306 T) sur la même buse simulée.
#include <feature/od3d/mpc_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

using namespace od3d::mpc;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

constexpr float dt = 0.16f; ///< pas de la régulation de Prusa (PID_dT : 16 × 10 / 1000 Hz)
constexpr float pwm_max = 255.0f; ///< PID_MAX
constexpr float max_target = 290.0f; ///< 305 °C (max_nozzle_temp) − 15 (HEATER_MAXTEMP_SAFETY_MARGIN)

/// Buse simulée, plus riche que le modèle du MPC : cartouche (petite capacité) reliée au bloc, bloc qui perd vers
/// l'air (ventilateur, filament), sonde en retard sur le bloc, bruit de mesure. Ordres de grandeur du Nextruder
/// d'après le modèle de Prusa (steady_state_hotend : ~11 W à 215 °C sans ventilateur, ~31 W à 100 %).
struct Plant {
    float power = 40; ///< [W] à pwm_max
    float c_heater = 2.0f; ///< [J/K]
    float c_block = 9.5f; ///< [J/K]
    float k_heater_block = 1.5f; ///< [W/K]
    float a = 0.056f; ///< [W/K]
    float f = 0.105f; ///< [W/K] en plus à 100 % de ventilateur
    float h = 0.0056f; ///< [J/K/mm]
    float r = 0.35f; ///< [1/s]
    float ambient = 25;
    float noise = 0.05f; ///< écart type du bruit de mesure [K]

    float t_heater = 25, t_block = 25, t_sensor = 25;
    std::mt19937 rng { 1 };

    void set_temp(float t) {
        t_heater = t_block = t_sensor = t;
    }

    void step(float seconds, float pwm, float fan, float e_speed) {
        constexpr int n = 10;
        const float s = seconds / n;
        const float u = std::clamp(pwm / pwm_max, 0.0f, 1.0f);
        for (int i = 0; i < n; ++i) {
            const float q = k_heater_block * (t_heater - t_block);
            t_heater += (u * power - q) * s / c_heater;
            t_block += (q - (a + fan * f + e_speed * h) * (t_block - ambient)) * s / c_block;
            t_sensor += (t_block - t_sensor) * (1.0f - std::exp(-r * s));
        }
    }

    float measure() {
        std::normal_distribution<float> d { 0, noise };
        return t_sensor + d(rng);
    }
};

/// Paramètres « vrais » de la buse simulée, vus par le modèle à deux nœuds du MPC
Params plant_params(const Plant &p) {
    return Params {
        .heater_power = p.power,
        .block_heat_capacity = p.c_heater + p.c_block,
        .sensor_responsiveness = p.r,
        .ambient_xfer_coeff = p.a,
        .fan_xfer_coeff = p.f,
        .filament_heat_capacity_permm = p.h,
    };
}

/// Une boucle MPC + buse simulée
struct Loop {
    Plant plant;
    Model model;
    float pwm = 0;
    float time = 0;

    explicit Loop(const Params &params) {
        model.set_params(params);
    }

    void run(float seconds, float target, float fan, float e_speed, float target_max = max_target) {
        for (float end = time + seconds; time < end; time += dt) {
            pwm = model.step({ dt, plant.measure(), target, target_max, fan, e_speed, pwm, pwm_max });
            REQUIRE(pwm >= 0);
            REQUIRE(pwm <= pwm_max);
            plant.step(dt, pwm, fan, e_speed);
        }
    }

    /// Écart maximal à la consigne (sonde, sans bruit) pendant seconds
    float max_error(float seconds, float target, float fan, float e_speed) {
        float worst = 0;
        for (float end = time + seconds; time < end; time += dt) {
            pwm = model.step({ dt, plant.measure(), target, max_target, fan, e_speed, pwm, pwm_max });
            plant.step(dt, pwm, fan, e_speed);
            worst = std::max(worst, std::fabs(plant.t_sensor - target));
        }
        return worst;
    }
};

Params good_params() {
    return plant_params(Plant {});
}

/// PI simple à la place du PID de Prusa pendant les paliers de la calibration
struct HoldRegulator {
    float i = 60;
    float step(float error) {
        const float out = 14 * error + i;
        if (!((out > pwm_max && error > 0) || (out < 0 && error < 0))) {
            i += 0.16f * error;
        }
        return std::clamp(out, 0.0f, pwm_max);
    }
};

struct TuneRun {
    Autotune tune;
    Plant plant;
    float time = 0;
    float pwm = 0;
    float max_seen = 0;
    bool target_set_while_heating = true;

    void run(const TuneConfig &config, float max_seconds = 3000) {
        tune.start(config, 0);
        HoldRegulator hold;
        while (!tune.finished() && time < max_seconds) {
            const Command c = tune.step({ time, plant.measure(), pwm });
            switch (c.heater) {
            case Heater::off:
                pwm = 0;
                CHECK(c.target == 0);
                break;
            case Heater::full:
                pwm = pwm_max;
                target_set_while_heating = target_set_while_heating && c.target == std::clamp(config.tune_temp, min_tune_temp, max_tune_temp);
                break;
            case Heater::regulate:
                pwm = hold.step(c.target - plant.measure());
                break;
            }
            plant.step(dt, pwm, c.fan / 255.0f, 0);
            max_seen = std::max(max_seen, plant.t_sensor);
            time += dt;
        }
    }
};

} // namespace

TEST_CASE("mpc : validité des paramètres") {
    CHECK_FALSE(is_valid(Params {})); // pas de calibration : C = R = A = 0
    CHECK(is_valid(good_params()));

    Params p = good_params();
    p.block_heat_capacity = std::numeric_limits<float>::quiet_NaN();
    CHECK_FALSE(is_valid(p));
    p = good_params();
    p.heater_power = 0;
    CHECK_FALSE(is_valid(p));
    p = good_params();
    p.fan_xfer_coeff = -0.1f;
    CHECK_FALSE(is_valid(p));
    p = good_params();
    p.sensor_responsiveness = 50;
    CHECK_FALSE(is_valid(p));
}

TEST_CASE("mpc : sortie toujours bornée à 0…puissance max") {
    Model m;
    m.set_params(good_params());
    std::mt19937 rng { 42 };
    std::uniform_real_distribution<float> temp(-40, 450);
    std::uniform_real_distribution<float> target(-50, 1000);
    std::uniform_real_distribution<float> unit(-0.5f, 1.5f);
    std::uniform_real_distribution<float> step(-1.0f, 5.0f);
    std::uniform_real_distribution<float> e(-200, 200);
    float pwm = 0;
    for (int i = 0; i < 20000; ++i) {
        const float limit = (i % 3 == 0) ? 128.0f : pwm_max;
        pwm = m.step({ step(rng), temp(rng), target(rng), max_target, unit(rng), e(rng), unit(rng) * limit, limit });
        REQUIRE(pwm >= 0);
        REQUIRE(pwm <= limit);
    }

    SECTION("valeurs non finies : sortie 0") {
        constexpr float nan = std::numeric_limits<float>::quiet_NaN();
        constexpr float inf = std::numeric_limits<float>::infinity();
        CHECK(m.step({ dt, nan, 215, max_target, 0, 0, 0, pwm_max }) == 0);
        CHECK(m.step({ dt, 25, nan, max_target, 0, 0, 0, pwm_max }) == 0);
        CHECK(m.step({ dt, 25, inf, max_target, 0, 0, 0, pwm_max }) == 0);
        CHECK(m.step({ nan, 25, 215, max_target, 0, 0, 0, pwm_max }) == 0);
        CHECK(m.step({ dt, 25, 215, max_target, nan, 0, 0, pwm_max }) == 0);
        CHECK(m.step({ dt, 25, 215, max_target, 0, inf, 0, pwm_max }) == 0);
        CHECK(m.step({ dt, 25, 215, max_target, 0, 0, 0, 0 }) == 0);
    }

    SECTION("sans calibration valable : sortie 0") {
        Model empty;
        CHECK(empty.step({ dt, 25, 215, max_target, 0, 0, 0, pwm_max }) == 0);
    }

    SECTION("froid loin de la consigne : pleine puissance, pas plus") {
        Model cold;
        cold.set_params(good_params());
        CHECK(cold.step({ dt, 25, 280, max_target, 0, 0, 0, pwm_max }) == pwm_max);
    }
}

TEST_CASE("mpc : consigne nulle ou au-delà de la température maximale") {
    Model m;
    m.set_params(good_params());

    SECTION("consigne <= 0 : chauffe coupée") {
        for (int i = 0; i < 50; ++i) {
            CHECK(m.step({ dt, 25, 0, max_target, 0, 0, 0, pwm_max }) == 0);
            CHECK(m.step({ dt, 25, -10, max_target, 0, 0, 0, pwm_max }) == 0);
        }
    }

    SECTION("mesure au-delà de la consigne maximale + marge : sortie 0, même avec une consigne plus haute") {
        CHECK(m.step({ dt, max_target + max_target_cutoff, 400, max_target, 0, 0, 0, pwm_max }) == 0);
        CHECK(m.step({ dt, max_target + 20, 400, max_target, 1, 20, pwm_max, pwm_max }) == 0);
    }

    SECTION("consigne de 400 °C : la buse s'arrête à la consigne maximale") {
        Loop loop(good_params());
        loop.run(300, 400, 0, 0);
        float highest = 0;
        for (int i = 0; i < 1000; ++i) {
            loop.run(dt, 400, 0, 0);
            highest = std::max(highest, loop.plant.t_sensor);
        }
        CHECK(highest <= max_target + 1.0f);
        CHECK(highest >= max_target - 1.0f); // elle y va quand même : la consigne est ramenée, pas ignorée
    }
}

TEST_CASE("mpc : montée et palier sur une buse simulée") {
    Loop loop(good_params());
    constexpr float target = 215;

    // Montée de 25 à 215 °C : pas de dépassement notable, consigne atteinte en moins de 90 s
    float highest = 0;
    float reached = -1;
    for (float end = 150; loop.time < end;) {
        loop.run(dt, target, 0, 0);
        highest = std::max(highest, loop.plant.t_sensor);
        if (reached < 0 && loop.plant.t_sensor >= target - 1) {
            reached = loop.time;
        }
    }
    CHECK(reached > 0);
    CHECK(reached < 90);
    CHECK(highest < target + 2);

    // Palier : ±0,5 K
    CHECK(loop.max_error(60, target, 0, 0) < 0.5f);

    // Ventilateur à 100 % d'un coup (les pertes triplent) : écart limité, puis retour à ±0,5 K
    CHECK(loop.max_error(30, target, 1, 0) < 3);
    CHECK(loop.max_error(60, target, 1, 0) < 0.5f);

    // Extrusion à 12 mm³/s (5 mm/s de filament de 1,75 mm), prévue par le modèle (H)
    CHECK(loop.max_error(30, target, 1, 5) < 2);
    CHECK(loop.max_error(60, target, 1, 5) < 0.5f);

    // Changement de consigne vers le bas puis vers le haut
    loop.run(90, 200, 1, 0);
    CHECK(loop.max_error(30, 200, 1, 0) < 0.5f);
    loop.run(90, 250, 1, 0);
    CHECK(loop.max_error(30, 250, 1, 0) < 0.5f);
}

TEST_CASE("mpc : modèle approximatif, l'ambiante du modèle rattrape l'erreur") {
    Params off = good_params();
    off.block_heat_capacity *= 1.3f;
    off.ambient_xfer_coeff *= 0.7f;
    off.fan_xfer_coeff *= 1.3f;
    off.sensor_responsiveness *= 0.7f;
    Loop loop(off);
    loop.plant.ambient = 40; // chambre chaude
    loop.plant.set_temp(40);
    loop.run(200, 230, 0, 0);
    CHECK(loop.max_error(60, 230, 0, 0) < 0.5f);
    loop.run(120, 230, 0.5f, 0);
    CHECK(loop.max_error(60, 230, 0.5f, 0) < 0.5f);
}

TEST_CASE("mpc : calibration automatique sur une buse simulée") {
    TuneRun run;
    run.plant.set_temp(80); // buse encore tiède
    run.run(TuneConfig {});

    REQUIRE(run.tune.phase() == Phase::done);
    CHECK(run.tune.error() == Error::none);
    CHECK(run.tune.progress() == 100);
    CHECK(run.target_set_while_heating); // pleine puissance avec la consigne posée : protections armées
    CHECK(run.max_seen < 200 + overheat_margin);
    CHECK(run.time < 20 * 60); // dans la minuterie de sécurité de Prusa (30 min)

    const Params r = run.tune.result();
    const Params truth = plant_params(run.plant);
    REQUIRE(is_valid(r));
    CHECK(r.heater_power == default_heater_power);
    CHECK(r.filament_heat_capacity_permm == default_filament_heat_capacity);
    CHECK_THAT(r.ambient_xfer_coeff, WithinRel(truth.ambient_xfer_coeff, 0.15f));
    CHECK_THAT(r.fan_xfer_coeff, WithinRel(truth.fan_xfer_coeff, 0.15f));
    CHECK_THAT(r.block_heat_capacity, WithinRel(truth.block_heat_capacity, 0.25f));
    CHECK_THAT(run.tune.ambient_temp(), WithinAbs(run.plant.ambient, 4.0f));

    // Et la régulation avec les paramètres trouvés tient le palier
    Loop loop(r);
    loop.run(150, 220, 0, 0);
    CHECK(loop.max_error(60, 220, 0, 0) < 0.5f);
    loop.run(60, 220, 1, 0);
    CHECK(loop.max_error(60, 220, 1, 0) < 0.5f);
}

TEST_CASE("mpc : calibration, puissance réelle différente de P supposée") {
    TuneRun run;
    run.plant.power = 34; // 24 V − 5 %, résistance haute
    run.run(TuneConfig {});
    REQUIRE(run.tune.phase() == Phase::done);

    // P ne fixe que l'unité : la régulation reste bonne
    Loop loop(run.tune.result());
    loop.plant.power = 34;
    loop.run(150, 215, 0, 0);
    CHECK(loop.max_error(60, 215, 0, 0) < 0.5f);
}

TEST_CASE("mpc : calibration, arrêts et erreurs") {
    SECTION("Annuler : chauffe coupée tout de suite") {
        Autotune tune;
        tune.start(TuneConfig {}, 0);
        tune.step({ 0, 100, 0 });
        tune.cancel();
        CHECK(tune.phase() == Phase::failed);
        CHECK(tune.error() == Error::cancelled);
        const Command c = tune.step({ 1, 100, 0 });
        CHECK(c.heater == Heater::off);
        CHECK(c.target == 0);
    }

    SECTION("pas de pas avant start : rien ne chauffe") {
        Autotune tune;
        CHECK(tune.step({ 0, 25, 0 }).heater == Heater::off);
        tune.cancel();
        CHECK(tune.phase() == Phase::idle);
    }

    SECTION("dépassement de la température de calibration : arrêt") {
        TuneRun run;
        run.tune.start(TuneConfig {}, 0);
        // Jusqu'à la chauffe
        while (run.tune.phase() != Phase::heating && run.time < 1000) {
            run.tune.step({ run.time, run.plant.measure(), 0 });
            run.plant.step(dt, 0, 1, 0);
            run.time += dt;
        }
        REQUIRE(run.tune.phase() == Phase::heating);
        const Command c = run.tune.step({ run.time + dt, 200 + overheat_margin + 1, pwm_max });
        CHECK(c.heater == Heater::off);
        CHECK(run.tune.error() == Error::overheat);
    }

    SECTION("cartouche trop faible : la chauffe n'aboutit pas") {
        TuneRun run;
        run.plant.power = 8; // 25 + 8 / 0,056 = 168 °C au plus
        run.run(TuneConfig {});
        CHECK(run.tune.phase() == Phase::failed);
        CHECK(run.tune.error() == Error::heating_timeout);
        CHECK(run.max_seen < 200);
    }

    SECTION("mesure non finie : arrêt") {
        Autotune tune;
        tune.start(TuneConfig {}, 0);
        CHECK(tune.step({ 0, std::numeric_limits<float>::quiet_NaN(), 0 }).heater == Heater::off);
        CHECK(tune.error() == Error::interrupted);
    }

    SECTION("température de calibration bornée à 150…250 °C") {
        TuneRun run;
        TuneConfig config;
        config.tune_temp = 400;
        run.run(config);
        REQUIRE(run.tune.phase() == Phase::done);
        CHECK(run.max_seen < max_tune_temp + overheat_margin);
    }
}
