// OneDrop3D (F38) : sécurité collision / warping. Trace d'impression synthétique fidèle au relevé OD3D_PA3.CSV du
// 22/11 (cellule à 295 Hz, bruit ±15 g, extrudeur qui tire la buse vers le bas de 300 à 1 200 g, force qui remonte à
// chaque ralentissement aux mêmes coins, rétraction et déplacement extrudeur arrêté) : aucune alerte ; pics isolés :
// aucune alerte ; chocs répétés au même endroit : alerte ; chocs pendant les phases exclues : ignorés.
#include <feature/od3d/collision_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <numbers>
#include <random>
#include <vector>

using namespace od3d::collision;

namespace {

constexpr uint32_t dt_us = 3390; // 295 Hz : la cellule hors haute précision (12 échantillons sur 13, HX717 à 320 Hz)

/// Un choc ajouté à la trace : demi-sinus de amplitude_g (vers le haut) sur duration_ms, à l'instant t_s
struct Bump {
    double t_s;
    float amplitude_g;
    float duration_ms = 30;
    float jolt_g = NAN; ///< secousse vue par l'accéléromètre, NAN : pas d'accéléromètre
};

struct Trace {
    double seconds = 50;
    float noise_sd_g = 10;
    /// Impression : à chaque tour de 4 s, rétraction (0,8 mm à 40 mm/s, avec un à-coup de 300 g vers le haut),
    /// déplacement extrudeur arrêté jusqu'à 0,3 s, désrétraction, puis extrusion qui tire la buse vers le bas
    /// (flow_g, ou de 300 à 1 200 g selon le tour) avec un ralentissement de 60 ms à chaque coin (1, 2 et 3 s)
    bool print = false;
    float flow_g = NAN;
    float z_mm = 5;
    std::vector<Bump> bumps;
    std::function<bool(double)> armed = [](double) { return true; };
};

/// La tête fait le tour d'un carré de 100 mm (de (50, 50) à (150, 150)) à 100 mm/s : 4 s par tour
void head_position(double t, float &x, float &y) {
    const double s = std::fmod(t * 100.0, 400.0);
    if (s < 100) {
        x = float(50 + s), y = 50;
    } else if (s < 200) {
        x = 150, y = float(50 + s - 100);
    } else if (s < 300) {
        x = float(150 - (s - 200)), y = 150;
    } else {
        x = 50, y = float(150 - (s - 300));
    }
}

struct Outcome {
    int pulses = 0; ///< impulsions relevées (CSV)
    int hits = 0; ///< chocs comptés
    int alerts = 0;
    std::vector<double> alert_times;
};

Outcome run(const Trace &tr, Sensitivity sens) {
    const Params p = params_for(sens);
    std::mt19937 rng(1234);
    std::normal_distribution<float> noise(0.f, tr.noise_sd_g);
    std::uniform_real_distribution<float> level(300.f, 1200.f);
    const bool with_accel = std::any_of(tr.bumps.begin(), tr.bumps.end(), [](const Bump &b) { return std::isfinite(b.jolt_g); });

    Detector det;
    HitMap map;
    Outcome out;

    float lap_flow = 0;
    int lap_seen = -1;
    float pressure = 0; // force d'extrusion réelle (vers le bas), en retard de 30 ms sur la consigne
    double e = 0;
    const uint32_t start_us = 4'000'000'000u; // l'horloge en µs reboucle pendant la trace
    const auto steps = uint64_t(tr.seconds * 1e6 / dt_us);
    for (uint64_t i = 0; i < steps; ++i) {
        const double t = double(i) * dt_us * 1e-6;
        const double dt = dt_us * 1e-6;
        float x, y;
        head_position(t, x, y);

        float command = 0; // consigne de force d'extrusion [g]
        float suction = 0;
        double ve = 0; // vitesse de l'extrudeur [mm/s]
        if (tr.print) {
            const int lap = int(t / 4.0);
            if (lap != lap_seen) {
                lap_seen = lap;
                lap_flow = std::isfinite(tr.flow_g) ? tr.flow_g : level(rng);
            }
            const double u = std::fmod(t, 4.0);
            if (u < 0.02) {
                ve = -40, suction = 300; // rétraction
            } else if (u < 0.30) {
                ve = 0; // déplacement
            } else if (u < 0.32) {
                ve = 40; // désrétraction
            } else {
                double slow = 1;
                for (const double corner : { 1.0, 2.0, 3.0 }) {
                    slow = std::min(slow, std::min(1.0, std::fabs(u - corner) / 0.03)); // creux de 60 ms au coin
                }
                command = lap_flow * float(slow);
                ve = command / 300.0; // 1 200 g ≈ 4 mm/s de filament
            }
        }
        e += ve * dt;
        pressure += (command - pressure) * float(dt_us) / 30'000.f;

        float bump = 0;
        float jolt = NAN;
        for (const auto &b : tr.bumps) {
            const double u = (t - b.t_s) / (b.duration_ms * 1e-3);
            if (u >= 0 && u <= 1) {
                bump += b.amplitude_g * float(std::sin(std::numbers::pi * u));
                jolt = b.jolt_g;
            }
        }
        if (std::isfinite(jolt)) {
            det.accel(1.f + jolt); // gravité + secousse
        } else if (with_accel) {
            det.accel(1.f); // accéléromètre branché, au repos
        }

        // Charge non tarée, vers le haut positive : repos 12 kg qui dérive, extrudeur qui tire vers le bas
        const float up = 12'000.f + float(t) * 0.3f - pressure + suction + bump + noise(rng);
        if (det.sample(uint32_t(start_us + i * dt_us), up, x, y, tr.z_mm, float(e), tr.armed(t), p.threshold_g * log_fraction)) {
            ++out.pulses;
            const Pulse &pulse = det.pulse();
            if (counts(pulse, p.threshold_g)) {
                ++out.hits;
                if (map.add(uint32_t(t * 1000), pulse.x, pulse.y, confirmed(pulse), p.hits).alert) {
                    ++out.alerts;
                    out.alert_times.push_back(t);
                }
            }
        }
    }
    return out;
}

/// count chocs à l'instant lap_time de chaque tour (même endroit), à partir du tour first_lap
std::vector<Bump> lap_bumps(int count, float amplitude_g, double lap_time = 0.15, int first_lap = 3, float jolt_g = NAN) {
    std::vector<Bump> v;
    for (int i = 0; i < count; ++i) {
        v.push_back({ .t_s = 4.0 * (first_lap + i) + lap_time, .amplitude_g = amplitude_g, .jolt_g = jolt_g });
    }
    return v;
}

} // namespace

TEST_CASE("collision : seuils par sensibilité", "[od3d][collision]") {
    CHECK(params_for(Sensitivity::low).threshold_g == 600.f);
    CHECK(params_for(Sensitivity::normal).threshold_g == 400.f);
    CHECK(params_for(Sensitivity::high).threshold_g == 250.f);
    CHECK(params_for(Sensitivity::low).hits == 4);
    CHECK(params_for(Sensitivity::normal).hits == 3);
    CHECK(params_for(Sensitivity::high).hits == 2);
    // Le plus petit choc compté vaut le double du palpage de Prusa (125 g)
    CHECK(params_for(Sensitivity::high).threshold_g >= 2 * 125.f);
}

TEST_CASE("collision : trace d'impression sans choc, aucune alerte", "[od3d][collision]") {
    Trace tr;
    tr.seconds = 600;
    tr.noise_sd_g = 15;
    tr.print = true; // la force remonte de 300 à 1 200 g aux trois mêmes coins à chaque tour
    for (const auto s : { Sensitivity::low, Sensitivity::normal, Sensitivity::high }) {
        const Outcome o = run(tr, s);
        CHECK(o.hits == 0);
        CHECK(o.alerts == 0);
    }
}

TEST_CASE("collision : pics de bruit isolés, aucune alerte", "[od3d][collision]") {
    Trace tr;
    tr.seconds = 1520;
    // Un pic fort par minute, chaque fois 17 mm plus loin sur le tour : jamais deux à moins de 5 mm en 15 min
    for (int i = 0; i < 25; ++i) {
        tr.bumps.push_back({ .t_s = 10.0 + 60.17 * i, .amplitude_g = 900, .duration_ms = i % 2 ? 30.f : 8.f });
    }
    const Outcome o = run(tr, Sensitivity::high);
    CHECK(o.hits == 25); // chacun est bien un choc…
    CHECK(o.alerts == 0); // …mais jamais deux au même endroit
}

TEST_CASE("collision : deux chocs au même endroit à plus de 15 min d'écart, aucune alerte", "[od3d][collision]") {
    Trace tr;
    tr.seconds = 940;
    tr.bumps = { { .t_s = 10.0, .amplitude_g = 900 }, { .t_s = 10.0 + 4.0 * 230, .amplitude_g = 900 } };
    const Outcome o = run(tr, Sensitivity::high);
    CHECK(o.hits == 2);
    CHECK(o.alerts == 0);
}

TEST_CASE("collision : chocs répétés au même endroit, alerte", "[od3d][collision]") {
    Trace tr;

    SECTION("sensibilité normale : 3 chocs de 600 g au même endroit") {
        tr.bumps = lap_bumps(3, 600);
        const Outcome o = run(tr, Sensitivity::normal);
        CHECK(o.hits == 3);
        REQUIRE(o.alerts == 1);
        CHECK(o.alert_times[0] > 4.0 * 5); // au 3e choc, pas avant
    }
    SECTION("2 chocs ne suffisent pas en sensibilité normale") {
        tr.bumps = lap_bumps(2, 600);
        CHECK(run(tr, Sensitivity::normal).alerts == 0);
    }
    SECTION("sensibilité faible : 600 g, 4 chocs") {
        tr.bumps = lap_bumps(3, 700);
        CHECK(run(tr, Sensitivity::low).alerts == 0);
        tr.bumps = lap_bumps(4, 700);
        CHECK(run(tr, Sensitivity::low).alerts == 1);
        tr.bumps = lap_bumps(4, 500); // sous le seuil de la sensibilité faible
        CHECK(run(tr, Sensitivity::low).alerts == 0);
    }
    SECTION("pendant l'impression : déplacement extrudeur arrêté") {
        tr.print = true;
        tr.noise_sd_g = 15;
        tr.bumps = lap_bumps(3, 600, 0.20);
        CHECK(run(tr, Sensitivity::normal).alerts == 1);
    }
    SECTION("pendant l'extrusion : le choc doit dépasser en plus la poussée de l'extrudeur") {
        tr.print = true;
        tr.noise_sd_g = 15;
        tr.flow_g = 800;
        tr.bumps = lap_bumps(3, 800 + 600, 2.5); // milieu d'un côté, en pleine extrusion
        CHECK(run(tr, Sensitivity::normal).alerts == 1);
        tr.bumps = lap_bumps(3, 900, 2.5); // 100 g au-dessus du repos seulement
        CHECK(run(tr, Sensitivity::normal).alerts == 0);
    }
    SECTION("bien au-dessus de la force normale : noyés dans une impression très bruitée, pas des chocs") {
        tr.bumps = lap_bumps(3, 300);
        tr.noise_sd_g = 70; // 8 × bruit de fond ≈ 450 g
        CHECK(run(tr, Sensitivity::high).alerts == 0);
        tr.noise_sd_g = 10; // témoin
        CHECK(run(tr, Sensitivity::high).alerts == 1);
    }
    SECTION("l'accéléromètre confirme : un choc compte double, mais il en faut 2 distincts") {
        tr.bumps = lap_bumps(1, 600, 0.15, 3, 1.5f);
        CHECK(run(tr, Sensitivity::normal).alerts == 0);
        tr.bumps = lap_bumps(2, 600, 0.15, 3, 1.5f);
        CHECK(run(tr, Sensitivity::normal).alerts == 1);
        tr.bumps = lap_bumps(2, 600, 0.15, 3, 0.1f); // secousse trop faible : pas de confirmation
        CHECK(run(tr, Sensitivity::normal).alerts == 0);
    }
    SECTION("une marche (impulsion trop longue) n'est pas un choc") {
        for (auto &b : tr.bumps = lap_bumps(3, 600)) {
            b.duration_ms = 400;
        }
        const Outcome o = run(tr, Sensitivity::normal);
        CHECK(o.hits == 0);
        CHECK(o.alerts == 0);
    }
}

TEST_CASE("collision : chocs pendant les phases exclues, ignorés", "[od3d][collision]") {
    Trace tr;
    tr.bumps = lap_bumps(3, 800, 2.0);

    SECTION("désarmé (palpage, origine, changement d'outil, chargement, hors impression)") {
        tr.armed = [](double) { return false; };
        const Outcome o = run(tr, Sensitivity::high);
        CHECK(o.pulses == 0);
        CHECK(o.alerts == 0);
    }
    SECTION("désarmé autour de chaque choc seulement") {
        tr.armed = [](double t) { return std::fmod(t, 4.0) < 1.5 || std::fmod(t, 4.0) > 2.5; };
        const Outcome o = run(tr, Sensitivity::high);
        CHECK(o.pulses == 0);
        CHECK(o.alerts == 0);
    }
    SECTION("réarmé juste avant le choc : la moyenne se refait d'abord (1 s)") {
        tr.armed = [](double t) { return std::fmod(t, 4.0) < 1.0 || std::fmod(t, 4.0) > 1.5; };
        CHECK(run(tr, Sensitivity::high).pulses == 0);
    }
    SECTION("sous Z = 1 mm (purge, première couche)") {
        tr.z_mm = 0.2f;
        const Outcome o = run(tr, Sensitivity::high);
        CHECK(o.pulses == 0);
        CHECK(o.alerts == 0);
    }
    SECTION("pendant une rétraction et juste après") {
        Trace rt;
        rt.print = true;
        rt.flow_g = 600;
        // Les chocs tombent pendant la rétraction (10 ms après son début) et dans les 100 ms qui suivent
        for (int lap = 3; lap < 6; ++lap) {
            rt.bumps.push_back({ .t_s = 4.0 * lap + 0.005, .amplitude_g = 800 });
            rt.bumps.push_back({ .t_s = 4.0 * lap + 0.06, .amplitude_g = 800 });
        }
        const Outcome o = run(rt, Sensitivity::high);
        CHECK(o.hits == 0);
        CHECK(o.alerts == 0);
    }
    SECTION("avant d'avoir vu l'extrudeur au repos depuis le réarmement") {
        Trace ex;
        ex.print = true;
        ex.flow_g = 600;
        // Réarmé à 1,5 s de chaque tour, en pleine extrusion ; le repos ne revient qu'au tour suivant
        ex.armed = [](double t) { return std::fmod(t, 4.0) >= 1.5; };
        ex.bumps = lap_bumps(3, 600 + 800, 2.7);
        CHECK(run(ex, Sensitivity::high).pulses == 0);
    }
    SECTION("témoin : les mêmes chocs, armés, donnent l'alerte") {
        CHECK(run(tr, Sensitivity::high).alerts == 1);
    }
}

TEST_CASE("collision : rebond et oubli dans la carte des chocs", "[od3d][collision]") {
    HitMap map;
    // Deux impulsions à 0,1 s au même endroit : un seul choc
    CHECK(map.add(1000, 10, 10, false, 2).near == 1);
    const auto r = map.add(1100, 10.5f, 10, false, 2);
    CHECK(r.near == 1);
    CHECK_FALSE(r.alert);
    // Un vrai deuxième choc, 1 s plus tard : alerte, puis la carte repart de zéro
    CHECK(map.add(2100, 11, 10, false, 2).alert);
    CHECK(map.add(3100, 11, 10, false, 2).near == 1);
    // Plus de 8 chocs ailleurs : le plus ancien sort, rien ne déborde
    for (uint32_t i = 0; i < 20; ++i) {
        CHECK_FALSE(map.add(4000 + i * 1000, 200 + 10.f * float(i), 0, false, 2).alert);
    }
    // Position inconnue : jamais « au même endroit »
    CHECK_FALSE(map.add(30000, NAN, NAN, false, 2).alert);
    CHECK_FALSE(map.add(31000, NAN, NAN, false, 2).alert);
}
