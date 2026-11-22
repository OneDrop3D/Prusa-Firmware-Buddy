// OneDrop3D (F38) : sécurité collision / warping. Trace d'impression synthétique (cellule à 295 Hz, dérive, paliers
// d'extrusion, bruit, rétractions avec leur à-coup de force) : aucune alerte ; pics isolés : aucune alerte ; chocs
// répétés au même endroit : alerte ; chocs pendant les phases exclues : ignorés.
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

/// Un choc ajouté à la trace : demi-sinus de amplitude_g sur duration_ms, à l'instant t_s
struct Bump {
    double t_s;
    float amplitude_g;
    float duration_ms = 30;
    float jolt_g = NAN; ///< secousse vue par l'accéléromètre, NAN : pas d'accéléromètre
};

struct Trace {
    double seconds = 60;
    float noise_sd_g = 10;
    bool extrusion_steps = false; ///< paliers de force d'extrusion de 0 à 300 g, de 0,2 à 2 s chacun
    bool retractions = false; ///< rétraction de 0,8 mm à 40 mm/s à chaque tour, avec un à-coup de 500 g
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
    std::uniform_real_distribution<float> level(0.f, 300.f);
    std::uniform_real_distribution<double> length(0.2, 2.0);

    Detector det;
    HitMap map;
    Outcome out;

    float extrusion_target = 0, extrusion = 0;
    double next_segment = 0;
    double e = 0;
    const uint32_t start_us = 4'000'000'000u; // l'horloge en µs reboucle pendant la trace
    const auto steps = uint64_t(tr.seconds * 1e6 / dt_us);
    for (uint64_t i = 0; i < steps; ++i) {
        const double t = double(i) * dt_us * 1e-6;
        float x, y;
        head_position(t, x, y);

        if (tr.extrusion_steps && t >= next_segment) {
            extrusion_target = level(rng);
            next_segment = t + length(rng);
        }
        extrusion += (extrusion_target - extrusion) * float(dt_us) / 30'000.f; // réponse de la buse, 30 ms

        // Rétraction au coin (50, 50) à chaque tour : 20 ms en arrière, 100 ms de déplacement, 20 ms en avant
        float retract_force = 0;
        const double lap = std::fmod(t, 4.0);
        double e_now = e + 2.0 * t; // extrusion à 2 mm/s
        if (tr.retractions && lap < 0.14) {
            const double r = lap < 0.02 ? lap / 0.02 : lap < 0.12 ? 1.0 : 1.0 - (lap - 0.12) / 0.02;
            e_now -= 0.8 * r;
            retract_force = (lap < 0.02 || lap >= 0.12) ? 500.f : 0.f; // à-coup pendant les mouvements de l'extrudeur
        }

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
        } else if (std::any_of(tr.bumps.begin(), tr.bumps.end(), [](const Bump &b) { return std::isfinite(b.jolt_g); })) {
            det.accel(1.f); // accéléromètre branché, au repos
        }

        const float up = 12'000.f + float(t) * 0.3f + extrusion + retract_force + bump + noise(rng);
        if (det.sample(uint32_t(start_us + i * dt_us), up, x, y, tr.z_mm, float(e_now), tr.armed(t), p.threshold_g * log_fraction)) {
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

/// Chocs au coin (150, 150), atteint à 2 s de chaque tour, à partir du tour first_lap
std::vector<Bump> corner_bumps(int count, float amplitude_g, int first_lap = 3, float jolt_g = NAN) {
    std::vector<Bump> v;
    for (int i = 0; i < count; ++i) {
        v.push_back({ .t_s = 4.0 * (first_lap + i) + 2.0, .amplitude_g = amplitude_g, .jolt_g = jolt_g });
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
    tr.extrusion_steps = true;
    tr.retractions = true;
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
    tr.seconds = 50;

    SECTION("sensibilité normale : 3 chocs de 600 g au même coin") {
        tr.bumps = corner_bumps(3, 600);
        const Outcome o = run(tr, Sensitivity::normal);
        CHECK(o.hits == 3);
        REQUIRE(o.alerts == 1);
        CHECK(o.alert_times[0] > 4.0 * 5 + 1.9); // au 3e choc, pas avant
    }
    SECTION("2 chocs ne suffisent pas en sensibilité normale") {
        tr.bumps = corner_bumps(2, 600);
        CHECK(run(tr, Sensitivity::normal).alerts == 0);
    }
    SECTION("sensibilité faible : 600 g, 4 chocs") {
        tr.bumps = corner_bumps(3, 700);
        CHECK(run(tr, Sensitivity::low).alerts == 0);
        tr.bumps = corner_bumps(4, 700);
        CHECK(run(tr, Sensitivity::low).alerts == 1);
        tr.bumps = corner_bumps(4, 500); // sous le seuil de la sensibilité faible
        CHECK(run(tr, Sensitivity::low).alerts == 0);
    }
    SECTION("par-dessus le bruit et les paliers d'extrusion") {
        tr.noise_sd_g = 15;
        tr.extrusion_steps = true;
        tr.retractions = true;
        tr.bumps = corner_bumps(5, 1000); // un choc pris dans un palier d'extrusion peut se perdre : 5 pour 3
        CHECK(run(tr, Sensitivity::normal).alerts >= 1);
    }
    SECTION("bien au-dessus de la force normale : noyés dans une impression très bruitée, pas des chocs") {
        tr.bumps = corner_bumps(3, 300);
        tr.noise_sd_g = 70; // 8 × bruit de fond ≈ 450 g
        CHECK(run(tr, Sensitivity::high).alerts == 0);
        tr.noise_sd_g = 10; // témoin
        CHECK(run(tr, Sensitivity::high).alerts == 1);
    }
    SECTION("l'accéléromètre confirme : un choc compte double, mais il en faut 2 distincts") {
        tr.bumps = corner_bumps(1, 600, 3, 1.5f);
        CHECK(run(tr, Sensitivity::normal).alerts == 0);
        tr.bumps = corner_bumps(2, 600, 3, 1.5f);
        CHECK(run(tr, Sensitivity::normal).alerts == 1);
        tr.bumps = corner_bumps(2, 600, 3, 0.1f); // secousse trop faible : pas de confirmation
        CHECK(run(tr, Sensitivity::normal).alerts == 0);
    }
    SECTION("une marche (impulsion trop longue) n'est pas un choc") {
        for (auto &b : tr.bumps = corner_bumps(3, 600)) {
            b.duration_ms = 400;
        }
        const Outcome o = run(tr, Sensitivity::normal);
        CHECK(o.hits == 0);
        CHECK(o.alerts == 0);
    }
}

TEST_CASE("collision : chocs pendant les phases exclues, ignorés", "[od3d][collision]") {
    Trace tr;
    tr.seconds = 40;
    tr.bumps = corner_bumps(3, 800);

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
    SECTION("pendant une rétraction") {
        Trace rt;
        rt.seconds = 40;
        rt.retractions = true;
        // Les chocs tombent pendant la rétraction (coin (50, 50), 10 ms après son début) et le déplacement qui suit
        for (int lap = 3; lap < 6; ++lap) {
            rt.bumps.push_back({ .t_s = 4.0 * lap + 0.01, .amplitude_g = 800 });
            rt.bumps.push_back({ .t_s = 4.0 * lap + 0.06, .amplitude_g = 800 });
        }
        const Outcome o = run(rt, Sensitivity::high);
        CHECK(o.hits == 0);
        CHECK(o.alerts == 0);
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
