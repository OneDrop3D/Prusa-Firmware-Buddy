// OneDrop3D : choix du filtre de l'input shaper (M1959) accéléré — même type et même fréquence que le code de Prusa,
// recopié ci-dessous depuis M958.cpp et input_shaper.cpp (v8.0.2), sur des spectres synthétiques, bruités et réalistes.
#include <feature/od3d/shaper_fit_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace sf = od3d::shaper_fit;
using input_shaper::Type;

// ---------------------------------------------------------------------------------------------------------------------
// Code de Prusa, recopié à l'identique (seuls les types d'entrée et les SERIAL_ECHO changent)
namespace prusa {

#define sq(x) ((x) * (x))
#define SQR(x) ((x) * (x))
using std::max;

struct Shaper {
    Shaper(const float par_a[], const float par_t[], int par_num_pulses)
        : num_pulses(par_num_pulses) {
        for (int i = 0; i < 5; ++i) {
            if (i < num_pulses) {
                a[i] = par_a[i];
                t[i] = par_t[i];
            } else {
                a[i] = 0.;
                t[i] = 0.;
            }
        }
    }

    float a[5] = {};
    float t[5] = {};
    int num_pulses;
};

// input_shaper.cpp
Shaper get(const float damping_ratio, const float shaper_freq, const float vibration_reduction, const input_shaper::Type type) {
    switch (type) {
    case Type::zvd: {
        constexpr int num_pulses = 3;
        const float df = std::sqrt(1.f - SQR(damping_ratio));
        const float K = std::exp(-damping_ratio * float(M_PI) / df);
        const float t_d = 1.f / (shaper_freq * df);

        const float a[num_pulses] = { 1.f, 2.f * K, SQR(K) };
        const float t[num_pulses] = { 0.f, .5f * t_d, t_d };

        Shaper shaper(a, t, num_pulses);
        return shaper;
    }
    case Type::mzv: {
        constexpr int num_pulses = 3;
        const float df = std::sqrt(1.f - SQR(damping_ratio));
        const float K = std::exp(-.75f * damping_ratio * float(M_PI) / df);
        const float t_d = 1.f / (shaper_freq * df);

        const float a1 = 1.f - 1.f / std::sqrt(2.f);
        const float a2 = (std::sqrt(2.f) - 1.f) * K;
        const float a3 = a1 * K * K;

        const float a[num_pulses] = { a1, a2, a3 };
        const float t[num_pulses] = { 0.f, .375f * t_d, .75f * t_d };

        Shaper shaper(a, t, num_pulses);
        return shaper;
    }
    case Type::ei: {
        constexpr int num_pulses = 3;
        const float v_tol = 1.f / vibration_reduction; // vibration tolerance
        const float df = std::sqrt(1.f - SQR(damping_ratio));
        const float K = std::exp(-damping_ratio * float(M_PI) / df);
        const float t_d = 1.f / (shaper_freq * df);

        const float a1 = .25f * (1.f + v_tol);
        const float a2 = .5f * (1.f - v_tol) * K;
        const float a3 = a1 * K * K;

        float a[num_pulses] = { a1, a2, a3 };
        float t[num_pulses] = { 0.f, .5f * t_d, t_d };

        Shaper shaper(a, t, num_pulses);
        return shaper;
    }
    default:
        // Prusa n'essaie que ZVD, MZV et EI (filter_list)
        throw std::logic_error("type");
    }
}

struct FrequencyGain {
    float frequency;
    float gain;
};

// M1959.cpp : FrequencyRangeSpectrum (30 à 79 Hz par pas de 1 Hz)
struct Spectrum {
    std::vector<float> samples;
    float max() const { return *std::max_element(samples.begin(), samples.end()); }
    size_t size() const { return samples.size(); }
    FrequencyGain get(size_t index) const { return { .frequency = static_cast<float>(30 + index * 1), .gain = samples[index] }; }
};

using FindBestShaperProgressHook = std::function<bool(input_shaper::Type, float)>;

// M958.cpp
static constexpr float epsilon = 0.01f;

static double get_inv_D(const Shaper &shaper) {
    double d = 0.;
    for (int i = 0; i < shaper.num_pulses; ++i) {
        d += static_cast<double>(shaper.a[i]);
    }
    return 1. / d;
}

static double vibration_reduction(const Shaper &shaper, float system_damping_ratio, float frequency) {
    const double inv_D = get_inv_D(shaper);
    const double omega = 2. * std::numbers::pi_v<double> * static_cast<double>(frequency);
    const double damping = static_cast<double>(system_damping_ratio) * omega;
    const double omega_d = omega * sqrt(1. - sq(static_cast<double>(system_damping_ratio)));

    double s = 0.;
    double c = 0.;

    for (int i = 0; i < shaper.num_pulses; ++i) {
        const double t_i = static_cast<double>(shaper.t[i]);
        const double w = shaper.a[i] * exp(-damping * (static_cast<double>(shaper.t[shaper.num_pulses - 1]) - t_i));
        s += w * sin(omega_d * t_i);
        c += w * cos(omega_d * t_i);
    }
    return (sqrt(sq(s) + sq(c)) * inv_D);
}

static float remaining_vibrations(const Shaper &shaper, float system_damping_ratio, const Spectrum &psd, float default_vibration_reduction) {
    float vibr_threshold = psd.max() / default_vibration_reduction;
    float remaining_vibrations_sum = 0.f;
    float all_vibrations_sum = 0.f;
    for (size_t i = 0; i < psd.size(); ++i) {
        const FrequencyGain fg = psd.get(i);
        all_vibrations_sum += max(fg.gain, 0.f);
        const float vibration = max(fg.gain * static_cast<float>(vibration_reduction(shaper, system_damping_ratio, fg.frequency)) - vibr_threshold, 0.f);
        remaining_vibrations_sum += vibration;
    }
    return (remaining_vibrations_sum / all_vibrations_sum);
}

static float smoothing(const Shaper &shaper) {
    constexpr float accel = 5000.f;
    constexpr float scv = 5.f;
    constexpr float half_accel = accel / 2.f;

    const double inv_D = get_inv_D(shaper);

    double ts = 0.;
    for (int i = 0; i < shaper.num_pulses; ++i) {
        ts += static_cast<double>(shaper.a[i] * shaper.t[i]);
    }
    ts *= inv_D;

    double offset_90 = 0.;
    double offset_180 = 0.;

    for (int i = 0; i < shaper.num_pulses; ++i) {
        const double t_i = static_cast<double>(shaper.t[i]);
        const double a_i = static_cast<double>(shaper.a[i]);
        if (t_i >= ts) {
            /// Calculate offset for one of the axes
            offset_90 += a_i * (scv + half_accel * (t_i - ts)) * (t_i - ts);
        }
        offset_180 += a_i * half_accel * sq(t_i - ts);
    }
    offset_90 *= (inv_D * sqrt(2.));
    offset_180 *= inv_D;

    return static_cast<float>(max(offset_90, offset_180));
}

enum class Action {
    first,
    find_best_result = first,
    select,
    last = select,
};
Action &operator++(Action &action) {
    using IntType = typename std::underlying_type<Action>::type;
    action = static_cast<Action>(static_cast<IntType>(action) + 1);
    return action;
}

struct Shaper_result {
    float frequency;
    float score;
    float smoothing;
};
static Shaper_result fit_shaper(const FindBestShaperProgressHook &progress_hook, input_shaper::Type type, const Spectrum &psd, const Action final_action, input_shaper::AxisConfig default_config) {
    constexpr float start_frequency = 5.f;
    constexpr float end_frequency = 150.f;
    constexpr float frequency_step = .2f;
    constexpr float max_smoothing = std::numeric_limits<float>::max();

    const float default_damping_ratio = default_config.damping_ratio;
    const float default_vibration_reduction = default_config.vibration_reduction;

    struct Result {
        float frequency;
        float score;
        float smoothing;
        float vibrs;
    };

    Result best_result = { 0.f, 0.f, 0.f, std::numeric_limits<float>::max() };
    Result selected_result = { 0.f, 0.f, 0.f, std::numeric_limits<float>::max() };

    for (Action action = Action::first; action <= final_action; ++action) {
        for (float frequency = end_frequency; frequency >= start_frequency - epsilon; frequency -= frequency_step) {
            const float progress_ratio = (end_frequency - frequency) / (end_frequency - start_frequency);
            if (!progress_hook(type, progress_ratio)) {
                return {};
            }
            Shaper shaper = get(default_damping_ratio, frequency, default_vibration_reduction, type);
            const float shaper_smoothing = smoothing(shaper);
            if (Action::find_best_result == action && best_result.frequency && shaper_smoothing > max_smoothing) {
                Shaper_result result = { .frequency = best_result.frequency, .score = best_result.score, .smoothing = best_result.smoothing };
                return result;
            }
            float shaper_vibrations = 0.f;
            for (float damping_ratio = 0.05f; damping_ratio <= 0.15f + epsilon; damping_ratio += 0.05f) {
                float vibrations = remaining_vibrations(shaper, damping_ratio, psd, default_vibration_reduction);
                if (vibrations > shaper_vibrations) {
                    shaper_vibrations = vibrations;
                }
                if (!progress_hook(type, progress_ratio)) {
                    return {};
                }
            }
            const float shaper_score = shaper_smoothing * (std::pow(shaper_vibrations, 1.5f) + shaper_vibrations * .2f + .01f);

            if (Action::find_best_result == action && shaper_vibrations < best_result.vibrs) {
                Result result = { .frequency = frequency, .score = shaper_score, .smoothing = shaper_smoothing, .vibrs = shaper_vibrations };
                best_result = result;
                selected_result = best_result;
            }

            if (Action::select == action && (shaper_vibrations < (best_result.vibrs * 1.1f)) && shaper_score < selected_result.score) {
                Result result = { .frequency = frequency, .score = shaper_score, .smoothing = shaper_smoothing, .vibrs = shaper_vibrations };
                selected_result = result;
            }
        }
    }

    Shaper_result shaper_result = { .frequency = selected_result.frequency, .score = selected_result.score, .smoothing = selected_result.smoothing };
    return shaper_result;
}

static input_shaper::AxisConfig find_best_shaper(const FindBestShaperProgressHook &progress_hook, const Spectrum &psd, const Action final_action, input_shaper::AxisConfig default_config) {
    struct Best_score {
        Shaper_result result;
        input_shaper::Type type;
    };
    std::optional<Best_score> best_shaper;

    for (input_shaper::Type shaper_type = input_shaper::Type::first; shaper_type <= input_shaper::Type::last; ++shaper_type) {
        if (shaper_type == input_shaper::Type::null || !std::ranges::contains(input_shaper::filter_list, shaper_type)) {
            continue;
        }

        if (!progress_hook(shaper_type, 0.0f)) {
            break;
        }

        Shaper_result shaper = fit_shaper(progress_hook, shaper_type, psd, final_action, default_config);

        if (
            !best_shaper.has_value() //
            || shaper.score * 1.2f < best_shaper->result.score
            || ((shaper.score * 1.05f < best_shaper->result.score) && (shaper.smoothing * 1.1f < best_shaper->result.smoothing)) //
        ) {
            best_shaper = {
                .result = shaper,
                .type = shaper_type,
            };
        }
    }

    return input_shaper::AxisConfig {
        .type = best_shaper ? best_shaper->type : input_shaper::Type::null,
        .frequency = best_shaper ? best_shaper->result.frequency : 0,
        .damping_ratio = default_config.damping_ratio,
        .vibration_reduction = default_config.vibration_reduction,
    };
}

input_shaper::AxisConfig find_best_shaper(const FindBestShaperProgressHook &progress_hook, const Spectrum &psd, input_shaper::AxisConfig default_config) {
    return find_best_shaper(progress_hook, psd, Action::find_best_result, default_config);
}

#undef sq
#undef SQR

} // namespace prusa
// ---------------------------------------------------------------------------------------------------------------------

namespace {

// Réglages de la CORE One (input_shaper_config.hpp) : seuls l'amortissement 0,1 et la réduction 20 servent au calcul
constexpr input_shaper::AxisConfig config { .type = Type::mzv, .frequency = 60.f, .damping_ratio = 0.1f, .vibration_reduction = 20.f };

sf::Pulses make_pulses(Type type, float frequency, const input_shaper::AxisConfig &c) {
    const prusa::Shaper s = prusa::get(c.damping_ratio, frequency, c.vibration_reduction, type);
    sf::Pulses p;
    std::copy_n(s.a, 5, p.a.begin());
    std::copy_n(s.t, 5, p.t.begin());
    p.num_pulses = s.num_pulses;
    return p;
}

std::vector<sf::Point> points(const prusa::Spectrum &psd) {
    std::vector<sf::Point> v;
    for (size_t i = 0; i < psd.size(); ++i) {
        v.push_back({ psd.get(i).frequency, psd.get(i).gain });
    }
    return v;
}

struct Peak {
    float frequency;
    float zeta;
    float height = 1.f;
};

/// Ce que M1959 enregistre : gain² de l'axe (transmissibilité d'une masse sur ressort excitée par déplacement, moins
/// l'excitation : max(gain − 1, 0)), plus le gain² des deux autres axes et un plancher ; bruit multiplicatif en option
prusa::Spectrum spectrum(std::vector<Peak> peaks, float noise = 0.f, uint32_t seed = 1, float cross = 0.02f, float slope = 0.f) {
    prusa::Spectrum psd;
    uint32_t x = seed * 2654435761u + 12345u;
    const auto rnd = [&x] { // [-1, 1)
        x = x * 1664525u + 1013904223u;
        return static_cast<float>(x >> 8) / static_cast<float>(1u << 23) - 1.f;
    };
    for (int f = 30; f < 80; ++f) {
        double gain = 1.;
        for (const Peak &p : peaks) {
            const double r = f / p.frequency;
            const double z2r2 = 4. * p.zeta * p.zeta * r * r;
            gain += p.height * (std::sqrt((1. + z2r2) / ((1. - r * r) * (1. - r * r) + z2r2)) - 1.);
        }
        double g2 = std::max(gain - 1., 0.) * std::max(gain - 1., 0.) + cross * cross + slope * (f - 30);
        g2 *= 1. + noise * rnd();
        psd.samples.push_back(std::max(static_cast<float>(g2), 0.0011f)); // M1959 refuse un point ≤ 0,001
    }
    return psd;
}

struct Case {
    std::string name;
    prusa::Spectrum psd;
};

std::vector<Case> cases() {
    std::vector<Case> c;
    // Pics simples
    for (float f0 : { 32.f, 38.f, 45.f, 52.f, 60.f, 68.f, 76.f }) {
        for (float zeta : { 0.03f, 0.08f, 0.15f }) {
            c.push_back({ "pic " + std::to_string(f0) + " z" + std::to_string(zeta), spectrum({ { f0, zeta } }) });
        }
    }
    // Pics doubles
    c.push_back({ "double 40+55", spectrum({ { 40, 0.06f }, { 55, 0.06f, 0.8f } }) });
    c.push_back({ "double 45+70", spectrum({ { 45, 0.05f }, { 70, 0.08f, 1.2f } }) });
    c.push_back({ "double 50+53", spectrum({ { 50, 0.04f }, { 53, 0.04f } }) });
    c.push_back({ "double 35+78", spectrum({ { 35, 0.1f, 0.5f }, { 78, 0.05f } }) });
    c.push_back({ "double 48+62 egaux", spectrum({ { 48, 0.07f }, { 62, 0.07f } }) });
    c.push_back({ "double 42+44", spectrum({ { 42, 0.03f }, { 44, 0.03f, 0.9f } }) });
    // Bruités
    for (uint32_t seed = 1; seed <= 4; ++seed) {
        c.push_back({ "bruit 10% pic 55 #" + std::to_string(seed), spectrum({ { 55, 0.06f } }, 0.1f, seed) });
        c.push_back({ "bruit 30% double 46+66 #" + std::to_string(seed), spectrum({ { 46, 0.05f }, { 66, 0.07f, 0.7f } }, 0.3f, seed) });
        c.push_back({ "bruit 60% pic 40 #" + std::to_string(seed), spectrum({ { 40, 0.1f } }, 0.6f, seed) });
    }
    // Réalistes (forme des spectres de la CORE One : pic de courroie vers 50-60 Hz, second mode, remontée, bruit)
    c.push_back({ "reel X", spectrum({ { 57, 0.07f }, { 74, 0.12f, 0.25f } }, 0.15f, 11, 0.05f, 0.002f) });
    c.push_back({ "reel Y", spectrum({ { 46, 0.09f }, { 33, 0.2f, 0.3f } }, 0.15f, 12, 0.08f, 0.001f) });
    c.push_back({ "reel courroie molle", spectrum({ { 36, 0.12f } }, 0.2f, 13, 0.1f, 0.003f) });
    c.push_back({ "reel bord 80 Hz", spectrum({ { 82, 0.06f } }, 0.1f, 14, 0.05f) });
    c.push_back({ "reel bord 30 Hz", spectrum({ { 28, 0.08f } }, 0.1f, 15, 0.05f) });
    c.push_back({ "reel large", spectrum({ { 50, 0.3f }, { 65, 0.3f, 0.6f } }, 0.1f, 16, 0.1f, 0.001f) });
    // Limites : plat, plancher seul, une seule raie
    c.push_back({ "plat", prusa::Spectrum { std::vector<float>(50, 0.5f) } });
    c.push_back({ "plancher", prusa::Spectrum { std::vector<float>(50, 0.0011f) } });
    prusa::Spectrum raie { std::vector<float>(50, 0.002f) };
    raie.samples[25] = 3.f;
    c.push_back({ "raie 55 Hz", raie });
    // Tirages : 1 à 3 pics de 25 à 90 Hz, ζ de 0,02 à 0,3, bruit jusqu'à 50 %, plancher et pente variables
    for (uint32_t seed = 1; seed <= 300; ++seed) {
        uint32_t x = seed * 747796405u + 2891336453u;
        const auto u = [&x] { // [0, 1)
            x = x * 1664525u + 1013904223u;
            return static_cast<float>(x >> 8) / static_cast<float>(1u << 24);
        };
        std::vector<Peak> peaks;
        for (int i = 1 + static_cast<int>(u() * 3); i > 0; --i) {
            peaks.push_back({ 25.f + 65.f * u(), 0.02f + 0.28f * u(), 0.2f + 1.3f * u() });
        }
        c.push_back({ "tirage " + std::to_string(seed), spectrum(peaks, 0.5f * u(), seed, 0.1f * u(), 0.004f * u()) });
    }
    return c;
}

const auto no_progress = [](Type, float) { return true; };

} // namespace

TEST_CASE("input shaper : l'estimation float reste dans la tolérance, le calcul exact est celui de Prusa") {
    float worst = 0.f;
    for (const Case &k : cases()) {
        CAPTURE(k.name);
        const std::vector<sf::Point> pts = points(k.psd);
        const sf::Spectrum s = sf::prepare(pts, config);
        for (Type type : { Type::zvd, Type::mzv, Type::ei }) {
            CAPTURE(static_cast<int>(type));
            for (float frequency = 150.f; frequency >= 5.f - 0.01f; frequency -= .2f) {
                const sf::Pulses p = make_pulses(type, frequency, config);
                const prusa::Shaper shaper = prusa::get(config.damping_ratio, frequency, config.vibration_reduction, type);
                float prusa_v = 0.f;
                for (float damping_ratio = 0.05f; damping_ratio <= 0.15f + 0.01f; damping_ratio += 0.05f) {
                    prusa_v = std::max(prusa_v, prusa::remaining_vibrations(shaper, damping_ratio, k.psd, config.vibration_reduction));
                }
                const float exact = sf::vibrations_exact(p, s);
                size_t n = 0;
                const float estimate = sf::vibrations_screen(p, s, std::numeric_limits<float>::infinity(), n);
                CAPTURE(frequency, prusa_v, exact, estimate);
                REQUIRE(exact == prusa_v); // mêmes bits, malgré les points sautés sous le seuil
                worst = std::max(worst, std::abs(estimate - prusa_v));
            }
        }
    }
    // Au pire (phase de 100 rad en float) l'écart reste sous ~2,5e-5 : marge d'au moins 4 ; mesuré bien plus petit
    CHECK(worst * 20.f < sf::screen_tolerance);
    WARN("écart float maximal : " << worst << " (tolérance " << sf::screen_tolerance << ")");
}

TEST_CASE("input shaper : même filtre et même fréquence que Prusa, beaucoup plus vite") {
    using clock = std::chrono::steady_clock;
    clock::duration prusa_time {}, od3d_time {};
    sf::Stats total;
    size_t spectra = 0;
    for (const Case &k : cases()) {
        CAPTURE(k.name);
        const std::vector<sf::Point> pts = points(k.psd);

        const auto t0 = clock::now();
        const input_shaper::AxisConfig expected = prusa::find_best_shaper(no_progress, k.psd, config);
        const auto t1 = clock::now();
        sf::Stats stats;
        const input_shaper::AxisConfig got = sf::find_best_shaper(pts, config, make_pulses, no_progress, &stats);
        const auto t2 = clock::now();
        prusa_time += t1 - t0;
        od3d_time += t2 - t1;

        CAPTURE(static_cast<int>(expected.type), expected.frequency, stats.exact);
        CHECK(got.type == expected.type);
        CHECK(got.frequency == expected.frequency); // même valeur float de la grille de Prusa
        CHECK(got.damping_ratio == expected.damping_ratio);
        CHECK(got.vibration_reduction == expected.vibration_reduction);

        total.screened += stats.screened;
        total.exact += stats.exact;
        total.points += stats.points;
        total.exact_points += stats.exact_points;
        ++spectra;
    }

    const double host_speedup = std::chrono::duration<double>(prusa_time) / std::chrono::duration<double>(od3d_time);
    // Sur l'imprimante (Cortex-M4 à 168 MHz, double logiciel) : ~19 800 cycles par point exact (3 impulsions : exp, sin,
    // cos en double, 2 sqrt, 1 division), ~1 000 par point float (expf, sinf, cosf matériels) ; Prusa calcule
    // 726 × 3 × 3 × 50 points par axe et attend ~0,75 ms dans idle(true) 4 fois par fréquence.
    const double prusa_s = (726. * 3 * 3 * 50 * 19800 / 168e6) + 726. * 3 * 4 * 0.75e-3;
    const double points = double(total.points) / spectra, exact_points = double(total.exact_points) / spectra;
    const double od3d_s = (points * 1000 + exact_points * 19800) / 168e6 + (points + exact_points * 20) / 1024 * 0.75e-3;
    WARN("hôte : Prusa " << std::chrono::duration<double>(prusa_time).count() << " s, OneDrop3D "
                         << std::chrono::duration<double>(od3d_time).count() << " s pour " << spectra << " axes, ×" << host_speedup
                         << " ; par axe : " << double(total.exact) / spectra << " fréquences exactes sur 2178, " << points
                         << " points float, " << exact_points << " points exacts ; imprimante estimée : " << prusa_s << " s → "
                         << od3d_s << " s par axe");
    CHECK(host_speedup > 2.);
}

TEST_CASE("input shaper : abandon pendant le calcul") {
    const prusa::Spectrum psd = spectrum({ { 50, 0.05f } });
    const std::vector<sf::Point> pts = points(psd);
    int calls = 0;
    const auto abort_soon = [&calls](Type, float) { return ++calls < 3; };
    sf::find_best_shaper(pts, config, make_pulses, abort_soon);
    CHECK(calls == 3); // plus aucun appel après le refus
}
