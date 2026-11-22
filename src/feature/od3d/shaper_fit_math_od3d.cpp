/// @file
/// OneDrop3D : voir shaper_fit_math_od3d.hpp. Les fonctions marquées « copie de Prusa » reprennent M958.cpp à
/// l'identique (même ordre des opérations, donc mêmes bits) : ne pas les « améliorer ».
#include "shaper_fit_math_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <optional>

namespace od3d::shaper_fit {

namespace {

    using input_shaper::Type;

    // Grilles de Prusa (M958.cpp, fit_shaper), parcourues avec les mêmes additions en float
    constexpr float epsilon = 0.01f;
    constexpr float start_frequency = 5.f;
    constexpr float end_frequency = 150.f;
    constexpr float frequency_step = .2f;

    /// Grille grossière (2 Hz) puis ±2 Hz autour de son meilleur point au pas fin : un premier majorant U
    constexpr int coarse_stride = 10;

    /// Rendre la main (affichage, abandon, chauffe) tous les ~10 ms de calcul sur l'imprimante, au lieu de 4 fois par
    /// fréquence chez Prusa (chaque appel attend 1 ms dans idle(true))
    constexpr size_t progress_work = 2048;
    /// Un point exact (double logiciel : exp, sin, cos, sqrt) coûte ~20 points float
    constexpr size_t exact_work = 20;

    struct Fit {
        float frequency;
        float score;
        float smoothing;
    };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion" // copies de Prusa (compilées sans cet avertissement dans Marlin)

    /// Copie de Prusa
    double get_inv_D(const Pulses &shaper) {
        double d = 0.;
        for (int i = 0; i < shaper.num_pulses; ++i) {
            d += static_cast<double>(shaper.a[i]);
        }
        return 1. / d;
    }

    /// Copie de Prusa (sq() de Marlin développé)
    double vibration_reduction(const Pulses &shaper, float system_damping_ratio, float frequency) {
        const double inv_D = get_inv_D(shaper);
        const double omega = 2. * std::numbers::pi_v<double> * static_cast<double>(frequency);
        const double damping = static_cast<double>(system_damping_ratio) * omega;
        const double omega_d = omega * sqrt(1. - ((static_cast<double>(system_damping_ratio)) * (static_cast<double>(system_damping_ratio))));

        double s = 0.;
        double c = 0.;

        for (int i = 0; i < shaper.num_pulses; ++i) {
            const double t_i = static_cast<double>(shaper.t[i]);
            const double w = shaper.a[i] * exp(-damping * (static_cast<double>(shaper.t[shaper.num_pulses - 1]) - t_i));
            s += w * sin(omega_d * t_i);
            c += w * cos(omega_d * t_i);
        }
        return (sqrt(((s) * (s)) + ((c) * (c))) * inv_D);
    }

    /// remaining_vibrations de Prusa, seuil et somme totale calculés une fois (mêmes valeurs). Les points sous le seuil
    /// sont sautés : chez Prusa ils ajoutent max(g·vr − seuil, 0) = +0, car vr ≤ 1 (aᵢ > 0, e^(−x) ≤ 1 pour x ≥ 0).
    float remaining_vibrations(const Pulses &shaper, float system_damping_ratio, std::span<const Point> psd, float vibr_threshold, float all_vibrations_sum) {
        float remaining_vibrations_sum = 0.f;
        for (const Point &fg : psd) {
            if (fg.gain <= vibr_threshold) {
                continue;
            }
            const float vibration = std::max(fg.gain * static_cast<float>(vibration_reduction(shaper, system_damping_ratio, fg.frequency)) - vibr_threshold, 0.f);
            remaining_vibrations_sum += vibration;
        }
        return (remaining_vibrations_sum / all_vibrations_sum);
    }

    /// Copie de Prusa (smoothing)
    float smoothing(const Pulses &shaper) {
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
            offset_180 += a_i * half_accel * ((t_i - ts) * (t_i - ts));
        }
        offset_90 *= (inv_D * sqrt(2.));
        offset_180 *= inv_D;

        return static_cast<float>(std::max(offset_90, offset_180));
    }

#pragma GCC diagnostic pop

    /// fit_shaper de Prusa en mode find_best_result : première fréquence (de 150 à 5 Hz) où V est minimal.
    /// \returns nullopt sur abandon
    std::optional<Fit> fit_shaper(Type type, const Spectrum &s, const input_shaper::AxisConfig &config, MakePulses make_pulses, const Progress &progress, Stats &stats) {
        const auto pulses_at = [&](float frequency) {
            return make_pulses(type, frequency, config);
        };
        size_t work = 0;
        const auto breathe = [&](size_t points, float progress_ratio) {
            work += points;
            if (work < progress_work) {
                return true;
            }
            work = 0;
            return progress(type, progress_ratio);
        };
        const auto screen = [&](const Pulses &shaper, float limit) {
            const size_t before = stats.points;
            const float v = vibrations_screen(shaper, s, limit, stats.points);
            ++stats.screened;
            return std::pair { v, stats.points - before };
        };

        // Premier majorant U du minimum : grille de 2 Hz, puis pas fin autour de son meilleur point
        float upper = std::numeric_limits<float>::infinity();
        int coarse_best = 0;
        for (int pass = 0; pass < 2; ++pass) {
            int k = 0;
            const int best_k = coarse_best;
            for (float frequency = end_frequency; frequency >= start_frequency - epsilon; frequency -= frequency_step, ++k) {
                const bool wanted = pass == 0 ? k % coarse_stride == 0 : std::abs(k - best_k) < coarse_stride && k != best_k;
                if (!wanted) {
                    continue;
                }
                const auto [v, points] = screen(pulses_at(frequency), upper);
                if (v + screen_tolerance < upper) {
                    upper = v + screen_tolerance;
                    coarse_best = k;
                }
                if (!breathe(points, 0.f)) {
                    return std::nullopt;
                }
            }
        }

        // Parcours de Prusa ; le calcul exact seulement si la fréquence peut atteindre le minimum (Ve − tol ≤ U) et
        // battre la meilleure déjà trouvée (Prusa ne la remplace que si V est strictement plus petit)
        float best_frequency = 0.f;
        float best_vibrs = std::numeric_limits<float>::max();
        for (float frequency = end_frequency; frequency >= start_frequency - epsilon; frequency -= frequency_step) {
            const float progress_ratio = (end_frequency - frequency) / (end_frequency - start_frequency);
            const Pulses shaper = pulses_at(frequency);
            const auto [v, points] = screen(shaper, std::min(upper, best_vibrs));
            upper = std::min(upper, v + screen_tolerance);
            size_t work_done = points;
            if (v - screen_tolerance <= upper && v - screen_tolerance < best_vibrs) {
                ++stats.exact;
                stats.exact_points += points;
                const float shaper_vibrations = vibrations_exact(shaper, s);
                work_done += points * exact_work;
                if (shaper_vibrations < best_vibrs) {
                    best_vibrs = shaper_vibrations;
                    best_frequency = frequency;
                    if (best_vibrs == 0.f) {
                        break; // V ≥ 0 : plus rien ne peut être strictement plus petit
                    }
                }
            }
            if (!breathe(work_done, progress_ratio)) {
                return std::nullopt;
            }
        }

        if (best_vibrs == std::numeric_limits<float>::max()) {
            return std::nullopt; // impossible avec un spectre valide (M1959 : gains finis > 0,001) ; évite get(0) → BSOD
        }

        /// The score trying to minimize vibrations, but also accounting the growth of smoothing (Prusa)
        const float shaper_smoothing = smoothing(pulses_at(best_frequency));
        const float shaper_score = shaper_smoothing * (std::pow(best_vibrs, 1.5f) + best_vibrs * .2f + .01f);
        return Fit { .frequency = best_frequency, .score = shaper_score, .smoothing = shaper_smoothing };
    }

} // namespace

Spectrum prepare(std::span<const Point> psd, const input_shaper::AxisConfig &default_config) {
    float max_gain = psd.front().gain;
    float all = 0.f;
    for (const Point &fg : psd) {
        max_gain = std::max(max_gain, fg.gain);
        all += std::max(fg.gain, 0.f);
    }
    return { .psd = psd, .threshold = max_gain / default_config.vibration_reduction, .all = all };
}

float vibrations_exact(const Pulses &shaper, const Spectrum &s) {
    // Prusa : le pire des 3 amortissements supposés (0,05, 0,1, 0,15)
    float shaper_vibrations = 0.f;
    for (float damping_ratio = 0.05f; damping_ratio <= 0.15f + epsilon; damping_ratio += 0.05f) {
        float vibrations = remaining_vibrations(shaper, damping_ratio, s.psd, s.threshold, s.all);
        if (vibrations > shaper_vibrations) {
            shaper_vibrations = vibrations;
        }
    }
    return shaper_vibrations;
}

/// Même formule en float : |écart| ≤ screen_tolerance (V est une moyenne pondérée de vr, juste à ~1e-5 en float, la
/// phase ω·t allant jusqu'à 100 rad). Les sommes partielles ne font que croître : dès que l'une dépasse limit + tol,
/// la fréquence ne peut plus servir, et l'on renvoie +∞ sans finir.
float vibrations_screen(const Pulses &shaper, const Spectrum &s, float limit, size_t &points) {
    float sum_a = 0.f;
    for (int i = 0; i < shaper.num_pulses; ++i) {
        sum_a += shaper.a[i];
    }
    const float inv_D = 1.f / sum_a;
    const float t_last = shaper.t[shaper.num_pulses - 1];
    float shaper_vibrations = 0.f;
    for (float damping_ratio = 0.05f; damping_ratio <= 0.15f + epsilon; damping_ratio += 0.05f) {
        const float df = std::sqrt(1.f - damping_ratio * damping_ratio);
        float sum = 0.f;
        for (const Point &fg : s.psd) {
            if (fg.gain <= s.threshold) {
                continue;
            }
            ++points;
            const float omega = 2.f * std::numbers::pi_v<float> * fg.frequency;
            const float damping = damping_ratio * omega;
            const float omega_d = omega * df;
            float sn = 0.f;
            float cs = 0.f;
            for (int i = 0; i < shaper.num_pulses; ++i) {
                const float w = shaper.a[i] * std::exp(-damping * (t_last - shaper.t[i]));
                sn += w * std::sin(omega_d * shaper.t[i]);
                cs += w * std::cos(omega_d * shaper.t[i]);
            }
            sum += std::max(fg.gain * (std::sqrt(sn * sn + cs * cs) * inv_D) - s.threshold, 0.f);
            if (sum / s.all - screen_tolerance > limit) {
                return std::numeric_limits<float>::infinity();
            }
        }
        shaper_vibrations = std::max(shaper_vibrations, sum / s.all);
    }
    return shaper_vibrations;
}

input_shaper::AxisConfig find_best_shaper(std::span<const Point> psd, const input_shaper::AxisConfig &default_config,
    MakePulses make_pulses, const Progress &progress, Stats *stats) {
    Stats local_stats;
    Stats &st = stats ? *stats : local_stats;

    const Spectrum s = prepare(psd, default_config);

    // Copie de Prusa (find_best_shaper)
    struct Best_score {
        Fit result;
        Type type;
    };
    std::optional<Best_score> best_shaper;

    for (Type shaper_type = Type::first; shaper_type <= Type::last; ++shaper_type) {
        if (shaper_type == Type::null || !std::ranges::contains(input_shaper::filter_list, shaper_type)) {
            continue;
        }

        if (!progress(shaper_type, 0.0f)) {
            break;
        }

        const std::optional<Fit> shaper = fit_shaper(shaper_type, s, default_config, make_pulses, progress, st);
        if (!shaper) {
            break;
        }

        if (
            !best_shaper.has_value() //
            || shaper->score * 1.2f < best_shaper->result.score
            || ((shaper->score * 1.05f < best_shaper->result.score) && (shaper->smoothing * 1.1f < best_shaper->result.smoothing)) //
        ) {
            best_shaper = {
                .result = *shaper,
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

} // namespace od3d::shaper_fit
