/// @file
/// OneDrop3D : choix du meilleur filtre de l'input shaper (M1959) beaucoup plus rapide, avec exactement le même
/// résultat que Prusa (`find_best_shaper` de M958.cpp, mode `find_best_result`) — calculs purs, testés dans
/// tests/unit/feature/od3d/shaper_fit_test.cpp contre une copie du code de Prusa.
///
/// Prusa essaie chaque type (ZVD, MZV, EI) sur 726 fréquences (150 → 5 Hz par pas de 0,2 Hz) et, pour chacune, calcule
/// les vibrations restantes V(f) pour 3 amortissements et 50 points du spectre, en double (logiciel sur le Cortex-M4) :
/// exp, sin et cos par impulsion, soit ~3 millions d'appels par axe. Il garde la première fréquence (en descendant)
/// de V minimal.
///
/// Ici : V(f) est d'abord estimé en float (Ve, écart |Ve − V| ≤ `screen_tolerance`, vérifié par le test), et le
/// calcul exact de Prusa n'est refait que là où Ve laisse une chance d'égaler le minimum : Ve(f) − tol ≤ U, où U est
/// un majorant du minimum (Ve + tol du meilleur point déjà estimé, d'abord sur une grille grossière de 2 Hz affinée).
/// Toute fréquence qui atteint le minimum exact passe ce filtre, et le parcours garde l'ordre et la comparaison stricte
/// de Prusa : la fréquence choisie est la sienne, exactement. L'estimation s'arrête dès qu'une somme partielle exclut la
/// fréquence, et seuls les points du spectre au-dessus du seuil de Prusa (max / 20) sont calculés : les autres comptent
/// pour 0 chez lui aussi. Sur l'imprimante : ~45 s par axe chez Prusa, ~0,3 s ici (estimation, voir le test).
#pragma once

#include <array>
#include <cstddef>
#include <span>

#include <inplace_function.hpp>
#include <Marlin/src/feature/input_shaper/input_shaper_config.hpp>

namespace od3d::shaper_fit {

/// Un point du spectre mesuré (M1959 : gain² de 30 à 79 Hz, tous > 0,001)
struct Point {
    float frequency;
    float gain;
};

/// Impulsions d'un filtre, copiées de `input_shaper::Shaper` (mêmes noms de champs que chez Prusa)
struct Pulses {
    std::array<float, 5> a {};
    std::array<float, 5> t {};
    int num_pulses = 0;
};

/// `input_shaper::get()` de Prusa (input_shaper.cpp n'est pas compilable hors de l'imprimante)
using MakePulses = Pulses (*)(input_shaper::Type type, float frequency, const input_shaper::AxisConfig &config);

/// Avancement 0..1 du type en cours ; false = abandon demandé
using Progress = stdext::inplace_function<bool(input_shaper::Type type, float progress)>;

/// Écart maximal admis entre l'estimation float de V et la valeur de Prusa (V est entre 0 et 1) : au pire ~2,5e-5
/// (phase ω·t jusqu'à 100 rad en float), mesuré à 3e-7 par le test. Plus large ne coûte que des calculs exacts de plus.
inline constexpr float screen_tolerance = 1e-4f;

/// Compteurs pour le test (estimation du temps sur l'imprimante)
struct Stats {
    size_t screened = 0; ///< V estimés (fréquence × type)
    size_t exact = 0; ///< V refaits avec le calcul de Prusa
    size_t points = 0; ///< points du spectre calculés en float (fréquence × type × amortissement)
    size_t exact_points = 0; ///< points du spectre calculés en double (calcul de Prusa)
};

/// Spectre préparé comme chez Prusa (remaining_vibrations)
struct Spectrum {
    std::span<const Point> psd;
    float threshold; ///< psd.max() / default_vibration_reduction
    float all; ///< somme des max(gain, 0)
};
Spectrum prepare(std::span<const Point> psd, const input_shaper::AxisConfig &default_config);

/// V(f) de Prusa pour un filtre : pire des 3 amortissements des vibrations restantes (calcul exact, double)
float vibrations_exact(const Pulses &shaper, const Spectrum &s);

/// Estimation float de vibrations_exact, à screen_tolerance près, ou +∞ dès qu'elle dépasse sûrement limit + tol ;
/// ajoute à points le nombre de points calculés
float vibrations_screen(const Pulses &shaper, const Spectrum &s, float limit, size_t &points);

/// Même résultat que `find_best_shaper(progress_hook, psd, default_config)` de Prusa (M958.cpp).
/// En cas d'abandon (progress renvoie false), le résultat n'a pas de sens, comme chez Prusa.
input_shaper::AxisConfig find_best_shaper(std::span<const Point> psd, const input_shaper::AxisConfig &default_config,
    MakePulses make_pulses, const Progress &progress, Stats *stats = nullptr);

} // namespace od3d::shaper_fit
