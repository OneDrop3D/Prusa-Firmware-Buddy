/// @file
/// OneDrop3D (F14) : tension des courroies mesurée par l'accéléromètre — calculs purs, sans Marlin ni affichage,
/// testés dans tests/unit/feature/od3d/belt_tension_test.cpp.
///
/// - recherche du pic d'un spectre (proéminence, interpolation parabolique) ;
/// - verdict et conseil de réglage : la logique du réglage manuel des courroies de Prusa
///   (src/feature/manual_belt_tuning/manual_belt_tuning_wizard.cpp), reprise telle quelle, constantes comprises
///   (elles viennent de manual_belt_tuning_config.hpp côté firmware) ;
/// - géométrie des jauges et de la courbe du spectre.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace od3d::belts {

/// Sommet d'un spectre
struct Peak {
    bool found = false; ///< un pic assez net a été trouvé dans la fenêtre
    float frequency = 0.f; ///< Hz, interpolé entre les points de mesure
    float height = 0.f; ///< valeur du spectre au point le plus haut
    float relative_prominence = 0.f; ///< proéminence / hauteur, de 0 à 1
};

/// Décalage du sommet de la parabole qui passe par trois points équidistants, en pas, dans [-0.5, 0.5]
/// (0 si les trois points ne forment pas un sommet)
float parabolic_offset(float left, float centre, float right);

/// Le pic le plus proéminent du spectre values (values[i] mesuré à start_hz + i * step_hz) dont le sommet est entre
/// window_min_hz et window_max_hz. Proéminence : hauteur du sommet au-dessus du plus haut des deux creux qui le
/// séparent d'un point plus haut (ou du bord). Le pic est « trouvé » si proéminence / hauteur >= min_relative.
Peak find_peak(std::span<const float> values, float start_hz, float step_hz, float window_min_hz, float window_max_hz, float min_relative = 0.1f);

/// Cibles de Prusa (manual_belt_tuning_config.hpp ; CORE One : 95,5 et 92 Hz, plage 89 à 98 Hz, écart 8 Hz,
/// 15 et 12 Hz par tour de vis)
struct Targets {
    float higher_optimal; ///< fréquence optimale de la courroie la plus tendue
    float lower_optimal; ///< fréquence optimale de l'autre
    float result_min; ///< la plus basse des deux doit être au moins à cette fréquence
    float result_max; ///< la plus haute des deux doit être au plus à cette fréquence
    float diff_limit; ///< au-delà de cet écart entre les deux, le portique est sans doute de travers
    float hz_per_rev; ///< variation de fréquence d'une courroie par tour de sa propre vis
    float hz_per_rev2; ///< variation de fréquence d'une courroie par tour de la vis de l'autre
};

/// Position d'une fréquence par rapport à la plage de Prusa
enum class Zone : uint8_t {
    low,
    ok,
    high,
};
Zone zone(float frequency, const Targets &targets);

enum class Verdict : uint8_t {
    ok, ///< dans la plage (ou moins d'1/8 de tour à faire) : rien à tourner
    adjust, ///< tourner les deux vis de turn_eighths huitièmes de tour (> 0 : serrer, < 0 : desserrer)
    misaligned, ///< écart trop grand entre les deux courroies : vérifier l'alignement du portique (Prusa)
};

struct Advice {
    Verdict verdict;
    int8_t turn_eighths; ///< signé, en huitièmes de tour, comme screw_revs::turn_eights de Prusa
};

/// Formule de Prusa (calc_revs_from_freq) : tours de la première vis pour corriger df1 et df2 (optimal - mesuré)
float revs_from_freq(float df1, float df2, float k1, float k2);

/// Verdict et conseil d'après les fréquences des deux courroies (l'ordre A/B ne compte pas : comme Prusa, la plus haute
/// est comparée à la cible haute et la plus basse à la cible basse)
Advice advise(float freq_a, float freq_b, const Targets &targets);

/// Nombre de tours lisible, valeur absolue : « 3/8 », « 1 », « 1 3/8 » ; ASCII seulement
void format_turns(int eighths, char *out, size_t size);

/// Abscisse (0 à width - 1) de frequency sur une jauge allant de lo à hi Hz, bornée
int16_t gauge_x(float frequency, float lo, float hi, int16_t width);

/// Courbe du spectre sur un tracé de w × h pixels : hauteur (0 = en bas, h - 1 = en haut) de la colonne x, par
/// interpolation linéaire entre les valid premiers points de values, à l'échelle scale (valeur du haut du tracé).
/// -1 si la colonne est au-delà des points déjà mesurés (rien à dessiner).
int16_t curve_height(std::span<const float> values, size_t valid, int x, int w, int h, float scale);

/// Abscisse (0 à w - 1) de la fréquence frequency sur un tracé de w pixels allant de start_hz à end_hz
int16_t plot_x(float frequency, float start_hz, float end_hz, int w);

} // namespace od3d::belts
