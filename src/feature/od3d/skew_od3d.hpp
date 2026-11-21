/// @file
/// OneDrop3D (F15) : correction d'équerrage XY.
///
/// Prusa a retiré le SKEW_CORRECTION de Marlin (planificateur et M852). Notre correction passe par la conversion
/// des positions natives en positions machine (lib/Marlin/Marlin/src/module/machine_pos_conv.cpp), comme le MBL, et
/// seulement quand le MBL est actif (impressions) : la prise d'origine et les calibrations de Prusa restent sans
/// correction. Facteur XY seulement (pas XZ ni YZ) : x_machine = x - (y - centre du plateau) * k.
///
/// Mesure (pièce de calibration de Marlin, un carré posé à plat) :
///
///       Y
///       ^     B-------C
///       |    /       /
///       |   /       /
///       |  A-------D
///       +-------------> X
///
/// AC et BD sont les diagonales, AD le côté avant (le long de X).
#pragma once

#include <cstdint>

namespace od3d::skew {

/// |k| au plus : 0,01, soit 0,57° (1 mm sur 100 mm). Au-delà, c'est le cadre ou les courroies qu'il faut régler.
inline constexpr float max_factor = 0.01f;

enum class Status : uint8_t {
    ok,
    inconsistent, ///< mesures impossibles ou incohérentes (faute de frappe, mauvais côté)
    too_large, ///< écart au-delà de max_factor
};

struct Result {
    Status status;
    float factor; ///< facteur k (tangente de l'écart), 0 si status != ok
};

/// Facteur XY d'après les diagonales AC, BD et le côté AD (mm), formule de Marlin (_SKEW_FACTOR, planner.h) :
/// AB = sqrt(2·AC² + 2·BD² - 4·AD²) / 2, puis k = tan(90° - acos((AC² - AB² - AD²) / (2·AD·AB))).
/// Refuse les longueurs hors de 10 à 400 mm, un côté AB déduit hors de 0,5 à 2 fois AD et |k| > max_factor.
Result factor_from_measures(float ac, float bd, float ad);

/// Écart à l'équerre en degrés (positif : B penche vers +X)
float factor_to_degrees(float factor);

/// Facteur acceptable (fini et |k| <= max_factor)
bool is_valid_factor(float factor);

// ---- Firmware seulement (M852_od3d.cpp), fil de Marlin ----

/// Applique k sans déplacer la tête (la position machine reste, la position native est recalculée), sans l'enregistrer
void set_factor(float factor);

/// Au démarrage : applique le facteur enregistré dans config_store (ignoré s'il n'est pas valide)
void load();

} // namespace od3d::skew
