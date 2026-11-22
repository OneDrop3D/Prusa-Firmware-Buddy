/// @file
/// OneDrop3D (F4, puis F5, F6, F9) : trace de force de la cellule de charge, partie pure (sans firmware), testée dans
/// tests/unit/feature/od3d. Une trace est une suite d'échantillons à pas de temps constant : force relative au premier
/// échantillon et position X de la tête, en entiers de 16 bits (4 octets par point : 1 536 points = 6 Ko).
/// L'enregistrement (interruption de la cellule de charge) est dans force_probe_od3d.hpp.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace od3d::force {

/// Unités des points : 0,2 g (±6 553 g) et 0,01 mm (±327 mm). La force était en 0,02 g (±655 g) jusqu'à la 8.0.2.20 :
/// en imprimant, la poussée de l'extrudeur dépasse 655 g dès le départ (relevé OD3D_PA3.CSV du 22/11, saturé à −655 g
/// sur toute la ligne, d'où « 0 créneau »). Le bruit de la cellule (±15 g) reste bien au-dessus du pas.
inline constexpr float force_unit_g = 0.2f;
inline constexpr float x_unit_mm = 0.01f;

struct TracePoint {
    int16_t force; ///< force relative au premier échantillon de la trace [force_unit_g]
    int16_t x; ///< position X relative au premier échantillon [x_unit_mm]
};
static_assert(sizeof(TracePoint) == 4);

using Trace = std::span<const TracePoint>;

/// Conversion bornée vers les unités des points
TracePoint make_point(float force_g, float x_mm);

inline float force_g(const TracePoint &p) {
    return float(p.force) * force_unit_g;
}
inline float x_mm(const TracePoint &p) {
    return float(p.x) * x_unit_mm;
}

/// Force lissée sur 3 points (moyenne centrée, bords compris) [g]
float force_at(Trace t, size_t i);

/// Vitesse de la tête le long de X, en valeur absolue, par différence centrée sur ±2 points [mm/s]
float speed_at(Trace t, size_t i, float dt);

/// Moyenne de la force lissée sur [i0, i1) [g] ; NAN si l'intervalle est vide
float mean_force(Trace t, size_t i0, size_t i1);

/// Écart type de la force brute autour de la droite qui joint les moyennes des deux moitiés de [i0, i1) [g]
/// (bruit, sans la dérive lente) ; NAN avec moins de 4 points
float noise_force(Trace t, size_t i0, size_t i1);

/// Front de vitesse : passage de la vitesse au-dessus (montée) ou au-dessous (descente) du seuil, confirmé par
/// min_run points consécutifs du même côté
struct Edge {
    uint16_t index;
    bool rising;
};

/// Trouve les fronts de la vitesse autour du seuil threshold [mm/s] ; rend le nombre de fronts écrits dans out
size_t find_edges(Trace t, float dt, float threshold, uint8_t min_run, std::span<Edge> out);

} // namespace od3d::force
