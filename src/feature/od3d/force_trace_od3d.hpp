/// @file
/// OneDrop3D (F4, puis F5, F6, F9) : trace de force de la cellule de charge, partie pure (sans firmware), testée dans
/// tests/unit/feature/od3d. Une trace est une suite d'échantillons à pas de temps constant : force relative au premier
/// échantillon, position X de la tête et position du moteur E (PA compris) au même instant, en entiers de 16 bits
/// (6 octets par point). L'enregistrement (interruption de la cellule de charge) est dans force_probe_od3d.hpp.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace od3d::force {

/// Unités des points : 0,25 g (±8 191 g), 0,01 mm (±327 mm). La force était en 0,02 g (±655 g) jusqu'à la 8.0.2.20 :
/// la poussée de l'extrudeur dépasse 655 g dès le départ (relevé OD3D_PA3.CSV du 22/11, saturé à −655 g sur toute la
/// ligne, d'où « 0 créneau ») ; 0,2 g (±6 553 g) en 8.0.2.21 ; 0,25 g ici : dans le vide à 14 mm³/s, PrusaPATuner voit
/// des paliers de plusieurs kilogrammes. Le bruit de la cellule (±15 g) reste bien au-dessus du pas.
inline constexpr float force_unit_g = 0.25f;
inline constexpr float x_unit_mm = 0.01f;
inline constexpr float e_unit_mm = 0.01f;

struct TracePoint {
    int16_t force; ///< force relative au premier échantillon de la trace [force_unit_g]
    int16_t x; ///< position X relative au premier échantillon [x_unit_mm]
    int16_t e; ///< position du moteur E relative au premier échantillon [e_unit_mm] (avance du PA comprise)
};
static_assert(sizeof(TracePoint) == 6);

using Trace = std::span<const TracePoint>;

/// Conversion bornée vers les unités des points
TracePoint make_point(float force_g, float x_mm, float e_mm = 0.f);

inline float force_g(const TracePoint &p) {
    return float(p.force) * force_unit_g;
}
inline float x_mm(const TracePoint &p) {
    return float(p.x) * x_unit_mm;
}
inline float e_mm(const TracePoint &p) {
    return float(p.e) * e_unit_mm;
}

} // namespace od3d::force
