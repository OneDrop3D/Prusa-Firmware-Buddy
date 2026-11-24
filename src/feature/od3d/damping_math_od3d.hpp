/// @file
/// OneDrop3D (F33, vecteur n° 03 du document v3) : amortissement réel ζ et facteur de qualité Q d'un axe, d'après sa
/// réponse en fréquence mesurée par l'accéléromètre — calculs purs, testés dans tests/unit/feature/od3d/damping_test.cpp.
///
/// Méthode de la bande à −3 dB (demi-puissance) sur le carré du gain : sommet f0 du pic principal, puis les deux
/// fréquences où le gain² retombe à la moitié du sommet ; Δf = f_droite − f_gauche, Q = f0 / Δf, ζ ≈ 1 / (2Q).
/// Valable pour un amortissement faible (ζ < 0,3). L'excitation de Prusa (M958) impose un déplacement et non une force :
/// la hauteur du pic ne donne pas ζ (commentaire de Prusa dans M958.cpp), sa largeur si.
#pragma once

#include <cstddef>
#include <span>

namespace od3d::damping {

/// Balayage de M1997, par axe : 30 à 90 Hz par pas de 0,5 Hz (Prusa : 30 à 80 Hz par pas de 1 Hz). Le pas fin garde
/// au moins 6 points dans la bande à −3 dB dès ζ = 0,03 à 50 Hz ; la plage couvre les pics jusque vers 80 Hz.
inline constexpr float sweep_start_hz = 30.f;
inline constexpr float sweep_step_hz = 0.5f;
inline constexpr size_t sweep_points = 121;
inline constexpr float sweep_end_hz = sweep_start_hz + (sweep_points - 1) * sweep_step_hz;
static_assert(sweep_end_hz == 90.f);

struct Resonance {
    bool found = false; ///< un sommet hors des bords et ses deux coupures à −3 dB dans le balayage
    float frequency = 0.f; ///< f0, Hz (sommet de la parabole passant par les trois points les plus hauts)
    float width = 0.f; ///< Δf à −3 dB, Hz (coupures interpolées linéairement)
    float q = 0.f; ///< f0 / Δf
    float zeta = 0.f; ///< Δf / (2 f0)
};

/// Pic principal (le plus haut) de psd : psd[i] = gain² mesuré à start_hz + i × step_hz, tous les points mesurés
Resonance analyse(std::span<const float> psd, float start_hz, float step_hz);

/// Amortissements que l'écran propose d'appliquer à l'input shaper : ceux pour lesquels Prusa calcule ses filtres
/// (M958.cpp, fit_shaper). Au-delà, la mesure est douteuse (deux pics confondus, couplage CoreXY qui élargit le pic,
/// accéléromètre mal fixé) : un filtre conçu pour 0,25 quand le vrai ζ vaut 0,05 laisse ~17 % de vibration au lieu
/// de ~4 % (V-02, jusqu'ici 0,02 à 0,25)
inline constexpr float min_zeta = 0.05f;
inline constexpr float max_zeta = 0.15f;

constexpr bool applicable(const Resonance &r) {
    return r.found && r.zeta >= min_zeta && r.zeta <= max_zeta;
}

} // namespace od3d::damping
