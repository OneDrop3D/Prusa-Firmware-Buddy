/// @file
/// OneDrop3D (F4, F5, F6) : géométrie des motifs de mesure imprimés sur le plateau, partie pure (sans firmware), testée
/// dans tests/unit/feature/od3d.
///
/// Les mesures par la cellule de charge se font en imprimant des lignes à hauteur de première couche : le plastique
/// écrasé entre la buse et le plateau repousse la buse vers le haut, avec une force qui suit la pression dans la buse
/// (dans le vide, la poussée de l'extrudeur et la contre-pression de la buse se compensent presque dans l'ensemble
/// pesé par la cellule). Les lignes sont parallèles à X, de l'avant du plateau vers l'arrière ; chaque ligne de F4 porte
/// à sa droite des traits qui donnent son numéro (un sur cinq plus long), pour choisir la meilleure ligne à l'œil.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace od3d::pattern {

inline constexpr float filament_area_mm2 = 2.4053f; ///< filament de 1,75 mm

/// Rectangle du plateau [mm], x0 ≤ x1 et y0 ≤ y1
struct Rect {
    float x0 = 0;
    float y0 = 0;
    float x1 = 0;
    float y1 = 0;

    float width() const { return x1 - x0; }
    float height() const { return y1 - y0; }
    bool contains(const Rect &r) const { return r.x0 >= x0 && r.x1 <= x1 && r.y0 >= y0 && r.y1 <= y1; }
    /// Agrandi de dx en X et de dy en Y de chaque côté
    Rect grown(float dx, float dy) const { return { x0 - dx, y0 - dy, x1 + dx, y1 + dy }; }
    /// Partie commune (vide : largeur ou hauteur ≤ 0)
    Rect clipped(const Rect &to) const;
    bool empty() const { return !(x1 > x0) || !(y1 > y0); }
};

/// Section d'un cordon imprimé [mm]
struct Bead {
    float width;
    float height;
};

/// Filament par mm de ligne [mm/mm]
float e_per_mm(const Bead &b);

/// Vitesse de la tête pour un débit [mm³/s] → [mm/s] ; 0 si la section est nulle
float speed_for_flow(const Bead &b, float flow);

/// Cordon plausible pour une buse de 0,4 à 0,8 mm à hauteur de première couche
bool is_valid(const Bead &b);

// ---- Repères des lignes (F4) ----

/// Un trait perpendiculaire à la ligne, à dx du premier trait
struct Tick {
    float dx;
    float length;
};

inline constexpr float tick_pitch = 1.5f; ///< écart entre deux traits [mm]
inline constexpr float tick_short = 2.0f;
inline constexpr float tick_long = 3.5f; ///< un trait sur cinq
inline constexpr float tick_gap = 4.0f; ///< entre la fin de la ligne et le premier trait

/// Traits qui donnent le numéro de la ligne (1 : un trait ; 7 : cinq traits dont le 5e long, puis deux) ; rend le nombre
/// écrit dans out
size_t tally(uint8_t number, std::span<Tick> out);

/// Place prise en X par les traits du numéro, écart compris (0 sans trait)
float tally_width(uint8_t number);

// ---- Lignes ----

/// count lignes parallèles à X, de l'avant (y0) vers l'arrière, espacées de pitch ; chaque ligne commence au plus à x0 et
/// fait au plus length, plus marks de repères à sa droite
struct Lines {
    float x0 = 0;
    float y0 = 0;
    float pitch = 5.f;
    float length = 0;
    float marks = 0;
    uint8_t count = 0;
    Bead bead { 0.45f, 0.2f };

    float y(uint8_t i) const { return y0 + pitch * float(i); }
    /// Surface couverte (cordons et traits compris)
    Rect area() const;
};

/// Rectangle palpé avant le motif : la surface du motif agrandie de grow (comme Prusa agrandit la zone d'impression
/// d'un point majeur de la grille), bornée au plateau
Rect probe_rect(const Rect &pattern, float grow_x, float grow_y, const Rect &bed);

} // namespace od3d::pattern
