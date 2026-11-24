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

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
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

/// Point du plateau [mm]
struct Point {
    float x;
    float y;
};

/// count lignes parallèles à X, de l'avant (y0) vers l'arrière, espacées de pitch ; chaque ligne couvre au plus x0 à
/// x0 + length, plus marks de repères à sa droite. Lignes dans les deux sens (demande du 22/11) : la ligne i + 1 part
/// d'où la ligne i s'arrête (paires vers +X, impaires vers −X), sans retour à gauche entre deux lignes.
/// along_y (mesure automatique, demande du 24/11 : un bloc qui ne tient pas couché peut tenir debout) : motif tourné
/// de 90°, X et Y échangés autour de (x0, y0) ; les fonctions ci-dessous restent dans le repère des lignes parallèles
/// à X et at() donne le point du plateau
struct Lines {
    float x0 = 0;
    float y0 = 0;
    float pitch = 5.f;
    float length = 0;
    float marks = 0;
    uint8_t count = 0;
    Bead bead { 0.45f, 0.2f };
    bool along_y = false;

    /// Point du plateau pour (px, py) du repère des lignes parallèles à X
    Point at(float px, float py) const { return along_y ? Point { x0 + (py - y0), y0 + (px - x0) } : Point { px, py }; }
    float y(uint8_t i) const { return y0 + pitch * float(i); }
    bool reversed(uint8_t i) const { return i % 2 == 1; }
    /// Sens de la ligne i le long de X : 1 ou −1
    float dir(uint8_t i) const { return reversed(i) ? -1.f : 1.f; }
    /// Départ de la ligne i en X (sa fin : x_start(i) + dir(i)·length)
    float x_start(uint8_t i) const { return reversed(i) ? x0 + length : x0; }
    /// Surface couverte sur le plateau (cordons et traits compris)
    Rect area() const;
};

/// Rectangle palpé avant le motif : la surface du motif agrandie de grow (comme Prusa agrandit la zone d'impression
/// d'un point majeur de la grille), bornée au plateau
Rect probe_rect(const Rect &pattern, float grow_x, float grow_y, const Rect &bed);

/// Les rectangles se recouvrent (bords qui se touchent : non)
constexpr bool overlaps(const Rect &a, const Rect &b) {
    return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1;
}

// ---- Place des motifs d'une série (retours du 23/11 : « il a imprimé plusieurs fois sur la même zone », « il faut
// décaler le plateau ») ----

/// Distribue sur le plateau les motifs et les lignes de purge d'une série, sans chevauchement : rangées de l'avant vers
/// l'arrière, de gauche à droite dans une rangée (une rangée a la hauteur de son plus haut rectangle), chaque rectangle
/// à gap au moins des autres et des zones interdites (keep_out : points que la tête palpe encore pendant la série).
/// Plus de place : nullopt, sans rien changer (on repart de zéro avec un nouvel objet, plateau nettoyé).
class Shelf {
public:
    static constexpr size_t max_keep_out = 2;

    Shelf() = default;
    Shelf(const Rect &zone, std::span<const Rect> keep_out, float gap);

    std::optional<Rect> place(float w, float h);

private:
    Rect zone_ {};
    std::array<Rect, max_keep_out> keep_out_ {}; ///< agrandies de gap ; vides : aucune
    float gap_ = 0;
    float x_ = 0; ///< début de la place libre dans la rangée
    float y_ = 0; ///< avant de la rangée
    float row_ = 0; ///< hauteur de la rangée
};

/// Rétraction des motifs avant un déplacement à vide ou une levée [mm de filament], à retract_speed. shortcut: le
/// firmware ne connaît pas la rétraction des profils PrusaSlicer (elle est dans le G-code tranché) et les préréglages
/// du fork n'en ont pas : 0,8 mm pour les matières rigides (PrusaSlicer pour la CORE One : 0,7 à 0,8 mm), rien pour
/// les flexibles (la rétraction les coince dans l'extrudeur) ; à remplacer par une valeur par matière si elle arrive.
constexpr float retract_mm(bool flexible) {
    return flexible ? 0.f : 0.8f;
}
inline constexpr float retract_speed = 35.f; ///< [mm/s]

} // namespace od3d::pattern
