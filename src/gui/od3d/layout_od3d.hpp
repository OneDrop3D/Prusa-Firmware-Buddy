/// @file
/// OneDrop3D (interface G) : calculs purs de disposition, testés seuls (tests/unit/feature/od3d/layout_od3d_test.cpp).
/// - Bobines selon le nombre d'outils lu à l'exécution (VirtualToolIndex::enabled_range_size()) : MMU3 = 5, sans MMU
///   = 1, INDX = 3 à 8. Une rangée jusqu'à 5, deux au-delà (8 têtes : 2 rangées de 4, retour du 22/11).
/// - Défilement de la grille des fichiers : la molette avance d'un fichier, la grille d'une rangée ; le doigt la tire
///   rangée par rangée ; un lancer continue de quelques rangées.
#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <Rect16.h>

namespace od3d::layout {

struct Grid {
    uint8_t cols;
    uint8_t rows;
};

inline constexpr uint8_t max_per_row = 5;

/// Colonnes et rangées pour tools outils (au moins un : l'imprimante sans MMU a sa bobine)
constexpr Grid spool_grid(uint8_t tools) {
    if (tools <= max_per_row) {
        return { std::max<uint8_t>(tools, 1), 1 };
    }
    return { uint8_t((tools + 1) / 2), 2 };
}

/// Rect de la bobine i (rangée par rangée) dans area, cases séparées de gap px ; les pixels qui restent de la division
/// vont aux premières colonnes et rangées
constexpr Rect16 spool_cell(Rect16 area, uint8_t tools, uint8_t gap, uint8_t i) {
    const Grid g = spool_grid(tools);
    const int col = i % g.cols, row = i / g.cols;
    const int free_w = area.Width() - (g.cols - 1) * gap, free_h = area.Height() - (g.rows - 1) * gap;
    const int w = free_w / g.cols, extra_w = free_w % g.cols;
    const int h = free_h / g.rows, extra_h = free_h % g.rows;
    return Rect16(area.Left() + col * (w + gap) + std::min(col, extra_w), area.Top() + row * (h + gap) + std::min(row, extra_h),
        w + (col < extra_w ? 1 : 0), h + (row < extra_h ? 1 : 0));
}

// ---- Coins arrondis ----
// Retour du 22/11 (8.0.2.29) : « les bobines sont dans un bloc gris clair, coupé en bas, avec un début d'arrondi ». Le
// fond du texte d'une carte (rect de toute sa largeur) repeignait en carré l'arrondi des coins du bas. Le contenu d'une
// carte reste dans la partie pleine de son corps.

/// Pixel (x, y) d'un corps w × h aux coins de rayon r (relatif au corps) qui garde la couleur pleine : ni dans l'arrondi,
/// ni sur son bord lissé (display::draw_rounded_rect mélange dès que le pixel touche le cercle de rayon r - 3/8)
constexpr bool in_body(int w, int h, int r, int x, int y) {
    if (x < 0 || y < 0 || x >= w || y >= h) {
        return false;
    }
    const int cx = x < r ? x : w - 1 - x, cy = y < r ? y : h - 1 - y;
    if (cx >= r || cy >= r) {
        return true;
    }
    const int dx = 2 * (r - cx) - 1, dy = 2 * (r - cy) - 1; // en demi-pixels, du centre du coin au centre du pixel
    return 16 * (dx * dx + dy * dy) < (8 * r - 3) * (8 * r - 3);
}

/// rect (absolu) tout entier dans la partie pleine de body : ses 4 coins suffisent (elle est convexe)
constexpr bool inside_body(Rect16 body, uint8_t r, Rect16 rect) {
    const int x0 = rect.Left() - body.Left(), x1 = rect.Right() - body.Left();
    const int y0 = rect.Top() - body.Top(), y1 = rect.Bottom() - body.Top();
    const int w = body.Width(), h = body.Height();
    return in_body(w, h, r, x0, y0) && in_body(w, h, r, x1, y0) && in_body(w, h, r, x0, y1) && in_body(w, h, r, x1, y1);
}

/// rect (absolu) rétréci à gauche et à droite jusqu'à ne plus toucher les coins arrondis de body
constexpr Rect16 clear_of_corners(Rect16 body, uint8_t r, Rect16 rect) {
    const int w = body.Width(), h = body.Height();
    const int y0 = rect.Top() - body.Top(), y1 = rect.Bottom() - body.Top();
    int x0 = rect.Left() - body.Left(), x1 = rect.Right() - body.Left();
    while (x0 <= x1 && !(in_body(w, h, r, x0, y0) && in_body(w, h, r, x0, y1))) {
        ++x0;
    }
    while (x1 >= x0 && !(in_body(w, h, r, x1, y0) && in_body(w, h, r, x1, y1))) {
        --x1;
    }
    return Rect16(body.Left() + x0, rect.Top(), x1 >= x0 ? x1 - x0 + 1 : 0, rect.Height());
}

constexpr bool overlap(Rect16 a, Rect16 b) {
    return a.Left() <= b.Right() && b.Left() <= a.Right() && a.Top() <= b.Bottom() && b.Top() <= a.Bottom();
}

/// Hauteurs des polices small et normal (height() n'existe pas hors du firmware ; vérifiées dans screen_home_od3d.cpp)
inline constexpr uint16_t small_h = 16, normal_h = 19;

/// Contenu d'une carte de bobine de l'accueil (corps body, coins de rayon r) selon sa place : large (sans MMU, INDX 3
/// têtes) bobine Ø40 à gauche, texte à droite ; basse (INDX 8 têtes, deux rangées) petite bobine et « n MATIÈRE » ;
/// étroite (MMU3) bobine en haut, « n MATIÈRE » dessous. Rien ne touche les coins arrondis (testé, et vérifié à la
/// compilation pour 1 à 8 bobines).
struct SpoolParts {
    point_i16_t spool; ///< coin haut gauche du disque ; son image (anneau compris) déborde de 2 px tout autour
    uint8_t d; ///< diamètre
    Rect16 label; ///< « n MATIÈRE », ou la matière seule en gras (carte large d'une seule bobine)
    Rect16 state; ///< état (carte large), vide sinon

    constexpr Rect16 spool_image() const {
        return Rect16(spool.x - 2, spool.y - 2, d + 4, d + 4);
    }
};

constexpr SpoolParts spool_parts(Rect16 body, uint8_t r, bool single) {
    if (body.Height() < 40) { // avant la large : 6 et 7 têtes ont deux rangées de cartes larges
        constexpr uint8_t d = 20;
        const point_i16_t at { int16_t(body.Left() + r), int16_t(body.Top() + (body.Height() - d) / 2) }; // anneau hors des coins
        const int16_t x = at.x + d + 4;
        return { at, d, clear_of_corners(body, r, Rect16(x, body.Top() + (body.Height() - small_h) / 2, body.Left() + body.Width() - x, small_h)), Rect16() };
    }
    if (body.Width() >= 110) {
        constexpr uint8_t d = 40;
        const point_i16_t at { int16_t(body.Left() + 10), int16_t(body.Top() + (body.Height() - d) / 2) };
        const int16_t x = at.x + d + 10;
        const uint16_t w = body.Left() + body.Width() - 6 - x;
        const uint16_t label_h = single ? normal_h : small_h;
        const int16_t y = body.Top() + (body.Height() - label_h - small_h) / 2;
        return { at, d, clear_of_corners(body, r, Rect16(x, y, w, label_h)), clear_of_corners(body, r, Rect16(x, y + label_h, w, small_h)) };
    }
    // Le libellé finit 4 px au-dessus du bas : 7 caractères tiennent entre les arrondis (bobine Ø32 sur 56 px)
    const uint8_t d = std::min<int>(40, body.Height() - 24);
    const point_i16_t at { int16_t(body.Left() + (body.Width() - d) / 2), int16_t(body.Top() + 3) };
    return { at, d, clear_of_corners(body, r, Rect16(body.Left(), body.Top() + d + 5, body.Width(), small_h)), Rect16() };
}

/// Le contenu tient dans la partie pleine du corps, sans chevauchement
constexpr bool spool_parts_fit(Rect16 body, uint8_t r, bool single) {
    const SpoolParts p = spool_parts(body, r, single);
    return inside_body(body, r, p.spool_image()) && inside_body(body, r, p.label) && !overlap(p.spool_image(), p.label)
        && (p.state.IsEmpty() || (inside_body(body, r, p.state) && !overlap(p.spool_image(), p.state) && !overlap(p.label, p.state)));
}

} // namespace od3d::layout

namespace od3d::grid {

/// Ce que montre la grille : first = rang du fichier en haut à gauche (début de rangée), focus = rang du fichier ciblé
struct View {
    int first;
    int focus;
    constexpr bool operator==(const View &) const = default;
};

/// Rang du fichier en haut à gauche de la dernière page (la dernière rangée en bas)
constexpr int max_first(int count, int cols, int rows) {
    const int total_rows = (count + cols - 1) / cols;
    return std::max(0, total_rows - rows) * cols;
}

/// first ramené au début d'une rangée, entre la première et la dernière page
constexpr int clamp_first(int count, int cols, int rows, int first) {
    return std::clamp(first / cols * cols, 0, max_first(count, cols, rows));
}

/// Un cran de molette (step = ±1) : le fichier suivant ou précédent ; la grille avance d'une rangée quand il sort de
/// l'écran. nullopt au bout de la liste : la molette passe aux onglets ou à l'en-tête.
constexpr std::optional<View> step(int count, int cols, int rows, View v, int step) {
    const int next = v.focus + step;
    if (next < 0 || next >= count) {
        return std::nullopt;
    }
    int first = v.first;
    if (next < first) {
        first = next / cols * cols;
    } else if (next >= first + cols * rows) {
        first = (next / cols - rows + 1) * cols;
    }
    return View { first, next };
}

/// first quand le doigt a glissé de dy px depuis first0 (vers le haut, dy < 0 : la suite de la liste) : une rangée par
/// pitch px, dès la moitié ; le contenu suit le doigt rangée par rangée
constexpr int drag(int count, int cols, int rows, int first0, int dy, int pitch) {
    const int rows_moved = (-dy + (dy < 0 ? pitch / 2 : -(pitch / 2))) / pitch;
    return clamp_first(count, cols, rows, first0 + rows_moved * cols);
}

/// Rangées ajoutées par un lancer : rien sous 500 px/s, une par tranche de 500 px/s, 3 au plus (inertie légère).
/// Signe : celui du défilement (doigt vers le haut, vitesse négative : rangées positives).
constexpr int fling_rows(int velocity_px_s) {
    const int rows = std::min((velocity_px_s < 0 ? -velocity_px_s : velocity_px_s) / 500, 3);
    return velocity_px_s < 0 ? rows : -rows;
}

/// Début de la fenêtre du dossier gardée en mémoire (LazyDirView : window entrées à partir de start, -1 = « .. » ;
/// total entrées, « .. » compris) pour montrer les entrées [wanted, wanted + page). start si elles y sont : rien à relire
/// sur la clé. Sinon la fenêtre se recale (une lecture du dossier), la page au bord d'où l'on vient : la suite dans le
/// sens du défilement reste en mémoire.
constexpr int window_start(int total, int window, int start, int wanted, int page) {
    if (wanted >= start && wanted + page <= start + window) {
        return start;
    }
    return std::clamp(wanted < start ? wanted + page - window : wanted, -1, std::max(total - window - 1, -1));
}

} // namespace od3d::grid
