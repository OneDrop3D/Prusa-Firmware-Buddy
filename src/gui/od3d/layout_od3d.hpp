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

} // namespace od3d::grid
