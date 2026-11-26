/// @file
/// OneDrop3D : plan du plateau de « Annuler des objets » (retour du 26/11 : « un visuel du plateau et cliquer sur l'objet
/// à enlever »). Partie pure, testée seule (tests/unit/feature/od3d/object_map_test.cpp).
///
/// La place d'un objet est l'étendue de ses extrusions pendant l'impression (G1, G2/G3 qui poussent du filament entre
/// « M486 S<n> » et le suivant) : rien à relire dans le fichier, et elle vaut pour tout trancheur qui écrit M486. Un objet
/// pas encore imprimé n'a pas de place (il reste annulable par son nom). Une boîte tient en 4 octets, par pas de 2 mm.
#pragma once

#include <algorithm>
#include <cstdint>
#include <span>

namespace od3d::object_map {

/// Étendue d'un objet sur le plateau, en pas de unit_mm (0 à 510 mm : la CORE One L comprise)
struct Box {
    uint8_t x0, y0, x1, y1;
};

inline constexpr float unit_mm = 2.f;
inline constexpr uint32_t unknown = 0; ///< pas de place connue (une vraie boîte n'est jamais tout à 0 : 2 mm dans le coin)

constexpr uint32_t pack(Box b) {
    return uint32_t(b.x0) | uint32_t(b.y0) << 8 | uint32_t(b.x1) << 16 | uint32_t(b.y1) << 24;
}

constexpr Box unpack(uint32_t p) {
    return { uint8_t(p), uint8_t(p >> 8), uint8_t(p >> 16), uint8_t(p >> 24) };
}

constexpr uint8_t to_unit(float mm) {
    return uint8_t(std::clamp(mm / unit_mm + 0.5f, 0.f, 255.f));
}

/// La boîte p (ou unknown) agrandie au tracé de (xa, ya) à (xb, yb), en mm
constexpr uint32_t grow(uint32_t p, float xa, float ya, float xb, float yb) {
    const Box s { to_unit(std::min(xa, xb)), to_unit(std::min(ya, yb)), to_unit(std::max(xa, xb)), to_unit(std::max(ya, yb)) };
    if (p == unknown) {
        return pack(s);
    }
    const Box b = unpack(p);
    return pack({ std::min(b.x0, s.x0), std::min(b.y0, s.y0), std::max(b.x1, s.x1), std::max(b.y1, s.y1) });
}

/// Rect en pixels sur le plan, (0, 0) en haut à gauche ; w == 0 : pas de place
struct Px {
    int16_t x = 0, y = 0, w = 0, h = 0;

    constexpr bool contains(int px, int py) const {
        return w > 0 && px >= x && px < x + w && py >= y && py < y + h;
    }
    constexpr bool operator==(const Px &) const = default;
};

/// Place de la boîte b sur un plan de w × h pixels d'un plateau de bed_x × bed_y mm, l'avant (Y = 0) en bas. Au moins
/// min_side pixels de côté, centré : un petit objet se voit et se touche.
constexpr Px to_screen(Box b, int w, int h, float bed_x, float bed_y, int min_side) {
    const auto px = [&](uint8_t u) { return int(std::clamp(u * unit_mm, 0.f, bed_x) * w / bed_x + 0.5f); };
    const auto py = [&](uint8_t u) { return h - int(std::clamp(u * unit_mm, 0.f, bed_y) * h / bed_y + 0.5f); };
    const auto widen = [min_side](int &a, int &c, int limit) {
        if (c - a < min_side) {
            a = std::clamp((a + c - min_side) / 2, 0, std::max(0, limit - min_side));
            c = std::min(a + min_side, limit);
        }
    };
    int x0 = px(b.x0), x1 = px(b.x1), y0 = py(b.y1), y1 = py(b.y0);
    widen(x0, x1, w);
    widen(y0, y1, h);
    return { int16_t(x0), int16_t(y0), int16_t(x1 - x0), int16_t(y1 - y0) };
}

/// L'objet touché au point (x, y) : le plus petit qui le contient (un objet posé dans un autre reste touchable), sinon le
/// plus proche à moins de reach pixels ; -1 si aucun
constexpr int hit(std::span<const Px> rects, int x, int y, int reach) {
    int best = -1;
    long best_score = 0;
    for (size_t i = 0; i < rects.size(); ++i) {
        const Px &r = rects[i];
        if (r.w <= 0) {
            continue;
        }
        const int dx = std::max({ r.x - x, 0, x - (r.x + r.w - 1) });
        const int dy = std::max({ r.y - y, 0, y - (r.y + r.h - 1) });
        const long d2 = long(dx) * dx + long(dy) * dy;
        if (d2 > long(reach) * reach) {
            continue;
        }
        // Dedans : l'aire (petite d'abord) ; dehors : après tout objet qui contient le point, le plus proche d'abord
        const long score = d2 == 0 ? long(r.w) * r.h : (1L << 30) + d2;
        if (best < 0 || score < best_score) {
            best = int(i);
            best_score = score;
        }
    }
    return best;
}

} // namespace od3d::object_map
