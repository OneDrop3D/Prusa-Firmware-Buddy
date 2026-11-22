/// @file
/// Givrage (frost_od3d.hpp) : partie pure, sans l'afficheur
#include "frost_od3d.hpp"
#include "spool_draw.hpp"

#include <algorithm>

namespace od3d::frost {

std::array<Area, 4> margins(Area area, Area hole, uint16_t top_rows, uint16_t bottom_rows) {
    const int ax1 = area.x + area.w, ay1 = area.y + area.h;
    const int hx0 = std::clamp<int>(hole.x, area.x, ax1), hx1 = std::clamp<int>(hole.x + hole.w, area.x, ax1);
    int hy0 = std::clamp<int>(hole.y, area.y, ay1), hy1 = std::clamp<int>(hole.y + hole.h, area.y, ay1);
    if (hx0 >= hx1 || hy0 >= hy1) {
        return { area, Area {}, Area {}, Area {} };
    }
    // Lignes des coins arrondis : avec les bandes du haut et du bas, sur toute la largeur
    const int top_end = std::min(hy0 + int(top_rows), hy1);
    hy1 = std::max(hy1 - int(bottom_rows), top_end);
    hy0 = top_end;
    const auto band = [](int x0, int y0, int x1, int y1) {
        return Area { int16_t(x0), int16_t(y0), uint16_t(std::max(x1 - x0, 0)), uint16_t(std::max(y1 - y0, 0)) };
    };
    return {
        band(area.x, area.y, ax1, hy0),
        band(area.x, hy0, hx0, hy1),
        band(hx1, hy0, ax1, hy1),
        band(area.x, hy1, ax1, ay1),
    };
}

namespace {

    constexpr int max_w = 480;
    constexpr int min_step = 8;

    /// Nœuds de toutes les bandes (3 composantes, moyennes en unités de l'afficheur), lus avant la première écriture.
    /// Fiche de bobine : 610 nœuds, éditeur de valeur : 671, écran de chargement : 849.
    std::array<uint8_t, 960 * 3> nodes;

    /// Nœuds sur len pixels : un tous les step, et un sur le dernier
    constexpr int count(int len, int step) {
        return len <= 1 ? 1 : (len - 2) / step + 2;
    }
    /// Position du nœud i
    constexpr int at(int i, int len, int step) {
        return std::min(i * step, len - 1);
    }
    /// Pente de a à b sur len pixels, en 16.16
    constexpr int32_t slope(int a, int b, int len) {
        return len ? (b - a) * 65536 / len : 0;
    }

    size_t node_count(const std::array<Area, 4> &bands, int step) {
        size_t n = 0;
        for (const Area &b : bands) {
            if (!b.empty()) {
                n += size_t(count(b.w, step)) * count(b.h, step);
            }
        }
        return n;
    }

} // namespace

namespace {

    /// Pixels du trou dans la ligne y de la bande b (px : ses pixels givrés) : coins hors de l'arrondi givrés, bord lissé
    /// mêlé au corps, le reste au corps
    void round_hole(uint8_t *px, const Area &b, int y, Area hole, const Corners &corners) {
        if (y < hole.y || y >= hole.y + hole.h) {
            return;
        }
        const int r = corners.r, ly = y - hole.y;
        const bool top = ly < r, bottom = hole.h - 1 - ly < r;
        const int x0 = std::max<int>(hole.x, b.x), x1 = std::min<int>(hole.x + hole.w, b.x + b.w);
        for (int x = x0; x < x1; ++x) {
            uint8_t *p = px + (x - b.x) * 3;
            const int lx = x - hole.x;
            const bool left = lx < r, right = !left && hole.w - 1 - lx < r;
            const uint8_t bit = (left || right) ? (top ? (left ? Corners::top_left : Corners::top_right) : bottom ? (left ? Corners::bottom_left : Corners::bottom_right) : 0) : 0;
            const uint8_t q = (corners.mask & bit) ? spool::corner_beyond_quarters(left ? lx : hole.w - 1 - lx, top ? ly : hole.h - 1 - ly, r) : 0;
            for (int c = 0; c < 3; ++c) {
                p[c] = static_cast<uint8_t>((corners.body[c] * (4 - q) + p[c] * q + 2) / 4); // 0 quart dehors : le corps
            }
        }
    }

} // namespace

void run(Area area, Area hole, size_t capacity, Read read, Write write, const Corners &corners) {
    const uint16_t top_rows = corners.mask & (Corners::top_left | Corners::top_right) ? corners.r : 0;
    const uint16_t bottom_rows = corners.mask & (Corners::bottom_left | Corners::bottom_right) ? corners.r : 0;
    const auto bands = margins(area, hole, top_rows, bottom_rows);
    for (const Area &b : bands) {
        if (!b.empty() && (b.w > max_w || 2 * size_t(b.w) > capacity)) {
            return; // le tampon ne tient pas deux lignes : rien
        }
    }
    int step = min_step;
    while (node_count(bands, step) * 3 > nodes.size()) {
        step *= 2; // grandes zones : nœuds plus espacés (jamais avec nos dialogues)
    }

    // 1. Lectures : pour chaque ligne de nœuds, 2 lignes de pixels ; chaque nœud, la moyenne de 8 colonnes autour
    uint8_t *buffer = nullptr;
    uint8_t *node = nodes.data();
    for (const Area &b : bands) {
        if (b.empty()) {
            continue;
        }
        const int w = b.w, h = b.h, nc = count(w, step), nr = count(h, step);
        const int k = std::min(h, 2);
        for (int j = 0; j < nr; ++j) {
            const int r0 = std::min(at(j, h, step), h - k);
            buffer = read(Area { b.x, int16_t(b.y + r0), b.w, uint16_t(k) });
            if (!buffer) {
                return;
            }
            for (int i = 0; i < nc; ++i) {
                const int cx = at(i, w, step);
                const int x0 = std::max(cx - step / 2, 0), x1 = std::min(cx + step / 2, w);
                const unsigned n = unsigned(k * (x1 - x0));
                for (int c = 0; c < 3; ++c, ++node) {
                    unsigned sum = 0;
                    for (int r = 0; r < k; ++r) {
                        for (int x = x0; x < x1; ++x) {
                            sum += buffer[(r * w + x) * 3 + c] >> 2;
                        }
                    }
                    *node = static_cast<uint8_t>((sum * 4 + n / 2) / n);
                }
            }
        }
    }

    // 2. Écritures : chaque pixel entre ses 4 nœuds, par paquets de lignes que le tampon tient
    node = nodes.data();
    for (const Area &b : bands) {
        if (b.empty()) {
            continue;
        }
        const int w = b.w, h = b.h, nc = count(w, step), nr = count(h, step);
        const int rows = int(capacity / w);
        std::array<uint8_t, (max_w / min_step + 2) * 3> line; // nœuds de la ligne, interpolés entre deux lignes de nœuds
        int j = 0;
        for (int y = 0; y < h;) {
            const int n = std::min(rows, h - y);
            for (int r = 0; r < n; ++r, ++y) {
                while (j + 1 < nr && at(j + 1, h, step) <= y) {
                    ++j;
                }
                const int j1 = std::min(j + 1, nr - 1), y0 = at(j, h, step);
                const uint8_t *above = node + j * nc * 3, *below = node + j1 * nc * 3;
                for (int i = 0; i < nc * 3; ++i) {
                    line[i] = static_cast<uint8_t>(above[i] + ((slope(above[i], below[i], at(j1, h, step) - y0) * (y - y0) + 0x8000) >> 16));
                }
                uint8_t *px = buffer + r * w * 3;
                for (int i = 0; i < nc; ++i) {
                    const int i1 = std::min(i + 1, nc - 1), x0 = at(i, w, step);
                    const int x1 = i1 == i ? w : at(i1, w, step); // le dernier nœud ne couvre que lui-même
                    for (int c = 0; c < 3; ++c) {
                        const int32_t d = slope(line[i * 3 + c], line[i1 * 3 + c], x1 - x0);
                        int32_t v = (int32_t(line[i * 3 + c]) << 16) + 0x8000;
                        for (int x = x0; x < x1; ++x, v += d) {
                            px[x * 3 + c] = shade(unsigned(v >> 16));
                        }
                    }
                }
                if (corners.mask) {
                    round_hole(px, b, b.y + y, hole, corners);
                }
            }
            write(Area { b.x, int16_t(b.y + y - n), b.w, uint16_t(n) }, buffer);
        }
        node += nc * nr * 3;
    }
}

} // namespace od3d::frost
