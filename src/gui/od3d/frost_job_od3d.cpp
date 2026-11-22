/// @file
/// Givrage (frost_od3d.hpp) : partie pure, sans l'afficheur
#include "frost_od3d.hpp"

#include <algorithm>

namespace od3d::frost {

std::array<Area, 4> margins(Area area, Area hole) {
    const int ax1 = area.x + area.w, ay1 = area.y + area.h;
    const int hx0 = std::clamp<int>(hole.x, area.x, ax1), hx1 = std::clamp<int>(hole.x + hole.w, area.x, ax1);
    const int hy0 = std::clamp<int>(hole.y, area.y, ay1), hy1 = std::clamp<int>(hole.y + hole.h, area.y, ay1);
    if (hx0 >= hx1 || hy0 >= hy1) {
        return { area, Area {}, Area {}, Area {} };
    }
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

void Job::start(Area area, Area hole) {
    bands = margins(area, hole);
    index = 0;
    primed = false;
}

namespace {

    /// Sommes horizontales des n lignes de w pixels du tampon, à la place des pixels (bords de la bande répétés) : chaque
    /// composante ramenée à 6 bits, somme de 3 ≤ 189
    void sum_rows(uint8_t *buf, int n, int w) {
        uint8_t *p = buf;
        for (int r = 0; r < n; ++r, p += 3 * w) {
            for (int c = 0; c < 3; ++c) {
                unsigned left = p[c] >> 2; // pixel d'avant, gardé avant d'être remplacé par sa somme
                for (int x = 0; x < w; ++x) {
                    const unsigned cur = p[3 * x + c] >> 2;
                    const unsigned right = x + 1 < w ? p[3 * (x + 1) + c] >> 2 : cur;
                    p[3 * x + c] = static_cast<uint8_t>(left + cur + right);
                    left = cur;
                }
            }
        }
    }

} // namespace

void Job::step(size_t capacity, Read read, Write write) {
    while (!done() && bands[index].empty()) {
        ++index;
    }
    if (done()) {
        return;
    }
    const Area band = bands[index];
    const int w = band.w;
    const int last = band.y + band.h - 1;
    const int max_rows = int(capacity / w);
    if (w > max_w || max_rows < 2) {
        stop(); // le tampon ne tient pas deux lignes : rien
        return;
    }
    if (!primed) {
        row = band.y;
    }

    // Lignes à lire : la 1re de la bande avec celles d'après, ou celles qui suivent les deux sommes en cache. La dernière
    // ligne de la bande (bord répété) sort avec la tranche qui la lit : il faut alors une ligne de place de plus.
    const bool first = !primed;
    const int from = first ? row : row + 1;
    int to = std::min(last, from + max_rows - 1);
    if (!first && to == last && to - from + 2 > max_rows) {
        --to;
    }
    const int n = to - from + 1;
    const int outs = to == last ? (first ? n : n + 1) : (first ? n - 1 : n);

    uint8_t *buf = read(Area { band.x, int16_t(from), uint16_t(w), uint16_t(n) });
    if (!buf) {
        stop();
        return;
    }
    sum_rows(buf, n, w);

    // Flou vertical, colonne par colonne : la ligne k sort à la place de la k-ième somme, déjà lue
    uint8_t *prev = sums.data(), *cur = sums.data() + max_w * 3;
    const int stride = 3 * w;
    for (int i = 0; i < stride; ++i) {
        unsigned hp, hc;
        int j;
        if (first) {
            hp = hc = buf[i]; // bord haut répété
            j = 1;
        } else {
            hp = prev[i];
            hc = cur[i];
            j = 0;
        }
        for (int k = 0; k < outs; ++k, ++j) {
            const unsigned hn = j < n ? buf[j * stride + i] : hc; // bord bas répété
            buf[k * stride + i] = shade(hp + hc + hn);
            hp = hc;
            hc = hn;
        }
        prev[i] = static_cast<uint8_t>(hp);
        cur[i] = static_cast<uint8_t>(hc);
    }
    if (outs > 0) {
        write(Area { band.x, row, uint16_t(w), uint16_t(outs) }, buf);
    }

    row = int16_t(row + outs);
    primed = true;
    if (row > last) {
        ++index;
        primed = false;
    }
}

} // namespace od3d::frost
