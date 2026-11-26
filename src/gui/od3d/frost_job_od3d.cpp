/// @file
/// Ouverture des fenêtres sur le voile (frost_od3d.hpp) : partie pure, sans l'afficheur
#include "frost_od3d.hpp"
#include "spool_draw.hpp"

#include <algorithm>
#include <array>

namespace od3d::frost {

namespace {

    constexpr uint8_t corner_bits[] = { MIC_TOP_LEFT, MIC_TOP_RIGHT, MIC_BOT_LEFT, MIC_BOT_RIGHT };

    /// Sans Rect16::HasIntersection (Rect16.cpp) : la partie pure se teste seule
    constexpr bool overlap(const Rect16 &a, const Rect16 &b) {
        return !a.IsEmpty() && !b.IsEmpty() && a.Left() <= b.Right() && b.Left() <= a.Right() && a.Top() <= b.Bottom() && b.Top() <= a.Bottom();
    }

    /// Carré radius × radius du coin bit (MIC_*) de rect
    constexpr Rect16 square(const Rect16 &rect, uint8_t bit) {
        const bool left = bit & (MIC_TOP_LEFT | MIC_BOT_LEFT), top = bit & (MIC_TOP_LEFT | MIC_TOP_RIGHT);
        return Rect16(rect.Left() + (left ? 0 : rect.Width() - radius), rect.Top() + (top ? 0 : rect.Height() - radius), radius, radius);
    }

    /// [a0, a1) ramené dans [t0, t1), long d'au moins side (autour de son milieu)
    void axis(int a0, int a1, int t0, int t1, int side, int &pos, int &len) {
        int lo = std::clamp(a0, t0, t1), hi = std::clamp(a1, t0, t1);
        if (hi - lo < side) {
            lo = std::clamp((lo + hi) / 2 - side / 2, t0, std::max(t0, t1 - side));
            hi = std::min(lo + side, t1);
        }
        pos = lo;
        len = hi - lo;
    }

} // namespace

std::pair<int, int> corner_row(Rect16 sq, uint8_t bit, Rect16 piece, int y) {
    const bool left = bit & (MIC_TOP_LEFT | MIC_BOT_LEFT), top = bit & (MIC_TOP_LEFT | MIC_TOP_RIGHT);
    const int cy = y - sq.Top();
    const auto outside = [&](int x) { // (0, 0) : coin extérieur ; 4 quarts : tout le pixel hors de l'arrondi
        const int cx = x - sq.Left();
        return spool::corner_beyond_quarters(left ? cx : radius - 1 - cx, top ? cy : radius - 1 - cy, radius) >= 4;
    };
    int x0 = piece.Left(), x1 = piece.Left() + piece.Width();
    while (x0 < x1 && outside(x0)) {
        ++x0;
    }
    while (x1 > x0 && outside(x1 - 1)) {
        --x1;
    }
    return { x0, x1 };
}

Rect16 start(Rect16 origin, Rect16 to, uint16_t side) {
    if (origin == to) {
        return to; // redessin à sa place
    }
    if (origin.IsEmpty() || 2 * uint32_t(origin.Width()) * origin.Height() >= uint32_t(to.Width()) * to.Height()) {
        origin = Rect16(to.Left() + to.Width() / 2, to.Top() + to.Height() / 2, 0, 0);
    }
    int x, y, w, h;
    axis(origin.Left(), origin.Left() + origin.Width(), to.Left(), to.Left() + to.Width(), side, x, w);
    axis(origin.Top(), origin.Top() + origin.Height(), to.Top(), to.Top() + to.Height(), side, y, h);
    return Rect16(x, y, w, h);
}

Rect16 step(Rect16 from, Rect16 to, int k, int n) {
    if (k >= n) {
        return to;
    }
    // Ralentit vers la fin : 1 - (1 - k/n)², en entiers
    const int num = n * n - (n - k) * (n - k), den = n * n;
    const auto at = [&](int a, int b) { return a + (b - a) * num / den; };
    const int left = at(from.Left(), to.Left()), top = at(from.Top(), to.Top());
    const int right = at(from.Left() + from.Width(), to.Left() + to.Width()), bottom = at(from.Top() + from.Height(), to.Top() + to.Height());
    return Rect16(left, top, right - left, bottom - top);
}

void plan(Rect16 rect, uint8_t corners, Rect16 from, std::span<const Rect16> parts, Painter &painter, bool veil) {
    const size_t n = std::min(parts.size(), max_parts);
    std::array<Rect16, max_parts + 4> keep; // les éléments, puis les carrés des coins arrondis de l'image : hors du corps
    std::copy_n(parts.begin(), n, keep.begin());
    const std::span<const Rect16> elements(keep.data(), n);
    std::array<bool, max_parts> done {};

    const Rect16 first = start(from, rect, 2 * radius);
    const int last = first == rect ? 0 : frames; // redessin à sa place : une seule image
    Rect16 was;
    uint8_t was_round = 0;
    for (int k = 0; k <= last; ++k) {
        if (k > 0 && k < last && painter.stop()) {
            k = last;
        }
        const Rect16 now = step(first, rect, k, last);

        // 1. Les éléments atteints, avec ceux qui les chevauchent (l'un peint par-dessus l'autre), dans leur ordre
        std::array<bool, max_parts> group {};
        for (size_t i = 0; i < n; ++i) {
            group[i] = !done[i] && overlap(keep[i], now);
        }
        for (bool grew = true; grew;) {
            grew = false;
            for (size_t i = 0; i < n; ++i) {
                for (size_t j = 0; j < n && group[i]; ++j) {
                    if (!done[j] && !group[j] && overlap(keep[j], keep[i])) {
                        group[j] = grew = true;
                    }
                }
            }
        }
        for (size_t i = 0; i < n; ++i) {
            if (group[i]) {
                done[i] = true;
                painter.part(i);
            }
        }

        // 2. Coins arrondis de l'image : ceux de la fenêtre, et tous tant qu'ils ne sont pas à leur place (un coin carré
        // touche le bord de l'écran)
        uint8_t round = 0;
        size_t m = n;
        for (const uint8_t bit : corner_bits) {
            if ((corners & bit) || square(now, bit) != square(rect, bit)) {
                round |= bit;
                keep[m++] = square(now, bit);
            }
        }
        const std::span<const Rect16> kept(keep.data(), m);
        const auto body = [&](Rect16 r) { style::for_each_gap(r, kept, [&](Rect16 g) { painter.body(g); }); };

        // 3. Le corps : les coins de l'image d'avant (maintenant dedans), puis la couronne nouvelle
        for (const uint8_t bit : corner_bits) {
            if (was_round & bit) {
                body(square(was, bit));
            }
        }
        const Rect16 hole[] = { was };
        style::for_each_gap(now, std::span<const Rect16>(hole, was.IsEmpty() ? 0 : 1), body);

        // 4. Les coins arrondis, sauf ceux qui n'ont pas bougé
        for (const uint8_t bit : corner_bits) {
            if ((round & bit) && !((was_round & bit) && square(was, bit) == square(now, bit))) {
                const Rect16 sq = square(now, bit);
                style::for_each_gap(sq, elements, [&](Rect16 piece) { painter.corner(sq, bit, piece); });
            }
        }
        was = now;
        was_round = round;
    }

    if (veil) {
        const Rect16 hole[] = { rect };
        style::for_each_gap(style::below_header, hole, [&](Rect16 r) { painter.veil(r); });
    }
}

} // namespace od3d::frost
