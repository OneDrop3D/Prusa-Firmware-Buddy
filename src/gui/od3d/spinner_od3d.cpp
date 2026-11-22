/// @file
#include "spinner_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace od3d::spinner {

namespace {

    constexpr float two_pi = 2 * std::numbers::pi_v<float>;
    constexpr float arc_len = two_pi / 4; ///< un quart
    constexpr float step_angle = two_pi / steps;

    /// front sur back, couvert à t (0 à 1) ; aux bouts, les couleurs exactes (Color::mix les assombrit d'un cran)
    Color blend(Color back, Color front, float t) {
        return t >= 1 ? front : t <= 0 ? back : Color::mix(back, front, uint8_t(t * 255));
    }

} // namespace

Color pixel(int x, int y, uint8_t d, uint8_t w, uint8_t step, Color bg) {
    const float half = d / 2.0f;
    const float dx = x + 0.5f - half, dy = y + 0.5f - half;
    const float r = std::sqrt(dx * dx + dy * dy);
    // Couverture du bord (anticrénelage radial) ; hors de l'anneau, le fond
    const float radial = std::min(half - r, r - (half - w)) + 0.5f;
    if (radial <= 0) {
        return bg;
    }
    // Angle depuis le haut, sens horaire ; position dans l'arc, puis distance (en px) à son bord le plus proche
    float rel = std::atan2(dx, -dy) - step * step_angle;
    while (rel < 0) {
        rel += two_pi;
    }
    const float inside = rel <= arc_len ? std::min(rel, arc_len - rel) : -std::min(rel - arc_len, two_pi - rel);
    return blend(bg, blend(style::line, style::accent, inside * r + 0.5f), radial);
}

std::array<Rect16, 2> changed(uint8_t d, uint8_t w, uint8_t from, uint8_t to) {
    const int moved = (to + steps - from) % steps;
    if (moved == 0) {
        return {};
    }
    // Secteur de l'anneau balayé par un bout de l'arc, élargi de l'anticrénelage (1 px au rayon intérieur)
    const float r_in = std::max(d / 2.0f - w - 1, 1.0f), r_out = d / 2.0f + 1;
    const float margin = 1.5f / r_in;
    const float span = moved * step_angle + 2 * margin;
    const auto sector = [&](float start) {
        if (span >= two_pi) {
            return Rect16(0, 0, d, d);
        }
        const float half = d / 2.0f;
        float x0 = d, y0 = d, x1 = 0, y1 = 0;
        const auto add = [&](float a, float r) {
            const float x = half + r * std::sin(a), y = half - r * std::cos(a);
            x0 = std::min(x0, x), x1 = std::max(x1, x), y0 = std::min(y0, y), y1 = std::max(y1, y);
        };
        // Les deux bouts du secteur, aux deux rayons, et les points cardinaux qu'il traverse, au rayon extérieur
        for (const float r : { r_in, r_out }) {
            add(start, r);
            add(start + span, r);
        }
        for (int k = int(std::ceil(start / arc_len)); k * arc_len <= start + span; ++k) {
            add(k * arc_len, r_out);
        }
        const int l = std::max(int(std::floor(x0)), 0), t = std::max(int(std::floor(y0)), 0);
        const int r = std::min(int(std::ceil(x1)), int(d)), b = std::min(int(std::ceil(y1)), int(d));
        return Rect16(l, t, std::max(r - l, 0), std::max(b - t, 0));
    };
    const float tail = from * step_angle - margin;
    return { sector(tail), sector(tail + arc_len) };
}

} // namespace od3d::spinner
