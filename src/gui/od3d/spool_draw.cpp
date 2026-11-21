/// @file
#include "spool_draw.hpp"

#include <algorithm>
#include <cmath>

namespace od3d::spool {

namespace {

    // Couleurs fixes de la bobine (SPEC_G.md §1.12)
    constexpr Color rim_color = Color::from_raw(0x5F636A);
    constexpr Color flange_color = Color::from_raw(0x2E3035);
    constexpr Color flange_edge_color = Color::from_raw(0x55595F);
    constexpr Color empty_color = Color::from_raw(0x17181B);
    constexpr Color hub_edge_color = Color::from_raw(0x44474D);
    constexpr Color hub_color = Color::from_raw(0x26282C);
    constexpr Color hub_ring_color = Color::from_raw(0x7A7E85);
    constexpr Color hole_color = Color::from_raw(0x111215);

    /// p % du rayon r, arrondi au pixel
    constexpr int pct(int p, int r) {
        return (p * r + 50) / 100;
    }

} // namespace

Disc::Disc(const Look &look, uint8_t diameter)
    : filament(look.filament.value_or(empty_color))
    , filament_dark(shade(filament))
    , has_filament(look.filament.has_value())
    , d(diameter) {
    const int r = d / 2;
    // Bandes de la maquette en % du rayon ; les bords fins gardent au moins 1 px
    rim = std::max(0, std::min(pct(96, r), r - 1));
    flange = pct(87, r);
    flange_edge = std::max(0, std::min(pct(84, r), flange - 1));
    hub_edge = pct(33, r);
    // Bord de l'enroulement : 34 % (vide) à 82 % (pleine) du rayon
    const int edge_pct = has_filament ? 34 + (48 * std::max<int>(8, std::min<int>(100, look.fill_percent)) + 50) / 100 : 34;
    winding = std::max<int>(hub_edge, std::min<int>(pct(edge_pct, r), flange_edge));
    hub = std::max(0, std::min(pct(30, r), hub_edge - 1));
    hub_ring = pct(19, r);
    hole = std::max(0, std::min(pct(15, r), hub_ring - 1));
}

std::optional<Color> Disc::pixel(int x, int y) const {
    // Distance du centre du pixel au centre du disque, en demi-pixels : tout reste entier
    const int dx = 2 * x + 1 - d;
    const int dy = 2 * y + 1 - d;
    const int q = dx * dx + dy * dy;
    if (x < 0 || y < 0 || x >= d || y >= d || q >= d * d) {
        return std::nullopt;
    }
    const int rho = static_cast<int>(std::sqrt(static_cast<float>(q))) / 2; // rayon du pixel [px]

    if (rho >= rim) {
        return rim_color;
    } else if (rho >= flange) {
        return flange_color;
    } else if (rho >= flange_edge) {
        return flange_edge_color;
    } else if (rho >= winding) {
        return empty_color;
    } else if (rho >= hub_edge) {
        // Anneaux depuis le bord : 2 px de la couleur, 1 px sombre ; sous Ø20 ils tomberaient sous le pixel
        return (d >= 20 && (winding - 1 - rho) % 3 == 2) ? filament_dark : filament;
    } else if (rho >= hub) {
        return hub_edge_color;
    } else if (rho >= hub_ring) {
        return hub_color;
    } else if (rho >= hole) {
        return hub_ring_color;
    }
    return hole_color;
}

Color shade(Color c) {
    const auto dim = [](uint8_t v) { return static_cast<uint8_t>((v * 72 + 50) / 100); };
    return Color::from_rgb(dim(c.r), dim(c.g), dim(c.b));
}

Color default_color(std::string_view material) {
    struct Entry {
        std::string_view name;
        Color color;
    };
    // Couleurs de filament de la maquette, une par matière courante
    static constexpr Entry table[] = {
        { "PLA", Color::from_raw(0xF8651B) }, // orange
        { "PETG", Color::from_raw(0x2F6FD6) }, // bleu
        { "ASA", Color::from_raw(0xD7263D) }, // rouge
        { "ABS", Color::from_raw(0xEDEDED) }, // blanc
        { "PVB", Color::from_raw(0xF2C14E) }, // jaune
        { "FLEX", Color::from_raw(0x2E9E5B) }, // vert
        { "TPU", Color::from_raw(0x2E9E5B) },
        { "PA", Color::from_raw(0x2A2A33) }, // noir
    };
    for (const Entry &e : table) {
        if (material == e.name) {
            return e.color;
        }
    }
    return Color::from_raw(0x8A8F96); // gris
}

} // namespace od3d::spool
