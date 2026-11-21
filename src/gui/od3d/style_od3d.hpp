/// @file
/// OneDrop3D (F1, interface G) : palette et géométrie communes à tous nos écrans (SPEC_G.md §1).
#pragma once

#include <algorithm>
#include <array>
#include <Rect16.h>
#include <span>
#include <utils/color.hpp>

namespace od3d::style {

// Palette de la maquette G (SPEC_G.md §1.1)
inline constexpr Color screen = Color::from_raw(0x1B1C1E); ///< fond de l'écran
inline constexpr Color tab_bar = Color::from_raw(0x141517); ///< fond de la barre d'onglets
inline constexpr Color surface = Color::from_raw(0x26282B); ///< cartes, boutons, bouton des températures
inline constexpr Color secondary = Color::from_raw(0x303337); ///< bouton secondaire, disque du retour
inline constexpr Color tile = Color::from_raw(0x34373C); ///< tuile d'icône neutre
inline constexpr Color line = Color::from_raw(0x3A3D42); ///< traits de séparation
inline constexpr Color thumb_back = Color::from_raw(0x22242A); ///< fond des vignettes
inline constexpr Color accent = COLOR_ORANGE; ///< #F8651B : bouton principal, onglet actif, emplacement chargé
inline constexpr Color on_accent = COLOR_BLACK; ///< texte et icônes sur l'orange
inline constexpr Color text_strong = COLOR_WHITE; ///< titres et valeurs
inline constexpr Color text = Color::from_raw(0xF2F2F2); ///< texte des boutons, de l'en-tête et des listes
inline constexpr Color text_soft = Color::from_raw(0xC9CCD0); ///< texte secondaire clair
inline constexpr Color text_muted = Color::from_raw(0xA9ADB3); ///< sous-titres, heure, onglets inactifs
inline constexpr Color text_faint = Color::from_raw(0x6B6F75); ///< éléments désactivés
inline constexpr Color warning = Color::from_raw(0xF2C14E);
inline constexpr Color danger = COLOR_RED_ALERT; ///< #E74626

/// Corps d'un élément ciblé par la molette : un ton plus clair, sans anneau blanc (retour de l'utilisateur, 21/11).
/// Une carte #26282B devient #34373C, la ligne de liste ciblée de SPEC_G.md §1.6 ; l'orange s'éclaircit davantage.
inline Color focused(Color c) {
    if (c == accent) {
        return Color::from_raw(0xFA8B54);
    }
    const auto up = [](uint8_t v, int d) { return uint8_t(std::min(255, v + d)); };
    return Color::from_rgb(up(c.r, 14), up(c.g, 15), up(c.b, 17));
}

// Tuiles d'icône par famille (SPEC_G.md §1.1)
inline constexpr Color tile_heat = Color::from_raw(0x4A2C1E);
inline constexpr Color tile_filament = Color::from_raw(0x1F3652);
inline constexpr Color tile_motion = Color::from_raw(0x463B1E);

// Géométrie (SPEC_G.md §1.3, §1.4, §1.11)
inline constexpr uint16_t screen_w = 480;
inline constexpr uint16_t screen_h = 320;
inline constexpr int16_t margin = 8; ///< marges gauche et droite
inline constexpr uint8_t ring = 2; ///< anneau de focus, dessiné dans une marge de 2 px autour du corps
inline constexpr uint8_t radius = 12; ///< cartes et boutons jusqu'à 64 px de haut
inline constexpr uint8_t radius_big = 14; ///< grandes cartes

// En-tête de 40 px (26 dans la maquette, jugé trop petit sur l'imprimante) ; barre d'onglets de 44 px pour le compenser
inline constexpr Rect16 header = { 0, 0, screen_w, 40 }; ///< en-tête : retour, titre, températures, réseau, heure
inline constexpr Rect16 header_line = { 0, 40, screen_w, 1 };
inline constexpr Rect16 back_touch = { 0, 0, 46, 44 }; ///< zone tactile du retour : 46 × 44, plus grande que son disque
inline constexpr Rect16 tab_line = { 0, 275, screen_w, 1 };
inline constexpr Rect16 tab_bar_rect = { 0, 276, screen_w, 44 };
inline constexpr uint16_t tab_w = screen_w / 4;
/// Zone des éléments, entre la zone tactile du retour et la barre d'onglets : un élément qui y tient n'est pas sous le retour
inline constexpr Rect16 content = Rect16::fromLTRB(0, back_touch.Bottom() + 1, screen_w, tab_line.Top());
inline constexpr int16_t top = content.Top() + ring; ///< haut du premier corps (46)

/// Rect d'un élément focalisable : son corps agrandi de la marge de l'anneau
constexpr Rect16 with_ring(Rect16 body) {
    return Rect16(body.Left() - ring, body.Top() - ring, body.Width() + 2 * ring, body.Height() + 2 * ring);
}

constexpr bool inside(Rect16 outer, Rect16 r) {
    return r.Left() >= outer.Left() && r.Top() >= outer.Top() && r.Right() <= outer.Right() && r.Bottom() <= outer.Bottom();
}

inline constexpr Rect16 screen_rect = { 0, 0, screen_w, screen_h };
static_assert(inside(screen_rect, header) && inside(screen_rect, tab_bar_rect));
static_assert(header_line.Top() == header.Bottom() + 1 && tab_bar_rect.Top() == tab_line.Bottom() + 1);
static_assert(tab_bar_rect.Bottom() == screen_h - 1, "la barre d'onglets touche le bas de l'écran");
static_assert(tab_w * 4 == screen_w);
static_assert(back_touch.Width() >= 44 && back_touch.Height() >= 44 && tab_bar_rect.Height() >= 44, "cibles tactiles d'au moins 44 px");
static_assert(top == 46 && content.Bottom() == 274);

/// Appelle fill(morceau) pour chaque partie d'area qu'aucun des rects ne couvre : bandes horizontales coupées aux bords
/// des rects, puis les intervalles libres de chaque bande. Plus de max_rects rects : fill(area), tout est repeint.
template <class Fill>
void for_each_gap(Rect16 area, std::span<const Rect16> rects, Fill &&fill) {
    constexpr size_t max_rects = 40;
    if (rects.size() > max_rects) {
        fill(area);
        return;
    }
    std::array<int16_t, 2 * max_rects + 2> edges; // bords hauts et bas+1, triés
    size_t n = 0;
    edges[n++] = area.Top();
    edges[n++] = area.Bottom() + 1;
    for (const Rect16 &r : rects) {
        edges[n++] = std::clamp<int16_t>(r.Top(), area.Top(), area.Bottom() + 1);
        edges[n++] = std::clamp<int16_t>(r.Bottom() + 1, area.Top(), area.Bottom() + 1);
    }
    std::sort(edges.begin(), edges.begin() + n);

    for (size_t e = 0; e + 1 < n; ++e) {
        const int16_t y0 = edges[e], y1 = edges[e + 1];
        if (y0 == y1) {
            continue;
        }
        // Les rects qui touchent la bande la couvrent sur toute sa hauteur (leurs bords sont des bords de bande)
        std::array<std::pair<int16_t, int16_t>, max_rects> spans; // [gauche, droite + 1)
        size_t k = 0;
        for (const Rect16 &r : rects) {
            if (!r.IsEmpty() && r.Top() <= y0 && r.Bottom() >= y1 - 1) {
                spans[k++] = { r.Left(), int16_t(r.Right() + 1) };
            }
        }
        std::sort(spans.begin(), spans.begin() + k);
        int16_t x = area.Left();
        const int16_t right = area.Right() + 1;
        for (size_t s = 0; s < k; ++s) {
            if (const int16_t end = std::min(spans[s].first, right); end > x) {
                fill(Rect16(x, y0, end - x, y1 - y0));
            }
            x = std::max(x, spans[s].second);
        }
        if (x < right) {
            fill(Rect16(x, y0, right - x, y1 - y0));
        }
    }
}

} // namespace od3d::style
