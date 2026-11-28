/// @file
/// OneDrop3D (F1, interface G) : palette et géométrie communes à tous nos écrans (SPEC_G.md §1).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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
inline constexpr Color text_tertiary = Color::from_raw(0x7D8188); ///< légendes, « stable », PA connu
inline constexpr Color text_danger = Color::from_raw(0xFF8A73);
inline constexpr Color choice = Color::from_raw(0x3A2618); ///< fond d'un choix sélectionné, ligne réglée à la molette
inline constexpr Color target = Color::from_raw(0x5B5F66); ///< ligne de cible des courbes, contour « Annuler »
inline constexpr Color handle = Color::from_raw(0x5B5B5B); ///< poignée de la fiche, cadre « MMU n » de l'assistant
inline constexpr Color grid = Color::from_raw(0x2C3036); ///< grille de la carte du plateau
inline constexpr Color crosshair = Color::from_raw(0x824223); ///< #F8651B à 45 % sur la carte du plateau
inline constexpr Color filament_green = Color::from_raw(0x2E9E5B); ///< vert des filaments proposés, mesure réussie
// Courbes et jauges : une couleur par rôle
inline constexpr Color curve_blue = Color::from_raw(0x4FA3F7);
inline constexpr Color curve_green = Color::from_raw(0x5FBF6A);
inline constexpr Color curve_grey = Color::from_raw(0x8A8E94);
inline constexpr Color band_green = Color::from_raw(0x2E5A3A); ///< plage conseillée sur une jauge
inline constexpr Color band_tint = Color::from_raw(0x2B3A30); ///< plage conseillée sous une courbe
/// Voile uni derrière nos fenêtres (frost_od3d.hpp) : le fond × 0,45, le noir à 55 % de la maquette
inline constexpr Color veil = Color::from_raw(0x0C0C0D);

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
inline constexpr Color tile_network = Color::from_raw(0x1F4436);

// Géométrie (SPEC_G.md §1.3, §1.4, §1.11)
inline constexpr uint16_t screen_w = 480;
inline constexpr uint16_t screen_h = 320;
inline constexpr int16_t margin = 8; ///< marges gauche et droite
inline constexpr uint8_t ring = 2; ///< anneau de focus, dessiné dans une marge de 2 px autour du corps
// Arrondis normés (retour de l'utilisateur, 21/11 : « les bords des cases ne sont pas normés ») : trois tailles
// seulement, plus les pastilles (moitié de la hauteur)
inline constexpr uint8_t radius_small = 8; ///< petites tuiles d'icône, bouton des températures, pastille d'onglet
inline constexpr uint8_t radius = 12; ///< cartes, boutons, listes, panneaux, feuilles, dialogues
inline constexpr uint8_t radius_big = radius; ///< grandes cartes : même arrondi que les autres (14 avant)

// En-tête de 40 px (26 dans la maquette, jugé trop petit sur l'imprimante) ; barre d'onglets de 44 px pour le compenser
inline constexpr Rect16 header = { 0, 0, screen_w, 40 }; ///< en-tête : retour, titre, températures, réseau, heure
inline constexpr Rect16 header_line = { 0, 40, screen_w, 1 };
inline constexpr Rect16 back_touch = { 0, 0, 46, 44 }; ///< zone tactile du retour : 46 × 44, plus grande que son disque
/// Sous l'en-tête : ce que couvrent nos fenêtres et leur voile (l'en-tête reste vivant au-dessus)
inline constexpr Rect16 below_header = Rect16::fromLTRB(0, header_line.Bottom() + 1, screen_w, screen_h);
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

inline constexpr size_t max_gap_rects = 40;

/// Appelle fill(morceau) pour chaque partie d'area qu'aucun des rects ne couvre : bandes horizontales coupées aux bords
/// des rects, puis les intervalles libres de chaque bande. Plus de max_gap_rects rects : fill(area), tout est repeint.
template <class Fill>
void for_each_gap(Rect16 area, std::span<const Rect16> rects, Fill &&fill) {
    constexpr size_t max_rects = max_gap_rects;
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

/// Peint area d'une passe, de haut en bas : chaque bande du fond (fill) vient juste après les éléments qui commencent à
/// sa hauteur ou au-dessus (draw(i), qui repeint tout rects[i]). Un écran n'apparaît plus en grille de fond vide avant
/// son contenu (retour du 22/11). Un élément qui en chevauche un plus ancien est peint après lui, comme chez Prusa.
template <class Draw, class Fill>
void paint_top_down(Rect16 area, std::span<const Rect16> rects, Draw &&draw, Fill &&fill) {
    if (rects.size() > max_gap_rects) {
        fill(area); // comme for_each_gap : tout le fond, puis les éléments
        for (size_t i = 0; i < rects.size(); ++i) {
            draw(i);
        }
        return;
    }
    std::array<bool, max_gap_rects> done {};
    const auto draw_one = [&](const auto &self, size_t k) -> void {
        for (size_t i = 0; i < k; ++i) {
            if (!done[i] && rects[i].HasIntersection(rects[k])) {
                self(self, i);
            }
        }
        done[k] = true;
        draw(k);
    };
    const auto draw_above = [&](int16_t y_end) { // les éléments qui commencent avant y_end, le plus haut d'abord
        for (;;) {
            size_t pick = rects.size();
            for (size_t i = 0; i < rects.size(); ++i) {
                if (!done[i] && rects[i].Top() < y_end && (pick == rects.size() || rects[i].Top() < rects[pick].Top())) {
                    pick = i;
                }
            }
            if (pick == rects.size()) {
                return;
            }
            draw_one(draw_one, pick);
        }
    };
    for_each_gap(area, rects, [&](Rect16 gap) {
        draw_above(gap.Bottom() + 1);
        fill(gap);
    });
    draw_above(INT16_MAX);
}

/// Carré de la distance de r au point p (0 dedans)
constexpr int distance2(Rect16 r, point_i16_t p) {
    const int dx = std::max({ r.Left() - p.x, 0, p.x - r.Right() });
    const int dy = std::max({ r.Top() - p.y, 0, p.y - r.Bottom() });
    return dx * dx + dy * dy;
}

inline constexpr int wave_step = 48; ///< largeur d'un anneau de la peinture en onde, en px

/// Comme paint_top_down, mais en onde depuis le point from (l'élément touché) : par anneaux de wave_step px, les
/// éléments qu'il atteint puis les bandes du fond qui y sont. On anime l'ordre de peinture, pas le contenu : chaque pixel
/// est peint une fois, sans un octet de plus, et l'animation dure le temps du dessin.
template <class Draw, class Fill>
void paint_from(Rect16 area, std::span<const Rect16> rects, point_i16_t from, Draw &&draw, Fill &&fill) {
    if (rects.size() > max_gap_rects) {
        paint_top_down(area, rects, draw, fill);
        return;
    }
    std::array<bool, max_gap_rects> done {};
    const auto draw_one = [&](const auto &self, size_t k) -> void {
        for (size_t i = 0; i < k; ++i) {
            if (!done[i] && rects[i].HasIntersection(rects[k])) {
                self(self, i); // un élément qui en chevauche un plus ancien est peint après lui
            }
        }
        done[k] = true;
        draw(k);
    };
    const int far_x = std::max(from.x - area.Left(), area.Right() - from.x), far_y = std::max(from.y - area.Top(), area.Bottom() - from.y);
    const int far = far_x * far_x + far_y * far_y; // le coin de area le plus loin
    for (int ring = 0, inner = -1; inner < far; inner = ring * ring, ring += wave_step) {
        for (size_t i = 0; i < rects.size(); ++i) {
            if (!done[i] && distance2(rects[i], from) <= ring * ring) {
                draw_one(draw_one, i);
            }
        }
        for_each_gap(area, rects, [&](Rect16 gap) {
            if (const int d = distance2(gap, from); d > inner && d <= ring * ring) {
                fill(gap);
            }
        });
    }
    for (size_t i = 0; i < rects.size(); ++i) {
        if (!done[i]) {
            draw_one(draw_one, i); // hors de area
        }
    }
}

/// Rectangle arrondi rect (coins de rayon r, contour de bw px ≤ r) peint autour de parts, que leur propriétaire repeint
/// lui-même en entier : plain(morceau, true) pour l'intérieur uni, plain(morceau, false) pour les traits droits du
/// contour, corner(morceau) pour ce qui reste des 4 carrés r × r des coins (lissés). Chaque pixel de rect hors de parts
/// une fois. Trop de parts : rect entier (le contenu se peint ensuite par-dessus, comme avant).
template <class Plain, class Corner>
void for_each_frame_piece(Rect16 rect, uint8_t r, uint8_t bw, std::span<const Rect16> parts, Plain &&plain, Corner &&corner) {
    r = uint8_t(std::min<int>({ r, rect.Width() / 2, rect.Height() / 2 }));
    bw = std::min(bw, r);
    if (parts.size() + 8 > max_gap_rects) {
        parts = {};
    }
    const int16_t l = rect.Left(), t = rect.Top(), w = rect.Width(), h = rect.Height();
    const Rect16 squares[] = { { l, t, r, r }, { int16_t(l + w - r), t, r, r }, { l, int16_t(t + h - r), r, r }, { int16_t(l + w - r), int16_t(t + h - r), r, r } };
    const Rect16 lines[] = { { int16_t(l + r), t, uint16_t(w - 2 * r), bw }, { int16_t(l + r), int16_t(t + h - bw), uint16_t(w - 2 * r), bw },
        { l, int16_t(t + r), bw, uint16_t(h - 2 * r) }, { int16_t(l + w - bw), int16_t(t + r), bw, uint16_t(h - 2 * r) } };
    std::array<Rect16, max_gap_rects> keep;
    size_t n = std::copy(parts.begin(), parts.end(), keep.begin()) - keep.begin();
    for (const Rect16 &k : squares) {
        keep[n++] = k;
    }
    for (const Rect16 &k : lines) {
        keep[n++] = k;
    }
    for_each_gap(rect, std::span<const Rect16>(keep.data(), n), [&](Rect16 p) { plain(p, true); });
    for (const Rect16 &line : lines) {
        if (!line.IsEmpty()) {
            for_each_gap(line, parts, [&](Rect16 p) { plain(p, false); });
        }
    }
    for (const Rect16 &square : squares) {
        if (!square.IsEmpty()) {
            for_each_gap(square, parts, corner);
        }
    }
}

/// Degrés entiers affichés pour la mesure t, quand shown est à l'écran : l'arrondi ne change qu'au-delà de 0,7 °C
/// d'écart (hystérésis de 0,2 °C autour de x,5), sinon les chiffres sautaient à chaque oscillation de la mesure
inline int16_t held_degrees(float t, int16_t shown) {
    return std::abs(t - shown) < 0.7f ? shown : static_cast<int16_t>(std::lround(t));
}

/// INDX sans tête prise (IF-03) : la buse à montrer parmi les mesures des têtes, la plus chaude (la première à égalité).
/// Une tête posée lit -1 °C : toutes posées, la première, montrée « --° »
inline size_t hottest(std::span<const float> temps) {
    return temps.empty() ? 0 : size_t(std::max_element(temps.begin(), temps.end()) - temps.begin());
}

} // namespace od3d::style
