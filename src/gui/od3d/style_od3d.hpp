/// @file
/// OneDrop3D (F1, interface G) : palette et géométrie communes à tous nos écrans (SPEC_G.md §1).
#pragma once

#include <Rect16.h>
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
inline constexpr Color focus_ring = COLOR_WHITE;

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

inline constexpr Rect16 header = { 0, 0, screen_w, 26 }; ///< en-tête : titre, températures, réseau, heure
inline constexpr Rect16 header_line = { 0, 26, screen_w, 1 };
inline constexpr Rect16 tab_line = { 0, 272, screen_w, 1 };
inline constexpr Rect16 tab_bar_rect = { 0, 273, screen_w, 47 };
inline constexpr uint16_t tab_w = screen_w / 4;
/// Zone libre entre l'en-tête et la barre d'onglets
inline constexpr Rect16 content = Rect16::fromLTRB(0, header_line.Bottom() + 1, screen_w, tab_line.Top());

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

} // namespace od3d::style
