/// @file
/// OneDrop3D (interface G) : fond derrière nos fenêtres (fiche de bobine, éditeur de valeur, chargement).
///
/// Décision de l'utilisateur du 23/11 (AUDIT_UI.md, lot C) : un voile uni à la place du flou givré du 22/11. Le flou
/// relisait l'écran (SPI à 5,25 MHz : 75 à 145 ms par ouverture) et la fermeture repeignait tout l'écran. Le voile
/// (style::veil) ne relit jamais l'écran : il n'est peint qu'autour de la fenêtre (~20 ms). À la fermeture, seule la
/// place du voile et de la fenêtre est repeinte, d'une passe.
///
/// Retour du 25/11 (« sur la partie du haut où on voit normalement le fond, on ne voyait que du noir ») : ce voile
/// était opaque (#0C0C0D) et cachait l'écran dessous, alors que la maquette le veut assombri mais visible. Une fenêtre
/// posée sur un écran (fiche, éditeur) le laisse donc tel quel, figé, et ne peint qu'un liseré sombre de 2 px au-dessus
/// d'elle ; seul l'écran de chargement, qui n'a pas d'écran dessous, peint encore le voile autour de son cadre.
///
/// La fenêtre s'ouvre depuis l'élément touché : son corps grandit en 4 images depuis le rect de cet élément, et chaque
/// image ne peint que la couronne nouvelle et ses coins arrondis (r × r). Chaque élément de la fenêtre est peint une
/// fois, à sa place, à l'image où le corps l'atteint : rien ne bouge, rien n'est peint deux fois. Les images se suivent
/// sans attente (l'ouverture dure le temps du dessin), et la molette tournée pendant ce temps saute à la dernière.
///
/// Partie pure (start, step, plan) testée seule ; l'afficheur est dans frost_od3d.cpp.
#pragma once

#include "style_od3d.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

class window_t;

namespace od3d::frost {

inline constexpr uint8_t radius = style::radius; ///< coins de toutes nos fenêtres sur le voile
inline constexpr int frames = 4; ///< images de l'ouverture
inline constexpr size_t max_parts = 16; ///< éléments d'une fenêtre peints chacun une fois ; au-delà, peints sur le corps

/// Rect de la première image : origin (l'élément touché) ramené dans to, d'au moins side × side ; depuis le centre de
/// to si origin est vide ou n'est pas un élément (la moitié de to ou plus : une liste entière). origin == to : to (redessin)
Rect16 start(Rect16 origin, Rect16 to, uint16_t side);

/// Rect de l'image k sur n, de from (k = 0) à to (k = n), en ralentissant ; chacun contient le précédent si from est dans to
Rect16 step(Rect16 from, Rect16 to, int k, int n);

/// Ce que l'ouverture peint, dans l'ordre
class Painter {
public:
    virtual void part(size_t i) = 0; ///< repeint tout parts[i]
    virtual void body(Rect16 r) = 0; ///< corps uni
    /// piece (dans square) du coin bit (MIC_*) de carré square : le corps lissé sur le voile hors de l'arrondi
    virtual void corner(Rect16 square, uint8_t bit, Rect16 piece) = 0;
    virtual void veil(Rect16 r) = 0;
    /// Une entrée de l'utilisateur : la dernière image tout de suite
    virtual bool stop() {
        return false;
    }

protected:
    ~Painter() = default;
};

/// Ouverture de la fenêtre rect (coins corners en MIC_* arrondis de radius), depuis from (start) ; puis, si veil, le
/// voile autour dans style::below_header. parts : ce que la fenêtre peint elle-même (dans rect), chacun peint une fois,
/// avec ceux qui le chevauchent, à l'image où le corps l'atteint ; le corps est peint autour d'eux.
void plan(Rect16 rect, uint8_t corners, Rect16 from, std::span<const Rect16> parts, Painter &painter, bool veil);

// ---- Sur l'imprimante ----

/// Rect de l'élément ciblé (touché ou cliqué), d'où s'ouvre la fenêtre qu'il ouvre ; vide sans élément
Rect16 origin();

/// Ouvre la fenêtre sur l'afficheur (plan) : paint(ctx, i) repeint tout parts[i]. veil : le voile autour (écran sans
/// écran dessous : le chargement) ; sinon l'écran dessous reste visible et un liseré sombre borde le haut de la fenêtre
using PaintPart = void (*)(void *ctx, size_t i);
void open(Rect16 rect, uint8_t corners, Rect16 from, std::span<const Rect16> parts, PaintPart paint, void *ctx, bool veil = false);

/// draw() d'un cadre posé sur un écran ou sur le voile (fiche, cadre du chargement), à repeindre en entier : open() avec
/// ses fenêtres visibles (first et les suivantes, qui repeignent chacune tout leur rect)
void draw(window_t *first, Rect16 rect, uint8_t corners, Rect16 from, bool veil = false);

/// Une fenêtre posée sur le voile vient d'être détruite : l'écran dessous repeint d'une passe la place qu'elle couvrait
/// (style::below_header), en-tête épargné ; tout l'écran au prochain dessin si ce n'est pas possible (un autre dialogue,
/// un écran qui va s'ouvrir). Note l'élément ciblé de l'écran (take_closed_origin).
void close();

/// L'élément ciblé à la dernière fermeture (la bobine dont la fiche lance un chargement), s'il date de moins de 3 s ;
/// vide sinon. Une seule fois.
Rect16 take_closed_origin();

} // namespace od3d::frost
