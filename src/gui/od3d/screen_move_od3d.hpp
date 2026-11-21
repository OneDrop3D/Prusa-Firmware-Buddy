/// @file
/// OneDrop3D (F1, interface G) : « Déplacer les axes » (SPEC_G.md §3.17), à la place de la liste de Prusa
/// (Contrôle > Déplacer axe). Croix XY avec l'origine XY au centre, colonnes Z et E, pas de 0.1 à 50 mm,
/// carte du plateau à toucher. Pas relatifs (G1, diagonales comprises) et carte (G123) aux vitesses du menu Prusa.
/// X et Y sans origine bougent quand même, comme la prise d'origine : arrêt au contact d'un bord (M1990). Z sans
/// origine propose la prise d'origine ; E reste bloqué tant que la buse est froide. « Axes du MMU » ouvre ses moteurs.
#pragma once

#include "screen_temperatures_od3d.hpp" // TextButton
#include "widgets_od3d.hpp"

#include <array>
#include <optional>

namespace od3d {

/// Touche de la croix ou des colonnes : chevron et libellé, ou l'icône de l'origine seule
class MoveKey : public Card {
public:
    enum class Arrow : uint8_t {
        none, ///< icône de l'origine, sur un fond plus clair
        up,
        down,
        left,
        right,
        up_left, ///< diagonales : chevron d'angle seul, sans libellé
        up_right,
        down_left,
        down_right,
    };

    MoveKey(window_t *parent, Rect16 body, Action action, Color back, Arrow arrow, const string_view_utf8 &label = {}, bool arrow_below = false, Color arrow_color = style::text);

    /// Grisée (E avec la buse froide) ; elle reste active pour dire pourquoi elle refuse
    void set_dimmed(bool set);

protected:
    void draw_content(Rect16 body) override;

private:
    string_view_utf8 label;
    Arrow arrow;
    bool arrow_below; ///< libellé au-dessus du chevron (Y-, Z-)
    bool dimmed = false;
    Color arrow_color;
};

/// Pas du déplacement (0.1, 1, 10 ou 50 mm) ; le choisi a le fond et le contour orange de la maquette
class StepChip : public Card {
public:
    StepChip(window_t *parent, uint8_t index);

protected:
    void unconditionalDraw() override;

private:
    uint8_t index;
};

/// Une ligne de texte qui ne se redessine que si son texte ou sa couleur change
class LiveText : public window_t {
public:
    LiveText(window_t *parent, Rect16 rect, Align_t align);

    void set(const char *text, Color color = style::text_muted);

protected:
    void unconditionalDraw() override;

private:
    std::array<char, 24> text {};
    Color color = style::text_muted;
    Align_t align;
};

/// Fond arrondi sous la croix XY
class CrossPanel : public window_t {
public:
    CrossPanel(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

/// Carte du plateau : grille, légende, réticule et point de la tête. Quand la tête bouge, seuls l'ancien
/// réticule et l'ancien point sont effacés. Elle n'est pas active : l'écran reçoit le toucher (pas la molette,
/// que la croix remplace).
class BedMap : public window_t {
public:
    explicit BedMap(window_t *parent);

    /// Centre du point en pixels, nullopt tant que X et Y n'ont pas d'origine
    void set(std::optional<point_i16_t> head);

protected:
    void unconditionalDraw() override;

private:
    void paint(Rect16 r) const; ///< fond, grille et légende dans r
    void draw_grid(Rect16 r) const; ///< grille et légende dans r, sans le fond

    string_view_utf8 hint;
    Rect16 hint_rect;
    std::optional<point_i16_t> head; ///< à dessiner
    std::optional<point_i16_t> drawn; ///< à l'écran
};

class ScreenMove final : public Screen {
public:
    ScreenMove();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();

    // Ordre de construction = ordre de la molette (SPEC_G.md §3.17 : [0] X- … [14] Désactiver les moteurs)
    Header header;
    CrossPanel cross;
    MoveKey x_minus;
    MoveKey x_plus;
    MoveKey y_plus;
    MoveKey y_minus;
    MoveKey origin_xy;
    MoveKey xy_up_left; ///< diagonales dans les coins de la croix : X et Y ensemble
    MoveKey xy_up_right;
    MoveKey xy_down_left;
    MoveKey xy_down_right;
    LiveText z_value;
    MoveKey z_plus;
    MoveKey z_minus;
    LiveText e_value;
    MoveKey extrude;
    MoveKey retract;
    BedMap map;
    LiveText step_title;
    StepChip steps[4];
    LiveText position;
    TextButton origin_all;
    TextButton motors;
    TextButton mmu_axes;
    TabBar tabs;

    uint8_t shown_step; ///< pas dessiné comme choisi
};

} // namespace od3d
