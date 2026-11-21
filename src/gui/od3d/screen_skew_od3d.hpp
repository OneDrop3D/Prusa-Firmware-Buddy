/// @file
/// OneDrop3D (F15) : écran « Équerrage XY » (SPEC_G.md, Mécanique › Équerrage XY), ouvert depuis « Fonctions OneDrop3D ».
/// On imprime un carré de 100 mm, on saisit ses diagonales AC, BD et son côté AD (éditeur de valeur commun), l'écran
/// affiche l'écart calculé, puis « Appliquer » l'ajoute à la correction actuelle (M852 I<k>) ou « Remettre à zéro »
/// l'annule.
/// Mesures et facteur : src/feature/od3d/skew_od3d.hpp.
#pragma once

#include <basic_screen_menu.hpp>
#include <WindowItemFormatableLabel.hpp>
#include <i_window_menu_item.hpp>

namespace od3d {

/// « Fonctions OneDrop3D » › « Équerrage XY » : ouvre l'écran
class MI_SKEW_XY final : public IWindowMenuItem {
public:
    MI_SKEW_XY();
    void click(IWindowMenu &) override;
};

/// Quelle pièce imprimer et comment la mesurer (message)
class MI_SKEW_HOWTO final : public IWindowMenuItem {
public:
    MI_SKEW_HOWTO();
    void click(IWindowMenu &) override;
};

/// Une mesure (0 : diagonale AC, 1 : diagonale BD, 2 : côté AD) ; un clic ouvre l'éditeur de valeur
class MI_SKEW_LENGTH : public WI_LAMBDA_LABEL_t {
public:
    explicit MI_SKEW_LENGTH(uint8_t which);
    void click(IWindowMenu &) override;

private:
    uint8_t which;
};

template <uint8_t which_>
class MI_SKEW_LENGTH_N final : public MI_SKEW_LENGTH {
public:
    MI_SKEW_LENGTH_N()
        : MI_SKEW_LENGTH(which_) {}
};

/// Écart calculé d'après les trois mesures (« -- » si elles sont incohérentes), en lecture seule
class MI_SKEW_MEASURED final : public WI_LAMBDA_LABEL_t {
public:
    MI_SKEW_MEASURED();

protected:
    void Loop() override;

private:
    uint8_t revision; ///< révision des mesures déjà affichée : la valeur se redessine quand une mesure change
};

/// Ajoute l'écart mesuré à la correction actuelle (la pièce a été imprimée avec elle), l'enregistre et l'applique
/// (pas pendant une impression)
class MI_SKEW_APPLY final : public IWindowMenuItem {
public:
    MI_SKEW_APPLY();
    void click(IWindowMenu &) override;
};

/// Correction enregistrée, en degrés, en lecture seule
class MI_SKEW_CURRENT final : public MenuItemAutoUpdatingLabel<float> {
public:
    MI_SKEW_CURRENT();
};

/// Remet la correction à zéro (pas pendant une impression)
class MI_SKEW_RESET final : public IWindowMenuItem {
public:
    MI_SKEW_RESET();
    void click(IWindowMenu &) override;
};

class ScreenSkew final : public BasicScreenMenu<MI_SKEW_HOWTO, MI_SKEW_LENGTH_N<0>, MI_SKEW_LENGTH_N<1>, MI_SKEW_LENGTH_N<2>,
                             MI_SKEW_MEASURED, MI_SKEW_APPLY, MI_SKEW_CURRENT, MI_SKEW_RESET> {
public:
    ScreenSkew();
};

} // namespace od3d
