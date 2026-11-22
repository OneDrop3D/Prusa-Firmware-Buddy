/// @file
/// OneDrop3D (F1, interface G) : éléments de menu à nous placés dans des menus de Prusa (en-tête léger, sans écran).
#pragma once

#include <i_window_menu_item.hpp>

namespace od3d {

/// Contrôle › « Axes du MMU » : ouvre les moteurs du MMU (sélecteur, galet, poulie)
class MI_MMU_AXES final : public IWindowMenuItem {
public:
    MI_MMU_AXES();
    void click(IWindowMenu &) override;
};

/// Contrôle › « Musique des moteurs » (F18) : ouvre l'écran de la musique des moteurs
class MI_MOTOR_MUSIC final : public IWindowMenuItem {
public:
    MI_MOTOR_MUSIC();
    void click(IWindowMenu &) override;
};

/// Réglages › Fonctions OneDrop3D › « Tension des courroies » (F14) : ouvre l'écran de mesure par l'accéléromètre
class MI_BELT_TENSION final : public IWindowMenuItem {
public:
    MI_BELT_TENSION();
    void click(IWindowMenu &) override;
};

/// Réglages › Fonctions OneDrop3D › « Vitesses critiques » (F22) : ouvre la carte des vitesses critiques
class MI_SPEED_MAP final : public IWindowMenuItem {
public:
    MI_SPEED_MAP();
    void click(IWindowMenu &) override;
};

/// Réglages › Fonctions OneDrop3D › « Amortissement des axes » (F33) : ζ et Q de X et Y, à appliquer à l'input shaper
class MI_DAMPING final : public IWindowMenuItem {
public:
    MI_DAMPING();
    void click(IWindowMenu &) override;
};

/// Réglages › Fonctions OneDrop3D › « Réponse du ventilateur » (N3) : temps de montée du ventilateur pièce mesuré
/// (M2001), qui règle la ventilation anticipée (F26)
class MI_FAN_RESPONSE final : public IWindowMenuItem {
public:
    MI_FAN_RESPONSE();
    void click(IWindowMenu &) override;
};

/// Réglages › « Impression » : sécurités et réactions pendant l'impression (éléments de Prusa regroupés)
class MI_SET_PRINTING final : public IWindowMenuItem {
public:
    MI_SET_PRINTING();
    void click(IWindowMenu &) override;
};

/// Réglages › « Filament et MMU » : capteurs, MMU, coupe-filament, OpenPrintTag (éléments de Prusa regroupés)
class MI_SET_FILAMENT final : public IWindowMenuItem {
public:
    MI_SET_FILAMENT();
    void click(IWindowMenu &) override;
};

/// Réglages › « Chambre » : filtration, lumières, caméra (éléments de Prusa regroupés)
class MI_SET_CHAMBER final : public IWindowMenuItem {
public:
    MI_SET_CHAMBER();
    void click(IWindowMenu &) override;
};

/// Réglages › Système › « Diagnostic » : outils de développement et de support de Prusa, sortis des autres menus
class MI_DIAGNOSTICS final : public IWindowMenuItem {
public:
    MI_DIAGNOSTICS();
    void click(IWindowMenu &) override;
};

} // namespace od3d
