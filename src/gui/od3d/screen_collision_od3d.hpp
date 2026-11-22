/// @file
/// OneDrop3D (F38) : Réglages › Fonctions OneDrop3D › « Sécurité collision » (feature/od3d/collision_od3d.hpp).
/// Interrupteur (coupé par défaut), sensibilité (faible / normale / élevée), action (mettre en pause / prévenir
/// seulement), dernier choc relevé et essai à la buse froide. Menu de Prusa dessiné en style G (menu_od3d) : doigt et
/// molette ; aucun windowEvent redéfini.
#pragma once

#include <basic_screen_menu.hpp>
#include <WindowItemFormatableLabel.hpp>
#include <WindowMenuItems.hpp>
#include <WindowMenuSwitch.hpp>

namespace od3d {

/// Fonctions OneDrop3D › « Sécurité collision »
class MI_COLLISION final : public IWindowMenuItem {
public:
    MI_COLLISION();
    void click(IWindowMenu &) override;
};

class MI_COLLISION_ENABLE final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_COLLISION_ENABLE();
    void OnChange(size_t) override;
};

class MI_COLLISION_SENSITIVITY final : public MenuItemSwitch {
public:
    MI_COLLISION_SENSITIVITY();

protected:
    bool on_item_selected(const OnItemSelectedArgs &args) override;
};

class MI_COLLISION_ACTION final : public MenuItemSwitch {
public:
    MI_COLLISION_ACTION();

protected:
    bool on_item_selected(const OnItemSelectedArgs &args) override;
};

/// Pic de la dernière impulsion relevée [g], mis à jour en direct (réglage des seuils, essai)
class MI_COLLISION_LAST final : public MenuItemAutoUpdatingLabel<int> {
public:
    MI_COLLISION_LAST();
};

/// Essai à la buse froide : 60 s armé hors impression
class MI_COLLISION_TEST final : public IWindowMenuItem {
public:
    MI_COLLISION_TEST();
    void click(IWindowMenu &) override;
};

class ScreenCollision final : public BasicScreenMenu<MI_COLLISION_ENABLE, MI_COLLISION_SENSITIVITY, MI_COLLISION_ACTION, MI_COLLISION_LAST, MI_COLLISION_TEST> {
public:
    ScreenCollision();
};

} // namespace od3d
