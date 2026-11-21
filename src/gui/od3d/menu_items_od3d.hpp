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

} // namespace od3d
