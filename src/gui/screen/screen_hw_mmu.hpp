#pragma once

#include <gui/menu_item/specific/menu_items_hw_mmu.hpp>
#include <gui/od3d/screen_ptfe_od3d.hpp> // OneDrop3D (F41)
#include <screen_menu.hpp>

using ScreenMenuHwMmu_ = ScreenMenu<EFooter::Off,
    MI_RETURN,
    MI_MMU_FRONT_PTFE_LENGTH,
    od3d::MI_PTFE_AUTO // OneDrop3D (F41) : mesure automatique
    >;

class ScreenMenuHwMmu final : public ScreenMenuHwMmu_ {
public:
    ScreenMenuHwMmu();
};
