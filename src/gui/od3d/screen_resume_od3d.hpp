/// @file
/// OneDrop3D (F30) : Réglages › Fonctions OneDrop3D › « Reprise sans cicatrice » (feature/od3d/clean_resume_od3d.hpp).
/// Interrupteur (coupé par défaut), purge, rétraction, vitesse de la dernière descente. Menu de Prusa dessiné en
/// style G (menu_od3d) : doigt et molette ; aucun windowEvent redéfini.
#pragma once

#include <basic_screen_menu.hpp>
#include <WindowMenuItems.hpp>
#include <WindowMenuSpin.hpp>

namespace od3d {

/// Fonctions OneDrop3D › « Reprise sans cicatrice »
class MI_CLEAN_RESUME final : public IWindowMenuItem {
public:
    MI_CLEAN_RESUME();
    void click(IWindowMenu &) override;
};

class MI_CLEAN_RESUME_ENABLE final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_CLEAN_RESUME_ENABLE();
    void OnChange(size_t) override;
};

class MI_CLEAN_RESUME_PURGE final : public WiSpin {
public:
    MI_CLEAN_RESUME_PURGE();
    void OnClick() override;
};

class MI_CLEAN_RESUME_RETRACT final : public WiSpin {
public:
    MI_CLEAN_RESUME_RETRACT();
    void OnClick() override;
};

class MI_CLEAN_RESUME_APPROACH final : public WiSpin {
public:
    MI_CLEAN_RESUME_APPROACH();
    void OnClick() override;
};

class ScreenCleanResume final : public BasicScreenMenu<MI_CLEAN_RESUME_ENABLE, MI_CLEAN_RESUME_PURGE, MI_CLEAN_RESUME_RETRACT, MI_CLEAN_RESUME_APPROACH> {
public:
    ScreenCleanResume();
};

} // namespace od3d
