/// @file
/// OneDrop3D (F39) : Réglages › « Mode veille » (feature/od3d/sleep_od3d.hpp) : veille automatique, délai, écran,
/// lumières, chauffes et leur délai, moteurs, ventilateurs, « Mettre en veille maintenant ». Menu de Prusa dessiné en
/// style G (menu_od3d) : doigt et molette ; aucun windowEvent redéfini. « Mettre en veille maintenant » est aussi
/// dans Contrôle (carte de l'accueil).
#pragma once

#include <feature/od3d/sleep_math_od3d.hpp>

#include <basic_screen_menu.hpp>
#include <WindowMenuItems.hpp>
#include <WindowMenuSwitch.hpp>

namespace od3d {

/// Réglages › « Mode veille »
class MI_SLEEP final : public IWindowMenuItem {
public:
    MI_SLEEP();
    void click(IWindowMenu &) override;
};

/// « Mettre en veille maintenant » (Mode veille et Contrôle) : refusé pendant une impression ou une opération
class MI_SLEEP_NOW final : public IWindowMenuItem {
public:
    MI_SLEEP_NOW();
    void click(IWindowMenu &) override;
};

/// Interrupteur d'un réglage du mode veille
class MI_SLEEP_SWITCH : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_SLEEP_SWITCH(bool sleep::Settings::*field, const char *label, const img::Resource *icon);
    void OnChange(size_t) override;

private:
    bool sleep::Settings::*field;
};

/// Délai en minutes (sleep::delays_min, et « Tout de suite » si with_zero)
class MI_SLEEP_MINUTES : public MenuItemSwitch {
public:
    MI_SLEEP_MINUTES(uint8_t sleep::Settings::*field, const char *label, const img::Resource *icon, bool with_zero);

protected:
    bool on_item_selected(const OnItemSelectedArgs &args) override;

private:
    uint8_t sleep::Settings::*field;
    bool with_zero;
};

class MI_SLEEP_ENABLE final : public MI_SLEEP_SWITCH {
public:
    MI_SLEEP_ENABLE();
};

class MI_SLEEP_DELAY final : public MI_SLEEP_MINUTES {
public:
    MI_SLEEP_DELAY();
};

class MI_SLEEP_SCREEN final : public MI_SLEEP_SWITCH {
public:
    MI_SLEEP_SCREEN();
};

class MI_SLEEP_LIGHT final : public MI_SLEEP_SWITCH {
public:
    MI_SLEEP_LIGHT();
};

class MI_SLEEP_HEATERS final : public MI_SLEEP_SWITCH {
public:
    MI_SLEEP_HEATERS();
};

class MI_SLEEP_HEATERS_DELAY final : public MI_SLEEP_MINUTES {
public:
    MI_SLEEP_HEATERS_DELAY();
};

class MI_SLEEP_MOTORS final : public MI_SLEEP_SWITCH {
public:
    MI_SLEEP_MOTORS();
};

class MI_SLEEP_FANS final : public MI_SLEEP_SWITCH {
public:
    MI_SLEEP_FANS();
};

class ScreenSleep final : public BasicScreenMenu<MI_SLEEP_ENABLE, MI_SLEEP_DELAY, MI_SLEEP_SCREEN, MI_SLEEP_LIGHT, MI_SLEEP_HEATERS,
                              MI_SLEEP_HEATERS_DELAY, MI_SLEEP_MOTORS, MI_SLEEP_FANS, MI_SLEEP_NOW> {
public:
    ScreenSleep();
};

} // namespace od3d
