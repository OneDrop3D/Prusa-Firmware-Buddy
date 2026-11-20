/// @file
#include "screen_home_od3d.hpp"

#include <i18n.h>
#include <img_resources.hpp>
#include <ScreenFactory.hpp>
#include <ScreenHandler.hpp>
#include <screen_filebrowser.hpp>
#include <screen_menu_control.hpp>
#include <screen_menu_filament_mmu.hpp>
#include <screen_menu_settings.hpp>
#include <screen_menu_temperature_and_fans.hpp>

namespace od3d {

namespace {

    // 480 × 320 : en-tête de 32 px en haut, barre d'onglets en bas, cartes juste au-dessus.
    constexpr int16_t marge = 8;
    constexpr uint16_t onglet_l = 480 / 4, onglet_h = 52;
    constexpr int16_t onglet_y = 320 - onglet_h;
    constexpr uint16_t carte_l = (480 - 4 * marge) / 3, carte_h = 52;
    constexpr int16_t carte_y = onglet_y - marge - carte_h;

    constexpr Rect16 carte(int i) {
        return Rect16(marge + i * (carte_l + marge), carte_y, carte_l, carte_h);
    }
    constexpr Rect16 onglet(int i) {
        return Rect16(i * onglet_l, onglet_y, onglet_l, onglet_h);
    }

    template <class Screen>
    void ouvrir(window_t &) {
        Screens::Access()->Open(ScreenFactory::Screen<Screen>);
    }

    // Étape 1 : l'accueil s'ouvre depuis « Fonctions OneDrop3D » ; « Accueil » et l'appui long y reviennent.
    // À l'étape 2, il devient l'écran de démarrage.
    void revenir(window_t &) {
        Screens::Access()->close_screen();
    }

} // namespace

ScreenHome::ScreenHome()
    : header(this, _("Home"))
    , cards {
        { this, carte(0), ouvrir<screen_filebrowser_data_t>, _("Files") },
        { this, carte(1), ouvrir<ScreenMenuTemperatureAndFans>, _("Temperatures") },
        { this, carte(2), ouvrir<ScreenMenuControl>, _("Control") },
    }
    , tabs {
        { this, onglet(0), revenir, _("Home") },
        { this, onglet(1), ouvrir<screen_filebrowser_data_t>, _("Print") },
        { this, onglet(2), ouvrir<ScreenMenuFilamentMMU>, _("Workshop") },
        { this, onglet(3), ouvrir<ScreenMenuSettings>, _("Settings") },
    } {
    header.SetIcon(&img::home_shape_16x16);
    cards[0].SetFocus(); // sans enfant focalisé, le cadre ignore les touchers
}

void ScreenHome::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::HELD_RELEASED) {
        revenir(*this);
        return;
    }
    screen_t::windowEvent(sender, event, param);
}

} // namespace od3d
