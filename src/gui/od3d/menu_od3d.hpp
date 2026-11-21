/// @file
/// OneDrop3D (F1, interface G) : tous les menus Prusa dans le style G (SPEC_G.md §1.3, §1.4, §1.7).
/// screen_menu.hpp s'en sert pour chaque ScreenMenu : l'arbre des réglages garde ses éléments MI_* (et leur
/// logique), seuls l'en-tête, le dessin de la liste et la barre d'onglets changent.
#pragma once

#include "widgets_od3d.hpp"

#include <array>
#include <utility>
#include <i_window_menu.hpp>
#include <i_window_menu_container.hpp>
#include <window_frame.hpp>

namespace od3d {

/// Liste G : 5 lignes de 40 px sous l'en-tête ; la barre de défilement est dans les 6 px de droite
inline constexpr Rect16 list_rect = { style::margin, 34, 470, 200 };
static_assert(style::inside(style::content, list_rect));

/// En-tête G d'un menu Prusa : leur titre en capitales prend la casse de la maquette, coupé à la place disponible.
/// has_back = false dans un dialogue : le retour de l'en-tête fermerait l'écran dessous.
class MenuHeader {
public:
    MenuHeader(window_t *parent, const string_view_utf8 &title, bool has_back = true);

    void SetText(const string_view_utf8 &title);
    void SetIcon(const img::Resource *) {} ///< pas d'icône dans l'en-tête G

private:
    std::array<char, 64> title_text {};
    Header header;
};

/// Dessine le menu qu'elle contient en liste G, à la place du dessin Prusa (partie commune, pour la flash)
class MenuListBase : public window_frame_t {
protected:
    MenuListBase(window_t *parent, Rect16 rect);

    void attach(IWindowMenu &menu);

    void draw() override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    IWindowMenu *list = nullptr;
    uint8_t drawn_rows = 0;
};

/// Remplace WindowExtendedMenu<Menu> : même membre « menu », la molette passe à l'en-tête et aux onglets aux bouts de la liste
template <class Menu>
class MenuList : public MenuListBase {
public:
    template <typename... Args>
    MenuList(window_t *parent, Rect16 rect, Args &&...args)
        : MenuListBase(parent, rect)
        , menu(this, rect, std::forward<Args>(args)...) {
        attach(menu);
    }

    Menu menu;
};

/// Cache les lignes « Retour » : le retour passe par l'en-tête (molette ou toucher) et l'appui long
void hide_return_items(IWinMenuContainer &container);

/// Onglet de la section ouverte (SPEC_G.md §1.4) : Réglages si l'écran des réglages est dans la pile, etc.
TabBar::Tab section_tab();

} // namespace od3d
