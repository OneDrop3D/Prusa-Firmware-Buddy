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
#include <type_traits>
#include <window_frame.hpp>
#include <window_menu_virtual.hpp>

namespace od3d {

/// Liste G : 5 lignes de 40 px sous l'en-tête, puis le haut de la 6e jusqu'à la ligne des onglets (retour du 23/11 : la
/// bande vide de 29 px ne disait pas qu'il y avait d'autres lignes ; photo du 24/11 : la demi-ligne laissait encore une
/// bande vide de 9 px au-dessus des onglets) ; la barre de défilement est dans les 6 px de droite.
/// Le menu Prusa compte 5 lignes (229 / 40) : le focus qui arrive sur la ligne coupée fait défiler d'une ligne.
inline constexpr Rect16 list_rect = Rect16::fromLTRB(style::margin, style::top, style::margin + 470, style::tab_line.Top());
static_assert(style::inside(style::content, list_rect));

/// En-tête G d'un menu Prusa : leur titre en capitales prend la casse de la maquette, coupé à la place disponible.
/// Deux façons de revenir (décision du 23/11) : le retour de l'en-tête, au style de la ligne « ‹ Retour » de la liste,
/// et cette ligne, que le balayage touche aussi (WindowMenu). has_back : le retour de l'en-tête, et l'appui long ferme
/// l'écran (ScreenMenuBase_) ; false dans un dialogue et au 1er démarrage (langue, configuration de l'imprimante).
class MenuHeader {
public:
    MenuHeader(window_t *parent, const string_view_utf8 &title, bool has_back = true, bool compact = false);

    void SetText(const string_view_utf8 &title);
    void show_back(bool show);
    bool back_shown() const {
        return back;
    }
    void SetIcon(const img::Resource *) {} ///< pas d'icône dans l'en-tête G
    void set_show_bed_info(bool) {} ///< écrans FSM de Prusa : l'en-tête G a déjà les températures

private:
    std::array<char, 64> title_text {};
    Header header;
    bool back;
};

/// Dessine le menu qu'elle contient en liste G, à la place du dessin Prusa (partie commune, pour la flash).
/// Sous les lignes entières, la place qui reste (moins d'une ligne) montre le haut de la ligne suivante, coupé net ;
/// rien au bout de la liste. La toucher fait défiler d'une ligne et la cible, sans l'activer : on ne lance pas ce
/// qu'on ne voit qu'à moitié, à quelques pixels des onglets.
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

/// Remplace WindowExtendedMenu<Menu> : même membre « menu », la molette passe à l'en-tête et aux onglets aux bouts de la liste.
template <class Menu>
class MenuList : public MenuListBase {
public:
    template <typename... Args>
    MenuList(window_t *parent, Rect16 rect, Args &&...args)
        : MenuListBase(parent, rect)
        , menu(this, rect, std::forward<Args>(args)...) {
        if constexpr (std::is_base_of_v<WindowMenuVirtualBase, Menu>) {
            menu.set_peek_item(); // la liste virtuelle prépare la demi-ligne
        }
        attach(menu);
    }

    Menu menu;
};

/// Onglet de la section ouverte (SPEC_G.md §1.4) : Réglages si l'écran des réglages est dans la pile, etc.
TabBar::Tab section_tab();

} // namespace od3d
