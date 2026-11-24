#pragma once

#include <window_header.hpp>
#include <status_footer.hpp>
#include <WinMenuContainer.hpp>
#include <WindowMenuItems.hpp>
#include <window_menu.hpp>
#include <screen.hpp>
#include <gui/od3d/menu_od3d.hpp>

/// Bits of ScreenMenuBase than can be made non-templated to save flash
class ScreenMenuBase_ : public screen_t {

public:
    virtual void InitState(screen_init_variant var) override {
        if (auto pos = var.GetMenuPosition()) {
            i_menu->restore_state(*pos);
        }
    }

    virtual screen_init_variant GetCurrentState() const override {
        screen_init_variant ret;
        ret.SetMenuPosition(i_menu->get_restore_state());
        return ret;
    }

protected:
    ScreenMenuBase_(window_t *parent, const string_view_utf8 &label, [[maybe_unused]] EFooter show_footer, IWindowMenu *menu)
        : screen_t(parent, parent != nullptr ? win_type_t::dialog : win_type_t::normal)
        , header(this, label)
        , i_menu(menu) //
    {
        // Do NOT work with i_menu here, it is not yet constructed!
        SetBackColor(od3d::style::screen); // OneDrop3D (interface G) : fond de la maquette
        if (!parent) {
            od3d::chrome::gap_screen(this); // OneDrop3D : l'en-tête et les onglets de l'écran d'avant peuvent rester
        }
    }

    /// OneDrop3D (interface G) : appui long = retour
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

    /// OneDrop3D (interface G) : le fond seulement entre l'en-tête, la liste et les onglets, qui se peignent eux-mêmes,
    /// de haut en bas avec chacun (od3d::draw_whole)
    void draw() override;
    void unconditionalDraw() override; ///< le fond entre eux d'abord, pour les écrans qui appellent window_frame_t::draw()

protected:
    /// OneDrop3D (interface G) : en-tête G, avec son retour et la ligne « Retour » de la liste ; la barre d'onglets
    /// remplace le pied de page
    od3d::MenuHeader header;

private:
    IWindowMenu *i_menu = nullptr;
};

template <typename Menu>
class ScreenMenuBase : public ScreenMenuBase_ {

protected:
    ScreenMenuBase(window_t *parent, const string_view_utf8 &label, EFooter show_footer)
        : ScreenMenuBase_(parent, label, show_footer, &menu.menu)
        , menu(this, od3d::list_rect)
        , tabs(this, od3d::section_tab()) //
    {
        // OneDrop3D (interface G) : pas de capture, la molette va de l'en-tête à la liste puis aux onglets
    }

protected:
    od3d::MenuList<Menu> menu; ///< OneDrop3D (interface G) : liste G au lieu de WindowExtendedMenu
    od3d::TabBar tabs; ///< après la liste : ordre de la molette
};

template <EFooter FOOTER, class... T>
class ScreenMenu : public ScreenMenuBase<WindowMenu> {
protected:
    // std::array<window_t*,sizeof...(T)> pElements;//todo menu item is not a window
    WinMenuContainer<T...> container;

public:
    /// Number of menu items this screen is composed of (compile-time).
    static constexpr size_t item_count = sizeof...(T);

    /// True if this screen is composed of a menu item of the given type (compile-time).
    template <class ITEM>
    static constexpr bool has_item = (std::is_same_v<ITEM, T> || ...);

    ScreenMenu(const string_view_utf8 &label, window_t *parent = nullptr)
        : ScreenMenuBase(parent, label, FOOTER) {
        // OneDrop3D (interface G) : un seul retour, la flèche de l'en-tête (avec l'appui long et le glissement). La ligne
        // « Retour » le doublait (retour de l'utilisateur, 24/11 : « des menus en double ») et prenait une ligne de la liste.
        // Les écrans sans retour dans l'en-tête (langue au 1er démarrage) cachent déjà la leur.
        if constexpr (has_item<MI_RETURN>) {
            Item<MI_RETURN>().hide();
        }
        menu.menu.BindContainer(container);
    }

    // compile time access by index
    template <std::size_t I>
    decltype(auto) Item() {
        return std::get<I>(container.menu_items);
    }
    // compile time access by type
    template <class TYPE>
    decltype(auto) Item() {
        return std::get<TYPE>(container.menu_items);
    }

    template <class ITEM1, class ITEM2>
    bool SwapVisibility() {
        return menu.menu.SwapVisibility(Item<ITEM1>(), Item<ITEM2>());
    }

    // ShowDevOnly intentionally not supported, can be set only in ctor
};
