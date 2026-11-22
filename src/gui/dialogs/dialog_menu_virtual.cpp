/// @file
#include "dialog_menu_virtual.hpp"

#include <ScreenHandler.hpp>
#include <IDialog.hpp>
#include <gui/od3d/menu_od3d.hpp>
#include <WindowMenuItems.hpp>
#include <img_resources.hpp>

namespace dialog_menu_virtual {

// Size-sensitive
static_assert(sizeof(MenuItem) == 56);

// This dialog gets allocated on the stack, so we want to have the WindowMenuVirtual as small as possible
// This means ditching the default alloc buffer size for the extact MenuItem sizeof, at the cost of extra flash usage
class Menu final : public MenuBase {

public:
    Menu(window_t *parent, Rect16 rect, const Config &config)
        : WindowMenuVirtualSized(parent, rect, CloseScreenReturnBehavior::yes)
        , config(config) {
        setup_items();

        if (config.focused_item >= 0) {
            move_focus_to_index(config.focused_item);
        }
    }

    int item_count() const final {
        return config.item_count;
    }

    void handle_item_click(IWindowMenuItem &item) {
        result = *item_index(&item);
        Screens::Access()->close_dialog();
    }

protected:
    void setup_item(ItemVariant &variant, int index) final {
        config.setup_f(index, variant);
    }

public:
    const Config &config;
    std::optional<int> result;
};

class Dialog final : public IDialog {

public:
    Dialog(const Config &config)
        : IDialog(GuiDefaults::RectScreen)
        , header_(this, config.title, false)
        , menu_(this, od3d::list_rect, config) //
    {
        SetBackColor(od3d::style::screen); // OneDrop3D (interface G) : en-tête et liste G
        CaptureNormalWindow(menu_);
    }

    std::optional<int> result() const {
        return menu_.menu.result;
    }

private:
    od3d::MenuHeader header_;
    od3d::MenuList<Menu, false> menu_; ///< OneDrop3D : le « Retour » reste, seule sortie sans choisir (en-tête sans retour)
};

// OneDrop3D : le dialogue est sur la pile de l'interface (6 Ko). En G (en-tête et liste od3d), 1 064 octets mesurés le
// 22/11, contre 1 248 pour celui de Prusa (window_header_t + WindowExtendedMenu) : il ne doit pas dépasser l'original
static_assert(sizeof(Dialog) <= 1248, "dialogue de liste de choix plus gros que celui de Prusa : pile de l'interface");

void MenuItem::click(IWindowMenu &menu) {
    handle_click(*this, menu);
}

std::optional<int> execute(const Config &config) {
    Dialog dlg(config);
    Screens::Access()->gui_loop_until_dialog_closed();
    return dlg.result();
}

void MenuItem::handle_click(IWindowMenuItem &item, IWindowMenu &menu) {
    static_cast<Menu &>(menu).handle_item_click(item);
}

} // namespace dialog_menu_virtual
