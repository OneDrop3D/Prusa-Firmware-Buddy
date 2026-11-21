/// @file
#include "menu_od3d.hpp"

#include "screen_files_od3d.hpp"
#include "screen_mmu_od3d.hpp"
#include "screen_printing_od3d.hpp"
#include "text_od3d.hpp"

#include <algorithm>
#include <cstring>
#include <display.hpp>
#include <iterator>
#include <i_window_menu_item.hpp>
#include <img_resources.hpp>
#include <raii/auto_restore.hpp>
#include <screen_menu_filament.hpp>
#include <screen_menu_filament_mmu.hpp>
#include <screen_menu_settings.hpp>
#include <WindowMenuItems.hpp>

namespace window_menu_item_private {
// Élément ciblé, défini dans i_window_menu_item.cpp (voir draw_row)
extern IWindowMenuItem *focused_menu_item;
} // namespace window_menu_item_private

namespace od3d {

namespace {

    constexpr Color tile_network = Color::from_raw(0x1F4436);
    constexpr Color scrollbar = style::text_faint;
    constexpr uint16_t row_w = 464; ///< le cadre : x 8 à 472
    constexpr int16_t tile_x = 12, tile_size = 26, label_x = 48, value_right = 12;

    /// Accès aux membres protégés d'IWindowMenuItem, pour dessiner ses parties dans le style G sans toucher à Prusa :
    /// un pointeur de membre formé par une classe dérivée vaut pour tout IWindowMenuItem
    struct ItemAccess : IWindowMenuItem {
        static const img::Resource *icon(const IWindowMenuItem &item) {
            return item.*(&ItemAccess::id_icon);
        }
        static Rect16 extension_rect(const IWindowMenuItem &item, Rect16 content) {
            return (item.*(&ItemAccess::getExtensionRect))(content);
        }
        static void print_extension(const IWindowMenuItem &item, Rect16 rect, Color fg, Color bg) {
            (item.*(&ItemAccess::printExtension))(rect, fg, bg, ropfn {});
        }
    };

    /// Couleur de la tuile d'après la famille de l'icône (SPEC_G.md §1.1)
    Color tile_color(const img::Resource *icon) {
        static constexpr const img::Resource *heat[] = { &img::temperature_16x16, &img::heatbed_16x16, &img::preheat_white_16x16, &img::nozzle_16x16, &img::fan_16x16, &img::turbine_16x16, &img::enclosure_16x16 };
        static constexpr const img::Resource *filament[] = { &img::spool_16x16, &img::finda_16x16, &img::filament_sensor_17x16, &img::user_filament_16x16 };
        static constexpr const img::Resource *network[] = { &img::wifi_16x16, &img::lan_16x16, &img::connect_16x16 };
        static constexpr const img::Resource *motion[] = { &img::calibrate_white_16x16, &img::selftest_16x16, &img::wizard_16x16, &img::z_axis_16x16, &img::move_16x16, &img::speed_16x16, &img::cycle_16x16 };
        const auto in = [icon](const auto &family) { return std::find(std::begin(family), std::end(family), icon) != std::end(family); };
        return in(heat) ? style::tile_heat : in(filament) ? style::tile_filament
            : in(network)                                ? tile_network
            : in(motion)                                 ? style::tile_motion
                                                         : style::tile;
    }

    /// Libellé sur une ligne, ou deux si le texte en a deux (nom, puis explication en gris)
    void draw_label(Rect16 rect, const string_view_utf8 &label, Color fg, Color bg) {
        std::array<char, 160> buf;
        label.copyToRAM(buf.data(), buf.size());
        char *second = strchr(buf.data(), '\n');
        if (second) {
            *second++ = '\0';
        }
        const size_t chars = rect.Width() / width(Font::small);
        text::ellipsize(buf.data(), chars);
        const uint16_t h = height(Font::small);
        if (!second) {
            draw_text(Rect16(rect.Left(), rect.Top() + (rect.Height() - h) / 2, rect.Width(), h), string_view_utf8::MakeRAM(buf.data()), Font::small, fg, bg);
            return;
        }
        text::ellipsize(second, chars);
        draw_text(Rect16(rect.Left(), rect.Top() + 1, rect.Width(), h), string_view_utf8::MakeRAM(buf.data()), Font::small, fg, bg);
        draw_text(Rect16(rect.Left(), rect.Top() + 18, rect.Width(), h), string_view_utf8::MakeRAM(second), Font::small, style::text_muted, bg);
    }

    /// Une ligne de la liste (SPEC_G.md §1.7) ; extension_only : seule la valeur a changé
    void draw_row(IWindowMenuItem &item, Rect16 row, bool first, bool last, bool extension_only) {
        const bool focused = item.is_focused();
        const bool enabled = item.IsEnabled();
        const Color bg = focused ? style::focused(style::surface) : style::surface;

        // Contenu entre l'anneau (2 px en haut et en bas) et les marges de 12 px
        const Rect16 content(row.Left() + label_x, row.Top() + 2, row.Width() - label_x - value_right, row.Height() - 4);
        Rect16 extension = ItemAccess::extension_rect(item, content);
        if (extension.Left() < content.Left() + 8 * width(Font::small)) {
            extension = Rect16::fromLTRB(content.Left() + 8 * width(Font::small), extension.Top(), content.Left() + content.Width(), extension.Top() + extension.Height());
        }
        const bool has_extension = extension.Width() > 0 && (enabled || item.DoesShowDisabledExtension());

        if (!extension_only) {
            // Ligne ciblée : fond #34373C (SPEC_G.md §1.6), sans anneau ; une passe, le trait par-dessus sa 1re ligne
            const uint8_t corners = (first ? MIC_TOP_LEFT | MIC_TOP_RIGHT : 0) | (last ? MIC_BOT_LEFT | MIC_BOT_RIGHT : 0);
            if (first) {
                display::draw_rounded_rect(row, style::screen, bg, style::radius_big, corners);
            } else {
                display::fill_rect(Rect16(row.Left(), row.Top(), row.Width(), 1), style::line);
                display::draw_rounded_rect(Rect16(row.Left(), row.Top() + 1, row.Width(), row.Height() - 1), style::screen, bg, style::radius_big, corners);
            }

            const img::Resource *icon = ItemAccess::icon(item);
            if (!icon) {
                icon = &img::settings_16x16;
            }
            const Color tile = tile_color(icon);
            const Rect16 tile_rect(row.Left() + tile_x, row.Top() + (row.Height() - tile_size) / 2, tile_size, tile_size);
            display::draw_rounded_rect(tile_rect, bg, tile, 8, MIC_ALL_CORNERS);
            draw_icon({ int16_t(tile_rect.Left() + (tile_size - icon->w) / 2), int16_t(tile_rect.Top() + (tile_size - icon->h) / 2) }, *icon, tile,
                enabled ? ropfn {} : ropfn { .shadow = is_shadowed::yes });

            const int16_t label_end = has_extension ? extension.Left() - 10 : content.Left() + content.Width();
            draw_label(Rect16(content.Left(), content.Top(), std::max<int>(label_end - content.Left(), 0), content.Height()), item.GetLabel(), enabled ? style::text : style::text_faint, bg);
        }

        if (has_extension) {
            if (extension_only) {
                display::fill_rect(extension, bg); // une valeur plus courte ne laisse rien (certaines valeurs Prusa ne peignent que leurs glyphes)
            }
            // Les valeurs Prusa prennent leur couleur « ciblée » (gris foncé) pour un fond blanc : sur le fond G sombre,
            // on les dessine comme non ciblées (la valeur en cours d'édition garde sa couleur)
            AutoRestore unfocus(window_menu_item_private::focused_menu_item, static_cast<IWindowMenuItem *>(nullptr), !item.is_edited());
            ItemAccess::print_extension(item, extension, enabled ? style::text_muted : style::text_faint, bg);
        }
    }

    void draw_scrollbar(const IWindowMenu &menu, int16_t pitch) {
        const int shown = menu.max_items_on_screen_count();
        const int count = menu.item_count();
        const Rect16 track(menu.Left() + row_w + 3, menu.Top(), 3, shown * pitch);
        display::fill_rect(track, style::screen);
        if (count <= shown) {
            return;
        }
        const int16_t h = std::max(track.Height() * shown / count, 6);
        const int16_t y = (track.Height() - h) * menu.scroll_offset() / (count - shown);
        display::draw_rounded_rect(Rect16(track.Left(), track.Top() + y, track.Width(), h), style::screen, scrollbar, 1, MIC_ALL_CORNERS);
    }

} // namespace

// ---- En-tête ----

MenuHeader::MenuHeader(window_t *parent, const string_view_utf8 &title, bool has_back)
    : header(parent, string_view_utf8::MakeRAM(title_text.data()), has_back) {
    SetText(title);
}

void MenuHeader::SetText(const string_view_utf8 &title) {
    title.copyToRAM(title_text.data(), title_text.size());
    text::title_case(title_text.data());
    text::ellipsize(title_text.data(), Header::title_chars());
    header.set_title(string_view_utf8::MakeRAM(title_text.data()));
}

// ---- Liste ----

MenuListBase::MenuListBase(window_t *parent, Rect16 rect)
    : window_frame_t(parent, rect) {
    SetBackColor(style::screen);
    Enable(); // la molette entre dans la liste depuis l'en-tête ou les onglets
}

void MenuListBase::attach(IWindowMenu &menu) {
    list = &menu;
    menu.hide_leading_return(); // listes virtuelles de Prusa : le « Retour » en double de l'en-tête
    menu.SetFocus();
}

void MenuListBase::draw() {
    if (!IsVisible() || !list) {
        return;
    }
    IWindowMenu &menu = *list;
    const int first = menu.scroll_offset();
    const int count = menu.current_items_on_screen_count();
    const int16_t pitch = menu.item_height() + GuiDefaults::MenuItemDelimeterHeight;
    const bool all = IsInvalid() || menu.IsInvalid() || drawn_rows != count;

    for (int slot = 0; slot < count; ++slot) {
        IWindowMenuItem *item = menu.item_at(first + slot);
        if (!item || (!all && !item->IsInvalid())) {
            continue;
        }
        const bool extension_only = !all && !item->IsIconInvalid() && !item->IsLabelInvalid();
        draw_row(*item, Rect16(menu.Left(), menu.slot_rect(slot).Top(), row_w, pitch), slot == 0, slot == count - 1, extension_only);
        item->Validate();
    }

    if (all) {
        const int16_t bottom = menu.Top() + count * pitch;
        display::fill_rect(Rect16(menu.Left(), bottom, row_w, menu.Top() + menu.Height() - bottom), style::screen);
        display::fill_rect(Rect16(menu.Left() + row_w, menu.Top(), 3, menu.Height()), style::screen);
        draw_scrollbar(menu, pitch);
    }

    drawn_rows = count;
    menu.Validate();
}

void MenuListBase::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    // Comme WindowExtendedMenu : le cadre a le focus, le menu reçoit la molette et le clic ; au bout de la liste,
    // le pas refusé fait passer le focus à l'élément suivant de l'écran (en-tête, onglets)
    if ((event == GUI_event_t::KNOB || event == GUI_event_t::CLICK) && list) {
        list->WindowEvent(sender, event, param);
        return;
    }
    window_frame_t::windowEvent(sender, event, param);
}

void hide_return_items(IWinMenuContainer &container) {
    // Seuls les « Retour » simples : un retour qui fait autre chose (« Enregistrer et revenir ») reste
    const string_view_utf8 back = _(MI_RETURN::label);
    for (int i = 0; i < container.GetRawCount(); ++i) {
        IWindowMenuItem *item = container.GetItemByRawIndex(i);
        if (item->has_return_behavior() && item->GetLabel().is_same_ref(back)) {
            item->hide();
        }
    }
}

TabBar::Tab section_tab() {
    const auto on_stack = [](bool (*is)(const screen_node &)) {
        return Screens::Access()->any_screen_on_stack(is);
    };
    if (on_stack([](const screen_node &n) { return n.creator.is_screen<ScreenMenuSettings>(); })) {
        return TabBar::Tab::settings;
    }
    if (on_stack([](const screen_node &n) { return n.creator.is_screen<ScreenMMU>() || n.creator.is_screen<ScreenMenuFilamentMMU>() || n.creator.is_screen<ScreenMenuFilament>(); })) {
        return TabBar::Tab::workshop;
    }
    if (on_stack([](const screen_node &n) { return n.creator.is_screen<ScreenFiles>() || n.creator.is_screen<ScreenPrinting>(); })) {
        return TabBar::Tab::print; // par exemple « Ajuster » (menu Prusa) ouvert depuis l'écran Impression
    }
    return TabBar::Tab::home;
}

} // namespace od3d
