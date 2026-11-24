/// @file
#include "menu_od3d.hpp"

#include "screen_files_od3d.hpp"
#include "screen_mmu_od3d.hpp"
#include "screen_printing_od3d.hpp"
#include "text_od3d.hpp"

#include <algorithm>
#include <cstring>
#include <display.hpp>
#include <fonts.hpp>
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

    // Indicateur de défilement (retour du 22/11 : « ce qui dit s'il y a du texte en dessous est trop petit ») : piste
    // visible de 4 px, à 2 px des lignes, et curseur clair d'au moins 24 px (3 px gris foncé avant)
    constexpr Color scroll_track = style::line;
    constexpr Color scroll_thumb = style::text_muted;
    constexpr int16_t scroll_gap = 2, scroll_w = 4, scroll_min = 24;
    constexpr uint8_t scroll_radius = 2;
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
            : in(network)                                ? style::tile_network
            : in(motion)                                 ? style::tile_motion
                                                         : style::tile;
    }

    /// Libellé sur une ligne, ou deux si le texte en a deux (nom, puis explication en gris) ; hint remplace l'explication,
    /// en orange ; tout rect est repeint, d'une passe
    void draw_label(Rect16 rect, const string_view_utf8 &label, Color fg, Color bg, const string_view_utf8 &hint = string_view_utf8::MakeNULLSTR()) {
        std::array<char, 160> buf;
        label.copyToRAM(buf.data(), buf.size());
        char *second = strchr(buf.data(), '\n');
        if (second) {
            *second++ = '\0';
        }
        const size_t chars = rect.Width() / width(Font::small);
        text::ellipsize(buf.data(), chars);
        Color second_fg = style::text_muted;
        if (!hint.isNULLSTR()) {
            second = buf.data() + strlen(buf.data()) + 1; // après la 1re ligne coupée (≤ 52 caractères) : la place reste
            hint.copyToRAM(second, buf.data() + buf.size() - second);
            second_fg = style::accent;
        }
        const uint16_t h = height(Font::small);
        if (!second) {
            draw_text(rect, string_view_utf8::MakeRAM(buf.data()), Font::small, fg, bg);
            return;
        }
        text::ellipsize(second, chars);
        draw_text(Rect16(rect.Left(), rect.Top(), rect.Width(), h + 2), string_view_utf8::MakeRAM(buf.data()), Font::small, fg, bg);
        draw_text(Rect16(rect.Left(), rect.Top() + h + 2, rect.Width(), rect.Height() - h - 2), string_view_utf8::MakeRAM(second), Font::small, second_fg, bg, Align_t::LeftTop());
    }

    /// Libellé de la demi-ligne : la 1re ligne à la place qu'elle a dans draw_label, ses glyphes coupés à bottom (exclu) ;
    /// rect ∩ [.., bottom) est repeint, d'une passe. Les glyphes passent par le tampon de l'écran (comme render_line) :
    /// seules ses premières rangées sont envoyées.
    void draw_label_cut(Rect16 rect, const string_view_utf8 &label, Color fg, Color bg, int16_t bottom) {
        const font_t &font = *resource_font(Font::small);
        std::array<char, 160> buf;
        label.copyToRAM(buf.data(), buf.size());
        char *second = strchr(buf.data(), '\n');
        if (second) {
            *second = '\0';
        }
        text::ellipsize(buf.data(), rect.Width() / font.w);
        size_t chars = 0;
        for (const char *p = buf.data(); *p; ++p) {
            chars += (*p & 0xC0) != 0x80;
        }
        const int16_t y = second ? rect.Top() + 1 : rect.Top() + (rect.Height() - font.h) / 2; // draw_label : LeftCenter sur h + 2 ou sur rect
        const Rect16 shown(rect.Left(), rect.Top(), rect.Width(), uint16_t(std::clamp<int>(bottom - rect.Top(), 0, rect.Height())));
        const Rect16 glyphs = Rect16(rect.Left(), y, uint16_t(chars * font.w), font.h).Intersection(shown);
        style::for_each_gap(shown, std::span<const Rect16>(&glyphs, 1), [bg](Rect16 r) {
            display::fill_rect(r, bg);
        });
        if (glyphs.IsEmpty()) {
            return;
        }
        StringReaderUtf8 reader(string_view_utf8::MakeRAM(buf.data()));
        const size_t capacity = display::buffer_pixel_size() / (font.w * font.h);
        int16_t x = rect.Left();
        for (size_t left = chars; left > 0;) {
            const size_t n = std::min(left, capacity);
            for (size_t i = 0; i < n; ++i) {
                display::store_char_in_buffer(uint16_t(n), uint16_t(i), reader.getUtf8Char(), &font, bg, fg);
            }
            display::draw_from_buffer(point_ui16(x, y), uint16_t(n * font.w), glyphs.Height());
            x += n * font.w;
            left -= n;
        }
    }

    /// Une ligne de la liste (SPEC_G.md §1.7) ; extension_only : seule la valeur a changé. visible < row.Height() :
    /// demi-ligne, seul son haut est peint (fond, tuile coupée et son icône, haut du libellé, pas de valeur)
    void draw_row(IWindowMenuItem &item, Rect16 row, bool first, bool last, bool extension_only, uint16_t visible) {
        const bool focused = item.is_focused();
        const bool enabled = item.IsEnabled();
        // Réglage à la molette (WiSpin) : la molette change la valeur au lieu de passer à la ligne suivante, et la ligne
        // le dit (avant, seule la valeur passait à l'orange : on croyait l'écran figé, BUG-6)
        const bool edited = item.is_edited();
        const Color bg = edited ? style::choice : focused ? style::focused(style::surface) : style::surface; // réglée : le fond du choix sélectionné

        // Contenu entre l'anneau (2 px en haut et en bas) et les marges de 12 px
        const Rect16 content(row.Left() + label_x, row.Top() + 2, row.Width() - label_x - value_right, row.Height() - 4);
        Rect16 extension = ItemAccess::extension_rect(item, content);
        if (extension.Left() < content.Left() + 8 * width(Font::small)) {
            extension = Rect16::fromLTRB(content.Left() + 8 * width(Font::small), extension.Top(), content.Left() + content.Width(), extension.Top() + extension.Height());
        }
        const bool cut = visible < row.Height();
        const Rect16 shown(row.Left(), row.Top(), row.Width(), visible);
        const bool has_extension = !cut && extension.Width() > 0 && (enabled || item.DoesShowDisabledExtension());

        if (!extension_only) {
            // Ligne ciblée : fond #34373C (SPEC_G.md §1.6), sans anneau. Une passe : le fond seulement autour de la tuile, du
            // libellé et de la valeur, qui repeignent leur rect (le fond puis le texte : chaque pas de la molette faisait
            // clignoter deux lignes) ; les bouts arrondis sont deux bandes de 12 px, sans contenu
            const uint8_t corners = (first ? MIC_TOP_LEFT | MIC_TOP_RIGHT : 0) | (last ? MIC_BOT_LEFT | MIC_BOT_RIGHT : 0);
            const Rect16 body = (first ? row : Rect16(row.Left(), row.Top() + 1, row.Width(), row.Height() - 1)).Intersection(shown);
            if (!first) {
                display::fill_rect(Rect16(row.Left(), row.Top(), row.Width(), 1), style::line);
            }
            const auto end_strip = [&](int16_t x, uint8_t strip_corners) {
                const Rect16 strip(x, body.Top(), tile_x, body.Height());
                if (strip_corners) {
                    display::draw_rounded_rect(strip, style::screen, bg, style::radius_big, strip_corners);
                } else {
                    display::fill_rect(strip, bg);
                }
            };
            static_assert(tile_x == value_right && tile_x >= style::radius_big, "les bouts arrondis tiennent dans les marges de la ligne");
            end_strip(body.Left(), corners & (MIC_TOP_LEFT | MIC_BOT_LEFT));
            end_strip(body.Left() + body.Width() - value_right, corners & (MIC_TOP_RIGHT | MIC_BOT_RIGHT));

            const img::Resource *icon = ItemAccess::icon(item);
            if (!icon) {
                icon = &img::settings_16x16;
            } else if (icon == &img::folder_up_16x16) {
                icon = &img::arrow_left_16x16; // « ‹ Retour » : la flèche de l'ancien retour de l'en-tête
            }
            const Color tile = tile_color(icon);
            const Rect16 tile_rect(row.Left() + tile_x, row.Top() + (row.Height() - tile_size) / 2, tile_size, tile_size);
            const Rect16 tile_shown = tile_rect.Intersection(shown);
            const int16_t label_end = has_extension ? extension.Left() - 10 : content.Left() + content.Width();
            const Rect16 label_rect(content.Left(), content.Top(), std::max<int>(label_end - content.Left(), 0), content.Height());
            const std::array<Rect16, 3> parts { tile_shown, label_rect.Intersection(shown), has_extension ? extension : Rect16() };
            style::for_each_gap(Rect16(body.Left() + tile_x, body.Top(), body.Width() - tile_x - value_right, body.Height()), std::span<const Rect16>(parts.data(), has_extension ? 3 : 2), [&](Rect16 r) {
                display::fill_rect(r, bg);
            });

            const Color fg = enabled ? style::text : style::text_faint;
            const Rect16 glyph(tile_rect.Left() + (tile_size - icon->w) / 2, tile_rect.Top() + (tile_size - icon->h) / 2, icon->w, icon->h);
            if (cut) {
                // L'icône (lue d'un bloc) quand elle tient entière (listes G : 29 px visibles, icône de 12 à 27), dans sa
                // tuile coupée net en bas comme le libellé (retour du 24/11 : « dnas le demi menu qui est affiché en bas
                // l'icone n'est pas afficher ») ; sinon pas de tuile : son haut seul se voyait comme une pastille vide
                const int16_t shown_end = shown.Top() + shown.Height();
                if (glyph.Bottom() < shown_end) {
                    // Coins du bas descendus sous la coupe (cachés) : côtés droits jusqu'à elle
                    const int16_t seen = shown_end - tile_rect.Top();
                    const Rect16 frame(tile_rect.Left(), tile_rect.Top(), tile_size, seen >= tile_size ? tile_size : seen + style::radius_small);
                    const std::array<Rect16, 2> painted_apart { glyph, Rect16(frame.Left(), shown_end, tile_size, std::max(frame.Top() + frame.Height() - shown_end, 0)) };
                    draw_rounded_frame(frame, bg, tile, tile, style::radius_small, 0, painted_apart);
                    draw_icon({ glyph.Left(), glyph.Top() }, *icon, tile, enabled ? ropfn {} : ropfn { .shadow = is_shadowed::yes });
                } else if (!tile_shown.IsEmpty()) {
                    display::fill_rect(tile_shown, bg);
                }
                draw_label_cut(label_rect, item.GetLabel(), fg, bg, shown_end);
                return;
            }
            // La tuile autour de l'icône, puis l'icône : chaque pixel une fois
            draw_rounded_frame(tile_rect, bg, tile, tile, style::radius_small, 0, std::span<const Rect16>(&glyph, 1));
            draw_icon({ glyph.Left(), glyph.Top() }, *icon, tile, enabled ? ropfn {} : ropfn { .shadow = is_shadowed::yes });
            draw_label(label_rect, item.GetLabel(), fg, bg,
                edited ? _("Knob: adjust, click: OK") : string_view_utf8::MakeNULLSTR());
        }

        if (has_extension) {
            display::fill_rect(extension, bg); // une valeur plus courte ne laisse rien (certaines valeurs Prusa ne peignent que leurs glyphes)
            // Les valeurs Prusa prennent leur couleur « ciblée » (gris foncé) pour un fond blanc : sur le fond G sombre,
            // on les dessine comme non ciblées (la valeur en cours d'édition garde sa couleur)
            AutoRestore unfocus(window_menu_item_private::focused_menu_item, static_cast<IWindowMenuItem *>(nullptr), !item.is_edited());
            ItemAccess::print_extension(item, extension, enabled ? style::text_muted : style::text_faint, bg);
        }
    }

    void draw_scrollbar(const IWindowMenu &menu) {
        const int shown = menu.max_items_on_screen_count();
        const int count = menu.item_count();
        const Rect16 track(menu.Left() + row_w + scroll_gap, menu.Top(), scroll_w, menu.Height());
        if (count <= shown) {
            display::fill_rect(track, style::screen);
            return;
        }
        // Piste au-dessus, curseur, piste en dessous : chaque pixel peint une fois
        const int16_t h = std::max<int16_t>(track.Height() * shown / count, scroll_min);
        const int16_t y = (track.Height() - h) * menu.scroll_offset() / (count - shown);
        const int16_t below = track.Height() - y - h;
        if (y > 0) {
            display::draw_rounded_rect(Rect16(track.Left(), track.Top(), scroll_w, y), style::screen, scroll_track, scroll_radius, MIC_TOP_LEFT | MIC_TOP_RIGHT);
        }
        // Le curseur se fond dans la piste, sauf à ses bouts où il touche le fond de l'écran
        const uint8_t ends = (y == 0 ? MIC_ALT_CL_TOP_LEFT | MIC_ALT_CL_TOP_RIGHT : 0) | (below == 0 ? MIC_ALT_CL_BOT_LEFT | MIC_ALT_CL_BOT_RIGHT : 0);
        display::draw_rounded_rect(Rect16(track.Left(), track.Top() + y, scroll_w, h), scroll_track, scroll_thumb, scroll_radius, uint8_t(MIC_ALL_CORNERS | ends), style::screen);
        if (below > 0) {
            display::draw_rounded_rect(Rect16(track.Left(), track.Top() + y + h, scroll_w, below), style::screen, scroll_track, scroll_radius, MIC_BOT_LEFT | MIC_BOT_RIGHT);
        }
    }

} // namespace

static_assert(row_w + scroll_gap + scroll_w <= list_rect.Width(), "l'indicateur de défilement tient dans la liste");

// ---- En-tête ----

MenuHeader::MenuHeader(window_t *parent, const string_view_utf8 &title, bool has_back, bool compact)
    : header(parent, string_view_utf8::MakeRAM(title_text.data()), has_back, menu_back, compact)
    , back(has_back) {
    SetText(title);
}

void MenuHeader::show_back(bool show) {
    back = show;
    header.show_back(show);
}

void MenuHeader::SetText(const string_view_utf8 &title) {
    title.copyToRAM(title_text.data(), title_text.size());
    text::title_case(title_text.data());
    // Coupé à sa place par le titre lui-même (« ... ») : 16 caractères en police normale à côté du retour, 19 sans
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
    const int16_t bottom = menu.Top() + count * pitch;
    // Demi-ligne : la ligne qui suit les lignes entières, s'il y en a une (une liste virtuelle la prépare aussi,
    // window_menu_virtual.cpp) ; sa valeur ne compte pas, elle n'est pas peinte
    const int16_t peek_h = std::min<int>(menu.Top() + menu.Height() - bottom, pitch - 1);
    IWindowMenuItem *next = (count == menu.max_items_on_screen_count() && peek_h > 0 && first + count < menu.item_count()) ? menu.item_at(first + count) : nullptr;
    const uint8_t rows = count + (next ? 1 : 0);
    const bool all = IsInvalid() || menu.IsInvalid() || drawn_rows != rows;

    for (int slot = 0; slot < rows; ++slot) {
        IWindowMenuItem *item = slot < count ? menu.item_at(first + slot) : next;
        if (!item || (!all && !item->IsInvalid())) {
            continue;
        }
        const bool extension_only = !all && !item->IsIconInvalid() && !item->IsLabelInvalid();
        if (slot < count || !extension_only) {
            // Coins du bas sur la dernière ligne entière, sauf si la demi-ligne la prolonge ; ordre de haut en bas
            draw_row(*item, Rect16(menu.Left(), menu.Top() + slot * pitch, row_w, pitch), slot == 0, slot == count - 1 && !next, extension_only, slot < count ? pitch : peek_h);
        }
        item->Validate();
    }

    if (all) {
        const int16_t end = next ? bottom + peek_h : bottom;
        display::fill_rect(Rect16(menu.Left(), end, row_w, menu.Top() + menu.Height() - end), style::screen);
        display::fill_rect(Rect16(menu.Left() + row_w, menu.Top(), scroll_gap, menu.Height()), style::screen);
        draw_scrollbar(menu);
    }

    drawn_rows = rows;
    menu.Validate();
}

void MenuListBase::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    // Comme WindowExtendedMenu : le cadre a le focus, le menu reçoit la molette et le clic ; au bout de la liste,
    // le pas refusé fait passer le focus à l'élément suivant de l'écran (en-tête, onglets)
    if ((event == GUI_event_t::KNOB || event == GUI_event_t::CLICK) && list) {
        list->WindowEvent(sender, event, param);
        return;
    }
    // Demi-ligne touchée : la liste défile d'une ligne et la cible (le menu Prusa n'a pas de ligne à cet endroit)
    if (event == GUI_event_t::TOUCH_CLICK && list) {
        const int shown = list->max_items_on_screen_count();
        if (event_conversion_union { .pvoid = param }.point.y >= list->Top() + shown * (list->item_height() + GuiDefaults::MenuItemDelimeterHeight)) {
            list->move_focus_to_index(list->scroll_offset() + shown);
            return;
        }
    }
    window_frame_t::windowEvent(sender, event, param);
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
