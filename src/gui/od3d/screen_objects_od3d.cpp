/// @file
#include "screen_objects_od3d.hpp"

#include "spool_draw.hpp"
#include "thumbnail_od3d.hpp" // draw_pixels

#include <algorithm>
#include <common/marlin_vars.hpp>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <feature/cancel_object/cancel_object.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <timing.h>
#include <window_msgbox.hpp>

// Marlin en dernier : ses macros ne doivent pas toucher les en-têtes de l'interface
#include <inc/MarlinConfig.h>

namespace od3d {

namespace {

    namespace om = object_map;
    static_assert(om::max_objects == marlin_vars_t::CANCEL_OBJECTS_NAME_COUNT, "places et noms : les mêmes objets");

    // ---- Géométrie ----
    // Le plan à gauche, à l'échelle du plateau et le plus grand possible ; une colonne d'au moins 180 px à droite
    constexpr float bed_x = X_BED_SIZE;
    constexpr float bed_y = Y_BED_SIZE;
    constexpr float scale = std::min(272 / bed_x, 262 / bed_y);
    constexpr Rect16 map_rect = { 8, 48, uint16_t(bed_x * scale + 0.5f), uint16_t(bed_y * scale + 0.5f) };
    constexpr int16_t pad = 6; ///< le plateau dans le plan, loin des coins arrondis
    constexpr int plan_w = map_rect.Width() - 2 * pad;
    constexpr int plan_h = map_rect.Height() - 2 * pad;
    constexpr int min_side = 10; ///< un petit objet se voit et se touche
    constexpr int reach = 12; ///< un toucher à côté d'un objet le choisit encore

    constexpr int16_t col_x = map_rect.Left() + map_rect.Width() + 12;
    constexpr uint16_t col_w = style::screen_w - style::margin - col_x;
    constexpr Rect16 info_rect = { col_x, 48, col_w, 160 };
    constexpr uint16_t half_w = (col_w - 4 - 8) / 2;
    constexpr Rect16 previous_body = { int16_t(col_x + 2), 216, half_w, 40 };
    constexpr Rect16 next_body = { int16_t(col_x + 2 + half_w + 8), 216, half_w, 40 };
    constexpr Rect16 cancel_body = { int16_t(col_x + 2), 266, uint16_t(col_w - 4), 40 };

    // CORE One 250 × 220, INDX 250 × 205,5, CORE One L 301 × 275 : plan de 272 px de large, colonne de 180 px
    static_assert(col_w >= 180 && map_rect.Top() + map_rect.Height() <= style::screen_h - 4);
    static_assert(info_rect.Top() + info_rect.Height() < style::with_ring(previous_body).Top());
    static_assert(style::with_ring(previous_body).Bottom() < style::with_ring(cancel_body).Top() && style::with_ring(cancel_body).Bottom() < style::screen_h);

    // ---- Couleurs ----
    constexpr Color map_back = style::thumb_back; ///< comme la carte de Déplacer les axes
    constexpr Color map_grid = style::grid;
    constexpr Color canceled_fill = Color::from_raw(0x6E2A22); ///< rouge sombre
    constexpr Color selected_ring = style::text_strong;

    constexpr Color fill_of(ObjectState s) {
        switch (s) {
        case ObjectState::printing:
            return style::accent;
        case ObjectState::canceled:
            return canceled_fill;
        default:
            return style::target;
        }
    }

    /// Grille tous les 50 mm en X et 55 mm en Y, comme la carte de Déplacer les axes
    constexpr bool grid_x(int px) {
        for (int mm = 50; mm < bed_x; mm += 50) {
            if (px == int(mm * plan_w / bed_x + 0.5f)) {
                return true;
            }
        }
        return false;
    }
    constexpr bool grid_y(int py) {
        for (int mm = 55; mm < bed_y; mm += 55) {
            if (py == plan_h - int(mm * plan_h / bed_y + 0.5f)) {
                return true;
            }
        }
        return false;
    }

    // ---- Choix ----

    int16_t selected = -1; ///< objet choisi ; un seul écran à la fois
    bool refresh = false; ///< tout de suite, sans attendre le prochain relevé

    ObjectState state_of(int16_t object, int16_t current) {
        if (buddy::cancel_object().is_object_cancelled(object)) {
            return ObjectState::canceled;
        }
        return object == current ? ObjectState::printing : ObjectState::waiting;
    }

    /// « 3 · Nom » (numéro de Prusa : à partir de 1), « 3 · Objet » sans nom
    void write_label(int16_t object, std::span<char> out) {
        char name[marlin_vars_t::CANCEL_OBJECT_NAME_LEN] = "";
        if (object < int16_t(marlin_vars_t::CANCEL_OBJECTS_NAME_COUNT)) {
            marlin_vars().cancel_object_names[object].copy_to(name, sizeof(name));
        }
        if (!name[0]) {
            _("Object").copyToRAM(name, sizeof(name));
        }
        snprintf(out.data(), out.size(), "%d · %s", object + 1, name);
    }

    void step(int d) {
        if (const int count = buddy::cancel_object().object_count(); count > 0) {
            selected = int16_t(((std::max<int>(selected, 0) + d) % count + count) % count);
            refresh = true;
        }
    }
    void select_previous() {
        step(-1);
    }
    void select_next() {
        step(1);
    }

    void cancel_selected() {
        if (selected < 0 || buddy::cancel_object().is_object_cancelled(selected)) {
            return;
        }
        std::array<char, 40> label;
        write_label(selected, label);
        char fmt[128];
        _("Cancel object %s? It will not be printed any more. This cannot be undone.").copyToRAM(fmt, sizeof(fmt));
        const std::span<char> text = message_text(); // la boîte le lit tant qu'elle est ouverte
        snprintf(text.data(), text.size(), fmt, label.data());
        if (MsgBoxQuestion(string_view_utf8::MakeRAM(text.data()), Responses_YesNo) == Response::Yes) {
            marlin_client::set_object_cancelled(selected, true);
        }
        refresh = true;
    }

} // namespace

// ---- Plan ----

ObjectMap::ObjectMap(window_t *parent)
    : window_t(parent, map_rect) {}

void ObjectMap::set(const View &v) {
    if (v == view) {
        return;
    }
    view = v;
    Invalidate();
}

int ObjectMap::hit(point_ui16_t p) const {
    std::array<om::Px, om::max_objects> rects;
    for (size_t i = 0; i < view.count; ++i) {
        rects[i] = view.items[i].rect;
    }
    return om::hit(std::span<const om::Px>(rects.data(), view.count), p.x - map_rect.Left() - pad, p.y - map_rect.Top() - pad, reach);
}

void ObjectMap::unconditionalDraw() {
    const int w = map_rect.Width(), h = map_rect.Height();
    constexpr int r = style::radius;
    int row = -1;
    uint32_t row_objects = 0; ///< objets coupés par la ligne en cours : seuls ceux-là sont regardés à chaque pixel
    draw_pixels(map_rect, [&](int x, int y) {
        const int px = x - pad, py = y - pad;
        if (y != row) {
            row = y;
            row_objects = 0;
            for (size_t i = 0; i < view.count; ++i) {
                const om::Px &o = view.items[i].rect;
                if (o.w > 0 && py >= o.y && py < o.y + o.h) {
                    row_objects |= 1u << i;
                }
            }
        }
        Color c = map_back;
        if (px >= 0 && px < plan_w && py >= 0 && py < plan_h && (grid_x(px) || grid_y(py))) {
            c = map_grid;
        }
        // Les objets qui se recouvrent : l'annulé sur celui à imprimer, celui en cours dessus, le choisi tout dessus
        int top = -1;
        for (uint32_t m = row_objects; m; m &= m - 1) {
            const int i = __builtin_ctz(m);
            const Item &it = view.items[i];
            if (px < it.rect.x || px >= it.rect.x + it.rect.w) {
                continue;
            }
            const bool chosen = i == view.selected;
            if (const int layer = int(it.state) + (chosen ? 4 : 0); layer > top) {
                top = layer;
                const bool ring = chosen && std::min({ px - it.rect.x, it.rect.x + it.rect.w - 1 - px, py - it.rect.y, it.rect.y + it.rect.h - 1 - py }) < 2;
                c = ring ? selected_ring : fill_of(it.state);
            }
        }
        // Coins arrondis, lissés sur le fond de l'écran
        const int cx = x < r ? x : (x >= w - r ? w - 1 - x : -1);
        const int cy = y < r ? y : (y >= h - r ? h - 1 - y : -1);
        if (cx >= 0 && cy >= 0) {
            c = spool::mix_quarters(c, style::screen, spool::corner_beyond_quarters(cx, cy, r));
        }
        return c;
    });

    // Numéros : seulement s'ils tiennent dans l'objet sans toucher un autre objet
    for (size_t i = 0; i < view.count; ++i) {
        const Item &it = view.items[i];
        char number[12];
        snprintf(number, sizeof(number), "%d", int(i + 1));
        const int16_t tw = int16_t(strlen(number) * width(Font::small)), th = int16_t(height(Font::small));
        if (it.rect.w < tw + 6 || it.rect.h < th + 4) {
            continue;
        }
        const om::Px label { int16_t(it.rect.x + (it.rect.w - tw) / 2), int16_t(it.rect.y + (it.rect.h - th) / 2), tw, th };
        const bool alone = std::none_of(view.items.begin(), view.items.begin() + view.count, [&](const Item &o) {
            return &o != &it && o.rect.w > 0 && o.rect.x < label.x + label.w && label.x < o.rect.x + o.rect.w && o.rect.y < label.y + label.h && label.y < o.rect.y + o.rect.h;
        });
        if (alone) {
            draw_text(Rect16(map_rect.Left() + pad + label.x, map_rect.Top() + pad + label.y, label.w, label.h), string_view_utf8::MakeRAM(number), Font::small,
                it.state == ObjectState::printing ? style::on_accent : style::text_strong, fill_of(it.state), Align_t::Center());
        }
    }
}

// ---- Objet choisi ----

ObjectInfo::ObjectInfo(window_t *parent)
    : window_t(parent, info_rect) {}

void ObjectInfo::set(const View &v) {
    if (v == view) {
        return;
    }
    view = v;
    Invalidate();
}

void ObjectInfo::unconditionalDraw() {
    display::fill_rect(info_rect, style::screen);
    const int16_t x = info_rect.Left();
    const uint16_t w = info_rect.Width();
    constexpr int16_t line = height(Font::small) + 4;
    int16_t y = info_rect.Top();
    if (view.object >= 0) {
        // Gras s'il tient (16 caractères), sinon en petit (20) : « 3 · Shape-Box id:2 » se lit encore en entier
        const Font font = string_view_utf8::MakeRAM(view.label.data()).computeNumUtf8Chars() * width(Font::normal) <= w ? Font::normal : Font::small;
        draw_text(Rect16(x, y, w, height(Font::normal)), string_view_utf8::MakeRAM(view.label.data()), font, style::text_strong, style::screen);
        y += height(Font::normal) + 6;
        switch (view.state) {
        case ObjectState::printing:
            draw_text(Rect16(x, y, w, line), _("Printing now"), Font::small, style::accent, style::screen);
            break;
        case ObjectState::canceled:
            draw_text(Rect16(x, y, w, line), _("Canceled"), Font::small, style::text_danger, style::screen);
            break;
        case ObjectState::waiting:
            draw_text(Rect16(x, y, w, line), _("Still to print"), Font::small, style::text_soft, style::screen);
            break;
        }
        y += line;
        if (!view.placed && view.state != ObjectState::canceled) {
            draw_text(Rect16(x, y, w, line), _("Place not known yet"), Font::small, style::text_tertiary, style::screen);
        }
        y += line;
    }
    draw_text(Rect16(x, y, w, line), _("Tap an object"), Font::small, style::text_tertiary, style::screen);

    // Légende, en bas du cadre
    struct Key {
        ObjectState state;
        const char *text;
    };
    static constexpr Key keys[] = {
        { ObjectState::printing, N_("Printing now") },
        { ObjectState::canceled, N_("Canceled") },
        { ObjectState::waiting, N_("Still to print") },
    };
    y = info_rect.Top() + info_rect.Height() - int16_t(std::size(keys)) * line;
    for (const Key &k : keys) {
        display::fill_rect(Rect16(x, y + (line - 10) / 2, 10, 10), fill_of(k.state));
        draw_text(Rect16(x + 16, y, w - 16, line), _(k.text), Font::small, style::text_muted, style::screen);
        y += line;
    }
}

// ---- Écran ----

ScreenObjects::ScreenObjects()
    : Screen(true)
    , header(this, _("Cancel objects"), true)
    , map(this)
    , info(this)
    , previous(this, previous_body, select_previous, string_view_utf8::MakeNULLSTR(), &img::arrow_left_10x16, style::surface, style::text_strong)
    , next(this, next_body, select_next, string_view_utf8::MakeNULLSTR(), &img::arrow_right_10x16, style::surface, style::text_strong)
    , cancel(this, cancel_body, cancel_selected, _("Cancel this object"), nullptr, style::secondary, style::text_danger) {
    ClrMenuTimeoutClose(); // comme la liste de Prusa : l'écran reste ouvert pendant l'impression
    selected = -1; // l'objet en cours, au premier relevé
    update();
    next.SetFocus(); // pas sur « Annuler » : un clic de molette par mégarde n'annule rien
}

void ScreenObjects::update() {
    updated_ms = ticks_ms();
    refresh = false;
    const auto &objects = buddy::cancel_object();
    const int16_t count = objects.object_count();
    const int16_t current = objects.current_object();
    if (selected >= count) {
        selected = int16_t(count - 1);
    }
    if (selected < 0 && count > 0) {
        selected = current >= 0 && current < count ? current : 0;
    }

    ObjectMap::View plan;
    plan.count = uint8_t(std::min<int>(count, om::max_objects));
    plan.selected = selected;
    for (int16_t i = 0; i < plan.count; ++i) {
        ObjectMap::Item &item = plan.items[i];
        item.state = state_of(i, current);
        if (const uint32_t b = om::box(i); b != om::unknown) {
            item.rect = om::to_screen(om::unpack(b), plan_w, plan_h, bed_x, bed_y, min_side);
        }
    }
    map.set(plan);

    ObjectInfo::View chosen;
    if (selected >= 0) {
        chosen.object = selected;
        chosen.state = state_of(selected, current);
        chosen.placed = om::box(selected) != om::unknown;
        write_label(selected, chosen.label);
    }
    info.set(chosen);

    previous.enable(count > 1);
    next.enable(count > 1);
    cancel.enable(selected >= 0 && chosen.state != ObjectState::canceled);
}

void ScreenObjects::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::TOUCH_CLICK:
        // Le plan n'est pas actif (hors de la molette) : son toucher est lu ici
        if (const point_ui16_t p = event_conversion_union { .pvoid = param }.point; map_rect.Contain(p)) {
            if (const int i = map.hit(p); i >= 0) {
                selected = int16_t(i);
                update();
            }
            return;
        }
        break;
    case GUI_event_t::LOOP:
        // Places et états relevés 4 fois par seconde ; tout de suite après un choix
        if (refresh || ticks_ms() - updated_ms >= 250) {
            update();
        }
        break;
    default:
        break;
    }
    Screen::windowEvent(sender, event, param);
}

} // namespace od3d
