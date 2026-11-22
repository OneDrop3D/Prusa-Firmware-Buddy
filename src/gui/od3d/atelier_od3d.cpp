/// @file
#include "atelier_od3d.hpp"

#include <algorithm>
#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <display_helper.h>
#include <feature/filament_sensor/filament_sensors_handler.hpp>
#include <feature/od3d/spool_usage_od3d.hpp>
#include <feature/prusa/MMU2/mmu2_state.h>
#include <i18n.h>
#include <marlin_client.hpp>
#include <MItem_tools.hpp>
#include <timing.h>
#include <utils/string_builder.hpp>

namespace od3d {

// ---- Bouton ----

Button::Button(window_t *parent, Rect16 body, uint8_t tag, Look look, const string_view_utf8 &label, const img::Resource *icon, Color back, uint8_t radius)
    : Card(parent, style::with_ring(body), nullptr, style::surface, back, radius)
    , tag(tag)
    , label(label)
    , icon(icon)
    , look(look) {}

void Button::set(Look set_look, const string_view_utf8 &set_label, const img::Resource *set_icon) {
    look = set_look;
    label = set_label;
    icon = set_icon;
    Invalidate();
}

void Button::set_look(Look set) {
    if (look != set) {
        look = set;
        Invalidate();
    }
}

void Button::set_font(Font set) {
    font = set;
    Invalidate();
}

void Button::enable(bool set) {
    if (set != IsEnabled()) {
        set_enabled(set);
        Invalidate();
    }
}

Button::Colors Button::colors() const {
    // Couleurs de la maquette (SPEC_G.md §1.5)
    static constexpr Color brown = Color::from_raw(0x3A2618);
    Colors c { back, back, style::text_soft, 0 };
    switch (look) {
    case Look::primary:
        c = { style::accent, style::accent, style::on_accent, 0 };
        break;
    case Look::secondary:
        c = { style::secondary, style::secondary, style::text, 0 };
        break;
    case Look::outline:
        c.border = style::line;
        c.border_w = 1;
        break;
    case Look::alert:
        c.border = c.fg = style::warning;
        c.border_w = 1;
        break;
    case Look::selected:
        c = { brown, style::accent, style::text_strong, 2 };
        break;
    case Look::card:
        c = { style::surface, style::surface, style::text, 0 };
        break;
    }
    if (!IsEnabled()) {
        c.fg = style::text_faint;
        if (look == Look::primary) {
            c.fill = c.border = style::secondary; // un bouton orange inactif tromperait
        }
    }
    if (IsFocused()) {
        c.fill = style::focused(c.fill); // focus : corps plus clair, sans anneau
    }
    return c;
}

Rect16 Button::draw_frame(Rect16 b, const Colors &c) {
    // Corps et contour d'une passe, marge comprise (le rect englobe la marge)
    od3d::draw_frame(style::with_ring(b), back, c.border, c.fill, radius, c.border_w);
    return Rect16(b.Left() + c.border_w, b.Top() + c.border_w, b.Width() - 2 * c.border_w, b.Height() - 2 * c.border_w);
}

void Button::unconditionalDraw() {
    const Colors c = colors();
    const Rect16 b = body();
    draw_frame(b, c);
    const bool enabled = IsEnabled();
    const ropfn rop = (look == Look::primary && enabled) ? ropfn { .swap_bw = has_swapped_bw::yes } : (enabled ? ropfn {} : ropfn { .shadow = is_shadowed::yes });
    draw_label(Rect16(b.Left() + 6, b.Top() + 2, b.Width() - 12, b.Height() - 4), label, icon, font, c.fg, c.fill, rop);
}

void Button::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::CLICK || event == GUI_event_t::TOUCH_CLICK) {
        if (IsEnabled() && GetParent()) {
            GetParent()->WindowEvent(this, GUI_event_t::CHILD_CLICK, reinterpret_cast<void *>(static_cast<uintptr_t>(tag)));
        }
        return; // toujours consommé, comme Card
    }
    Card::windowEvent(sender, event, param);
}

void draw_label(Rect16 area, const string_view_utf8 &label, const img::Resource *icon, Font font, Color fg, Color bg, ropfn rop) {
    constexpr int16_t gap = 6;
    const font_t &f = *resource_font(font); // width() et height() ne servent qu'à la compilation
    const uint16_t chars = label.isNULLSTR() ? 0 : label.computeNumUtf8Chars();
    const uint16_t text_w = chars * f.w;
    const uint16_t icon_w = icon ? icon->w + (chars ? gap : 0) : 0;
    if (icon_w + text_w <= area.Width()) {
        // Une ligne : icône et texte centrés ensemble
        int16_t x = area.Left() + (area.Width() - icon_w - text_w) / 2;
        if (icon) {
            draw_icon({ x, int16_t(area.Top() + (area.Height() - icon->h) / 2) }, *icon, bg, rop);
            x += icon_w;
        }
        if (chars) {
            draw_text(Rect16(x, area.Top() + (area.Height() - f.h) / 2, text_w, f.h), label, font, fg, bg);
        }
        return;
    }
    // Trop long : icône à gauche, texte sur deux lignes à côté
    int16_t x = area.Left();
    if (icon) {
        draw_icon({ x, int16_t(area.Top() + (area.Height() - icon->h) / 2) }, *icon, bg, rop);
        x += icon_w;
    }
    const uint16_t h = std::min<uint16_t>(area.Height(), 2 * f.h);
    render_text_align(Rect16(x, area.Top() + (area.Height() - h) / 2, area.Left() + area.Width() - x, h), label, font, bg, fg, { 0, 0, 0, 0 },
        text_flags(icon ? Align_t::LeftCenter() : Align_t::Center(), is_multiline::yes, check_overflow::no));
}

namespace atelier {

    // Teintes de la maquette (SPEC_G.md §1.1) ; le nom se place selon la langue
    const std::array<Paint, 8> paints { {
        { Color::from_raw(0x2A2A33), N_("black %s") },
        { Color::from_raw(0xEDEDED), N_("white %s") },
        { Color::from_raw(0x8A8F96), N_("grey %s") },
        { Color::from_raw(0xD7263D), N_("red %s") },
        { Color::from_raw(0xF8651B), N_("orange %s") },
        { Color::from_raw(0xF2C14E), N_("yellow %s") },
        { Color::from_raw(0x2E9E5B), N_("green %s") },
        { Color::from_raw(0x2F6FD6), N_("blue %s") },
    } };

    int paint_index(FilamentType type) {
        if (!type) {
            return -1;
        }
        const FilamentTypeParameters params = type.parameters();
        if (!params.color.has_value()) {
            return -1;
        }
        for (size_t i = 0; i < paints.size(); ++i) {
            if (paints[i].color == *params.color) {
                return int(i);
            }
        }
        return -1;
    }

    void spool_name(std::span<char> out, FilamentType type) {
        if (!type) {
            strlcpy(out.data(), "-", out.size()); // « — » n'est pas dans la police
            return;
        }
        const FilamentTypeParameters params = type.parameters();
        if (const int paint = paint_index(type); paint >= 0) {
            StringViewUtf8Parameters<filament_name_buffer_size + 2> args;
            _(paints[paint].name).formatted(args, params.name.data()).copyToRAM(out.data(), out.size());
        } else {
            strlcpy(out.data(), params.name.data(), out.size());
        }
    }

    void paint_name(std::span<char> out, int paint) {
        out[0] = '\0';
        if (paint < 0 || paint >= int(paints.size())) {
            return;
        }
        // Le format sans matière, puis sans les espaces qui l'entouraient (« blanc %s » / « %s blanc »)
        StringViewUtf8Parameters<2> args;
        std::array<char, 24> text;
        _(paints[paint].name).formatted(args, "").copyToRAM(text.data(), text.size());
        const char *begin = text.data();
        while (*begin == ' ') {
            ++begin;
        }
        strlcpy(out.data(), begin, out.size());
        for (size_t len = strlen(out.data()); len && out[len - 1] == ' '; --len) {
            out[len - 1] = '\0';
        }
    }

    bool mmu_active() {
        return MMU2::xState(marlin_vars().mmu2_state.get()) == MMU2::xState::Active;
    }

    bool mmu_enabled() {
        return config_store().mmu2_enabled.get();
    }

    uint8_t visible_tools() {
        return std::clamp<uint8_t>(VirtualToolIndex::enabled_range_size(), 1, VirtualToolIndex::count);
    }

    string_view_utf8 slot_state(const Slot &slot, uint8_t index, std::span<char> buf, Color &color) {
        color = style::text_muted;
        if (slot.loaded) {
            color = style::accent;
            return _("Loaded");
        }
        if (!slot.look.filament) {
            return _("Empty");
        }
        if (const auto usage = spool_usage::report(index)) {
            // Reste (F13 ou étiquette OpenPrintTag), en jaune sous le seuil d'alerte
            snprintf(buf.data(), buf.size(), "~%d g", int(usage->remaining_g));
            if (usage->low()) {
                color = style::warning;
            }
            return string_view_utf8::MakeRAM(buf.data());
        }
        paint_name(buf, paint_index(FilamentType::for_tool(VirtualToolIndex::from_raw(index))));
        return string_view_utf8::MakeRAM(buf.data());
    }

    bool filament_in_nozzle() {
        return FSensors_instance().WhereIsFilament() == MMU2::FilamentState::AT_FSENSOR;
    }

    bool can_run() {
        return !marlin_client::is_printing();
    }

    namespace {
        Request request {};
        bool request_valid = false;
        bool served = false;
        uint32_t valid_until_ms = 0;

        constexpr uint32_t first_wait_ms = 30'000; ///< le G-code peut attendre que la file se vide
        constexpr uint32_t open_ms = 3'600'000; ///< tant que l'écran de progression est ouvert (attente du filament…)
        constexpr uint32_t chain_ms = 3'000; ///< entre deux écrans d'une même opération

        bool send(const char *gcode, Op op, uint8_t slot) {
            if (!gui_try_gcode_with_msg(gcode)) {
                return false;
            }
            request = { op, slot };
            request_valid = true;
            served = false;
            valid_until_ms = ticks_ms() + first_wait_ms;
            return true;
        }
    } // namespace

    bool run(Op op, uint8_t slot) {
        ArrayStringBuilder<MAX_CMD_SIZE> gcode;
        switch (op) {
        case Op::load:
            // Comme MI_MMU_LOAD_TO_NOZZLE_I : la matière de l'emplacement donne la température. Sans matière (imprimante
            // sans MMU, buse vide), pas de S : Prusa demande la matière à charger.
            if (const FilamentType type = FilamentType::for_tool(VirtualToolIndex::from_raw(slot))) {
                gcode.append_string("M701 S\"");
                type.build_encoded_gcode_param(gcode);
                gcode.append_printf("\" W2 P%i", slot);
            } else {
                gcode.append_printf("M701 W2 P%i", slot);
            }
            break;
        case Op::unload:
            gcode.append_string("M702 W2");
            break;
        case Op::preload:
            gcode.append_printf("M704 P%i", slot);
            break;
        case Op::preload_all: {
            // Une commande par emplacement, comme MI_MMU_PRELOAD_ALL, sans changer leur matière
            bool any = false;
            for (const VirtualToolIndex tool : VirtualToolIndex::all()) {
                if (tool.is_enabled()) {
                    char line[16];
                    snprintf(line, sizeof(line), "M704 P%i", tool.to_raw());
                    if (!send(line, op, slot)) {
                        return any;
                    }
                    any = true;
                }
            }
            return any;
        }
        case Op::eject:
            gcode.append_printf("M705 P%i", slot);
            break;
        case Op::cut:
            gcode.append_printf("M706 P%i", slot);
            break;
        }
        return send(gcode.str(), op, slot);
    }

    std::optional<Request> take_request() {
        if (!request_valid || ticks_diff(valid_until_ms, ticks_ms()) < 0) {
            request_valid = false;
            return std::nullopt;
        }
        served = true;
        valid_until_ms = ticks_ms() + open_ms; // l'écran peut être recréé (retour d'un autre écran) sans perdre le nom
        return request;
    }

    void release_request() {
        if (request_valid) {
            valid_until_ms = ticks_ms() + chain_ms;
        }
    }

    void restore_request(Request r) {
        request = r;
        request_valid = true;
        served = true;
        valid_until_ms = ticks_ms() + open_ms;
    }

    bool request_served(Op op, uint8_t slot) {
        return served && request.op == op && request.slot == slot;
    }

    bool write_register(uint8_t reg, uint16_t value) {
        char gcode[24];
        snprintf(gcode, sizeof(gcode), "M708 A0x%02x X%u", reg, unsigned(value));
        return gui_try_gcode_with_msg(gcode);
    }

} // namespace atelier

} // namespace od3d
