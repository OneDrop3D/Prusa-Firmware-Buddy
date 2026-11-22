/// @file
#include "screen_climate_od3d.hpp"
#include "screen_filament_presets_od3d.hpp"

#include "widgets_od3d.hpp"

#include <algorithm>
#include <common/marlin_vars.hpp>
#include <cstdio>
#include <cstring>
#include <feature/od3d/chamber_climate_od3d.hpp>
#include <i18n.h>
#include <iterator>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <tool_index.hpp>
#include <window_msgbox.hpp>

namespace od3d {

namespace cc = chamber_climate;

namespace {

    constexpr NumericInputConfig step_config = {
        .min_value = cc::CooldownParams::step_min,
        .max_value = cc::CooldownParams::step_max,
        .unit = Unit::celsius,
    };
    constexpr NumericInputConfig interval_config = {
        .min_value = cc::CooldownParams::interval_min_min,
        .max_value = cc::CooldownParams::interval_min_max,
        .unit = Unit::minute,
    };
    constexpr NumericInputConfig floor_config = {
        .min_value = cc::CooldownParams::floor_min,
        .max_value = cc::CooldownParams::floor_max,
        .step = 5,
        .unit = Unit::celsius,
    };

    const char *const recipe_names[] = { N_("no recipe"), N_("ceiling"), N_("slow cool"), N_("both") };

    /// État pour l'élément : bit 31 actif, bits 16 à 30 consigne, bits 0 à 15 minutes avant le palier suivant
    uint32_t packed_status() {
        const cc::Status s = cc::status();
        if (!s.active) {
            return 0;
        }
        const uint32_t minutes = (s.next_step_s + 59) / 60;
        return (1ul << 31) | (uint32_t(std::max<int16_t>(s.target, 0) & 0x7fff) << 16) | (minutes & 0xffff);
    }

} // namespace

// ---- Fonctions OneDrop3D › Climat de la chambre ----

MI_CHAMBER_CLIMATE::MI_CHAMBER_CLIMATE()
    : IWindowMenuItem(_("Chamber climate\nMaterial recipes, slow bed cool-down"), &img::enclosure_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_CHAMBER_CLIMATE::click(IWindowMenu &) {
    open_screen<ScreenChamberClimate>();
}

MI_CLIMATE_RECIPES::MI_CLIMATE_RECIPES()
    : IWindowMenuItem(_("Material recipes\nChamber ceiling or slow cool-down"), &img::spool_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_CLIMATE_RECIPES::click(IWindowMenu &) {
    open_screen<ScreenClimateRecipes>();
}

MI_SOFT_COOLDOWN_STATE::MI_SOFT_COOLDOWN_STATE()
    : MenuItemAutoUpdatingLabel(
        _("Slow cool-down\nTouch to stop it"),
        [this](const std::span<char> &buffer) {
            const uint32_t v = value();
            if (!(v >> 31)) {
                _("Off").copyToRAM(buffer.data(), buffer.size());
                return;
            }
            snprintf(buffer.data(), buffer.size(), "%u\xC2\xB0"
                                                   "C %u min",
                unsigned((v >> 16) & 0x7fff), unsigned(v & 0xffff));
        },
        [](auto *) -> uint32_t { return packed_status(); }) {
    SetIconId(&img::heatbed_16x16);
    set_show_disabled_extension(true); // « Arrêt » reste lisible, grisé
    set_enabled(cc::status().active);
}

void MI_SOFT_COOLDOWN_STATE::Loop() {
    MenuItemAutoUpdatingLabel::Loop();
    set_enabled(value() >> 31);
}

void MI_SOFT_COOLDOWN_STATE::click(IWindowMenu &) {
    if (!cc::status().active) {
        return;
    }
    if (MsgBoxQuestion(_("Stop the slow cool-down? The bed heater turns off."), Responses_YesNo) == Response::Yes) {
        cc::request_stop();
    }
}

MI_COOLDOWN_STEP::MI_COOLDOWN_STEP()
    : WiSpin(cc::params().step_c, step_config, _("Step\nBed drop at each step"), &img::heatbed_16x16) {}

void MI_COOLDOWN_STEP::OnClick() {
    cc::CooldownParams p = cc::params();
    p.step_c = static_cast<uint8_t>(value());
    cc::set_params(p);
}

MI_COOLDOWN_INTERVAL::MI_COOLDOWN_INTERVAL()
    : WiSpin(cc::params().interval_min, interval_config, _("Step length\nMinutes between two steps"), &img::heatbed_16x16) {}

void MI_COOLDOWN_INTERVAL::OnClick() {
    cc::CooldownParams p = cc::params();
    p.interval_min = static_cast<uint8_t>(value());
    cc::set_params(p);
}

MI_COOLDOWN_FLOOR::MI_COOLDOWN_FLOOR()
    : WiSpin(cc::params().floor_c, floor_config, _("Down to\nBelow it: bed off, cool-down over"), &img::heatbed_16x16) {}

void MI_COOLDOWN_FLOOR::OnClick() {
    cc::CooldownParams p = cc::params();
    p.floor_c = static_cast<uint8_t>(value());
    cc::set_params(p);
}

MI_COOLDOWN_START::MI_COOLDOWN_START()
    : IWindowMenuItem(_("Start now\nFrom the current bed temperature"), &img::heatbed_16x16, is_enabled_t::yes, is_hidden_t::no) {}

void MI_COOLDOWN_START::click(IWindowMenu &) {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    for (const auto tool : PhysicalToolIndex::all()) {
        if (marlin_vars().hotend(tool).target_nozzle.get() > 0) {
            MsgBoxWarning(_("A nozzle is heating: the slow cool-down only drives a bed left alone."), Responses_Ok);
            return;
        }
    }
    if (marlin_vars().temp_bed.get() <= cc::params().floor_c + 1) {
        MsgBoxWarning(_("The bed is already below the floor temperature."), Responses_Ok);
        return;
    }
    marlin_client::gcode("M1998 S1");
}

MI_CLIMATE_FILAMENTS::MI_CLIMATE_FILAMENTS()
    : IWindowMenuItem(_("Chamber per filament\nTarget, min and max in Filaments"), &img::enclosure_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

// Réglages › Filaments : préréglages de Prusa modifiables (UI-13), et « Mes filaments » (ceux de l'utilisateur)
void MI_CLIMATE_FILAMENTS::click(IWindowMenu &) {
    open_screen<ScreenFilamentPresets>();
}

ScreenChamberClimate::ScreenChamberClimate()
    : BasicScreenMenu(_("Chamber climate")) {}

// ---- Recettes par matière ----

MI_CLIMATE_RECIPE::MI_CLIMATE_RECIPE(bool user, uint8_t list_index)
    : MenuItemSelectMenu({})
    , user(user)
    , index(0) {
    FilamentType type;
    if (user) {
        index = list_index;
        type = UserFilamentType { list_index };
    } else if (list_index < preset_filament_type_count) {
        const PresetFilamentType preset = preset_filament_types[list_index];
        index = static_cast<uint8_t>(preset);
        type = preset;
    }
    // Lignes en trop (préréglages absents de cette imprimante) et filaments de l'utilisateur non définis : cachés
    const bool shown = (user || list_index < preset_filament_type_count) && (!user || type.is_visible());
    set_is_hidden(!shown);
    SetIconId(user ? &img::user_filament_16x16 : &img::spool_16x16);
    if (!shown) {
        return;
    }

    // « PLA », puis « cible 20°C, max 38°C, filtre » (valeurs de Prusa)
    const FilamentTypeParameters p = type.parameters();
    char target[24];
    if (p.chamber_target_temperature.has_value()) {
        char word[16];
        _("target").copyToRAM(word, sizeof(word));
        snprintf(target, sizeof(target), "%s %d\xC2\xB0"
                                         "C",
            word, int(*p.chamber_target_temperature));
    } else {
        _("no target").copyToRAM(target, sizeof(target));
    }
    char max[16] = "";
    if (p.chamber_max_temperature.has_value()) {
        snprintf(max, sizeof(max), ", max %d\xC2\xB0"
                                   "C",
            int(*p.chamber_max_temperature));
    }
    char filter[16] = "";
    if (p.requires_filtration) {
        _(", filter").copyToRAM(filter, sizeof(filter));
    }
    snprintf(label.data(), label.size(), "%s\n%s%s%s", p.name.data(), target, max, filter);
    SetLabel(string_view_utf8::MakeRAM(label.data()));

    set_current_item(static_cast<int>(user ? cc::user_recipe(index) : cc::preset_recipe(index)));
}

int MI_CLIMATE_RECIPE::item_count() const {
    return static_cast<int>(std::size(recipe_names));
}

string_view_utf8 MI_CLIMATE_RECIPE::build_item_text(int i, StringViewUtf8ParamBase &) const {
    return _(recipe_names[i]);
}

bool MI_CLIMATE_RECIPE::on_item_selected(const OnItemSelectedArgs &args) {
    const auto r = static_cast<cc::Recipe>(args.new_index);
    if (user) {
        cc::set_user_recipe(index, r);
    } else {
        cc::set_preset_recipe(index, r);
    }
    return true;
}

ScreenClimateRecipes::ScreenClimateRecipes()
    : BasicScreenMenu(_("Material recipes")) {}

} // namespace od3d
