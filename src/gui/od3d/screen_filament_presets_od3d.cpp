/// @file
#include "screen_filament_presets_od3d.hpp"

#include "value_editor_od3d.hpp"
#include "widgets_od3d.hpp"

#include <cmath>
#include <config_store/store_instance.hpp>
#include <cstdio>
#include <cstring>
#include <feature/od3d/filament_presets_od3d.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <iterator>
#include <numeric_input_config_common.hpp>
#include <screen/filament/screen_filament_management.hpp>
#include <span>
#include <tool_index.hpp>
#include <utility>

static_assert(HAS_CHAMBER_API(), "Filaments : chambre de la CORE One");

namespace od3d {

namespace {

    /// Préréglage ouvert (fil de l'interface seulement) et son nom, titre de son écran
    PresetFilamentType current = PresetFilamentType::PLA;
    FilamentTypeParameters::Name current_name;

    constexpr const char *field_names[] = {
        N_("Nozzle"),
        N_("Nozzle standby"),
        N_("Bed"),
        N_("Chamber"),
        N_("Chamber minimum"),
        N_("Chamber maximum"),
    };
    static_assert(std::size(field_names) == fp::field_count);

    fp::Override override_of(PresetFilamentType t) {
        return config_store().od3d_preset_overrides.get(std::to_underlying(t));
    }

    void set_override(PresetFilamentType t, const fp::Override &o) {
        config_store().od3d_preset_overrides.set(std::to_underlying(t), o);
    }

    /// Valeurs de la table de Prusa, et valeurs appliquées (modifications comprises)
    fp::Values prusa_values() {
        return fp::values(preset_filament_parameters(current));
    }
    fp::Values current_values() {
        return fp::values(FilamentType(current).parameters());
    }

    /// « 20 °C », ou « Aucun » (chambre sans valeur)
    void format_value(std::span<char> buf, int16_t v) {
        if (v == fp::none) {
            _("None").copyToRAM(buf.data(), buf.size());
        } else {
            snprintf(buf.data(), buf.size(), "%d\xC2\xB0"
                                             "C",
                v);
        }
    }

    /// Éditeur de f : limites de Prusa ; pour la chambre (0 = « Aucun »), bornes qui gardent min <= cible <= max
    NumericInputConfig field_config(fp::Field f, const fp::Values &now) {
        switch (f) {
        case fp::nozzle:
            return numeric_input_config::filament_nozzle_temperature(AllTools {});
        case fp::nozzle_preheat:
            return numeric_input_config::nozzle_temperature(AllTools {});
        case fp::bed:
            return numeric_input_config::bed_temperature;
        default:
            break;
        }
        NumericInputConfig config = numeric_input_config::chamber_temp_with_none();
        const auto bound = [&](fp::Field a, fp::Field b, float otherwise) {
            return now[a] != fp::none ? float(now[a]) : now[b] != fp::none ? float(now[b]) : otherwise;
        };
        if (f == fp::chamber_target) {
            config.min_value = bound(fp::chamber_min, fp::chamber_min, config.min_value);
            config.max_value = bound(fp::chamber_max, fp::chamber_max, config.max_value);
        } else if (f == fp::chamber_min) {
            config.max_value = bound(fp::chamber_target, fp::chamber_max, config.max_value);
        } else {
            config.min_value = bound(fp::chamber_target, fp::chamber_min, config.min_value);
        }
        return config;
    }

} // namespace

// ---- Filaments ----

MI_FILAMENT_PRESETS::MI_FILAMENT_PRESETS()
    : IWindowMenuItem(_("Filaments\nTemperatures of PLA, PETG... editable"), &img::spool_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_FILAMENT_PRESETS::click(IWindowMenu &) {
    open_screen<ScreenFilamentPresets>();
}

MI_PRESET::MI_PRESET(uint8_t list_index)
    : IWindowMenuItem(string_view_utf8::MakeNULLSTR(), &img::spool_16x16, is_enabled_t::yes,
        list_index < preset_filament_type_count ? is_hidden_t::no : is_hidden_t::yes, expands_t::yes)
    , list_index(list_index) {
    if (list_index >= preset_filament_type_count) {
        return; // ligne en trop : préréglage absent de cette imprimante
    }
    // « PLA · modifié », puis « 215/60 °C · chambre 20 °C » (valeurs appliquées)
    const PresetFilamentType t = preset_filament_types[list_index];
    const FilamentTypeParameters p = FilamentType(t).parameters();
    char changed[24] = "";
    if (override_of(t).any()) {
        strcpy(changed, " \xC2\xB7 ");
        _("modified").copyToRAM(changed + strlen(changed), sizeof(changed) - strlen(changed));
    }
    char chamber[32] = "";
    if (p.chamber_target_temperature.has_value()) {
        char word[16];
        _("chamber").copyToRAM(word, sizeof(word));
        snprintf(chamber, sizeof(chamber), " \xC2\xB7 %s %d\xC2\xB0"
                                           "C",
            word, int(*p.chamber_target_temperature));
    }
    snprintf(label.data(), label.size(), "%s%s\n%d/%d\xC2\xB0"
                                         "C%s",
        p.name.data(), changed, int(p.nozzle_temperature), int(p.heatbed_temperature), chamber);
    SetLabel(string_view_utf8::MakeRAM(label.data()));
}

void MI_PRESET::click(IWindowMenu &) {
    current = preset_filament_types[list_index];
    current_name = preset_filament_parameters(current).name;
    open_screen<ScreenFilamentPreset>();
}

MI_USER_FILAMENTS::MI_USER_FILAMENTS()
    : IWindowMenuItem(_("My filaments\nYour own, order and visibility (Prusa)"), &img::user_filament_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_USER_FILAMENTS::click(IWindowMenu &) {
    open_screen<ScreenFilamentManagement>();
}

ScreenFilamentPresets::ScreenFilamentPresets()
    : BasicScreenMenu(_("Filaments")) {}

// ---- Un préréglage ----

// Sans icône au constructeur : WI_LAMBDA_LABEL_t réserverait alors à la valeur la place d'une icône
MI_PRESET_FIELD::MI_PRESET_FIELD(fp::Field field)
    : WI_LAMBDA_LABEL_t(string_view_utf8::MakeNULLSTR(), nullptr, is_enabled_t::yes, is_hidden_t::no, [field](const std::span<char> &buffer) {
        format_value(buffer, current_values()[field]);
    })
    , field(field) {
    SetIconId(field == fp::bed ? &img::heatbed_16x16 : field >= fp::chamber_target ? &img::enclosure_16x16
                                                                                   : &img::nozzle_16x16);
    update_label();
}

// Nom, puis « Valeur Prusa », ou « modifié · Prusa 20 °C »
void MI_PRESET_FIELD::update_label() {
    char name[24], note[32];
    _(field_names[field]).copyToRAM(name, sizeof(name));
    if (override_of(current).modified(field)) {
        char changed[16], prusa[12];
        _("modified").copyToRAM(changed, sizeof(changed));
        format_value(prusa, prusa_values()[field]);
        snprintf(note, sizeof(note), "%s \xC2\xB7 Prusa %s", changed, prusa);
    } else {
        _("Prusa value").copyToRAM(note, sizeof(note));
    }
    snprintf(label.data(), label.size(), "%s\n%s", name, note);
    SetLabel(string_view_utf8::MakeRAM(label.data()));
}

// Vaut tout de suite pour la suite (préchauffage, chargement, impression suivante) ; une valeur en cours ne change pas
void MI_PRESET_FIELD::click(IWindowMenu &) {
    const fp::Values now = current_values();
    const NumericInputConfig config = field_config(field, now);
    const auto v = ValueEditor::exec(_(field_names[field]), now[field] == fp::none ? 0 : now[field], config);
    if (!v) {
        return;
    }
    set_override(current, fp::with(override_of(current), field, static_cast<int16_t>(std::lround(*v)), prusa_values()));
    update_label();
    InValidateLabel(); // même tampon : SetLabel() ne voit pas le changement
    InValidateExtension();
}

MI_PRESET_RESTORE::MI_PRESET_RESTORE()
    : IWindowMenuItem(_("Restore Prusa value\nOne value or the whole preset"), &img::arrow_left_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_PRESET_RESTORE::click(IWindowMenu &) {
    open_screen<ScreenPresetRestore>();
}

ScreenFilamentPreset::ScreenFilamentPreset()
    : BasicScreenMenu(string_view_utf8::MakeRAM(current_name.data())) {}

// ---- Rétablir la valeur de Prusa ----

MI_PRESET_RESET::MI_PRESET_RESET(uint8_t field)
    : IWindowMenuItem(string_view_utf8::MakeNULLSTR(), &img::arrow_left_16x16, is_enabled_t::yes, is_hidden_t::no)
    , field(field) {
    if (field == fp::field_count) {
        SetLabel(_("Whole preset\nAll values back to Prusa's"));
    } else {
        char name[24], prusa[12];
        _(field_names[field]).copyToRAM(name, sizeof(name));
        format_value(prusa, prusa_values()[field]);
        snprintf(label.data(), label.size(), "%s\nPrusa %s", name, prusa);
        SetLabel(string_view_utf8::MakeRAM(label.data()));
    }
    set_enabled(modified());
}

bool MI_PRESET_RESET::modified() const {
    const fp::Override o = override_of(current);
    return field == fp::field_count ? o.any() : o.modified(fp::Field(field));
}

void MI_PRESET_RESET::Loop() {
    set_enabled(modified()); // ne redessine que si l'état change
}

void MI_PRESET_RESET::click(IWindowMenu &) {
    set_override(current, field == fp::field_count ? fp::Override {} : fp::without(override_of(current), fp::Field(field)));
}

ScreenPresetRestore::ScreenPresetRestore()
    : BasicScreenMenu(_("Prusa values")) {}

} // namespace od3d
