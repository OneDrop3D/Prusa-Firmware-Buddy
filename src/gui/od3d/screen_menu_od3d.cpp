/// @file
#include "screen_menu_od3d.hpp"

#include "value_editor_od3d.hpp"
#include "widgets_od3d.hpp"

#include <cmath>
#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <cstdio>
#include <feature/nozzle_thermal_compensation/nozzle_thermal_compensation.hpp>
#include <feature/od3d/lookahead_od3d.hpp>
#include <feature/od3d/pa_od3d.hpp>
#include <feature/od3d/pairing_od3d.hpp>
#include <feature/od3d/spool_calib_od3d.hpp>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <window_msgbox.hpp>

namespace od3d {

namespace nozzle_comp = buddy::nozzle_thermal_compensation;

// ---- Fonctions OneDrop3D ----

MI_FEATURES::MI_FEATURES()
    : IWindowMenuItem(_("OneDrop3D Features"), &img::wizard_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_FEATURES::click(IWindowMenu &) {
    open_screen<ScreenMenuFeatures>();
}

MI_NOZZLE_TEMPERATURE::MI_NOZZLE_TEMPERATURE()
    : IWindowMenuItem(_("Nozzle temperature\nExpansion, PID or MPC control"), &img::temperature_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_NOZZLE_TEMPERATURE::click(IWindowMenu &) {
    open_screen<ScreenNozzleTemperature>();
}

MI_SPOOL_JOIN_SLOTS::MI_SPOOL_JOIN_SLOTS()
    : IWindowMenuItem(_("Spool join\nEmpty spool: another one takes over"), &img::cycle_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_SPOOL_JOIN_SLOTS::click(IWindowMenu &) {
    open_screen<ScreenSpoolJoin>();
}

MI_PAIR_PRINTFORGE::MI_PAIR_PRINTFORGE()
    : IWindowMenuItem(_("Pair PrintForge\n6-digit code to link PrintForge"), &img::connect_16x16, is_enabled_t::yes, is_hidden_t::no) {}

void MI_PAIR_PRINTFORGE::click(IWindowMenu &) {
    // PrintForge reçoit le mot de passe PrusaLink : sans lui, il n'y a rien à transmettre.
    if (!config_store().prusalink_enabled.get() || !config_store().prusalink_password.get()[0]) {
        MsgBoxWarning(_("Turn on PrusaLink first: Settings > Network > PrusaLink."), Responses_Ok);
        return;
    }
    char code[8];
    snprintf(code, sizeof(code), "%06lu", static_cast<unsigned long>(od3d::pairing::start()));
    StringViewUtf8Parameters<8> params;
    MsgBoxInfo(_("Type this code in PrintForge:\n\n%s\n\nValid for 5 minutes.").formatted(params, code), Responses_Ok);
    od3d::pairing::stop();
}

ScreenMenuFeatures::ScreenMenuFeatures()
    : BasicScreenMenu(_("OneDrop3D Features")) {}

// ---- Ventilation anticipée (F26) ----

namespace {

    constexpr NumericInputConfig fan_lead_config {
        .min_value = 0,
        .max_value = 1,
        .step = 0.1f,
        .special_value = 0, ///< « Arrêt » : comme Prusa
        .max_decimal_places = 1,
        .unit = Unit::second,
    };

} // namespace

// Vaut tout de suite, impression comprise (seules les hausses du ventilateur sont avancées)
MI_FAN_LEAD::MI_FAN_LEAD()
    // Sans icône au constructeur : WI_LAMBDA_LABEL_t réserverait alors à la valeur la place d'une icône
    : WI_LAMBDA_LABEL_t(_("Fan lead\nPart fan speeds up to 1 s early"), nullptr, is_enabled_t::yes, is_hidden_t::no, [](const std::span<char> &buffer) {
        const unsigned ds = lookahead::fan_lead_ds();
        if (ds == 0) {
            _("Off").copyToRAM(buffer.data(), buffer.size());
        } else {
            snprintf(buffer.data(), buffer.size(), "%u.%u s", ds / 10, ds % 10);
        }
    }) {
    SetIconId(&img::fan_16x16);
}

void MI_FAN_LEAD::click(IWindowMenu &) {
    if (const auto v = ValueEditor::exec(_("Fan lead"), lookahead::fan_lead_ds() / 10.0f, fan_lead_config)) {
        lookahead::set_fan_lead_ds(static_cast<uint8_t>(std::lround(*v * 10)));
        InValidateExtension();
    }
}

// ---- PA de l'imprimante (F4) ----

// Vaut dès le prochain changement d'outil ou M572 ; la mesure se lance depuis la fiche de la bobine (puce « PA »).
MI_PA_PRINTER_WINS::MI_PA_PRINTER_WINS()
    : WI_ICON_SWITCH_OFF_ON_t(pa::printer_wins(), _("Printer PA wins\nMeasured slot PA replaces the slicer's M572"), &img::nozzle_16x16) {}

void MI_PA_PRINTER_WINS::OnChange(size_t) {
    pa::set_printer_wins(value());
}

// ---- Limite de débit (F5) ----

// Vaut dès maintenant pour l'emplacement chargé ; la mesure se lance depuis la fiche de la bobine (PA › Débit, temp.).
MI_FLOW_LIMIT::MI_FLOW_LIMIT()
    : WI_ICON_SWITCH_OFF_ON_t(calib::flow_limit_enabled(), _("Limit to measured flow\nSlows moves above the spool's max flow"), &img::nozzle_16x16) {}

void MI_FLOW_LIMIT::OnChange(size_t) {
    calib::set_flow_limit_enabled(value());
}

// ---- Température de la buse (F2) ----

// Pas pendant une impression : le Z changerait d'un coup (jusqu'à 0,05 mm à 280 °C).
MI_NOZZLE_THERMAL_COMPENSATION::MI_NOZZLE_THERMAL_COMPENSATION()
    : WI_ICON_SWITCH_OFF_ON_t(nozzle_comp::is_enabled(), _("Nozzle expansion\nFixes Z as the hot nozzle grows longer"), &img::nozzle_16x16,
        marlin_client::is_printing() ? is_enabled_t::no : is_enabled_t::yes) {}

void MI_NOZZLE_THERMAL_COMPENSATION::OnChange(size_t) {
    nozzle_comp::set_enabled(value());
}

// Même calcul que le firmware, d'après la cible de la buse (lue côté interface, pas le terme du serveur Marlin)
MI_NOZZLE_THERMAL_CORRECTION::MI_NOZZLE_THERMAL_CORRECTION()
    : MenuItemAutoUpdatingLabel(_("Current correction"), "%+.3f mm", [](auto *) -> float {
        return nozzle_comp::is_enabled() ? nozzle_comp::elongation_vs_reference_mm(marlin_vars().active_hotend().target_nozzle.get()) : 0.0f;
    }) {
    SetIconId(&img::z_axis_16x16);
}

ScreenNozzleTemperature::ScreenNozzleTemperature()
    : BasicScreenMenu(_("Nozzle temp.")) {}

// ---- Relais de bobine (F3) ----

// Les relais sont lus au départ de chaque impression : un changement vaut pour la suivante.
MI_SPOOL_JOIN::MI_SPOOL_JOIN()
    : WI_ICON_SWITCH_OFF_ON_t(config_store().od3d_spool_join.get(), _("Automatic join\nSame material and colour, or the rules below"), &img::cycle_16x16) {}

void MI_SPOOL_JOIN::OnChange(size_t) {
    config_store().od3d_spool_join.set(value());
}

MI_SPOOL_JOIN_TARGET::MI_SPOOL_JOIN_TARGET(uint8_t slot)
    : MenuItemSelectMenu({})
    , slot(slot) {
    SetLabel(_("When %i runs out").formatted(label_params, slot + 1));
    SetIconId(&img::cycle_16x16);
    set_behavior(Behavior::quick_cycle);
    set_is_hidden(!VirtualToolIndex::from_raw(slot).is_enabled());
    const uint8_t target = config_store().od3d_spool_join_target.get(slot);
    set_current_item(target < VirtualToolIndex::count ? target + 1 : 0);
}

int MI_SPOOL_JOIN_TARGET::item_count() const {
    return VirtualToolIndex::count + 1;
}

string_view_utf8 MI_SPOOL_JOIN_TARGET::build_item_text(int index, StringViewUtf8ParamBase &params) const {
    return index == 0 ? _("do nothing") : _("Slot %i").formatted(params, index);
}

bool MI_SPOOL_JOIN_TARGET::on_item_selected(const OnItemSelectedArgs &args) {
    int index = args.new_index;
    if (index == slot + 1) {
        index = (index + 1) % item_count(); // un emplacement ne prend pas le relais de lui-même : valeur suivante
    }
    config_store().od3d_spool_join_target.set(slot, index == 0 ? uint8_t(0xff) : uint8_t(index - 1));
    set_current_item(index);
    return false; // valeur déjà posée
}

ScreenSpoolJoin::ScreenSpoolJoin()
    : BasicScreenMenu(_("Spool Join")) {}

} // namespace od3d
