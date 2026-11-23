/// @file
#include "screen_skew_od3d.hpp"

#include "value_editor_od3d.hpp"
#include "widgets_od3d.hpp"

#include <config_store/store_instance.hpp>
#include <algorithm>
#include <cstdio>
#include <feature/od3d/calib_assistant_od3d.hpp>
#include <feature/od3d/skew_od3d.hpp>
#include <img_resources.hpp>
#include <iterator>
#include <marlin_client.hpp>
#include <numeric_input_config.hpp>
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    /// Mesures saisies (mm), gardées tant que l'imprimante est allumée ; au départ, un carré de 100 mm d'équerre
    constexpr float square[3] = { 141.42f, 141.42f, 100.0f };
    float measures[3] = { square[0], square[1], square[2] };
    uint8_t measures_revision = 0;

    constexpr const char *length_labels[3] = { N_("Diagonal AC"), N_("Diagonal BD"), N_("Side AD") };

    // SPEC_G.md : diagonales de 100 à 200 mm au 1/100, côté de 50 à 200 mm au 1/10
    constexpr NumericInputConfig diagonal_config { .min_value = 100, .max_value = 200, .step = 0.01f, .max_decimal_places = 2, .unit = Unit::millimeter };
    constexpr NumericInputConfig side_config { .min_value = 50, .max_value = 200, .step = 0.1f, .max_decimal_places = 1, .unit = Unit::millimeter };

    skew::Result measured() {
        return skew::factor_from_measures(measures[0], measures[1], measures[2]);
    }

    /// Angle en degrés, signé, 3 décimales (« +0.073° ») ; ASCII et ° seulement (polices de l'imprimante)
    void format_degrees(float factor, char *out, size_t size) {
        snprintf(out, size, "%+.3f\xC2\xB0", static_cast<double>(skew::factor_to_degrees(factor)));
    }

    is_enabled_t enabled_unless_printing() {
        // Pendant une impression, la correction décalerait la suite de la pièce
        return marlin_client::is_printing() ? is_enabled_t::no : is_enabled_t::yes;
    }

    /// Loop() des éléments grisés pendant une impression : construits pendant elle, ils restaient grisés ensuite
    void follow_printing(IWindowMenuItem &item) {
        item.set_enabled(enabled_unless_printing() == is_enabled_t::yes);
    }

} // namespace

// ---- Élément de « Fonctions OneDrop3D » ----

MI_SKEW_XY::MI_SKEW_XY()
    : IWindowMenuItem(_("XY squareness\nFixes a frame that is not square"), &img::move_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_SKEW_XY::click(IWindowMenu &) {
    open_screen<ScreenSkew>();
}

// ---- Écran « Équerrage XY » ----

MI_SKEW_HOWTO::MI_SKEW_HOWTO()
    : IWindowMenuItem(_("Test piece\nPrint a 100 mm square, then measure it"), &img::print_16x16, is_enabled_t::yes, is_hidden_t::no) {}

void MI_SKEW_HOWTO::click(IWindowMenu &) {
    MsgBoxInfo(_("Print a solid 100 x 100 x 3 mm square in the middle of the bed. "
                 "Seen from the front: A front left, B back left, C back right, D front right. "
                 "Measure AC, BD (diagonals) and AD (side) with a caliper."),
        Responses_Ok);
}

MI_SKEW_LENGTH::MI_SKEW_LENGTH(uint8_t which)
    : WI_LAMBDA_LABEL_t(_(length_labels[which]), [which](const std::span<char> &buffer) {
        snprintf(buffer.data(), buffer.size(), which == 2 ? "%.1f mm" : "%.2f mm", static_cast<double>(measures[which]));
    })
    , which(which) {
    SetIconId(&img::move_16x16);
}

void MI_SKEW_LENGTH::click(IWindowMenu &menu) {
    const NumericInputConfig &config = which == 2 ? side_config : diagonal_config;
    if (const auto value = ValueEditor::exec(_(length_labels[which]), measures[which], config, nullptr, nullptr, ValueEditor::row(menu, *this))) {
        measures[which] = *value;
        ++measures_revision;
        InValidateExtension();
    }
}

MI_SKEW_MEASURED::MI_SKEW_MEASURED()
    : WI_LAMBDA_LABEL_t(_("Measured skew"), [](const std::span<char> &buffer) {
        const skew::Result result = measured();
        if (result.status == skew::Status::ok) {
            format_degrees(result.factor, buffer.data(), buffer.size());
        } else {
            snprintf(buffer.data(), buffer.size(), "--"); // incohérent ou trop grand : « Appliquer » dit pourquoi
        }
    })
    , revision(measures_revision) {
    SetIconId(&img::info_16x16);
}

void MI_SKEW_MEASURED::Loop() {
    if (revision != measures_revision) {
        revision = measures_revision;
        InValidateExtension();
    }
}

MI_SKEW_APPLY::MI_SKEW_APPLY()
    : IWindowMenuItem(_("Apply correction"), &img::ok_color_16x16, enabled_unless_printing(), is_hidden_t::no) {}

void MI_SKEW_APPLY::click(IWindowMenu &) {
    const skew::Result result = measured();
    // La pièce a été imprimée avec la correction actuelle : l'écart mesuré est le reste, il s'y ajoute
    // (deux cisaillements s'additionnent). Pour repartir de zéro : « Remettre à zéro », puis réimprimer.
    const float total = config_store().od3d_skew_xy.get() + result.factor;
    if (result.status == skew::Status::inconsistent) {
        MsgBoxWarning(_("Inconsistent measurements: check AC, BD and AD."), Responses_Ok);
        return;
    }
    if (result.status == skew::Status::too_large || !skew::is_valid_factor(total)) {
        MsgBoxWarning(_("Skew over 0.57\xC2\xB0: adjust the frame and belts rather than correcting it."), Responses_Ok);
        return;
    }
    // M852 enregistre le facteur (config_store) et l'applique dans le fil de Marlin
    marlin_client::gcode_printf("M852 I%.6f", static_cast<double>(total));
    calib_assistant::record_done(calib_registry::Test::skew); // l'assistant ne la demande plus (jusqu'à sa prochaine version)
    // Mesures consommées : un 2e « Appliquer » ajoutait encore l'écart (REV-1). Revenues au carré d'équerre (écart
    // nul, testé dans skew_od3d_test.cpp), elles attendent la pièce réimprimée ; la boîte refermée les redessine.
    std::copy(std::begin(square), std::end(square), measures);
    ++measures_revision;
    char angle[16];
    format_degrees(total, angle, sizeof(angle));
    StringViewUtf8Parameters<16> params;
    MsgBoxInfo(_("XY correction saved: %s (measured skew added to the current one). Used for prints with bed leveling. Print the test piece again to check.").formatted(params, angle), Responses_Ok);
}

void MI_SKEW_APPLY::Loop() {
    follow_printing(*this);
}

MI_SKEW_CURRENT::MI_SKEW_CURRENT()
    : MenuItemAutoUpdatingLabel(_("Current correction"), "%+.3f\xC2\xB0", [](auto *) -> float {
        return skew::factor_to_degrees(config_store().od3d_skew_xy.get());
    }) {
    SetIconId(&img::calibrate_white_16x16);
}

MI_SKEW_RESET::MI_SKEW_RESET()
    : IWindowMenuItem(_("Reset to zero"), &img::cycle_16x16, enabled_unless_printing(), is_hidden_t::no) {}

void MI_SKEW_RESET::click(IWindowMenu &) {
    marlin_client::gcode("M852 I0");
}

void MI_SKEW_RESET::Loop() {
    follow_printing(*this);
}

ScreenSkew::ScreenSkew()
    : BasicScreenMenu(_("XY squareness")) {}

} // namespace od3d
