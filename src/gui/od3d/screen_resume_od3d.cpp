/// @file
#include "screen_resume_od3d.hpp"

#include "widgets_od3d.hpp"

#include <cmath>
#include <feature/od3d/clean_resume_od3d.hpp>
#include <i18n.h>
#include <img_resources.hpp>

namespace od3d {

namespace cr = clean_resume;

namespace {

    constexpr NumericInputConfig purge_config = {
        .max_value = cr::Settings::purge_max,
        .unit = Unit::millimeter,
    };
    constexpr NumericInputConfig retract_config = {
        .max_value = cr::Settings::retract_max / 10.0f,
        .step = 0.1f,
        .max_decimal_places = 1,
        .unit = Unit::millimeter,
    };
    constexpr NumericInputConfig approach_config = {
        .min_value = cr::Settings::approach_min,
        .max_value = cr::Settings::approach_max,
    };

    /// Lit, modifie et enregistre les réglages (ramenés dans leurs plages)
    template <typename F>
    void modify(F &&f) {
        cr::Settings s = cr::settings();
        f(s);
        cr::set_settings(s);
    }

} // namespace

MI_CLEAN_RESUME::MI_CLEAN_RESUME()
    : IWindowMenuItem(_("Clean resume\nPurge off the part before going back"), &img::print_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_CLEAN_RESUME::click(IWindowMenu &) {
    open_screen<ScreenCleanResume>();
}

// Vaut pour la prochaine reprise (après pause ou coupure)
MI_CLEAN_RESUME_ENABLE::MI_CLEAN_RESUME_ENABLE()
    : WI_ICON_SWITCH_OFF_ON_t(cr::settings().enabled, _("Clean resume\nAfter a pause or a power loss"), &img::print_16x16) {}

void MI_CLEAN_RESUME_ENABLE::OnChange(size_t) {
    modify([this](cr::Settings &s) { s.enabled = value(); });
}

MI_CLEAN_RESUME_PURGE::MI_CLEAN_RESUME_PURGE()
    : WiSpin(cr::settings().purge_mm, purge_config, _("Purge\nPushed out at Prusa's purge spot"), &img::spool_16x16) {}

void MI_CLEAN_RESUME_PURGE::OnClick() {
    modify([this](cr::Settings &s) { s.purge_mm = static_cast<uint8_t>(value()); });
}

MI_CLEAN_RESUME_RETRACT::MI_CLEAN_RESUME_RETRACT()
    : WiSpin(cr::settings().retract_tenths / 10.0f, retract_config, _("Retraction\nPushed back exactly on return"), &img::spool_16x16) {}

void MI_CLEAN_RESUME_RETRACT::OnClick() {
    modify([this](cr::Settings &s) { s.retract_tenths = static_cast<uint8_t>(std::lround(value() * 10)); });
}

MI_CLEAN_RESUME_APPROACH::MI_CLEAN_RESUME_APPROACH()
    : WiSpin(cr::settings().approach_mm_s, approach_config, _("Approach (mm/s)\nLast 5 mm down onto the part"), &img::z_axis_16x16) {}

void MI_CLEAN_RESUME_APPROACH::OnClick() {
    modify([this](cr::Settings &s) { s.approach_mm_s = static_cast<uint8_t>(value()); });
}

ScreenCleanResume::ScreenCleanResume()
    : BasicScreenMenu(_("Clean resume")) {}

} // namespace od3d
