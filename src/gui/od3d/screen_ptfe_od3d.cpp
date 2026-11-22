/// @file
#include "screen_ptfe_od3d.hpp"

#include "widgets_od3d.hpp"

#include <common/marlin_vars.hpp>
#include <cstdio>
#include <cstring>
#include <feature/od3d/ptfe_od3d.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    namespace pt = ptfe;

    // Les numéros s'écrivent pareil en français : pas de traduction (_() rend le texte tel quel)
    constexpr const char *slot_items[] = { "1", "2", "3", "4", "5" };
    uint8_t chosen_slot = 0; ///< gardé tant que l'imprimante reste allumée

    const char *result_text(pt::Result result) {
        switch (result) {
        case pt::Result::none:
            return N_("Not measured yet");
        case pt::Result::running:
            return N_("Measuring...");
        case pt::Result::saved:
            return nullptr; // chiffres : show()
        case pt::Result::refused_printing:
            return N_("Refused: printing");
        case pt::Result::refused_mmu:
            return N_("Refused: MMU not ready");
        case pt::Result::refused_sensor:
            return N_("Refused: extruder sensor inactive");
        case pt::Result::refused_extruder:
            return N_("Refused: filament in the extruder");
        case pt::Result::refused_selector:
            return N_("Refused: filament in the selector");
        case pt::Result::no_trigger:
            return N_("Failed: the sensor saw nothing");
        case pt::Result::too_short:
            return N_("Tube under 341 mm: unchanged");
        }
        return nullptr;
    }

} // namespace

MI_PTFE_AUTO::MI_PTFE_AUTO()
    : IWindowMenuItem(_("Measure automatically\nWith a preloaded spool"), &img::spool_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_PTFE_AUTO::click(IWindowMenu &) {
    open_screen<ScreenPtfe>();
}

MI_PTFE_LENGTH_NOW::MI_PTFE_LENGTH_NOW()
    : MenuItemAutoUpdatingLabel(_("Front PTFE Length"), "%d mm", [](auto *) -> int {
        return marlin_vars().mmu2_bowden_length.get();
    }) {
    SetIconId(&img::metrics_16x16);
}

MI_PTFE_SLOT::MI_PTFE_SLOT()
    : MenuItemSwitch(_("Measured slot\nIts spool must be preloaded"), slot_items, chosen_slot) {
    SetIconId(&img::spool_16x16);
}

bool MI_PTFE_SLOT::on_item_selected(const OnItemSelectedArgs &args) {
    chosen_slot = uint8_t(args.new_index);
    return MenuItemSwitch::on_item_selected(args);
}

MI_PTFE_START::MI_PTFE_START()
    : IWindowMenuItem(_("Start the measurement\nAbout 1 minute, cold nozzle OK"), &img::wizard_16x16, is_enabled_t::yes, is_hidden_t::no) {}

void MI_PTFE_START::click(IWindowMenu &) {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    StringViewUtf8Parameters<4> params;
    const string_view_utf8 question = _("The MMU pushes the spool of slot %d to the extruder gears, measures the front PTFE tube, then unloads it. Is that spool preloaded?").formatted(params, chosen_slot + 1);
    if (MsgBoxQuestion(question, Responses_YesNo) != Response::Yes) {
        return;
    }
    marlin_client::gcode_printf("M1999 B1 P%u", unsigned(chosen_slot));
}

MI_PTFE_RESULT::MI_PTFE_RESULT()
    : IWindowMenuItem(_("Last measurement"), &img::info_16x16, is_enabled_t::yes, is_hidden_t::no) {
    show();
}

void MI_PTFE_RESULT::Loop() {
    show();
}

void MI_PTFE_RESULT::show() {
    const pt::Status s = pt::status();
    const uint32_t key = (uint32_t(s.result) << 24) | (uint32_t(s.measured_mm) << 12) | s.saved_mm; // mm ≤ 1000 : 12 bits chacun
    if (key == shown_key) {
        return;
    }
    shown_key = key;
    _("Last measurement").copyToRAM(text.data(), text.size());
    size_t n = strlen(text.data());
    if (n + 1 >= text.size()) {
        return;
    }
    text[n++] = '\n';
    if (const char *line = result_text(s.result)) {
        _(line).copyToRAM(text.data() + n, text.size() - n);
    } else {
        char format[48];
        _("%u mm measured, %u mm saved").copyToRAM(format, sizeof(format));
        snprintf(text.data() + n, text.size() - n, format, unsigned(s.measured_mm), unsigned(s.saved_mm));
    }
    SetLabel(string_view_utf8::MakeRAM(text.data()));
}

ScreenPtfe::ScreenPtfe()
    : BasicScreenMenu(_("Front PTFE Length")) {}

} // namespace od3d
