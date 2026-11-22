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

    uint8_t chosen_slot = 0; ///< gardé tant que l'imprimante reste allumée

} // namespace

const char *ptfe_result_text(ptfe::Result result) {
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
        return N_("Failed: sensor not reached within 1000 mm");
    case pt::Result::too_short:
        return N_("Tube under 341 mm: unchanged");
    case pt::Result::too_long:
        return N_("Tube over 1000 mm (MMU max): unchanged");
    case pt::Result::interrupted:
        return N_("Failed: load interrupted (MMU error?)");
    case pt::Result::not_applied:
        return N_("Measured, but not applied by the MMU");
    }
    return nullptr;
}

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
    : MenuItemSelectMenu(_("Spool to use\nPreloaded in the MMU")) {
    SetIconId(&img::spool_16x16);
    set_behavior(Behavior::quick_cycle); // comme les relais de bobine (MI_SPOOL_JOIN_TARGET)
    set_current_item(chosen_slot);
}

int MI_PTFE_SLOT::item_count() const {
    return pt::slots;
}

string_view_utf8 MI_PTFE_SLOT::build_item_text(int index, StringViewUtf8ParamBase &params) const {
    return _("Slot %i").formatted(params, index + 1);
}

bool MI_PTFE_SLOT::on_item_selected(const OnItemSelectedArgs &args) {
    chosen_slot = uint8_t(args.new_index);
    return true;
}

MI_PTFE_START::MI_PTFE_START()
    : IWindowMenuItem(_("Start the measurement\nAbout 1 minute, cold nozzle OK"), &img::wizard_16x16, is_enabled_t::yes, is_hidden_t::no) {}

void MI_PTFE_START::click(IWindowMenu &) {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    StringViewUtf8Parameters<4> params;
    // Retour du 22/11 : « ça ralentit toujours au même endroit » pendant la mesure, c'est voulu : on le dit avant
    const string_view_utf8 question = _("The MMU pushes the spool of slot %d to the extruder, slowly after 341 mm (normal during the measurement), then unloads it. Is that spool preloaded?").formatted(params, chosen_slot + 1);
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
    const bool not_applied = s.result == pt::Result::not_applied;
    const uint32_t key = (uint32_t(s.result) << 24) | (uint32_t(not_applied ? s.mmu_mm : s.measured_mm) << 12) | s.saved_mm; // mm ≤ 4095 : 12 bits chacun
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
    char format[64];
    if (not_applied) {
        // Les deux valeurs : la longueur écrite et celle que le MMU a gardée (relue)
        _("Not applied: MMU %u mm, asked %u").copyToRAM(format, sizeof(format));
        snprintf(text.data() + n, text.size() - n, format, unsigned(s.mmu_mm), unsigned(s.saved_mm));
    } else if (const char *line = ptfe_result_text(s.result)) {
        _(line).copyToRAM(text.data() + n, text.size() - n);
    } else {
        // Une seule longueur réglée ; la distance mesurée entre parenthèses (« 2 données » : laquelle compte ?)
        _("Saved %u mm (sensor at %u mm)").copyToRAM(format, sizeof(format));
        snprintf(text.data() + n, text.size() - n, format, unsigned(s.saved_mm), unsigned(s.measured_mm));
    }
    SetLabel(string_view_utf8::MakeRAM(text.data()));
}

ScreenPtfe::ScreenPtfe()
    : BasicScreenMenu(_("Front PTFE Length")) {}

} // namespace od3d
