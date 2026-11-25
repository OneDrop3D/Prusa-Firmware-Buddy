/// @file
#include "screen_settings_od3d.hpp"

#include "widgets_od3d.hpp"

#include <img_resources.hpp>

namespace od3d {

MI_SET_PRINTING::MI_SET_PRINTING()
    : IWindowMenuItem(_("Printing\nSafety checks, runout, G-code"), &img::print_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_SET_PRINTING::click(IWindowMenu &) {
    open_screen<ScreenSetPrinting>();
}

ScreenSetPrinting::ScreenSetPrinting()
    : BasicScreenMenu(_("Printing")) {}

MI_SET_FILAMENT::MI_SET_FILAMENT()
    : IWindowMenuItem(HAS_MMU2() ? _("Filament and MMU\nSensors, MMU, cutter, OpenPrintTag") : _("Filament\nFilament sensors"), &img::spool_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_SET_FILAMENT::click(IWindowMenu &) {
    open_screen<ScreenSetFilament>();
}

ScreenSetFilament::ScreenSetFilament()
    : BasicScreenMenu(HAS_MMU2() ? _("Filament and MMU") : _("Filament")) {} // INDX : pas de MMU

MI_SET_CHAMBER::MI_SET_CHAMBER()
    : IWindowMenuItem(_("Chamber\nFiltration, lights, camera"), &img::enclosure_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_SET_CHAMBER::click(IWindowMenu &) {
    open_screen<ScreenSetChamber>();
}

ScreenSetChamber::ScreenSetChamber()
    : BasicScreenMenu(_("Chamber")) {}

MI_DIAGNOSTICS::MI_DIAGNOSTICS()
    : IWindowMenuItem(_("Diagnostics\nLogs, crash dump, metrics (support)"), &img::selftest_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_DIAGNOSTICS::click(IWindowMenu &) {
    open_screen<ScreenDiagnostics>();
}

ScreenDiagnostics::ScreenDiagnostics()
    : BasicScreenMenu(_("Diagnostics")) {}

} // namespace od3d
