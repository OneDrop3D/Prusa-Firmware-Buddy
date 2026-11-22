/// @file
#include "menu_items_hw_mmu.hpp"

#include <MItem_tools.hpp>
#include <array>
#include <common/marlin_vars.hpp>
#include <numeric_input_config.hpp>
#include <option/has_mmu2.h>

static_assert(HAS_MMU2());

static constexpr NumericInputConfig mmu_bowden_limits {
    .min_value = 341,
    .max_value = 1000,
    .unit = Unit::millimeter,
};

MI_MMU_FRONT_PTFE_LENGTH::MI_MMU_FRONT_PTFE_LENGTH()
    : WiSpin {
        static_cast<float>(marlin_vars().mmu2_bowden_length.get()),
        mmu_bowden_limits,
        _("Front PTFE Length"),
    } {
}

void MI_MMU_FRONT_PTFE_LENGTH::OnClick() {
    const auto new_mmu_bowden_length = static_cast<uint16_t>(value());

    std::array<char, 32> gcode;
    snprintf(gcode.data(), gcode.size(), "M708 A0x22 X%u", new_mmu_bowden_length);
    gui_try_gcode_with_msg(gcode.data());
}

void MI_MMU_FRONT_PTFE_LENGTH::Loop() {
    // OneDrop3D (F41) : le registre 0x22 relu à chaque interrogation du MMU, sauf pendant le réglage. Chez Prusa, la
    // valeur n'était lue qu'à l'ouverture du menu : une écriture perdue ou une mesure en cours ne se voyait pas.
    const uint16_t mmu = marlin_vars().mmu2_bowden_length.get();
    if (!is_edited() && mmu >= mmu_bowden_limits.min_value && mmu <= mmu_bowden_limits.max_value) {
        set_value(float(mmu));
    }
}
