/// @file
#include "menu_items_hw_mmu.hpp"

#include <MItem_tools.hpp>
#include <array>
#include <common/marlin_vars.hpp>
#include <feature/od3d/ptfe_od3d.hpp>
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
    // OneDrop3D (F41) : M1999 L écrit puis relit le registre 0x22 (M708 ne vérifiait rien)
    snprintf(gcode.data(), gcode.size(), "M1999 L%u", new_mmu_bowden_length);
    gui_try_gcode_with_msg(gcode.data());
}

void MI_MMU_FRONT_PTFE_LENGTH::Loop() {
    // OneDrop3D (F41) : le registre 0x22 relu à chaque interrogation du MMU, sauf pendant le réglage. Chez Prusa, la
    // valeur n'était lue qu'à l'ouverture du menu : une écriture perdue ou une mesure en cours ne se voyait pas.
    const uint16_t mmu = marlin_vars().mmu2_bowden_length.get();
    if (!is_edited() && mmu >= mmu_bowden_limits.min_value && mmu <= mmu_bowden_limits.max_value) {
        set_value(float(mmu));
    }
    // Dernière écriture relue différente (menu ou mesure) : les deux valeurs, jusqu'à une écriture prise
    const od3d::ptfe::Write w = od3d::ptfe::last_write();
    const uint32_t refused = w.asked && w.mmu != w.asked ? (uint32_t(w.asked) << 16) | w.mmu : 0;
    if (refused != warned) {
        warned = refused;
        SetLabel(refused ? _("Front PTFE Length\nNot applied: MMU %u, asked %u").formatted(label_params, unsigned(w.mmu), unsigned(w.asked))
                         : _("Front PTFE Length"));
    }
}
