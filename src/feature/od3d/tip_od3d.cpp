/// @file
#include "tip_od3d.hpp"

#include <array>

#include <config_store/store_instance.hpp>
#include <feature/prusa/MMU2/mmu2_mk4.h>
#include <marlin_server.hpp>
#include <timing.h>
#include <tool_index.hpp>

namespace od3d::tip {

static_assert(slots <= VirtualToolIndex::count); // « OD3D Tip Stats » a une place par outil

namespace {

    constexpr uint32_t step_interval_ms = 250; ///< un changement d'outil dure bien plus

    // Fil de Marlin seulement (step)
    std::array<Stats, slots> live;
    bool started = false;
    bool dirty = false; ///< mesures pas encore écrites
    uint16_t changes_seen = 0; ///< MMU2::ToolChangeCounter au tour d'avant
    uint8_t tool_seen = 0xff; ///< emplacement dans la buse au tour d'avant : celui qui est déchargé au changement suivant
    uint32_t last_step_ms = 0;

} // namespace

Stats stats(uint8_t slot) {
    return slot < slots ? config_store().od3d_tip_stats.get(slot) : Stats {};
}

void step() {
    const uint32_t now_ms = ticks_ms();
    if (ticks_diff(now_ms, last_step_ms) < int32_t(step_interval_ms)) {
        return;
    }
    last_step_ms = now_ms;

    MMU2::MMU2 &mmu = MMU2::mmu2;
    const uint16_t changes = mmu.ToolChangeCounter();
    const bool printing = marlin_server::is_printing();
    if (!started) {
        for (uint8_t i = 0; i < slots; ++i) {
            live[i] = config_store().od3d_tip_stats.get(i);
        }
        started = true;
    } else if (changes != changes_seen) {
        // Un seul changement depuis le tour d'avant : le recul mesuré est celui de la pointe de tool_seen. Hors impression
        // (M1704, T à la main), Prusa décharge d'abord : rien n'est mesuré.
        if (printing && uint16_t(changes - changes_seen) == 1 && tool_seen < slots) {
            live[tool_seen] = add(live[tool_seen], mmu.UnloadEPosOnFSOff());
            dirty = true;
        }
    }
    changes_seen = changes;
    tool_seen = mmu.get_current_tool();

    if (dirty && !printing) { // une écriture par impression
        for (uint8_t i = 0; i < slots; ++i) {
            if (!(live[i] == config_store().od3d_tip_stats.get(i))) {
                config_store().od3d_tip_stats.set(i, live[i]);
            }
        }
        dirty = false;
    }
}

} // namespace od3d::tip
