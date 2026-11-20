/// @file
#include "spool_join_od3d.hpp"

#include <config_store/store_instance.hpp>
#include <module/prusa/spool_join.hpp>
#include <module/prusa/tool_mapper.hpp>
#include <tool_index.hpp>
#include <variant>

namespace od3d {

namespace {

    bool used_by_print(VirtualToolIndex slot) {
        return std::holds_alternative<GcodeToolIndex>(tool_mapper.to_gcode(slot));
    }

} // namespace

void apply_persistent_spool_joins() {
    if (!config_store().od3d_spool_join.get()) {
        return;
    }
    // shortcut: un seul tour dans l'ordre des emplacements ; une chaîne réglée à rebours (4 → 2 puis 1 → 4) perd son 2e maillon, faire deux tours si on le voit
    for (const auto slot : VirtualToolIndex::all()) {
        const uint8_t target = config_store().od3d_spool_join_target.get(slot.to_raw());
        if (target >= VirtualToolIndex::count || spool_join.get_spool_2(slot)) {
            continue;
        }
        const auto to = VirtualToolIndex::from_raw(target);
        if (used_by_print(spool_join.get_first_spool_1_from_chain(slot)) && !used_by_print(to)) {
            spool_join.add_join(slot, to); // refuse seul les boucles, les doublons et les emplacements désactivés
        }
    }
}

} // namespace od3d
