/// @file
#include "spool_join_od3d.hpp"

#include <config_store/store_instance.hpp>
#include <cstring>
#include <filament.hpp>
#include <module/prusa/spool_join.hpp>
#include <module/prusa/tool_mapper.hpp>
#include <option/has_mmu2.h>
#include <tool_index.hpp>
#include <variant>

namespace od3d {

namespace {

    bool used_by_print(VirtualToolIndex slot) {
        return std::holds_alternative<GcodeToolIndex>(tool_mapper.to_gcode(slot));
    }

    /// Dans un relais (premier ou suivant) : ne peut plus servir de relais à un autre
    bool in_a_join(VirtualToolIndex slot) {
        return spool_join.get_spool_2(slot).has_value() || spool_join.get_first_spool_1_from_chain(slot) != slot;
    }

    /// Même matière et même couleur, les deux couleurs connues
    bool same_spool(VirtualToolIndex a, VirtualToolIndex b) {
        const FilamentType ta = FilamentType::for_tool(a), tb = FilamentType::for_tool(b);
        if (!ta || !tb) {
            return false;
        }
        const FilamentTypeParameters pa = ta.parameters(), pb = tb.parameters();
        return strcmp(pa.name.data(), pb.name.data()) == 0 && pa.color.has_value() && pb.color.has_value() && *pa.color == *pb.color;
    }

    void apply_automatic_joins() {
        for (const auto slot : VirtualToolIndex::all()) {
            if (!slot.is_enabled() || !used_by_print(slot)) {
                continue;
            }
            // Bout de la chaîne de cet emplacement, puis un relais de plus tant qu'il y a un emplacement pareil et libre
            VirtualToolIndex end = slot;
            for (int guard = 0; guard < int(VirtualToolIndex::count); ++guard) {
                while (const auto next = spool_join.get_spool_2(end)) {
                    end = *next;
                }
                bool added = false;
                for (const auto candidate : VirtualToolIndex::all()) {
                    if (candidate != end && candidate.is_enabled() && !used_by_print(candidate) && !in_a_join(candidate) && same_spool(slot, candidate)
                        && spool_join.add_join(end, candidate)) {
                        added = true;
                        break;
                    }
                }
                if (!added) {
                    break;
                }
            }
        }
    }

} // namespace

void apply_persistent_spool_joins() {
    // INDX : pas de MMU ; les relais entre têtes ne sont ni réglables à l'écran ni essayés : comme Prusa
    if (!HAS_MMU2() || !config_store().od3d_spool_join.get()) {
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
    apply_automatic_joins(); // après les règles manuelles, qui gardent la priorité
}

} // namespace od3d
