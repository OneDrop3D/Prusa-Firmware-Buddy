/// @file
#include "lookahead_od3d.hpp"
#include "lookahead_math_od3d.hpp"

#include <algorithm>
#include <atomic>

#include <config_store/store_instance.hpp>
#include <inc/MarlinConfig.h>
#include <module/planner.h>

namespace od3d::lookahead {

static_assert(BLOCK_BUFFER_SIZE <= 128 && (BLOCK_BUFFER_SIZE & (BLOCK_BUFFER_SIZE - 1)) == 0, "index sur 8 bits, masque");

namespace {

    constexpr uint8_t max_fan_lead_ds = static_cast<uint8_t>(max_fan_lead_s * 10 + 0.5f);

    std::atomic<uint8_t> fan_lead = 0; ///< [dixièmes de seconde]

    /// Une case de la file, résumée. Les champs lus (drapeaux, longueur, vitesse nominale, pas, sens, ventilateur)
    /// sont écrits par la tâche de Marlin avant que head ne passe la case, et ne sont plus modifiés ensuite que par
    /// cette même tâche (recalcul des vitesses de jonction, qui ne touche pas à ces champs).
    BlockInfo read_block(uint8_t index) {
        block_t &b = Planner::block_buffer[index];
        if (b.flag.sync_position) {
            return BlockInfo { .duration_s = 0, .e_mm = 0, .fan = 0, .is_move = false };
        }
        // Le filament d'un mouvement du seul extrudeur (recul, retour, chargement) n'est pas du débit d'impression
        const bool moves_xyz = b.msteps.x != 0 || b.msteps.y != 0 || b.msteps.z != 0;
        float e_mm = moves_xyz ? float(b.msteps.e) * Planner::mm_per_mstep[E_AXIS] : 0.0f;
        if (TEST(b.direction_bits, E_AXIS)) {
            e_mm = -e_mm;
        }
        return BlockInfo {
            .duration_s = block_duration(b.millimeters, b.nominal_speed),
            .e_mm = e_mm,
            .fan = b.print_fan_speed,
            .is_move = true,
        };
    }

} // namespace

float planner_e_speed_ahead(float max_e_speed) {
    // Une seule copie des index ; head ne bouge pas pendant la lecture (même tâche que le remplissage)
    const uint8_t head = Planner::block_buffer_head;
    const uint8_t tail = Planner::block_buffer_tail;
    FlowScan scan(flow_window_s, max_e_speed);
    scan_ring(tail, head, uint8_t(BLOCK_BUFFER_SIZE), scan, read_block);
    return std::clamp(scan.mean_e_speed(), 0.0f, max_e_speed);
}

uint8_t fan_speed_ahead(uint8_t current) {
    const uint8_t lead_ds = fan_lead.load();
    if (lead_ds == 0) {
        return current; // comportement de Prusa
    }
    const uint8_t head = Planner::block_buffer_head;
    const uint8_t tail = Planner::block_buffer_tail;
    FanScan scan(lead_ds / 10.0f);
    scan_ring(tail, head, uint8_t(BLOCK_BUFFER_SIZE), scan, read_block);
    // Jamais moins que ce que Prusa appliquerait maintenant : seules les hausses sont avancées
    return std::max(current, scan.fan().value_or(current));
}

void load() {
    fan_lead = std::min<uint8_t>(config_store().od3d_fan_lead.get(), max_fan_lead_ds);
}

uint8_t fan_lead_ds() {
    return fan_lead.load();
}

void set_fan_lead_ds(uint8_t lead_ds) {
    const uint8_t v = std::min(lead_ds, max_fan_lead_ds);
    fan_lead = v;
    config_store().od3d_fan_lead.set(v);
}

} // namespace od3d::lookahead
