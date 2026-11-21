/// @file
#include "island_od3d.hpp"

#include "atelier_od3d.hpp"
#include "screen_progress_od3d.hpp"

#include <fsm/filament_change_phases.hpp>
#include <fsm_loadunload_type.hpp>
#include <ScreenFactory.hpp>
#include <ScreenHandler.hpp>

namespace od3d::island {

namespace {

    bool is_minimized = false;
    fsm::BaseData last_data;
    std::optional<Origin> last_origin;
    uint8_t last_progress = 0;
    uint16_t rev = 0;

    void take(fsm::BaseData data) {
        last_data = data;
        // MMU_ERRWaitingForUser porte un pointeur vers l'erreur, pas d'avancement (et rouvre l'écran de toute façon)
        if (GetEnumFromPhaseIndex<PhasesLoadUnload>(data.GetPhase()) != PhasesLoadUnload::MMU_ERRWaitingForUser) {
            last_progress = fsm::deserialize_data<FSMLoadUnloadData>(data.GetData()).progress;
        }
        ++rev;
    }

} // namespace

bool minimized() {
    return is_minimized;
}

void minimize(fsm::BaseData data, std::optional<Origin> origin) {
    is_minimized = true;
    last_origin = origin;
    take(data);
}

void update(fsm::BaseData data) {
    if (is_minimized) {
        take(data);
    }
}

void end() {
    is_minimized = false;
    last_origin.reset();
    ++rev;
}

void prepare_reopen() {
    is_minimized = false;
    if (last_origin) {
        atelier::restore_request({ atelier::Op(last_origin->op), last_origin->slot });
    }
    ++rev;
}

void restore() {
    if (!is_minimized) {
        return;
    }
    prepare_reopen();
    screen_init_variant init;
    init.SetFsmBaseData(last_data);
    Screens::Access()->Open(screen_node(ScreenFactory::Screen<ScreenLoadUnload>, init));
}

uint8_t progress() {
    return last_progress;
}

std::optional<uint8_t> slot() {
    return last_origin ? std::optional<uint8_t>(last_origin->slot) : std::nullopt;
}

uint16_t revision() {
    return rev;
}

} // namespace od3d::island
