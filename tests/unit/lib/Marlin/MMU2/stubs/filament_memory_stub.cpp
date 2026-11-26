// OneDrop3D (F43) : jamais de départ d'impression dans ces tests, le changement d'outil de Prusa est testé tel quel
#include "../../../../../../src/feature/od3d/filament_memory_od3d.hpp"

namespace od3d::filament_memory {

void on_print_start(bool) {}

bool take_print_start_tool_change() {
    return false;
}

void start_short_purge() {}

} // namespace od3d::filament_memory
