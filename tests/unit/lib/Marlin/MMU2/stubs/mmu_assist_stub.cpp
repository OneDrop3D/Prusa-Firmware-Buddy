// OneDrop3D (F11) : sans assistance, le protocole de Prusa est testé tel quel
#include "protocol_logic.h"

namespace od3d::mmu_assist {

Exchange next_exchange(uint16_t &) {
    return Exchange::none;
}

void read_back(uint16_t) {}

void restart() {}

} // namespace od3d::mmu_assist
