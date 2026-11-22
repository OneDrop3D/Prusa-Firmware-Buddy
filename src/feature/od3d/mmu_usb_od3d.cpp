/// @file
#include "mmu_usb_od3d.hpp"

#include <atomic>

#include <config_store/store_instance.hpp>
#include <feature/prusa/MMU2/mmu2_mk4.h>
#include <marlin_server.hpp>

namespace od3d::mmu_usb {

namespace {
    std::atomic<bool> on = false; ///< remis à faux au démarrage : un redémarrage de l'imprimante y met fin
}

bool active() {
    return on;
}

bool start() {
    if (marlin_server::is_printing() || !config_store().mmu2_enabled.get()) {
        return false;
    }
    // Plus de requête vers l'extension, donc plus rien sur l'UART du MMU ; l'alimentation et nreset restent tels quels
    MMU2::mmu2.StopForUsbUpdate();
    on = true;
    return true;
}

void end() {
    on = false;
}

} // namespace od3d::mmu_usb
