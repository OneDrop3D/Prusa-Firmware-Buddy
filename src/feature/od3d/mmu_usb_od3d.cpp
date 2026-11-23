/// @file
#include "mmu_usb_od3d.hpp"
#include "mmu_flash_od3d.hpp"

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
    // Plus de requête vers l'extension, donc plus rien sur l'UART du MMU ; nreset reste tel quel
    MMU2::mmu2.StopForUsbUpdate();
    // Alimenté même si l'imprimante l'avait coupé (test des capteurs de filament, MMU2::Stop) : sans courant, aucun USB.
    // Sur la CORE One, simple consigne à la carte d'extension, sans effet si le MMU l'est déjà
    MMU2::mmu2.PowerOn();
    mmu_flash::forget_result();
    on = true;
    return true;
}

void end() {
    on = false;
}

} // namespace od3d::mmu_usb
