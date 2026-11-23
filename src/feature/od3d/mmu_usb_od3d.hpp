/// @file
/// OneDrop3D : mise à jour du firmware du MMU par son micro-USB, sur la CORE One. L'imprimante ne peut pas écrire le
/// firmware du MMU elle-même (son UART est relié à la carte d'extension xBuddy, MMU_USE_BOOTLOADER() = 0) : c'est
/// PrintForge ou PrusaSlicer qui l'écrit par l'USB. Pendant ce temps, l'imprimante interrogeait le MMU (S0) et pouvait le
/// redémarrer. `M709 S2` l'alimente (ou le garde alimenté) et coupe toute communication (MMU2::StopKeepPowered) ;
/// toute autre commande M709 (`M709 X2` : coupure de l'alimentation puis reprise, « Terminer ») ou un redémarrage de
/// l'imprimante y met fin.
#pragma once

namespace od3d::mmu_usb {

/// Le mode est actif : MMU alimenté, aucune communication
bool active();

/// Fil de Marlin (M709 S2) : refusé pendant une impression ou MMU désactivé dans les réglages
bool start();

/// Fil de Marlin : toute autre commande M709 (le MMU redémarre, s'éteint ou reprend)
void end();

} // namespace od3d::mmu_usb
