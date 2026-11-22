/// @file
/// OneDrop3D (F43) : emplacement chargé jusqu'à la buse retenu après un redémarrage ou un flash.
///
/// - Mémoire : config_store « OD3D Loaded Filament » (emplacement, matière, date), tenue à jour par step() d'après
///   l'état de Prusa (MMU2::get_current_tool) : effacée dès qu'une commande du MMU commence (chargement, déchargement,
///   changement d'outil, éjection, coupe), réécrite quand l'imprimante est au repos hors impression.
/// - Démarrage du MMU : si FINDA et le capteur de l'extrudeur voient du filament, que le MMU ne contredit pas
///   l'emplacement et que sa matière n'a pas changé, Prusa le reprend comme après un changement d'outil réussi
///   (MMU2::RestoreLoadedTool). Sinon, comportement de Prusa et mémoire effacée.
/// - Tous ceux qui lisent VirtualToolIndex::currently_selected() le voient (fiche de bobine, atelier, T<n>, départ
///   d'impression mono-matière, PA) ; loaded_slot() y ajoute l'accord des capteurs.
#pragma once

#include <cstdint>
#include <optional>

namespace od3d::filament_memory {

/// Emplacement dans la buse : celui de Prusa, seulement si FINDA et le capteur de l'extrudeur voient du filament
/// (sans MMU actif : l'outil de Prusa, comme VirtualToolIndex::currently_selected)
std::optional<uint8_t> loaded_slot();

/// Boucle de Marlin (marlin_server, idle) : décision au démarrage du MMU, puis mémoire tenue à jour
void step();

} // namespace od3d::filament_memory
