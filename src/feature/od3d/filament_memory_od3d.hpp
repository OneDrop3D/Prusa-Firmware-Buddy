/// @file
/// OneDrop3D (F43) : emplacement chargé jusqu'à la buse retenu après un redémarrage ou un flash.
///
/// - Mémoire : config_store « OD3D Loaded Filament » (emplacement, matière, date), tenue à jour par step() d'après
///   l'état de Prusa (MMU2::get_current_tool) quand il est sûr (certain_slot : aucun changement d'outil interrompu) :
///   effacée dès qu'une commande du MMU commence (chargement, déchargement, changement d'outil, éjection, coupe) ou que
///   l'emplacement n'est plus sûr, réécrite quand l'imprimante est au repos hors impression. La matière est celle du
///   chargement : changer ensuite celle de l'emplacement chargé ne la réécrit pas.
/// - Démarrage du MMU : si FINDA et le capteur de l'extrudeur voient du filament, que le MMU ne contredit pas
///   l'emplacement et que sa matière n'a pas changé, Prusa le reprend comme après un changement d'outil réussi
///   (MMU2::RestoreLoadedTool). Sinon, comportement de Prusa et mémoire effacée.
/// - « Chargé » (étagère, accueil, fiche de bobine), PA, « Charger » : loaded_slot(), jamais l'outil 0 que Prusa rend
///   quand le MMU ne répond pas, ni un emplacement dont un changement d'outil a été interrompu.
#pragma once

#include "filament_memory_math_od3d.hpp"

#include <cstdint>
#include <optional>

namespace od3d::filament_memory {

/// Emplacement dans la buse (nozzle_slot() de filament_memory_math_od3d.hpp) : celui de Prusa, seulement s'il est sûr
/// et si FINDA et le capteur de l'extrudeur voient du filament ; aucun si le MMU activé ne répond pas ; sans MMU,
/// l'outil unique sauf si la tête voit qu'il n'y a rien. Lisible de toute tâche
std::optional<uint8_t> loaded_slot();

/// Emplacement où imprimer le T0 d'un G-code mono-outil de matière file_material (single_tool_choice), MMU actif ;
/// l'emplacement 1 sinon. « Avant l'impression » le montre, « Imprimer maintenant » y associe le T0
/// (marlin_print_preview). Lisible de toute tâche
uint8_t single_tool_slot(const char *file_material);

/// Boucle de Marlin (marlin_server, idle) : décision au démarrage du MMU, puis mémoire tenue à jour
void step();

// ---- Filament gardé dans la buse d'une impression à l'autre (retour du 24/11) ----
// Prusa le déchargeait à la fin et à l'arrêt, parce que le G-code de départ de PrusaSlicer pour la CORE One MMU3 compte
// que T<n> n'amène le filament qu'aux roues de l'extrudeur : il pousse ensuite start_load_mm (« load to the nozzle »)
// à 15 mm au-dessus du plateau, puis la ligne de purge amorce. Le premier T d'une impression reprend donc le filament
// gardé : le même est reculé de start_load_mm (il revient au bout de la buse, rien ne sort en l'air), un autre est
// d'abord déchargé avec sa pointe façonnée (ramming), comme Prusa hors impression.

/// parking_pos_retraction 84 + extra_loading_move −52 − 15 du profil PrusaSlicer de la CORE One MMU3
/// shortcut: valeur du profil de PrusaSlicer ; à lire dans le G-code si un autre trancheur ou profil la change
inline constexpr float start_load_mm = 17.f;

/// Départ d'une impression (marlin_server, PrintInit) : le prochain changement d'outil du MMU est celui du G-code de
/// départ, seulement si l'impression part du début du fichier. Reprise à une position du fichier : son G-code de départ
/// ne repasse pas, il n'y a ni « load to the nozzle » à compenser ni pointe à refaire (audit U-07)
void on_print_start(bool from_file_start);

/// MMU2::tool_change : premier changement d'outil depuis on_print_start(), rendu une seule fois
bool take_print_start_tool_change();

} // namespace od3d::filament_memory
