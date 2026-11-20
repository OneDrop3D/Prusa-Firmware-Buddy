/// @file
/// OneDrop3D (F3) : relais de bobine permanents, réglés dans « Fonctions OneDrop3D ».
#pragma once

namespace od3d {

/// Au départ d'une impression, ajoute les relais « emplacement n vide → emplacement k » des réglages :
/// seulement si la chaîne de n part d'un emplacement que l'impression utilise (Prusa l'exige), si k n'est
/// pas utilisé par l'impression, et sans remplacer un relais déjà choisi pour cette impression.
void apply_persistent_spool_joins();

} // namespace od3d
