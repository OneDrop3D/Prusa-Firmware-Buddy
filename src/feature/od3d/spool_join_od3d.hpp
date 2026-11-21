/// @file
/// OneDrop3D (F3) : relais de bobine permanents, réglés dans « Fonctions OneDrop3D ».
#pragma once

namespace od3d {

/// Au départ d'une impression, ajoute les relais « emplacement n vide → emplacement k » des réglages :
/// seulement si la chaîne de n part d'un emplacement que l'impression utilise (Prusa l'exige), si k n'est
/// pas utilisé par l'impression, et sans remplacer un relais déjà choisi pour cette impression.
///
/// Puis le relais automatique (retour de l'utilisateur, 21/11) : chaque emplacement de l'impression sans relais est
/// relayé par un emplacement libre de la même matière et de la même couleur, les deux couleurs étant connues (une
/// matière choisie sans couleur ne relaie rien : un PLA rouge ne doit pas devenir bleu). Les relais forment une
/// chaîne si plusieurs emplacements conviennent (1 → 3 → 4).
void apply_persistent_spool_joins();

} // namespace od3d
