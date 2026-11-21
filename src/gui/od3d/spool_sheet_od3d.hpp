/// @file
/// OneDrop3D (F1, interface G) : fiche d'une bobine (SPEC_G.md §3.3), ouverte en touchant ou en cliquant une
/// bobine de l'accueil ou de l'étagère. Elle couvre le bas de l'écran ; l'écran d'origine reste dessiné au-dessus
/// (sans voile : pas d'alpha sur l'écran). « Fermer », le toucher au-dessus de la fiche ou l'appui long la ferment.
#pragma once

#include <cstdint>

namespace od3d {

/// Ouvre la fiche de l'emplacement index (0 à 4), attend qu'elle se ferme, puis lance l'action choisie
void open_sheet(uint8_t index);

} // namespace od3d
