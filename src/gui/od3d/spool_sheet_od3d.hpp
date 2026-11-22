/// @file
/// OneDrop3D (F1, interface G) : fiche d'une bobine (SPEC_G.md §3.3), ouverte en touchant ou en cliquant une
/// bobine de l'accueil ou de l'étagère. Elle couvre le bas de l'écran ; au-dessus, l'écran d'origine givré
/// (frost_od3d.hpp). « Fermer », le toucher du fond givré, le balayage de côté, l'appui long, ou la fiche tirée vers
/// le bas par sa poignée au-delà du tiers de sa hauteur la ferment.
#pragma once

#include <cstdint>

namespace od3d {

/// Ouvre la fiche de l'emplacement index (0 à 4), attend qu'elle se ferme, puis lance l'action choisie
void open_sheet(uint8_t index);

} // namespace od3d
