/// @file
/// OneDrop3D : association de PrintForge par un code à 6 chiffres affiché sur l'imprimante.
/// Le code ouvre une fenêtre de 5 minutes, ne sert qu'une fois, et 5 mauvais codes la ferment : un code permanent à
/// 6 chiffres se devinerait sur le réseau, alors que la clé renvoyée (le mot de passe PrusaLink) reste longue.
#pragma once

#include <cstdint>
#include <string_view>

namespace od3d::pairing {

/// Ouvre la fenêtre d'association et renvoie le code à afficher (100000 à 999999).
uint32_t start();

/// Ferme la fenêtre (dialogue refermé sur l'imprimante).
void stop();

/// Code reçu par PrusaLink : vrai une seule fois, pendant la fenêtre ; 5 mauvais codes la ferment.
bool check(std::string_view code);

} // namespace od3d::pairing
