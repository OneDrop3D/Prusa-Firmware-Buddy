/// @file
/// OneDrop3D (retour du 24/11, « optimise le menu imprimer ») : départ d'impression demandé par « Imprimer maintenant »
/// de notre écran « Avant l'impression ».
///
/// Un filament détecté dans l'extrudeur que le MMU ne sait pas placer fait poser à Prusa « Décharger le filament
/// maintenant ? » : depuis « Imprimer maintenant », c'est la réponse « Oui », sans question. Seulement ce départ-là :
/// la reprise après une coupure de courant, l'impression lancée par Connect ou AUTO.GCO passent aussi l'aperçu
/// (PreviewSkipIfAble::all, même valeur que tool_mapping), et une reprise ne doit jamais décharger en pleine pièce.
#pragma once

#include <atomic>

namespace od3d::print_start {

namespace detail {
    inline std::atomic<bool> auto_unload = false;
}

/// Interface, juste avant marlin_client::print_start() de « Imprimer maintenant »
inline void request_auto_unload() {
    detail::auto_unload = true;
}

/// Fil de Marlin, au début de l'aperçu de chaque départ : la demande (s'il y en a une) pour ce départ, effacée
inline bool take_auto_unload() {
    return detail::auto_unload.exchange(false);
}

} // namespace od3d::print_start
