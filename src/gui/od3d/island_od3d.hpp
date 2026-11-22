/// @file
/// OneDrop3D (F1, interface G) : la pastille « Dynamic Island » de l'en-tête, comme celle d'Apple (retour de
/// l'utilisateur, 21/11). Elle montre, par priorité (island_face_od3d.hpp) :
/// - l'opération du filament mise en arrière-plan : « Continuer en arrière-plan » (ou la flèche) de l'écran de
///   progression la réduit, et pendant une impression les changements d'outil du MMU s'y ouvrent d'emblée quand on est
///   ailleurs que sur l'écran Impression ; la machine à états de Prusa continue. Toucher la pastille rouvre l'écran ; il
///   revient aussi tout seul dès que Prusa attend l'utilisateur (question, couleur, erreur du MMU) ;
/// - l'impression en cours quand on a quitté l'écran Impression (UI-10) : la toucher y ramène ;
/// - la buse qui chauffe hors impression (préchauffage) : la toucher ouvre l'aperçu des températures.
/// Pas de fichier d'interface ici : l'en-tête (widgets_od3d) n'inclut que ce fichier léger.
#pragma once

#include "island_face_od3d.hpp"

#include <common/fsm_base_types.hpp>
#include <cstdint>
#include <optional>

namespace od3d::island {

/// L'opération demandée par l'atelier (atelier::Request en octets : op, emplacement)
struct Origin {
    uint8_t op;
    uint8_t slot;
};

/// L'écran de progression est réduit dans l'en-tête
bool minimized();

/// Réduit l'opération (l'écran de progression se ferme ensuite)
void minimize(fsm::BaseData data, std::optional<Origin> origin);

/// Une machine Load_unload s'ouvre : vrai si elle part directement en arrière-plan (open_in_background), sans écran
bool open_minimized(fsm::BaseData data);

/// Nouvelles données de la machine pendant la réduction (DialogHandler)
void update(fsm::BaseData data);

/// La machine s'est arrêtée : plus de pastille
void end();

/// Avant que l'écran de progression ne se rouvre : plus de pastille, et l'atelier retrouve le nom de l'opération
void prepare_reopen();

/// Toucher la pastille : rouvre l'écran de progression avec les dernières données
void restore();

/// Ce que la pastille montre maintenant (lit l'imprimante et l'écran ouvert ; à appeler depuis l'interface)
Face face();

/// Toucher la pastille (ou la choisir à la molette et cliquer) : rouvre le chargement, l'écran Impression ou
/// l'aperçu des températures
void activate();

} // namespace od3d::island
