/// @file
/// OneDrop3D (F1, interface G) : la pastille « Dynamic Island » de l'en-tête, comme celle d'Apple (retour de
/// l'utilisateur, 21/11). Elle montre, par priorité (island_face_od3d.hpp) :
/// - un de nos processus (mesure du PA, débit, courroies, MPC, musique...) hors de son écran : son nom et son étape ;
///   la toucher rouvre son écran. Les machines Wait (origine, palpage) et Load_unload qu'il ouvre restent cachées ;
/// - l'impression en cours quand on a quitté l'écran Impression (UI-10) : la toucher y ramène ;
/// - l'opération du filament mise en arrière-plan : « Continuer en arrière-plan » (ou la flèche) de l'écran de
///   progression la réduit, et pendant une impression les changements d'outil du MMU s'y ouvrent d'emblée quand on est
///   ailleurs que sur l'écran Impression ; la machine à états de Prusa continue. Toucher la pastille rouvre l'écran ; il
///   revient aussi tout seul dès que Prusa attend l'utilisateur (question, couleur, erreur du MMU) ;
/// - le « Patientez » de Prusa (origine G28, palpage G29) réduit par « Continuer en arrière-plan » : le toucher le rouvre ;
/// - la buse qui chauffe hors impression (préchauffage) : la toucher ouvre l'aperçu des températures.
/// Pas de fichier d'interface ici : l'en-tête (widgets_od3d) n'inclut que ce fichier léger.
#pragma once

#include "island_face_od3d.hpp"

#include <common/fsm_base_types.hpp>
#include <cstddef>
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

/// Un de nos processus est en cours : les machines Wait et Load_unload qu'il ouvre restent dans la pastille
bool in_process();

// « Patientez » de Prusa (machine Wait), réductible (dialog_wait_od3d.hpp, DialogHandler)

/// La machine Wait s'ouvre : vrai si elle reste réduite (pendant un de nos processus, ou à la suite d'une attente
/// réduite, G29 enchaînant ses commandes)
bool wait_opens_minimized();

/// Le dialogue « Patientez » est réduit : DialogHandler le ferme, ou ne le rouvre pas
bool wait_minimized();

/// « Continuer en arrière-plan » ou la flèche du dialogue (DialogHandler le ferme au tour suivant, hors de ses événements)
void minimize_wait();

/// La machine Wait s'est arrêtée
void wait_end();

/// Texte de la pastille (traduit)
void text(const Face &face, char *buf, size_t size);

/// Ce que la pastille montre maintenant (lit l'imprimante et l'écran ouvert ; à appeler depuis l'interface)
Face face();

/// Toucher la pastille (ou la choisir à la molette et cliquer) : rouvre l'écran du processus, l'écran Impression, le
/// chargement, le « Patientez » ou l'aperçu des températures
void activate();

} // namespace od3d::island
