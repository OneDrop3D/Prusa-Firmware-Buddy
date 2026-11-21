/// @file
/// OneDrop3D (F13) : suivi du poids des bobines des 5 emplacements du MMU.
///
/// L'extrusion par outil vient de FilamentTracker (le planificateur y déclare chaque mouvement de E, par outil
/// virtuel = emplacement du MMU), comme pour le suivi OpenPrintTag de Prusa (filament_usage_tracker) : on garde la
/// longueur déjà comptée par emplacement, et l'écart × la masse d'un millimètre de la matière est retiré du reste.
/// La mémoire (config_store, « OD3D Spool Usage ») n'est écrite qu'avec parcimonie : fin d'impression, changement
/// d'outil, extrusion arrêtée hors impression, ou tous les 25 g (spool_usage::should_commit).
///
/// Une bobine avec une étiquette OpenPrintTag suivie par Prusa garde ses données : report() les donne en priorité.
#pragma once

#include "spool_usage_math_od3d.hpp"

#include <optional>

namespace od3d::spool_usage {

/// Ce que montrent les écrans pour un emplacement
struct Report {
    float remaining_g; ///< reste, extrusion pas encore écrite comprise
    float net_g; ///< poids net neuf
    float alert_g; ///< seuil d'alerte (0 pour une étiquette : pas de seuil)
    float used_g; ///< consommé depuis la pose (NAN pour une étiquette)
    float last_print_g; ///< dernière impression (NAN pour une étiquette)
    float g_per_mm; ///< masse d'un millimètre
    bool from_tag; ///< données de l'étiquette OpenPrintTag (non modifiables ici)

    uint8_t fill_percent() const { return spool_usage::fill_percent(remaining_g, net_g); }
    bool low() const { return !from_tag && is_low(remaining_g, alert_g); }
};

/// Données de l'emplacement (0 à 4) : l'étiquette OpenPrintTag si Prusa la suit, sinon notre suivi s'il a un
/// poids net ; nullopt sinon
std::optional<Report> report(uint8_t slot);

/// Ce qui est enregistré pour l'emplacement (pour l'écran d'utilisation)
Record record(uint8_t slot);

/// Thread de Marlin, à chaque tour de boucle (limité à une fois par seconde)
void step();

/// Thread de Marlin, au départ d'une impression : point de départ de « dernière impression »
void on_print_start();

/// Thread de Marlin, à la fin d'une impression (finie ou arrêtée) : tout est écrit ; appelable plusieurs fois
void on_print_end();

// Modifications par l'interface (thread de l'interface) ; chacune écrit en mémoire tout de suite

/// Bobine neuve de net_g grammes (« Changer la bobine », étape 3)
void new_spool(uint8_t slot, uint16_t net_g);

/// Poids net seul (« Matière et couleur », utilisation) ; démarre le suivi d'un emplacement qui n'en avait pas
void set_net(uint8_t slot, uint16_t net_g);

/// Reste corrigé après une pesée
void set_remaining(uint8_t slot, float remaining_g);

/// Seuil d'alerte de fin
void set_alert(uint8_t slot, uint16_t alert_g);

/// Interface : un emplacement vient de passer sous son seuil (une fois par passage) ; nullopt sinon
std::optional<uint8_t> take_low_alert();

} // namespace od3d::spool_usage
