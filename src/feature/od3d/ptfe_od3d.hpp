/// @file
/// OneDrop3D (F41) : mesure automatique du tube PTFE avant (Réglages › Matériel › MMU, registre 0x22 du MMU).
///
/// `M1999 B1 P<emplacement 0-4>` : la longueur du MMU est mise au minimum (341 mm), puis le test de chargement de
/// Prusa (M1704 : jusqu'aux engrenages de l'extrudeur, buse froide permise, puis déchargement vers le MMU) pousse la
/// bobine préchargée de cet emplacement. Pendant ce temps, step() relève la position de la poulie (registre 0x1a, lue à
/// chaque interrogation du MMU) jusqu'à ce que le capteur de l'extrudeur voie le filament ; ptfe_math_od3d.hpp en tire
/// la longueur, écrite dans le registre 0x22. En cas d'échec, l'ancienne longueur est remise.
/// Pas d'étalonnage de Prusa à réutiliser : le firmware du MMU parle d'« autotuning » dans un commentaire, sans code.
#pragma once

#include <cstdint>

namespace od3d::ptfe {

enum class Result : uint8_t {
    none, ///< pas encore mesuré depuis le démarrage
    running,
    saved, ///< longueur écrite dans le MMU
    refused_printing,
    refused_mmu, ///< MMU absent, désactivé ou pas prêt, emplacement invalide
    refused_sensor, ///< capteur de l'extrudeur absent, désactivé, non calibré ou en panne
    refused_extruder, ///< filament déjà dans l'extrudeur
    refused_selector, ///< filament dans le sélecteur (FINDA)
    no_trigger, ///< le capteur de l'extrudeur n'a jamais vu le filament
    too_short, ///< capteur vu pendant l'avance rapide minimale : tube plus court que 341 mm
};

struct Status {
    Result result;
    uint16_t measured_mm; ///< position de la poulie relevée avant le capteur (0 sans mesure)
    uint16_t saved_mm; ///< longueur écrite (0 sans)
};

/// Pour l'interface et l'assistant de calibration (tout fil)
Status status();

/// Fil de Marlin (M1999 B1) : bloquant, toute la mesure ; slot de 0 à 4
Result measure(uint8_t slot);

/// Fil de Marlin, à chaque tour de idle() (marlin_server cycle) : relevé pendant la mesure
void step();

} // namespace od3d::ptfe
