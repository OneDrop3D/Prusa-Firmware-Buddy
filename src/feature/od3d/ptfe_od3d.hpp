/// @file
/// OneDrop3D (F41) : mesure automatique du tube PTFE avant (Réglages › Matériel › MMU, registre 0x22 du MMU).
///
/// `M1999 B1 P<emplacement 0-4>` : la longueur du MMU est mise au minimum (341 mm), puis le test de chargement de
/// Prusa (M1704 : jusqu'aux engrenages de l'extrudeur, buse froide permise, puis déchargement vers le MMU) pousse la
/// bobine préchargée de cet emplacement. Pendant ce temps, step() suit l'avance lente du MMU et la position de la poulie
/// (registre 0x1a, lue à chaque interrogation) jusqu'à ce que le capteur de l'extrudeur voie le filament ;
/// ptfe_math_od3d.hpp (Tracker) en tire la distance et la longueur, écrite dans le registre 0x22. En cas d'échec,
/// l'ancienne longueur est remise. Pendant la mesure, le filament ralentit donc toujours à 341 mm : c'est voulu.
/// Pas d'étalonnage de Prusa à réutiliser : le firmware du MMU parle d'« autotuning » dans un commentaire, sans code.
#pragma once

#include <cstdint>

namespace od3d::ptfe {

inline constexpr uint8_t slots = 5; ///< MMU3 : emplacements 0 à 4

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
    too_long, ///< distance au-delà de 1000 mm, le maximum du MMU
    interrupted, ///< chargement arrêté avant l'avance lente (erreur du MMU, arrêt) : rien mesuré
    not_applied, ///< mesure faite, mais la longueur relue dans le MMU n'est pas celle écrite (mmu_mm)
};

struct Status {
    Result result;
    uint16_t measured_mm; ///< distance FINDA → capteur de l'extrudeur mesurée (0 sans mesure)
    uint16_t saved_mm; ///< longueur écrite (0 sans)
    uint16_t mmu_mm; ///< longueur relue dans le MMU après l'écriture (0 sans)
};

/// Dernière écriture de la longueur (menu « Tube PTFE avant » par M1999 L, mesure) : demandée et relue dans le MMU
struct Write {
    uint16_t asked; ///< 0 : aucune depuis le démarrage
    uint16_t mmu; ///< relue juste après ; différente de asked : le MMU ne l'a pas prise
};

/// Pour l'interface et l'assistant de calibration (tout fil)
Status status();

/// Résultat remis à « pas encore mesuré » : l'assistant le fait avant son étape, pour ne pas lire le résultat d'avant
/// si M1999 ne part pas
void forget();

/// Fil de Marlin (M1999 B1) : bloquant, toute la mesure ; slot de 0 à 4
Result measure(uint8_t slot);

/// Fil de Marlin (M1999 L) : écrit la longueur dans le MMU (registre 0x22) puis la relit, la seule preuve qu'il l'a
/// prise (par le pont MODBUS, une écriture est « acceptée » dès que le bus répond). Hors de 341 à 1000 mm, rien n'est
/// écrit : le MMU remplacerait la valeur par 360 sans le dire. Rend la valeur relue (0 si illisible).
uint16_t write_length(uint16_t mm);

/// Pour l'interface (tout fil)
Write last_write();

/// Fil de Marlin, à chaque tour de idle() (marlin_server cycle) : relevé pendant la mesure
void step();

} // namespace od3d::ptfe
