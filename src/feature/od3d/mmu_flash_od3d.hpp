/// @file
/// OneDrop3D : le firmware du MMU écrit par l'imprimante, le micro-USB de la carte du MMU branché sur le port USB avant
/// (l'hôte USB de la xBuddy, celui de la clé). Une petite classe CDC s'ajoute à celle de la clé dans l'hôte USB de ST :
/// l'hôte choisit la classe d'après la 1re interface (0x02 pour le MMU, 0x08 pour une clé), et la nôtre ne prend que les
/// appareils Prusa 0x2c99:0x0004 (firmware du MMU) et 0x2c99:0x0003 (son chargeur Caterina). Le firmware écrit est celui
/// que contient l'imprimante (/system/res/mmu/fw.bin, dépendance « firmware-mmu » de utils/bootstrap.py). Seulement en
/// mode `M709 S2` (mmu_usb_od3d.hpp) : l'imprimante ne parle plus au MMU par la carte d'extension.
#pragma once

#include <cstdint>

namespace od3d::mmu_flash {

enum class Device : uint8_t {
    none,
    firmware, ///< 0x2c99:0x0004, renvoyé dans son chargeur par une ligne à 1200 bauds
    bootloader, ///< 0x2c99:0x0003
};

/// Le MMU branché sur le port avant
Device device();

struct Version {
    uint8_t major = 0;
    uint8_t minor = 0;
    uint8_t patch = 0;
    uint16_t build = 0;
};

/// Version du firmware contenu dans l'imprimante (lue à 0xAC, comme src/mmu2/mmu2_bootloader.cpp) ; faux s'il manque
bool bundled(Version &version);

enum class Phase : uint8_t {
    idle,
    connecting, ///< passage dans le chargeur, présentation
    writing,
    done, ///< écrit, relu, chargeur quitté
    failed,
};

enum class Error : uint8_t {
    none,
    file, ///< firmware de l'imprimante absent ou illisible : rien n'a été écrit
    no_mmu, ///< MMU débranché
    no_bootloader, ///< le MMU n'est pas passé dans son chargeur
    no_answer, ///< `command` sans réponse
    refused, ///< `command` refusée
    wrong_chip, ///< pas un ATmega32U4, ou chargeur qui n'écrit pas par pages
    verify, ///< relecture différente à `address`
};

struct Status {
    Phase phase;
    uint8_t percent;
    Error error;
    char command;
    uint16_t address;
};

Status status();

/// Lance l'écriture sur la tâche des travaux asynchrones : refusé hors du mode M709 S2 ou pendant une écriture
bool start();

/// Mise à jour du MMU en cours : le MMU est inutilisable, du passage en mode M709 S2 à « Terminer » (M709 X2), et tant
/// que l'écriture tourne. L'assistant de calibration y reporte les tests qui ont besoin du MMU. Lisible de toute tâche.
bool busy();

/// Nouvelle séance du mode M709 S2 (mmu_usb_od3d.cpp) : oublie le résultat de la précédente
void forget_result();

/// Hôte USB (usb_host.cpp) : ajoute la classe du MMU après celle de la clé
void register_usb_class();

} // namespace od3d::mmu_flash
