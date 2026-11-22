/// @file
/// OneDrop3D (N4) : vitesse de la poulie du MMU mesurée (Atelier › Réglages du MMU › Vitesse de la poulie, assistant de
/// calibration).
///
/// `M2002 P<emplacement 0-4>` : le test de chargement de Prusa (M1704 : jusqu'aux engrenages de l'extrudeur, buse froide
/// permise, puis déchargement vers le MMU) est fait avec la bobine préchargée de cet emplacement, à chaque palier de
/// mmu_speed_math_od3d.hpp (95 à 190 mm/s, chargement et déchargement). Un palier avec une erreur, un réessai, un
/// patinage ou un cycle plus lent arrête la montée ; 80 % du dernier palier passé est gardé (au moins 95), vérifié par
/// un dernier cycle, puis renvoyé au MMU à chaque démarrage. La durée d'un cycle est mesurée à 95 mm/s et à la vitesse
/// gardée : c'est le gain d'un changement d'outil. Une erreur au palier de Prusa ou à la vérification : rien ne change
/// (vitesse de Prusa). `M2002 R` : vitesse de Prusa.
#pragma once

#include "mmu_speed_math_od3d.hpp"

#include <array>
#include <cstdint>

namespace od3d::mmu_speed {

inline constexpr uint8_t slots = 5; ///< MMU3 : emplacements 0 à 4
inline constexpr uint8_t cycles = levels.size() + 1; ///< paliers, puis la vérification

enum class Result : uint8_t {
    none, ///< pas encore mesuré depuis le démarrage
    running,
    saved, ///< vitesse choisie (peut être celle de Prusa : aucun palier au-dessus n'est passé)
    stopped, ///< arrêt demandé : vitesse d'avant remise
    refused_printing,
    refused_mmu, ///< MMU absent, désactivé ou pas prêt, emplacement invalide
    refused_sensor, ///< capteur de l'extrudeur absent, désactivé, non calibré ou en panne
    refused_extruder, ///< filament déjà dans l'extrudeur
    refused_selector, ///< filament dans le sélecteur (FINDA)
    failed, ///< erreur à la vitesse de Prusa : rien ne change
    failed_check, ///< erreur à la vérification de la vitesse choisie : vitesse de Prusa
};

/// Lu par l'interface pendant la mesure (comme damping::last() : revision() dit quand il change)
struct Measurement {
    Result result = Result::none;
    uint8_t slot = 0;
    uint8_t step = 0; ///< cycle en cours ou dernier fait : 0 à levels.size() - 1 paliers, levels.size() vérification
    uint8_t speed = 0; ///< vitesse gardée (résultat saved)
    std::array<uint16_t, cycles> ds {}; ///< durée de chaque cycle [0,1 s], 0 pas fait
    std::array<bool, cycles> passed {}; ///< cycle fait et passé
};

const Measurement &last();
uint16_t revision();
bool is_running();

/// Résultat remis à « pas encore mesuré » : l'assistant le fait avant son étape
void forget();

/// Demande l'arrêt (interface) : pris en compte entre deux cycles (~30 s au plus)
void request_stop();

/// Ce qui est enregistré (vitesse choisie et durées mesurées)
Saved saved();

/// Au démarrage, avant celui du MMU : la vitesse enregistrée part avec les registres d'initialisation
void plan_saved();

/// Fil de Marlin (M2002 P) : bloquant, toute la mesure
Result measure(uint8_t slot);

/// Fil de Marlin (M2002 R) : vitesse de Prusa, mesure oubliée
void reset();

/// Fil de Marlin, à chaque tour de idle() (marlin_server cycle) : temps passé dans l'avance lente pendant un cycle
void step();

} // namespace od3d::mmu_speed
