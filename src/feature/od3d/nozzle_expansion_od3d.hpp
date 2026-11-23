/// @file
/// OneDrop3D (M2003) : dilatation réelle de la buse mesurée par la cellule de charge, gardée par buse réglée dans Prusa
/// (« OD3D Nozzle Expansion », lue par feature/nozzle_thermal_compensation à la place du modèle de Prusa).
///
/// Buse vide (refusé si le capteur de l'extrudeur voit du filament), plateau à 60 °C (stabilisé pendant l'origine et
/// le 1er palier), origine, puis à chaque palier de temperatures (170, 215, 260, 215, 170) : nettoyage de la buse à
/// chaque température jamais atteinte (new_high()), buse à la consigne et stable, 3 palpages du centre du plateau,
/// sans correction. nozzle_expansion_math_od3d.hpp en tire la pente ; acceptée, elle
/// remplace celle de Prusa pour cette buse, et le maillage gardé (F44), palpé avec l'ancienne, est oublié.
///
/// Les paliers restent en mémoire jusqu'au redémarrage ; l'interface les lit pendant la mesure (points et droite).
/// Réutilisable (N5, palpage pendant la chauffe) : pente par slope_mm_per_c() de la compensation, paliers par last().
#pragma once

#include "nozzle_expansion_math_od3d.hpp"

#include <cstdint>

namespace od3d::nozzle_expansion {

inline constexpr int16_t bed_temperature = 60; ///< une seule consigne du début à la fin
inline constexpr uint32_t bed_soak_ms = 120'000; ///< plateau à sa consigne depuis 2 min avant le 1er palpage
/// Buse à ±1,5 °C de chaque palier depuis 30 s avant de palper (90 s jusqu'à la 8.0.2.33, retour du 23/11 : trop
/// long). La buse (21 mm de laiton dans le bloc de chauffe, celle que corrige Prusa) suit la sonde du bloc en
/// quelques secondes ; ce qui chauffe lentement (radiateur, cadre) n'était pas stable en 90 s non plus : l'aller-retour
/// annule sa dérive régulière, et l'écart à la droite (10 µm au plus) refuse une dérive qui ne l'est pas.
inline constexpr uint32_t settle_ms = 30'000;
inline constexpr float hover_z = 5.f; ///< buse au-dessus du point pendant les chauffes [mm]

enum class Status : uint8_t {
    idle,
    running,
    saved, ///< pente acceptée et gardée pour cette buse
    stopped,
    printing, ///< refusé : impression ou imprimante occupée
    filament, ///< refusé : filament dans l'extrudeur
    heating, ///< buse ou plateau pas à la consigne à temps
    homing,
    probing, ///< un palpage n'a pas abouti
    spread, ///< refusée : palpages trop dispersés
    residual, ///< refusée : paliers loin de la droite
    slope, ///< refusée : pente impossible
};

enum class Phase : uint8_t {
    bed, ///< plateau, origine, nettoyage
    heating, ///< buse vers le palier, puis stabilisation
    probing,
};

struct Measurement {
    std::array<Stage, stages> stage {};
    uint8_t done = 0; ///< paliers mesurés
    Outcome outcome {}; ///< après le dernier palier
    uint8_t key = 0; ///< buse mesurée (nozzle_thermal_compensation::nozzle_key)
    uint16_t before_nm = 0; ///< pente gardée avant pour cette buse [nm/°C], 0 : modèle de Prusa
};

/// Corps de M2003 : bloque jusqu'à la fin de la mesure ou request_stop()
void run();

/// M2003 R : la buse réglée revient au modèle de Prusa
void forget();

/// Le capteur de l'extrudeur voit du filament
bool filament_loaded();

void request_stop();
bool is_running();
Status status();
Phase phase();
uint8_t current_stage();
uint8_t progress_percent();
uint16_t run_count();
uint16_t revision(); ///< change à chaque palier, palpage ou fin
const Measurement &last();

} // namespace od3d::nozzle_expansion
