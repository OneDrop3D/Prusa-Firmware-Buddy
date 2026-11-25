/// @file
/// OneDrop3D (F29) : climat de la chambre et descente douce du plateau, au-dessus de ce que Prusa fait déjà.
///
/// Déjà chez Prusa (non refait ici) : cible, minimum et maximum de chambre par filament (préréglages et filaments de
/// l'utilisateur, M865 C D E), filtration par filament (requires_filtration), régulation de la chambre par les
/// ventilateurs de la xBuddy Extension en automatique (cooling.cpp, refroidissement seulement : la CORE One chauffe sa
/// chambre par le plateau), volets ouverts ou fermés selon la cible (> 45 °C : fermés), filtration après impression,
/// protections de la chambre (surchauffe : ventilateurs à fond ; 85 °C : erreur).
///
/// Ajouté : une recette par matière (chamber_climate_math_od3d.hpp) avec deux options, coupées par défaut :
/// - « plafond » : pendant l'impression, sans cible du trancheur, la cible de la chambre devient le maximum de la
///   matière (PLA 38 °C, PETG 45 °C chez Prusa) ;
/// - « descente douce » : après une impression réussie, le plateau descend par paliers. La chambre reste sans cible
///   (Prusa la remet à zéro en fin d'impression) : ses ventilateurs ne tournent pas, sauf la filtration après
///   impression de Prusa (réglage de l'utilisateur, les fumées passent avant).
/// Une nouvelle impression arrête la descente tout de suite : elle n'est jamais retardée.
#pragma once

#include "chamber_climate_math_od3d.hpp"

#include <cstdint>

namespace od3d::chamber_climate {

/// Matières des recettes : les préréglages de Prusa (bits 0 à 15 du mot des préréglages, indice = PresetFilamentType)
/// et les 8 filaments de l'utilisateur (mot à part)
Recipe preset_recipe(uint8_t preset);
void set_preset_recipe(uint8_t preset, Recipe r);
Recipe user_recipe(uint8_t index);
void set_user_recipe(uint8_t index, Recipe r);

CooldownParams params();
void set_params(const CooldownParams &p);

// ---- Thread de Marlin ----

/// À chaque tour de boucle de Marlin (limité à une fois par seconde)
void step();

/// Départ d'une impression : arrête une descente en cours (l'impression suivante n'attend jamais)
void on_print_start();

/// Fin d'une impression réussie : lance la descente si une matière de l'impression la demande
void on_print_finished();

/// M1998 S1 : descente depuis la température actuelle du plateau (hors impression) ; false si rien à faire
bool start_now();

// ---- Tous threads ----

/// Arrêt demandé (interface, M1998 S0) : le plateau est coupé au prochain tour de Marlin
void request_stop();

/// Cible de la chambre choisie à l'écran (Températures, Ajuster), posée au prochain tour de Marlin, même pendant une
/// attente (absorption de chaleur, M109, M190, M191) : un M141 mis en file attendait sa fin (retour du 25/11). 0 = sans cible.
void request_target(float celsius);

struct Status {
    bool active = false;
    int16_t target = 0; ///< consigne du palier en cours
    uint16_t next_step_s = 0; ///< avant le palier suivant
    uint16_t steps_left = 0;
    uint8_t floor_c = 0;
};
Status status();

} // namespace od3d::chamber_climate
