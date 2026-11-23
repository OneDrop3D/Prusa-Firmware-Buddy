/// @file
/// OneDrop3D (F39) : mode veille, ce qui est partagé entre l'interface et le fil de Marlin.
///
/// Il remplace ce que faisait Prusa :
/// - l'atténuation des lumières de la chambre après 2 min (leds::SideStripHandler, réglages « Atténuation » masqués) :
///   les lumières restent allumées, et s'éteignent en veille ;
/// - la coupure des moteurs après 10 min d'immobilité (buddy::StepperTimeoutManager) : quand le mode veille est actif,
///   seule la veille coupe les moteurs (réglage « Moteurs ») ; mode coupé, la coupure de Prusa revient.
/// Ce qui reste à Prusa : la coupure de sécurité des chauffes après 30 min (buddy::SafetyTimer, M86) et toutes les
/// protections thermiques ; le retour à l'accueil des menus après 30 s (Interface › Délai des menus).
///
/// L'interface (gui/od3d/sleep_gui_od3d.*) décide de l'endormissement et publie ici des drapeaux ; le fil de Marlin
/// fait les coupures (step()), jamais pendant une impression, une pause ou un G-code en cours.
#pragma once

#include "sleep_math_od3d.hpp"

namespace od3d::sleep {

/// Réglages (fil de l'interface) : lus une fois dans config_store, gardés en mémoire
Settings settings();
void set_settings(const Settings &s);

// ---- Publié par l'interface ----

/// @param takeover mode veille actif : la coupure des moteurs de Prusa n'agit plus
/// @param lights_off lumières éteintes ; @param fans_off ventilateurs de la chambre et filtration coupés
void publish(bool takeover, bool lights_off, bool fans_off);

/// Coupures à faire par le fil de Marlin (abandonnées s'il est occupé)
void request(Due due);

/// Un écran attend l'utilisateur avec l'accéléromètre à fixer ou fixé (retour du 23/11) : aucune coupure des moteurs
/// par inactivité, ni de Prusa ni de la veille, sinon l'origine se perdrait et serait refaite accéléromètre fixé.
/// Compté : chaque true est suivi d'un false (quitter l'écran, fin de l'étape) ; à zéro, comportement normal.
void hold_motors(bool hold);

// ---- Lu par les lumières (interface) et les ventilateurs (Marlin) ----

bool lights_off();

/// Fil de Marlin : faux pendant une impression ou une pause, même avant que l'interface se soit réveillée
bool fans_off();

// ---- Fil de Marlin ----

/// À chaque tour de boucle (cycle() de marlin_server), avant la coupure des moteurs de Prusa
void step();

} // namespace od3d::sleep
