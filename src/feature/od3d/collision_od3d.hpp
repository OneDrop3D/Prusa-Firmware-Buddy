/// @file
/// OneDrop3D (F38, document Vecteurs v3 n° 20) : sécurité collision / warping pendant l'impression.
///
/// Détection (collision_math_od3d.hpp) : impulsions de force de la cellule de charge, répétées au même endroit.
/// L'accéléromètre de la buse, s'il est branché et répond, confirme un choc (il compte double) ; il n'est jamais requis.
/// Il est vérifié une seule fois, au départ de l'impression (od3d::accel::ready(), ACC-1 : moins de 0,3 s, tête à
/// l'arrêt, planificateur vide) ; ensuite l'interruption ne fait que lire la file du poller de Prusa, toujours actif,
/// et jamais quand une mesure l'a emprunté. Pas pendant l'essai à la buse froide.
///
/// Branchement :
/// - interruption de la cellule (Loadcell::ProcessSample, au même endroit que force_probe) : quelques calculs en
///   mémoire fixe, rien quand la fonction est désarmée ; charge non tarée (une tare ne fait pas de marche) ;
/// - fil de Marlin (cycle() de marlin_server) : armement, comptage, pause, CSV.
///
/// Armé seulement pendant une impression (état Printing de marlin_server, donc ni pause ni fin), réglage actif, et
/// jamais pendant :
/// - un palpage, une prise d'origine, une calibration Z ou un palpage XY (cellule en haute précision, butée Z ou
///   XY de la cellule actives) : vu dans l'interruption, échantillon par échantillon ;
/// - ce que Prusa protège déjà de sa détection de filament coincé (BlockEStallDetection) : G28, G29, changement
///   d'outil et chargement du MMU, M600, M701/M702, relais de bobine, rétraction automatique, nos mesures F4/F5/F6 ;
/// - la purge et la première couche (Z < 1 mm), les rétractions, désrétractions et essuyages (extrudeur rapide).
/// Les protections thermiques et de mouvement de Prusa ne sont pas touchées : on ne fait que lire la cellule.
///
/// Action au moment de l'alerte : « Mettre en pause » (par défaut) = la pause normale de Prusa (print_pause), puis
/// le message (nombre de chocs, position) sur nos écrans ; « Prévenir seulement » = le message. Pas de Z-hop de
/// sauvetage : un mouvement glissé dans le planificateur au milieu de l'impression n'est pas sûr, et la pause de
/// Prusa lève déjà la buse.
///
/// Journal : les impulsions (dès la moitié du seuil) vont dans /usb/OD3D_COLL.CSV, ajoutées quand l'imprimante est
/// à l'arrêt (pause, fin d'impression, essai) pour ne jamais bloquer le fil de Marlin pendant un mouvement ; on garde
/// les 12 plus fortes en attendant.
#pragma once

#include "collision_math_od3d.hpp"

#include <cstdint>
#include <optional>

namespace od3d::collision {

struct Settings {
    bool enabled = false; ///< coupé par défaut
    Sensitivity sensitivity = Sensitivity::normal;
    Action action = Action::pause;
};

Settings settings();
void set_settings(const Settings &s);

/// Au démarrage (fil de Marlin) : lit les réglages
void load();

// ---- Interruption de la cellule de charge ----

/// up_g : charge non tarée, vers le haut positive [g] ; position de l'échantillon (positionLookback) ;
/// probing : palpage ou butée de la cellule en cours
void on_loadcell_sample(uint32_t time_us, float up_g, float x, float y, float z, float e, bool probing);

// ---- Fil de Marlin ----

/// À chaque tour de boucle de Marlin
void step();

/// Départ d'une impression (état PrintInit : boucle extérieure, rien dans le planificateur) : les chocs d'avant sont
/// oubliés ; fonction active : accéléromètre vérifié pour toute l'impression
void on_print_start();

// ---- Interface ----

struct Alert {
    uint8_t hits = 0; ///< chocs au même endroit
    int16_t x_mm = 0;
    int16_t y_mm = 0;
    bool paused = false; ///< l'impression a été mise en pause
    bool test = false; ///< pendant l'essai à la buse froide
};

/// Alerte à afficher (une fois)
std::optional<Alert> take_alert();

/// Pic de la dernière impulsion relevée [g] ; 0 : aucune depuis le démarrage
int last_peak_g();

/// Essai à la buse froide, hors impression : armé 60 s, sans pause ; les chocs donnés à la main sous la buse
/// s'affichent dans « Dernier choc » et donnent l'alerte comme en impression
void start_test();
bool test_running();

} // namespace od3d::collision
