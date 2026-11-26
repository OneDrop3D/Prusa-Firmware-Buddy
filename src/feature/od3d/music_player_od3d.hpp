/// @file
/// OneDrop3D (F18) : musique des moteurs, lecteur côté Marlin (M1991) et commandes de l'interface.
/// Le fichier, les moteurs et l'option « à la fin de chaque impression » sont dans le config_store
/// (od3d_music_file, od3d_music_motors, od3d_music_at_print_end) ; la vitesse et la hauteur de chaque morceau, dans
/// tuning_path sur la clé.
#pragma once

#include "music_od3d.hpp"

#include <cstdint>

namespace od3d::music {

/// Réglages par morceau (Tuning), à côté des morceaux
inline constexpr const char *tuning_path = "/usb/OD3D_MUS.BIN";

/// Réglages de la lecture en cours (l'éditeur de l'écran, à chaque pas) : appliqués dès l'accord suivant. play() part
/// de ceux retenus pour le morceau.
void set_live_tuning(Tuning tuning);
Tuning live_tuning();

/// Voix jouées (CORE One = CoreXY : les moteurs du haut A et B chantent chacun une voix dans un seul déplacement XY ;
/// le plateau, en Z, une 3e). Les valeurs de od3d_music_motors gardent leur place dans le config_store (plein) : les
/// anciens « X et Y », « X seul » et « Z » deviennent ces trois-là (retour du 26/11)
enum class Motors : uint8_t {
    three = 0, ///< « 3 voix » : mélodie sur A, 2e voix sur B, basse sur Z (le plateau monte et descend)
    two = 1, ///< « 2 voix » : mélodie sur A, basse sur B
    melody = 2, ///< « Mélodie » : la plus aiguë seule, sur A et B ensemble (déplacement en X, le plus fort)
};
inline constexpr uint8_t motors_count = 3;

/// État de la dernière lecture, pour l'interface
enum class Status : uint8_t {
    idle, ///< rien lancé depuis le démarrage
    playing,
    done, ///< morceau joué jusqu'au bout
    stopped, ///< arrêté sur demande (ou impression lancée)
    no_file, ///< pas de fichier choisi, ou il ne s'ouvre pas (clé retirée)
    bad_file, ///< pas un MIDI de format 0 ou 1
    no_notes, ///< aucune note à jouer
    printing, ///< refusé : une impression est en cours
    no_room, ///< pas assez de course sur l'axe à cette hauteur
    no_memory, ///< pas la place sur le tas pour lire le fichier
};

/// Corps de M1991 : joue le fichier choisi sur les moteurs choisis (motors_override < motors_count : à la place
/// du réglage). Bloque jusqu'à la fin du morceau ou jusqu'à request_stop().
void play(uint8_t motors_override = 0xff);

/// Demande l'arrêt (appelable depuis l'interface) : les mouvements déjà en file finissent (moins d'une seconde)
void request_stop();

bool is_playing();

Status status();

/// Nombre de M1991 traités (joués ou refusés) depuis le démarrage : l'interface sait quand sa demande a répondu
uint16_t run_count();

/// À la fin d'une impression réussie (marlin_server) : met M1991 en file si l'option est active et qu'un
/// fichier est choisi
void on_print_finished();

} // namespace od3d::music
