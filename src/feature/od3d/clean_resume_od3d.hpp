/// @file
/// OneDrop3D (F30) : reprise sans cicatrice (option, coupée par défaut ; voir clean_resume_math_od3d.hpp).
///
/// Déjà chez Prusa : à la pause, rétraction de STANDARD_RETRACT_LENGTH, tête garée hors de la pièce et plateau
/// descendu ; à la reprise, réchauffe, retour en Z puis en XY 5 mm au-dessus de la pièce (unpark_head_XY), descente à
/// 5 mm/s et désrétraction immobile au point de reprise (unpark_head_ZE).
///
/// Ajouté, option active et remplissage connu : juste avant ce retour, descente rétractée sur le remplissage de l'objet,
/// désrétraction et petite purge en bas, court cordon le long du remplissage, rétraction et remontée ; puis le retour de
/// Prusa. Sans remplissage connu, et après une coupure de courant (la mémoire du remplissage ne survit pas au
/// redémarrage) : la reprise de Prusa, sans aucun mouvement de plus.
#pragma once

#include "clean_resume_math_od3d.hpp"

#include <cstdint>

namespace od3d::clean_resume {

Settings settings();
void set_settings(const Settings &s);

// ---- Fil de Marlin ----

/// Au démarrage (settings_load) : marqueurs de type à la lecture du fichier selon le réglage (la lecture d'une impression
/// commence avant son PrintInit)
void load();

/// Départ d'une impression : remplissage oublié, marqueurs de type à la lecture du fichier selon le réglage
void on_print_start();

/// M2011 S<n>, rendu par la lecture du fichier à la place d'un « ;TYPE:… » (feature_marks_od3d.hpp)
void set_feature(uint8_t feature);

/// Changement de tête (INDX, tool_change.cpp) : le remplissage gardé est celui d'une autre buse (diamètre, matière,
/// débit par mm) : oublié ; les hauteurs de couche restent
void on_tool_change();

/// G0/G1 traité (G0_G1.cpp), positions natives avant et après ; rien si l'option est coupée
void on_print_move(float x0, float y0, float z0, float e0, float x1, float y1, float z1, float e1);

/// Reprise après pause, juste avant le retour de Prusa (unpark_head_XY). resume_z, resume_e : Z et E de la pièce
void before_unpark(float resume_z, float resume_e);

} // namespace od3d::clean_resume
