/// @file
/// OneDrop3D (F8) : lecture des mouvements à venir dans la file du planificateur de Prusa
/// (calculs purs dans lookahead_math_od3d.hpp).
///
/// Sûreté de la lecture : la file (Planner::block_buffer, [tail, head)) est remplie par la tâche de Marlin
/// (Planner::_buffer_msteps / buffer_raw_block, qui attend une case libre dans idle() *avant* d'écrire le bloc, puis
/// avance head) et vidée par l'interruption du mouvement (PreciseStepping), qui n'avance que tail et les drapeaux
/// busy. Ces fonctions sont appelées depuis la tâche de Marlin elle-même (LocalHotend::manage() par
/// Temperature::manage_heater(), dans idle()), comme la lecture que Prusa fait déjà dans
/// Planner::check_axes_activity() (get_first_move_block()) : aucune case de [tail, head) ne peut être
/// réécrite pendant la lecture. Si tail avance pendant ce temps, on lit au pire un bloc qui vient de finir (ses
/// données restent en place). Une copie de head et tail est faite une fois, le parcours est borné à
/// BLOCK_BUFFER_SIZE cases, et le résultat est borné ensuite (0 à max_e_speed).
#pragma once

#include <cstdint>

namespace od3d::lookahead {

/// F8 : vitesse moyenne du filament [mm/s] sur les flow_window_s (~500 ms) à venir ; reculs et vitesses au-delà de
/// max_e_speed comptés pour 0. Tâche de Marlin.
float planner_e_speed_ahead(float max_e_speed);

} // namespace od3d::lookahead
