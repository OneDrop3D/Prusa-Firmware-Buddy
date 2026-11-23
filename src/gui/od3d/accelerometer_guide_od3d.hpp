/// @file
/// OneDrop3D (ACC-1), côté écran : commun aux écrans des courroies (F14), des vitesses critiques (F22) et de
/// l'amortissement (F33).
/// - un test à l'accéléromètre a été refusé sans bouger parce qu'il ne répond pas (feature/od3d/accelerometer_od3d.hpp) ;
/// - jamais d'origine accéléromètre fixé (retour du 23/11 : « il demande de mettre l'accéléromètre et après il fait un
///   homing avec le plateau ») : sans origine, « Démarrer » la prend d'abord, puis l'écran demande de le fixer ; tant que
///   l'écran est ouvert, les moteurs restent alimentés.
#pragma once

#include <feature/od3d/sleep_od3d.hpp>

namespace od3d {

/// Demande de fixer l'accéléromètre sur la buse et de le brancher. Vrai pour « Réessayer » (l'écran relance alors sa
/// mesure, qui revérifie l'accéléromètre avant tout mouvement) ; faux pour « Annuler ».
bool ask_accelerometer_retry();

/// Ce que dit l'écran tant que l'origine manque (msgid) : « Démarrer prend d'abord l'origine », ou « origine en cours » ;
/// nullptr si elle est faite (l'écran demande alors de fixer l'accéléromètre)
const char *accelerometer_homing_text();

/// « Démarrer » : vrai si l'origine est faite (la mesure peut partir) ; sinon la prend (M1989 H : chauffe coupée,
/// origine, tête au centre), accéléromètre pas encore fixé, et rend faux
bool accelerometer_homed_or_home();

/// Membre des écrans à l'accéléromètre : moteurs tenus tant que l'écran est ouvert (od3d::sleep::hold_motors)
struct AccelerometerMotorHold {
    AccelerometerMotorHold() { sleep::hold_motors(true); }
    ~AccelerometerMotorHold() { sleep::hold_motors(false); }
    AccelerometerMotorHold(const AccelerometerMotorHold &) = delete;
    AccelerometerMotorHold &operator=(const AccelerometerMotorHold &) = delete;
};

} // namespace od3d
