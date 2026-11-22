/// @file
/// OneDrop3D (ACC-1), côté écran : un test à l'accéléromètre a été refusé sans bouger parce qu'il ne répond pas
/// (feature/od3d/accelerometer_od3d.hpp). Commun aux écrans des courroies (F14), des vitesses critiques (F22) et de
/// l'amortissement (F33).
#pragma once

namespace od3d {

/// Demande de fixer l'accéléromètre sur la buse et de le brancher. Vrai pour « Réessayer » (l'écran relance alors sa
/// mesure, qui revérifie l'accéléromètre avant tout mouvement) ; faux pour « Annuler ».
bool ask_accelerometer_retry();

} // namespace od3d
