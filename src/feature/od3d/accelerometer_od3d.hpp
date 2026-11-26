/// @file
/// OneDrop3D (ACC-1) : règle commune des tests qui ont besoin de l'accéléromètre (F14 courroies M1992, F22 vitesses
/// critiques M1993, F33 amortissement M1997).
///
/// Sur la CORE One, l'accéléromètre se clipse sur la buse et ne se branche que le temps d'une calibration :
/// - avant tout mouvement, le test vérifie qu'il répond (ready()) ; sinon il est refusé sans rien bouger, et son écran
///   demande de le fixer sur la buse et de le brancher, avec « Réessayer » / « Annuler » (ask_accelerometer_retry(),
///   gui/od3d/accelerometer_guide_od3d.hpp) ;
/// - aucune fonction ne suppose l'accéléromètre pendant une impression : les fonctions permanentes n'utilisent que des
///   valeurs enregistrées pendant une calibration (F33 : l'amortissement est écrit dans la configuration de l'input
///   shaper de Prusa).
#pragma once

namespace od3d::accel {

/// Moyenne des échantillons, tête à l'arrêt (m/s²) : la pesanteur si la puce répond vraiment (entre 0,5 et 1,5 g).
/// Une puce débranchée après avoir été vue laisse le bus SPI en l'air : octets tous à 0 ou tous à 1, soit presque 0 g.
constexpr bool gravity_plausible(float mean_x, float mean_y, float mean_z) {
    constexpr float g = 9.80665f;
    const float norm2 = mean_x * mean_x + mean_y * mean_y + mean_z * mean_z;
    return norm2 >= (0.5f * g) * (0.5f * g) && norm2 <= (1.5f * g) * (1.5f * g);
}

/// Fil de Marlin, tête à l'arrêt, avant tout mouvement : l'accéléromètre est détecté (Prusa le recherche à chaque
/// session, il peut donc être branché après le démarrage), ses échantillons arrivent et montrent la pesanteur.
/// Dure moins de 0,3 s. INDX sans tête prise : vrai sans lire, l'accéléromètre est dans chaque tête (take_head).
bool ready();

/// INDX (revue du 26/11) : l'accéléromètre est celui de la tête active, lu par MODBUS ; sans tête, il ne répond pas.
/// Après l'origine, une mesure prend une tête comme l'input shaper de Prusa (M1959) ; ailleurs, rien
void take_head();

} // namespace od3d::accel
