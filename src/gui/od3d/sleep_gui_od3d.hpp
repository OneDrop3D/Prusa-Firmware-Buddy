/// @file
/// OneDrop3D (F39) : mode veille côté interface (logique : feature/od3d/sleep_math_od3d.hpp ; coupures :
/// feature/od3d/sleep_od3d.hpp ; réglages : screen_sleep_od3d.hpp).
///
/// Écran éteint : rétroéclairage à 0 par les deux commandes de l'xLCD (luminosité de l'ILI9488 sur les écrans
/// récents, 4e sortie des LED d'état sur les anciens), écran peint en noir, plus aucun dessin. Au réveil, la
/// luminosité d'avant revient et tout l'écran courant (dialogues compris) est redessiné. Une réinitialisation de
/// l'écran pendant la veille (lcd::communication_check) remet la luminosité au maximum : elle est recoupée au tour
/// suivant.
#pragma once

#include <cstdint>
#include <window_types.hpp>

namespace od3d::sleep {

/// gui_loop, avant le dessin : relève l'imprimante, s'endort ou se réveille, éteint ou rallume
void gui_step();

/// Molette (gui.cpp) : retire la part du geste qui réveille l'écran (cran, appui jusqu'au relâchement)
void filter_knob(int32_t &encoder_diff, bool &is_btn, BtnState_t btn);

/// Toucher (gui.cpp). @return avalé : il a réveillé l'écran
bool swallow_touch();

/// Écran éteint : rien n'est dessiné
bool screen_off();

/// « Mettre en veille maintenant ». @return false : imprimante occupée
bool sleep_now();

} // namespace od3d::sleep
