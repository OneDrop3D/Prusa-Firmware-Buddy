/// @file
/// OneDrop3D : ce que fait la mesure en cours, en clair (retour du 23/11 : « voir plus en détail ce que l'imprimante
/// fait »). Le G-code qui tourne (fil de Marlin) publie une phrase et ses nombres (« Chauffe de la buse à 215 °C :
/// 182 °C », « Palpage 2/3 à 215 °C »), et la valeur mesurée dès qu'elle existe ; l'assistant de calibration les
/// affiche avec l'avancement du test. Mémoire vive seulement ; l'assistant efface tout au début de chaque étape.
#pragma once

#include <cstdint>

namespace od3d::live {

/// Une phrase N_() (au plus trois %d) et ses nombres
struct Line {
    const char *msgid = nullptr; ///< nullptr : rien de publié
    int16_t a = 0;
    int16_t b = 0;
    int16_t c = 0;

    bool operator==(const Line &) const = default;
};

/// Fil de Marlin : ce qui se fait (msgid à durée de vie statique, N_())
void set(const char *msgid, int a = 0, int b = 0, int c = 0);

/// Fil de Marlin : la dernière valeur mesurée, gardée jusqu'au clear()
void set_value(const char *msgid, int a = 0, int b = 0, int c = 0);

void clear();

/// Tout fil (lecture champ par champ : un nombre peut avoir un tour de retard, sans conséquence à l'écran)
Line now();
Line value();

} // namespace od3d::live
