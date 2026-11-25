/// @file
/// OneDrop3D (F30) : type de tracé du trancheur (« ;TYPE:Internal infill » de PrusaSlicer) jusqu'à la tâche de Marlin.
///
/// Les « ;TYPE:… » sont dans le fichier, texte comme .bgcode (PrusaSlicer encode le .bgcode en MeatPack avec
/// commentaires : lib/libbgcode/tests/data/mini_cube_ps2.8.1_ref.gcode en contient), mais Prusa jette tout commentaire
/// à la lecture (media_prefetch.cpp) : rien plus loin ne les voit. Quand la reprise sans cicatrice est active, chaque
/// ligne « ;TYPE:… » y devient « M2011 S<n> », à la place exacte du commentaire (mêmes positions de reprise).
/// Fonction coupée : la lecture du fichier est celle de Prusa. Sans dépendance : compilé aussi dans ses tests.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

namespace od3d::feature_marks {

/// Écrit par la tâche de Marlin (départ d'impression, réglage), lu par la lecture du fichier
inline std::atomic<bool> enabled = false;

/// S de M2011
enum Feature : uint8_t {
    other = 0,
    infill = 1, ///< remplissage intérieur : PrusaSlicer « Internal infill », Orca « Sparse infill », Cura « FILL »
};

/// Une ligne de commentaire lue caractère par caractère, après son ';' (sans le '\n'). Remise à zéro avec la ligne.
struct LineParser {
    static constexpr char prefix[] = "TYPE:";
    static constexpr uint8_t prefix_len = sizeof(prefix) - 1;
    static constexpr uint8_t name_max = 16; ///< plus long : un autre tracé

    uint8_t matched = 0; ///< caractères de prefix reconnus
    uint8_t name_len = 0;
    bool not_type = false; ///< commentaire ordinaire
    bool long_name = false;
    char name[name_max] = {};

    constexpr void feed(char ch) {
        if (not_type) {
            return;
        }
        if (matched < prefix_len) {
            if (ch == prefix[matched]) {
                ++matched;
            } else {
                not_type = true;
            }
            return;
        }
        if (ch == '\r') {
            return;
        }
        if (name_len == name_max) {
            long_name = true;
            return;
        }
        name[name_len++] = ch;
    }

    /// Fin de la ligne : -1 si ce n'était pas un « TYPE: », sinon S de M2011
    constexpr int finish() const {
        if (not_type || matched < prefix_len) {
            return -1;
        }
        if (long_name) {
            return other;
        }
        size_t len = name_len;
        while (len > 0 && name[len - 1] == ' ') {
            --len;
        }
        const std::string_view type(name, len);
        return (type == "Internal infill" || type == "Sparse infill" || type == "FILL") ? infill : other;
    }
};

/// Commande rendue pour un marqueur (feature : finish()), avec son '\0'. \returns sa longueur ; 0 si ce n'était pas
/// un marqueur ou si out est trop court (la ligne est alors sautée, comme chez Prusa)
inline size_t write_command(int feature, char *out, size_t size) {
    if (feature < 0) {
        return 0;
    }
    char command[] = "M2011 S0";
    command[sizeof(command) - 2] = char('0' + (feature == infill ? infill : other));
    if (size < sizeof(command)) {
        return 0;
    }
    memcpy(out, command, sizeof(command));
    return sizeof(command) - 1;
}

} // namespace od3d::feature_marks
