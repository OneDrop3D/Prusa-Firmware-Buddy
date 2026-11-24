/// @file
/// OneDrop3D (F1, interface G) : parties pures des écrans Fichiers, Avant l'impression et Impression (SPEC_G.md §3.9–3.12) :
/// métadonnées d'un G-code, durées, anneau de focus. Sans affichage ni fichier : testées dans tests/unit/feature/od3d.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>

namespace od3d::print_info {

inline constexpr uint8_t max_tools = 5; ///< emplacements du MMU3

/// Ce que l'écran montre d'un G-code, lu dans ses métadonnées (comme GCodeInfo::parse_comment)
struct FileInfo {
    uint32_t seconds = 0; ///< temps estimé (mode normal) [s], 0 s'il manque
    std::array<float, max_tools> grams {}; ///< filament par outil [g]
    float millimeters = 0; ///< filament de tous les outils [mm]
    std::array<std::array<char, 8>, max_tools> types {}; ///< matière par outil (« PLA »)
    std::array<int16_t, max_tools> nozzle {}; ///< température de buse par outil (« temperature », hors 1re couche) [°C] ; 0 : absente
    uint8_t used = 0; ///< bit i : l'outil i sert (au moins 0,0003 g, comme GCodeInfo::ExtruderInfo::used)

    float total_grams() const;
    bool is_used(uint8_t tool) const { return used & (1 << tool); }
};

/// Prend une ligne « nom = valeur » des métadonnées ; les autres noms sont ignorés
void parse_metadata(FileInfo &info, std::string_view name, std::string_view value);

/// « 1d 2h 3m 4s » (temps estimé du trancheur) → secondes
uint32_t parse_duration(std::string_view text);

/// « 2 h 14 » : heures, puis minutes sur deux chiffres (maquette)
void format_hm(uint32_t seconds, char *buf, size_t size);

/// Retrait du bord gauche (et du droit) d'un rectangle arrondi de hauteur h et de rayon r à la ligne y,
/// même règle que display::draw_rounded_rect (dedans si dx² + dy² < r²)
int corner_inset(int y, int h, int r);

/// Frise de progression de l'écran Impression (w × h) : ce que montre le pixel (x, y) à percent %
enum class StripPart : uint8_t {
    outside, ///< fond de l'écran
    track, ///< reste à imprimer
    done, ///< imprimé (orange)
    cursor, ///< curseur blanc de 3 px
};
StripPart strip_part(int x, int y, int w, int h, int percent);

/// Colonnes [first, second) de la frise qui changent de from à to % : tout ce qui diffère est dedans
std::pair<int, int> strip_changed(int w, int from, int to);

} // namespace od3d::print_info
