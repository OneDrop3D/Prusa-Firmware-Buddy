/// @file
/// OneDrop3D (F1, interface G) : parties pures des écrans Fichiers, Avant l'impression et Impression (SPEC_G.md §3.9–3.12) :
/// métadonnées d'un G-code, durées, anneau de focus. Sans affichage ni fichier : testées dans tests/unit/feature/od3d.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <option/has_toolchanger.h>
#include <string_view>
#include <utility>

namespace od3d::print_info {

/// Têtes de l'INDX (IF-02 : jusqu'à 8), emplacements du MMU3 ; 5 ailleurs : chaque outil de plus coûte 14 o par fichier
/// du cache de la grille
inline constexpr uint8_t max_tools = HAS_TOOLCHANGER() ? 8 : 5;
static_assert(max_tools <= 8, "used et given sur 8 bits");

/// Ce que l'écran montre d'un G-code, lu dans ses métadonnées (comme GCodeInfo::parse_comment)
struct FileInfo {
    uint32_t seconds = 0; ///< temps estimé (mode normal) [s], 0 s'il manque
    std::array<float, max_tools> grams {}; ///< filament par outil [g]
    float millimeters = 0; ///< filament de tous les outils [mm]
    std::array<std::array<char, 8>, max_tools> types {}; ///< matière par outil (« PLA »)
    std::array<int16_t, max_tools> nozzle {}; ///< température de buse par outil (« temperature », hors 1re couche) [°C] ; 0 : absente
    uint8_t used = 0; ///< bit i : l'outil i sert (au moins 0,0003 g, comme GCodeInfo::ExtruderInfo::used)
    uint8_t given = 0; ///< outils listés dans « filament used [g] »

    float total_grams() const;
    bool is_used(uint8_t tool) const { return used & (1 << tool); }
    /// G-code mono-outil (comme GCodeInfo::is_singletool_gcode) : un seul outil listé, le T0, utilisé
    bool single_tool() const { return given == 1 && is_used(0); }
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

/// « Avant l'impression » : place de la k-ième ligne parmi used outils utilisés. Cinq lignes tiennent au-dessus du
/// bandeau d'alertes ; au-delà (INDX 6 à 8 têtes), deux colonnes de quatre
struct RowPlace {
    uint8_t col;
    uint8_t row;
    constexpr bool operator==(const RowPlace &) const = default;
};
constexpr RowPlace row_place(uint8_t k, uint8_t used) {
    return used > 5 ? RowPlace { uint8_t(k / 4), uint8_t(k % 4) } : RowPlace { 0, k };
}

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

/// Ce que la grille des fichiers emprunte au tas (P-04)
struct CachePlan {
    bool folder = false; ///< fenêtre du dossier
    bool infos = false; ///< métadonnées
    uint16_t thumbs = 0; ///< place des vignettes [o], 0 : aucune
};

/// Par ordre d'utilité : la fenêtre du dossier (sans elle, le dossier est relu sur la clé tous les deux crans de molette),
/// les métadonnées, puis les vignettes avec ce qui reste (16, 8 ou 4 Ko de place, plus thumbs_overhead). Un bloc n'est
/// pris que s'il reste reserve octets libres après lui ; une taille 0 : bloc déjà pris (ou pas voulu)
CachePlan plan_caches(size_t free, size_t reserve, size_t folder, size_t infos, size_t thumbs_overhead);

} // namespace od3d::print_info
