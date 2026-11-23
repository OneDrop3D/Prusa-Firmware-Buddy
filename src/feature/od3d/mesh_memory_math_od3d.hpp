/// @file
/// OneDrop3D (F44, demande du 22/11 : « le maillage du plateau faudra le garder en mémoire pour pouvoir lancer une
/// impression plus vite ») : maillage du plateau gardé, partie pure (sans firmware), testée dans tests/unit/feature/od3d.
///
/// Le G-code de départ de PrusaSlicer pour la CORE One palpe la zone de la pièce à chaque impression (G29 P1, puis
/// G29 P1 X150 Y0 W100 H20 C près de la purge). Les points majeurs palpés (grille 7 × 7, en µm) sont gardés avec ce
/// qui les rend valables : plateau et chambre (± 5 °C), buse (diamètre, trempée, haut débit, type de hotend), buse à la
/// même consigne de palpage (± 5 °C : l'origine Z se prend à cette température), décalage Z de la sonde, dilatation de
/// la buse (F2) active ou non, date. À l'impression suivante, si tout concorde, que la zone à palper est couverte et
/// que le maillage n'a pas dépassé l'âge choisi, il est repris après un **palpage de contrôle** de 3 points aux coins
/// de la zone : leur écart moyen (dispersion de l'origine Z) corrige tout le maillage ; un écart qui varie d'un point à
/// l'autre (plateau bougé, autre plaque, buse changée sans le dire) fait tout repalper comme Prusa.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace od3d::mesh_memory {

inline constexpr size_t majors_x = 7; ///< GRID_MAJOR_POINTS_X de la CORE One (vérifié côté firmware)
inline constexpr size_t majors_y = 7;
inline constexpr size_t majors = majors_x * majors_y;
inline constexpr int16_t no_temperature = INT16_MIN;
inline constexpr float temperature_tolerance = 5.f; ///< plateau, chambre, buse de palpage [°C]
inline constexpr size_t check_points = 3;
inline constexpr float check_spread_mm = 0.02f; ///< écart d'un point de contrôle à l'écart moyen, au plus
inline constexpr float check_offset_mm = 0.05f; ///< écart moyen au plus (origine Z, plaque)

/// Indice d'un point majeur (ix, iy de 0 à 6) dans le tableau et dans les masques
constexpr size_t index_of(size_t ix, size_t iy) {
    return iy * majors_x + ix;
}
constexpr uint64_t major_bit(size_t i) {
    return uint64_t(1) << i;
}

/// Ce qui est gardé (config_store « OD3D Bed Mesh »). Taille fixe : pour changer la structure, créer un nouvel élément.
struct Record {
    static constexpr uint8_t current_version = 1;

    uint64_t valid = 0; ///< bit index_of(ix, iy) : point palpé
    uint32_t time = 0; ///< date du palpage (s depuis 1970, UTC) ; 0 : inconnue, jamais repris
    int16_t bed_c10 = no_temperature; ///< plateau mesuré au palpage [0,1 °C]
    int16_t chamber_c10 = no_temperature; ///< chambre [0,1 °C] ; no_temperature : pas de capteur
    int16_t probe_c = 0; ///< consigne de la buse pendant le palpage [°C]
    int16_t probe_offset_um = 0; ///< décalage Z de la sonde (M851) [µm]
    uint16_t nozzle_um = 0; ///< diamètre de la buse [µm]
    uint8_t version = 0; ///< 0 : vide
    uint8_t flags = 0; ///< bit 0 buse trempée, bit 1 haut débit, bit 2 dilatation de la buse (F2) active
    uint8_t hotend = 0; ///< type de hotend de Prusa
    uint8_t sheet = 0; ///< plaque active (imprimantes à profils de plaque)
    std::array<uint8_t, 4> reserved {}; ///< à 0 (taille sans bourrage)
    std::array<int16_t, majors> z_um {}; ///< hauteur des points majeurs [µm]

    bool empty() const { return version != current_version || valid == 0; }
    bool operator==(const Record &) const = default;
};
static_assert(sizeof(Record) == 128);

/// Hauteur en µm, bornée (±32 mm)
int16_t to_um(float mm);
inline float from_um(int16_t um) {
    return float(um) * 0.001f;
}
/// Température en 0,1 °C ; no_temperature si inconnue (NAN)
int16_t to_c10(float c);

enum class Refusal : uint8_t {
    none,
    disabled, ///< réglage désactivé
    empty, ///< rien de gardé
    clock, ///< date du palpage ou heure actuelle inconnue
    too_old, ///< plus vieux que l'âge choisi (ou date dans le futur)
    bed, ///< plateau à plus de 5 °C
    chamber, ///< chambre à plus de 5 °C, ou capteur apparu ou disparu
    probe_temperature, ///< buse de palpage à plus de 5 °C
    nozzle, ///< buse, hotend, plaque ou dilatation (F2) changés
    z_offset, ///< décalage Z de la sonde changé
    not_covered, ///< un point à palper manque
    too_few_points, ///< zone de moins de check_points + 1 points : rien à gagner
    check_spread, ///< les points de contrôle ne bougent pas ensemble
    check_offset, ///< écart moyen trop grand
    probing, ///< palpage de contrôle impossible
};

/// stored peut-il servir maintenant ? now : mêmes champs relevés maintenant (time : heure actuelle, 0 si inconnue ;
/// z_um et valid ignorés) ; max_age_s : 0 = réglage désactivé
Refusal compare(const Record &stored, const Record &now, uint32_t max_age_s);

/// compare() avant le départ (« Avant l'impression ») : plateau, chambre et buse ne sont pas encore à leur température
/// de palpage, ils ne comptent qu'au G29 (avec la zone à palper)
Refusal compare_before_print(const Record &stored, Record now, uint32_t max_age_s);

/// « Maillage : repris » choisi alors que le réglage est désactivé : âge maximal, le plus long que propose le réglage
inline constexpr uint8_t chosen_max_age_hours = 72;

/// Âge maximal [s] du maillage repris pour une impression ; 0 : pas de reprise (palpage complet). setting_hours : le
/// réglage (0 = désactivé) ; reuse : le choix d'« Avant l'impression », nullopt sans choix (PrusaLink, réimpression...)
constexpr uint32_t max_age_s(uint8_t setting_hours, std::optional<bool> reuse) {
    if (!reuse) {
        return uint32_t(setting_hours) * 3600u;
    }
    return *reuse ? uint32_t(setting_hours ? setting_hours : chosen_max_age_hours) * 3600u : 0;
}

/// Choix « Maillage » d'« Avant l'impression » (retour du 23/11 : plaque changée, ou impression à lancer vite), pour la
/// seule impression demandée juste après ; les autres départs (PrusaLink, Connect, réimpression, port série) gardent le
/// réglage. En mémoire vive : config_store est plein.
class PrintChoice {
public:
    /// Interface, juste avant marlin_client::print_start : true repris, false palpage complet
    void choose(bool reuse) { next = reuse ? 1 : 0; }

    /// Fil de Marlin, à chaque demande d'impression : elle prend le choix posé juste avant, ou aucun (le réglage)
    void on_print_request() { now = next.exchange(-1); }

    /// G29 : le choix pour l'impression demandée ; nullopt : aucun, le réglage
    std::optional<bool> current() const { return now < 0 ? std::nullopt : std::optional<bool>(now == 1); }

private:
    std::atomic<int8_t> next { -1 };
    int8_t now = -1;
};

/// Couverture : needed (points que G29 palperait) tous gardés, et assez nombreux pour que la reprise gagne du temps
Refusal covers(const Record &stored, uint64_t needed);

/// Points de contrôle parmi needed : vers les coins de la zone (min et max de ix + iy, max et min de ix − iy), sans
/// doublon ; rend leur masque (check_points points au plus)
uint64_t pick_checks(uint64_t needed);

/// Résultat du contrôle : écart moyen des points palpés de nouveau par rapport aux gardés
struct Check {
    Refusal refusal = Refusal::none;
    float offset_mm = 0; ///< à ajouter aux points gardés
    float spread_mm = 0; ///< plus grand écart d'un point à offset_mm
};

/// fresh_mm : hauteurs palpées de nouveau (NAN : pas palpé), aux indices de checks
Check evaluate(const Record &stored, uint64_t checks, std::span<const float, majors> fresh_mm);

} // namespace od3d::mesh_memory
