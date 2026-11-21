/// @file
/// OneDrop3D (F13) : suivi du poids des bobines, partie pure (sans mémoire ni affichage), testée dans
/// tests/unit/feature/od3d. Longueur extrudée → grammes (filament 1,75 mm × densité de la matière), reste, alerte
/// de fin, quand écrire en mémoire. La partie qui compte l'extrusion est spool_usage_od3d.hpp.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace od3d::spool_usage {

inline constexpr uint16_t default_alert_g = 50; ///< alerte de fin par défaut [g]
inline constexpr uint16_t min_net_g = 100; ///< poids net neuf, valeur libre [g]
inline constexpr uint16_t max_net_g = 3000;
inline constexpr uint16_t max_alert_g = 500;
inline constexpr std::array<uint16_t, 5> net_presets { 250, 500, 750, 1000, 2000 }; ///< choix de l'assistant [g]

inline constexpr float filament_diameter_mm = 1.75f;
inline constexpr float default_density = 1.24f; ///< matière inconnue : celle du PLA [g/cm³]

/// Ce qui est gardé par emplacement (config_store, « OD3D Spool Usage »). Taille fixe : pour changer la structure,
/// créer un nouvel élément (nouveau nom) et migrer l'ancien, sans réutiliser ce nom.
struct Record {
    uint16_t net_g = 0; ///< poids net de la bobine neuve [g] ; 0 = pas de suivi (bobine jamais déclarée)
    uint16_t alert_g = default_alert_g; ///< seuil d'alerte de fin [g]
    float remaining_g = 0; ///< reste [g], diminué à chaque écriture, corrigé après une pesée
    float used_g = 0; ///< consommé depuis la pose de la bobine [g]
    float last_print_g = 0; ///< consommé par la dernière impression qui a utilisé l'emplacement [g]

    bool operator==(const Record &) const = default;
};
static_assert(sizeof(Record) == 16);

/// Densité de la matière [g/cm³] d'après son nom (« PLA », « PETG-CF », « pla+ ») : le plus long préfixe connu,
/// sans tenir compte de la casse ; default_density sinon
float density(std::string_view material);

/// Masse d'un millimètre de filament de 1,75 mm [g/mm]
float grams_per_mm(float density);

/// Masse d'une longueur de filament [g]
float grams(float length_mm, float density);

/// Longueur restante [m] d'une masse de filament
float meters(float weight_g, float density);

/// La bobine est suivie : son poids net a été déclaré
constexpr bool is_tracked(const Record &r) {
    return r.net_g > 0;
}

/// Bobine neuve posée (« Changer la bobine ») : reste = poids net, consommation remise à zéro ; le seuil reste
Record new_spool(const Record &previous, uint16_t net_g);

/// Nouveau poids net sans changer de bobine (« Matière et couleur », utilisation) : reste = min(reste, net) ;
/// une bobine encore jamais suivie part pleine
Record set_net(const Record &previous, uint16_t net_g);

/// Consommation de grams : le reste ne descend pas sous 0
Record consume(const Record &previous, float grams);

/// Remplissage dessiné [%] : reste / net, borné à 0–100
uint8_t fill_percent(float remaining_g, float net_g);

/// Reste au seuil d'alerte ou en dessous
bool is_low(float remaining_g, float alert_g);

/// Le reste vient de passer au seuil d'alerte ou en dessous (une seule alerte par passage)
bool crossed_alert(float before_g, float after_g, float alert_g);

/// Les grammes prévus par le fichier dépassent le reste (faux si l'une des valeurs est inconnue : NAN)
bool not_enough(float planned_g, float remaining_g);

/// Quand écrire en mémoire ce qui a été extrudé (parcimonie) :
/// - jamais pour moins de 0,1 g ;
/// - toujours au-delà de flush_g (coupure de courant : au plus cette perte) ;
/// - en impression, quand l'outil n'est plus l'outil actif (changement d'outil) et qu'il a au moins tool_change_g ;
/// - hors impression, quand l'extrusion est arrêtée depuis idle_ms (chargement, purge, extrusion à la main).
/// La fin d'impression écrit tout, à part.
inline constexpr float flush_g = 25;
inline constexpr float tool_change_g = 5;
inline constexpr uint32_t idle_ms = 30'000;
bool should_commit(float pending_g, bool printing, bool active_tool, uint32_t still_ms);

/// « 250 g », « 1250 g » ; « 1 kg », « 2 kg » pour les multiples de 1000
void format_weight(char *buf, size_t size, uint16_t grams);

} // namespace od3d::spool_usage
