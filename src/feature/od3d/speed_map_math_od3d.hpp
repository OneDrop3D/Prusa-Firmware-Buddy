/// @file
/// OneDrop3D (F22) : carte des vitesses critiques (VFA) — calculs purs, sans Marlin ni affichage,
/// testés dans tests/unit/feature/od3d/speed_map_test.cpp.
///
/// - énergie vibratoire pendant un mouvement à vitesse constante : moyenne quadratique de l'accélération après un
///   passe-haut (la pesanteur et le mouvement lui-même disparaissent, restent les vibrations) ;
/// - longueur d'un passage pour avoir assez de mesure à vitesse constante ;
/// - vitesses critiques : sommets nets de l'énergie en fonction de la vitesse.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace od3d::speed_map {

/// Énergie vibratoire d'un flux d'échantillons (m/s²) : passe-haut du 1er ordre (moyenne glissante exponentielle
/// retranchée, constante de temps tau), puis moyenne des carrés des trois axes sur les échantillons « comptés »
class VibrationMeter {
public:
    /// sample_period et tau en secondes
    VibrationMeter(float sample_period, float tau);

    /// Ajoute un échantillon ; counted : il est pris dans la moyenne (sinon il ne fait que régler le filtre)
    void add(float x, float y, float z, bool counted);

    /// Échantillons comptés
    uint32_t count() const { return n; }

    /// Moyenne des carrés comptés (m²/s⁴), 0 sans échantillon
    float mean_square() const;

    /// Pour additionner deux passages (aller et retour) : somme des carrés et nombre
    double sum_square() const { return sum; }

private:
    float alpha;
    float mean[3] = { 0, 0, 0 };
    bool started = false;
    double sum = 0;
    uint32_t n = 0;
    uint16_t settling = 0; ///< échantillons avant que le filtre soit établi (pas comptés)
};

/// Géométrie d'un passage à vitesse constante v (mm/s), accélération accel (mm/s²)
struct Pass {
    float length; ///< mm à parcourir
    float ramp; ///< mm d'accélération (et autant de freinage)
    float margin; ///< mm ignorés en plus de chaque côté (retard de l'accéléromètre)
};

/// Passage le plus court qui donne cruise_s secondes de mesure à vitesse constante, borné à max_length (la mesure
/// raccourcit alors) ; margin = margin_mm + v × delay_s
Pass pass_for(float v, float accel, float cruise_s, float margin_mm, float delay_s, float max_length);

/// La distance parcourue d (mm, depuis le départ du passage) est dans la partie à vitesse constante mesurée
bool in_cruise(const Pass &pass, float d);

/// Bout commun (coordonnée basse) des passages d'un axe : chaque vitesse va de là à là + length et en revient. Elle
/// repart donc où la précédente s'est arrêtée, sans déplacement de recentrage, et l'aller vaut le retour. Le plus long
/// passage (top_length, la vitesse la plus haute) est centré sur centre.
float sweep_from(float centre, float top_length);

/// Médiane des valeurs finies (0 sans valeur)
float median(std::span<const float> values);

/// Vitesses critiques : indices des sommets (plus hauts que leurs voisins, un bord compte) dont la valeur dépasse
/// factor × la médiane et min_value, du plus fort au plus faible, au plus out.size() (64 valeurs lues au plus). Rend leur nombre.
size_t find_critical(std::span<const float> values, float factor, float min_value, std::span<uint8_t> out);

/// « 60, 120, 250 » : vitesses (start + index × step) dans l'ordre croissant ; ASCII
void format_speeds(std::span<const uint8_t> indices, float start, float step, char *out, size_t size);

/// Hauteur en pixels (0 à h) d'une barre de valeur value pour une échelle scale (valeur de la barre pleine)
int16_t bar_height(float value, float scale, int16_t h);

} // namespace od3d::speed_map
