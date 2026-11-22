/// @file
/// OneDrop3D (F1, interface G) : bobine vue de face, calculée pixel par pixel (SPEC_G.md §1.12).
/// Partie pure, sans affichage : testée dans tests/unit/feature/od3d. Le dessin est od3d::draw_spool().
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>
#include <utils/color.hpp>

namespace od3d::spool {

struct Look {
    std::optional<Color> filament; ///< couleur de l'enroulement ; nullopt = emplacement sans bobine
    uint8_t fill_percent = 100; ///< reste / poids net [%]

    bool operator==(const Look &) const = default;
};

/// Anticrénelage (retour du 22/11 : « les icônes de filament ne sont pas lisses ») : un pixel à cheval sur le bord de
/// deux zones rondes prend le mélange de leurs couleurs, en quarts, selon la part d'une fenêtre de 1 px posée le long du
/// rayon qui tombe de chaque côté du bord. Distance au centre en px, depuis le centre du pixel.

/// Distance du centre du pixel (x, y) au centre du carré side × side
float radius_in_square(int x, int y, int side);

/// Quarts (0 à 4) du pixel à la distance r du centre qui tombent au-delà du cercle de rayon edge
uint8_t beyond_quarters(float r, float edge);

/// inner pour 4 - q quarts, outer pour q quarts (q de 0 à 4)
Color mix_quarters(Color inner, Color outer, uint8_t q);

/// Couleur d'un pixel à la distance r : inside dans le cercle de rayon edge, outside au-delà, mélangées au bord
inline Color edge_color(float r, float edge, Color inside, Color outside) {
    return mix_quarters(inside, outside, beyond_quarters(r, edge));
}

/// Disque d'une bobine de diamètre d (pair) : jante, flasque, partie vide, enroulement en anneaux
/// (2 px de la couleur, 1 px plus sombre), moyeu. Chaque bord de bande est anticrénelé.
class Disc {
public:
    Disc(const Look &look, uint8_t diameter);

    /// Couleur du pixel (x, y) du carré d × d ; back hors du disque, mélangé au bord de la jante
    Color pixel(int x, int y, Color back) const;

    /// Même chose à la distance r du centre (déjà calculée par l'appelant)
    Color at_radius(float r, Color back) const;

private:
    /// Couleur de la bande au rayon entier rho (centre du pixel entre rho et rho + 1), back au-delà du disque
    Color band(int rho, Color back) const;

    Color filament;
    Color filament_dark;
    bool has_filament;
    uint8_t d;
    // Rayon intérieur de chaque bande [px], de l'extérieur vers le centre
    uint8_t rim, flange, flange_edge, winding, hub_edge, hub, hub_ring, hole;
};

/// Anneau sombre de l'enroulement : chaque canal × 0,72 (maquette : shade(c, -0.28))
Color shade(Color c);

/// Couleur d'une matière quand le préréglage n'en a pas (seuls les filaments ad hoc ont une couleur)
Color default_color(std::string_view material);

} // namespace od3d::spool
