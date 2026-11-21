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

/// Disque d'une bobine de diamètre d : jante, flasque, partie vide, enroulement en anneaux
/// (2 px de la couleur, 1 px plus sombre), moyeu.
class Disc {
public:
    Disc(const Look &look, uint8_t diameter);

    /// Couleur du pixel (x, y) du carré d × d, nullopt hors du disque
    std::optional<Color> pixel(int x, int y) const;

    /// Parcourt la ligne y par segments de même couleur, en sautant ce qui est hors du disque :
    /// f(x, longueur, couleur)
    template <class F>
    void for_each_run(int y, F &&f) const {
        int start = 0;
        std::optional<Color> current = pixel(0, y);
        for (int x = 1; x <= d; ++x) {
            const std::optional<Color> c = x < d ? pixel(x, y) : std::nullopt;
            if (c != current) {
                if (current) {
                    f(start, x - start, *current);
                }
                start = x;
                current = c;
            }
        }
    }

private:
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
