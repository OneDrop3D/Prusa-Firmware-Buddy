/// @file
/// OneDrop3D (interface G) : fond givré derrière nos dialogues (retour du 22/11 : « il n'y a pas de fond (celui de base),
/// peut-être mettre un léger flou en arrière-plan »). Ce que l'écran montre autour du dialogue est relu sur l'afficheur
/// (comme la capture d'écran de Prusa), flouté (boîte 3 × 3) et assombri (× 0,45 : le voile noir à 55 % de la maquette),
/// puis réécrit. Aucune image de l'écran en mémoire : une tranche de ≤ 1 920 pixels (le tampon de 4 lignes de
/// l'afficheur) et deux lignes de sommes (2 880 octets).
///
/// La relecture est lente (SPI à 5,25 MHz, 3 octets par pixel : 4,6 µs par pixel, ~175 ms pour les 38 000 pixels autour
/// de l'écran de chargement) : le dialogue s'affiche d'abord, le fond se givre ensuite, une tranche (~11 ms) par tour de
/// la boucle de l'interface quand rien n'est à redessiner, sans bloquer la molette ni le toucher.
///
/// Partie pure (Job), testée seule ; l'afficheur et la boucle de l'interface sont dans frost_od3d.cpp.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

class window_t;
class Rect16;

namespace od3d::frost {

/// Rect en pixels de l'écran
struct Area {
    int16_t x = 0;
    int16_t y = 0;
    uint16_t w = 0;
    uint16_t h = 0;

    bool empty() const {
        return w == 0 || h == 0;
    }
};

/// Une composante après flou et voile, d'après la somme des 9 composantes voisines ramenées à 6 bits (≤ 567) ; rendue
/// comme l'afficheur les prend, 6 bits en haut de l'octet. × 0,45 / 9 ≈ 51 / 1024 : le fond #1B1C1E devient #0C0C0C,
/// la couleur du voile (style::veil).
constexpr uint8_t shade(unsigned sum9) {
    return static_cast<uint8_t>(((sum9 * 51 + 512) >> 10) << 2);
}

/// area moins hole, en bandes dans l'ordre du givrage : haut, gauche, droite, bas (vides s'il n'y a rien)
std::array<Area, 4> margins(Area area, Area hole);

/// Givrage d'area moins hole, tranche par tranche. Le flou ne lit que dans sa bande (bords répétés) : il ne prend ni le
/// dialogue, ni une bande voisine déjà givrée.
class Job {
public:
    static constexpr uint16_t max_w = 480;

    /// Lit la fenêtre (3 octets par pixel, ligne après ligne) et rend le tampon qui la contient, nullptr si échec
    using Read = uint8_t *(*)(Area window);
    /// Réécrit la fenêtre depuis le début du tampon
    using Write = void (*)(Area window, const uint8_t *buffer);

    void start(Area area, Area hole);
    void stop() {
        index = bands.size();
    }
    bool done() const {
        return index >= bands.size();
    }

    /// Une tranche ; capacity : pixels que le tampon de read peut tenir (au moins 2 lignes de la bande la plus large)
    void step(size_t capacity, Read read, Write write);

private:
    std::array<Area, 4> bands {};
    size_t index = bands.size();
    int16_t row = 0; ///< prochaine ligne à réécrire dans la bande
    bool primed = false; ///< les sommes des lignes row - 1 et row sont en cache
    /// Sommes horizontales (3 composantes de 6 bits, ≤ 189) des lignes row - 1 et row, pour la tranche suivante
    std::array<uint8_t, 2 * max_w * 3> sums {};
};

// ---- Sur l'imprimante ----

/// Givre ce que l'écran montre dans area autour de hole, pour owner (le dialogue ou l'écran qui s'y dessine) : à
/// appeler à son premier dessin, une fois son cadre peint
void start(const window_t &owner, const Rect16 &area, const Rect16 &hole);

/// owner disparaît ou se repeint en entier : plus rien à givrer pour lui
void stop(const window_t &owner);

/// Un dialogue givré se ferme : stop(), et l'écran dessous se repeint d'une passe (le fond entre ses éléments, puis
/// chacun) au lieu d'un aplat sur toute la place du dialogue avant ses éléments
void close(const window_t &dialog);

/// Boucle de l'interface, quand rien n'est à redessiner : une tranche si owner est toujours au-dessus ; false s'il n'y a
/// plus rien à faire
bool step();

} // namespace od3d::frost
