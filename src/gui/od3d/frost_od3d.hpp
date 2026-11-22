/// @file
/// OneDrop3D (interface G) : fond givré derrière nos dialogues (retour du 22/11 : « il n'y a pas de fond (celui de base),
/// peut-être mettre un léger flou en arrière-plan »). Ce que l'écran montre autour du dialogue est relu sur l'afficheur
/// (comme la capture d'écran de Prusa), flouté et assombri (× 0,45 : le voile noir à 55 % de la maquette), puis réécrit.
///
/// Retour du 22/11 (8.0.2.25) : « le flou arrive après, de bas en haut, alors qu'il devait être en même temps ». La
/// relecture est lente (SPI à 5,25 MHz, 3 octets : 4,6 µs par pixel, et ~1 ms de pause par lecture) ; tout relire
/// prenait ~175 ms, en tranches après le dialogue. Le givrage se fait maintenant d'une traite, avant le dialogue :
/// - seules 2 lignes sur 8 sont relues ; un nœud toutes les 8 colonnes et 8 lignes prend la moyenne de ses 8 × 2 pixels,
///   et chaque pixel est l'interpolation des 4 nœuds autour (un flou plus large que le 3 × 3 d'avant, plus « givré ») ;
/// - tout est relu avant la première écriture (nœuds gardés : 2 880 octets), puis réécrit en ~20 ms : rien ne progresse
///   à l'œil.
/// Fiche de bobine (480 × 73) : 10 lectures, 9 600 pixels, ~55 ms de lecture et ~20 ms d'écriture, puis la fiche.
///
/// Partie pure (run avec read et write), testée seule ; l'afficheur est dans frost_od3d.cpp.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

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

/// Une composante après voile, d'après la moyenne de ses voisines en unités de l'afficheur (6 bits en haut de l'octet,
/// ≤ 252) ; rendue comme l'afficheur la prend. × 0,45 ≈ 459 / 1024 : le fond #1B1C1E devient #0C0C0C, la couleur du
/// voile (style::veil).
constexpr uint8_t shade(unsigned average) {
    return static_cast<uint8_t>(((average * 459 + 2048) >> 12) << 2);
}

/// area moins hole, en bandes : haut, gauche, droite, bas (vides s'il n'y a rien)
std::array<Area, 4> margins(Area area, Area hole);

/// Lit la fenêtre (3 octets par pixel, ligne après ligne) et rend le tampon qui la contient, nullptr si échec
using Read = uint8_t *(*)(Area window);
/// Réécrit la fenêtre depuis le début du tampon que read a rendu
using Write = void (*)(Area window, const uint8_t *buffer);

/// Givre area moins hole d'une traite : toutes les lectures, puis toutes les écritures. Le flou ne lit que dans sa bande
/// (bords répétés) : il ne prend ni le dialogue, ni une bande voisine. capacity : pixels que le tampon de read tient
/// (au moins 2 lignes de la bande la plus large).
void run(Area area, Area hole, size_t capacity, Read read, Write write);

// ---- Sur l'imprimante ----

/// Givre ce que l'écran montre dans area autour de hole : au premier dessin du dialogue, avant de le peindre
void run(const Rect16 &area, const Rect16 &hole);

/// Un dialogue givré se ferme : l'écran dessous se repeint d'une passe (le fond entre ses éléments, puis chacun) au lieu
/// d'un aplat sur toute la place du dialogue avant ses éléments
void close();

} // namespace od3d::frost
