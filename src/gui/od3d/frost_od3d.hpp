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
#include <utils/color.hpp>

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

/// area moins hole, en bandes : haut, gauche, droite, bas (vides s'il n'y a rien). Les bandes du haut et du bas
/// descendent (remontent) de top_rows (bottom_rows) lignes dans le trou, pour ses coins arrondis.
std::array<Area, 4> margins(Area area, Area hole, uint16_t top_rows = 0, uint16_t bottom_rows = 0);

/// Lit la fenêtre (3 octets par pixel, ligne après ligne) et rend le tampon qui la contient, nullptr si échec
using Read = uint8_t *(*)(Area window);
/// Réécrit la fenêtre depuis le début du tampon que read a rendu
using Write = void (*)(Area window, const uint8_t *buffer);

/// Coins arrondis du trou (le dialogue), bits de mask. Retour du 22/11 (8.0.2.30) : « un carré noir sous la fenêtre,
/// sans les angles arrondis, qui dépasse » : le trou était givré en rectangle et le dialogue peignait ses coins du voile
/// uni, plus sombre que l'écran givré dès qu'il y avait autre chose que le fond derrière eux. Maintenant le givrage
/// fait aussi les coins : givré hors de l'arrondi, mêlé à body (couleur du corps, octets de l'afficheur) sur son bord
/// lissé (comme display::draw_rounded_rect), body dedans ; le dialogue peint son corps sans ses coins (draw_body).
struct Corners {
    enum : uint8_t {
        top_left = 1,
        top_right = 2,
        bottom_left = 4,
        bottom_right = 8,
    };
    uint8_t r = 0;
    uint8_t mask = 0;
    std::array<uint8_t, 3> body {};
};

/// Givre area moins hole d'une traite : toutes les lectures, puis toutes les écritures. Le flou ne lit que dans sa bande
/// (bords répétés) : il ne prend ni une bande voisine, ni le dialogue hors des lignes de ses coins. capacity : pixels que
/// le tampon de read tient (au moins 2 lignes de la bande la plus large).
void run(Area area, Area hole, size_t capacity, Read read, Write write, const Corners &corners = {});

// ---- Sur l'imprimante ----

/// Givre ce que l'écran montre dans area autour de hole, coins arrondis de hole compris (rayon r, corners en MIC_*, corps
/// body) : au premier dessin du dialogue, avant de le peindre, et quand l'écran a été repeint dessous (!frosted())
void run(const Rect16 &area, const Rect16 &hole, uint8_t r = 0, uint8_t corners = 0, Color body = {});

/// L'écran autour du dialogue est encore givré : rien ne l'a repeint en entier depuis run (Screens::Draw appelle forget)
bool frosted();

/// Corps d'un dialogue givré : comme display::draw_rounded_rect(rect, voile, body, r, corners), sans repeindre les
/// coins que le givrage a faits (ils gardent l'écran givré au lieu d'un carré de voile uni)
void draw_body(const Rect16 &rect, Color body, uint8_t r, uint8_t corners);

/// Les coins faits par le givrage ont été recouverts (fiche tirée vers le bas) : draw_body les peint du voile
void forget_corners();

/// L'écran est repeint en entier (sortie de veille, afficheur réinitialisé, dialogue refermé...) : le givre sous un dialogue
/// est perdu, le dialogue regivre à son dessin (un fond uni devient le voile uni)
void forget();

/// Un dialogue givré se ferme : l'écran dessous se repeint d'une passe (le fond entre ses éléments, puis chacun) au lieu
/// d'un aplat sur toute la place du dialogue avant ses éléments
void close();

} // namespace od3d::frost
