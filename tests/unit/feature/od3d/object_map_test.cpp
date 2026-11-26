// OneDrop3D : plan du plateau de « Annuler des objets » : place suivie par les extrusions, mise à l'échelle, objet touché
#include <feature/od3d/object_map_math_od3d.hpp>

#include <array>
#include <catch2/catch_test_macros.hpp>

using namespace od3d::object_map;

TEST_CASE("object_map: la place grandit avec les extrusions") {
    uint32_t b = grow(unknown, 100, 50, 110, 50); // un premier tracé, de 100 à 110 mm en X
    CHECK(unpack(b).x0 == 50);
    CHECK(unpack(b).x1 == 55);
    CHECK(unpack(b).y0 == 25);
    CHECK(unpack(b).y1 == 25);

    b = grow(b, 104, 70, 104, 40); // dedans en X, plus loin des deux côtés en Y
    CHECK(unpack(b).x0 == 50);
    CHECK(unpack(b).x1 == 55);
    CHECK(unpack(b).y0 == 20);
    CHECK(unpack(b).y1 == 35);
    CHECK(grow(b, 102, 30, 106, 30) == grow(b, 106, 30, 102, 30)); // le sens du tracé n'y change rien

    // Hors du plateau : bornée à 0 et 510 mm ; une vraie place n'est jamais « inconnue »
    CHECK(unpack(grow(unknown, -5, -3, 600, 2)).x0 == 0);
    CHECK(unpack(grow(unknown, -5, -3, 600, 2)).x1 == 255);
    CHECK(grow(unknown, 10, 10, 10, 10) != unknown);
}

TEST_CASE("object_map: plan à l'échelle, l'avant en bas") {
    // CORE One : 250 × 220 mm sur 250 × 220 px
    CHECK(to_screen({ 0, 0, 25, 10 }, 250, 220, 250, 220, 8) == Px { 0, 200, 50, 20 });
    CHECK(to_screen({ 0, 0, 25, 10 }, 500, 440, 250, 220, 8) == Px { 0, 400, 100, 40 });
    // Un petit objet a au moins 8 px, centré
    CHECK(to_screen({ 100, 50, 100, 50 }, 250, 220, 250, 220, 8) == Px { 196, 116, 8, 8 });
    // Contre le bord, il reste dans le plan
    CHECK(to_screen({ 124, 0, 125, 1 }, 250, 220, 250, 220, 8) == Px { 242, 212, 8, 8 });
}

TEST_CASE("object_map: objet touché") {
    const std::array<Px, 4> rects { {
        { 0, 0, 100, 100 }, // grand
        { 40, 40, 20, 20 }, // posé dans le grand
        {}, // pas encore imprimé
        { 200, 0, 10, 10 }, // petit, à l'écart
    } };
    CHECK(hit(rects, 50, 50, 12) == 1); // le plus petit qui contient le point
    CHECK(hit(rects, 10, 10, 12) == 0);
    CHECK(hit(rects, 215, 5, 12) == 3); // à 6 px du petit : touché quand même
    CHECK(hit(rects, 105, 50, 12) == 0); // à côté du grand
    CHECK(hit(rects, 300, 300, 12) == -1);
}
