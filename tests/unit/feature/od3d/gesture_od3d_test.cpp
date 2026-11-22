/// OneDrop3D (interface G) : toucher, défilement et balayage de retour (gesture_od3d.hpp)
#include <gui/od3d/gesture_od3d.hpp>
#include <catch2/catch_test_macros.hpp>

using od3d::gesture::classify;
using E = GUI_event_t;

TEST_CASE("od3d gesture: a tap stays a tap") {
    CHECK(classify(0, 0, 80, 0) == E::TOUCH_CLICK);
    CHECK(classify(3, -2, 80, 3) == E::TOUCH_CLICK); // clavier numérique : toucher plus tolérant
    CHECK(classify(3, 0, 80, 0) == E::_count); // un toucher un peu glissé n'est rien, surtout pas un retour
}

TEST_CASE("od3d gesture: vertical list scroll is never a back swipe") {
    CHECK(classify(0, 40, 900, 0) == E::TOUCH_SWIPE_DOWN);
    CHECK(classify(20, -40, 200, 0) == E::TOUCH_SWIPE_UP); // défilement un peu de travers
    CHECK(classify(70, 90, 200, 0) == E::_count); // en diagonale : ni défilement ni retour
}

TEST_CASE("od3d gesture: back swipe needs 60 px, mostly horizontal, under 600 ms") {
    CHECK(classify(60, 0, 300, 0) == E::TOUCH_SWIPE_RIGHT);
    CHECK(classify(-120, 20, 250, 0) == E::TOUCH_SWIPE_LEFT);
    CHECK(classify(200, 30, 599, 0) == E::TOUCH_SWIPE_RIGHT);
    CHECK(classify(59, 0, 300, 0) == E::_count); // trop court (Prusa : 10 px suffisaient)
    CHECK(classify(120, 0, 600, 0) == E::_count); // trop lent : un curseur qu'on tire
    CHECK(classify(100, 51, 200, 0) == E::_count); // trop penché
}

TEST_CASE("od3d gesture: the spool sheet is grabbed only by a downward drag") {
    using namespace od3d::gesture::pull;
    CHECK_FALSE(grabs(0, 5)); // encore un toucher (ou un bouton)
    CHECK(grabs(0, 6));
    CHECK(grabs(-5, 6));
    CHECK_FALSE(grabs(6, 6)); // en diagonale : peut-être un balayage de retour
    CHECK_FALSE(grabs(40, 20)); // balayage de côté : il ferme la fiche par lui-même
    CHECK_FALSE(grabs(0, -30)); // vers le haut : rien
}

TEST_CASE("od3d gesture: the pulled sheet follows the finger, closes from a third, else springs back") {
    using namespace od3d::gesture::pull;
    constexpr int16_t height = 206, max = 194; // fiche de bobine (y 114 à 320), bord arrondi de 12 px visible
    CHECK(offset(-20, max) == 0); // jamais au-dessus de sa place
    CHECK(offset(50, max) == 50);
    CHECK(offset(400, max) == max);
    CHECK_FALSE(closes(68, height));
    CHECK(closes(69, height)); // un tiers de 206
    CHECK(closes(max, height));

    // Remontée depuis juste avant le tiers : 4 images à 25 images/s, toujours plus haut, jusqu'à sa place
    int16_t at = 68;
    int frames = 0;
    while (at > 0) {
        const int16_t next = spring(at);
        REQUIRE(next < at);
        at = next;
        ++frames;
    }
    CHECK(frames == 4);
    CHECK(frames * frame_ms <= 160);
    CHECK(frame_ms >= 40); // ≤ 25 images/s
}
