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
