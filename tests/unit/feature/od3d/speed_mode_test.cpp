// OneDrop3D (retour du 26/11) : modes de vitesse et bridage des ventilateurs en Silencieux.
#include <feature/od3d/speed_mode_od3d.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace od3d::speed_mode;

TEST_CASE("speed_mode: the user's speeds, increasing, within the manual range") {
    CHECK(percent(Mode::quiet) == 80);
    CHECK(percent(Mode::normal) == 100);
    CHECK(percent(Mode::fast) == 130);
    CHECK(percent(Mode::faster) == 160);
    CHECK(percent(Mode::fastest) == 250);
    for (size_t i = 1; i < percents.size(); ++i) {
        CHECK(percents[i] > percents[i - 1]);
    }
    CHECK(percents.back() <= 300); // MI_SPEED, le réglage à la main, va jusqu'à 300 %
}

TEST_CASE("speed_mode: quiet caps the part fan, never raises it") {
    CHECK(print_fan_pwm(255, true) == quiet_print_fan_pwm);
    CHECK(print_fan_pwm(100, true) == 100);
    CHECK(print_fan_pwm(0, true) == 0);
    CHECK(print_fan_pwm(255, false) == 255);
}

TEST_CASE("speed_mode: quiet lowers the chamber fans limit, keeps a lower one") {
    CHECK(chamber_fans_max_pwm(255, true) == quiet_chamber_fans_pwm);
    CHECK(chamber_fans_max_pwm(60, true) == 60);
    CHECK(chamber_fans_max_pwm(255, false) == 255);
}
