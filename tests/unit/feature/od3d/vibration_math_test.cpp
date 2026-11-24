// OneDrop3D : accélération max utile (seuil de remontée du résidu) et vibrations des ventilateurs (pic à la fréquence
// de rotation, comparaison à la référence), sur des signaux synthétiques.
#include <feature/od3d/vibration_math_od3d.hpp>
#include <feature/od3d/calib_registry_od3d.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <numbers>

using namespace od3d::vibration;
using Catch::Approx;

namespace {

constexpr float pi = std::numbers::pi_v<float>;
constexpr float rate = 1344.f; ///< échantillons par seconde de l'accéléromètre

/// Résidu après l'arrêt : sinusoïde amortie d'amplitude amp (m/s²) à f Hz, pesanteur sur z, bruit pseudo-aléatoire ±noise
float residual_of(float amp, float f, float noise, uint32_t seed = 1) {
    WindowMeter m;
    for (int i = 0; i < 336; ++i) { // 250 ms
        const float t = float(i) / rate;
        seed = seed * 1103515245u + 12345u;
        const float r = noise * (float(seed >> 16 & 0x7fff) / 16384.f - 1.f);
        m.add(amp * std::exp(-0.05f * 2.f * pi * f * t) * std::sin(2.f * pi * f * t) + r, 0.3f * r, 9.81f - r);
    }
    return m.variance();
}

std::array<uint16_t, accel_levels> levels() {
    std::array<uint16_t, accel_levels> a;
    for (size_t i = 0; i < a.size(); ++i) {
        a[i] = level_accel(i);
    }
    return a;
}

/// Magnitudes des bins pour une sinusoïde de f Hz et d'amplitude amp, f0 lu au tachymètre, bruit de fond floor
std::array<float, fan_bins> magnitudes(float f, float amp, float f0, float floor) {
    // Transformée en un point (comme FourierSeries3d de Prusa) sur 1 s
    std::array<float, fan_bins> out;
    const auto freq = bin_frequencies(f0);
    for (size_t k = 0; k < fan_bins; ++k) {
        double s = 0, c = 0;
        const int n = int(rate);
        for (int i = 0; i < n; ++i) {
            const float t = float(i) / rate;
            const float x = amp * std::sin(2.f * pi * f * t);
            s += x * std::sin(2.f * pi * freq[k] * t);
            c += x * std::cos(2.f * pi * freq[k] * t);
        }
        out[k] = float(2.0 * std::sqrt(s * s + c * c) / n) + floor;
    }
    return out;
}

} // namespace

TEST_CASE("vibration: window variance ignores gravity") {
    WindowMeter m;
    CHECK(m.variance() == 0.f);
    for (int i = 0; i < 1000; ++i) {
        const float s = std::sin(2.f * pi * 50.f * float(i) / rate);
        m.add(2.f * s, 0.f, 9.81f); // sinusoïde d'amplitude 2 : variance 2
    }
    CHECK(m.variance() == Approx(2.f).epsilon(0.02));
}

TEST_CASE("vibration: linear machine, no rise up to the top") {
    // Résidu proportionnel à a : la limite est le dernier palier, sans remontée
    std::array<float, accel_levels> r;
    for (size_t i = 0; i < r.size(); ++i) {
        r[i] = residual_of(level_accel(i) * 2e-5f, 48.f, 0.01f, uint32_t(i + 1));
    }
    const float noise = residual_of(0.f, 48.f, 0.01f, 99);
    const Limit l = find_limit(r, levels(), noise);
    CHECK(l.found);
    CHECK_FALSE(l.rose);
    CHECK(l.reference == 0);
    CHECK(l.last_ok == accel_levels - 1);
}

TEST_CASE("vibration: residual rises above 7000 mm/s2") {
    std::array<float, accel_levels> r;
    for (size_t i = 0; i < r.size(); ++i) {
        const float a = level_accel(i);
        const float gain = a > 7000.f ? 2.f : 1.f; // ne suit plus : résidu par unité d'accélération doublé
        r[i] = residual_of(a * 2e-5f * gain, 48.f, 0.01f, uint32_t(i + 1));
    }
    const Limit l = find_limit(r, levels(), residual_of(0.f, 48.f, 0.01f, 99));
    CHECK(l.found);
    CHECK(l.rose);
    CHECK(level_accel(l.last_ok) == 7000);
}

TEST_CASE("vibration: reference is the lowest level out of the noise") {
    // Bruit au repos fort : les premiers paliers s'y perdent, la référence monte ; le bruit est retiré du résidu
    std::array<float, accel_levels> r;
    const float noise = residual_of(0.f, 48.f, 0.1f, 99);
    for (size_t i = 0; i < r.size(); ++i) {
        r[i] = residual_of(level_accel(i) * 1e-4f, 48.f, 0.1f, uint32_t(i + 1));
    }
    const Limit l = find_limit(r, levels(), noise);
    CHECK(l.found);
    CHECK(l.reference > 0);
    CHECK_FALSE(l.rose);
}

TEST_CASE("vibration: all in the noise or nothing measured is not conclusive") {
    std::array<float, accel_levels> r;
    r.fill(0.001f);
    CHECK_FALSE(find_limit(r, levels(), 0.001f).found);
    r.fill(NAN);
    CHECK_FALSE(find_limit(r, levels(), 0.f).found);
    // Mesure arrêtée au 4e palier : la limite ne va pas plus loin
    r = { 1.f, 4.f, 8.f, NAN, NAN, NAN, NAN, NAN, NAN };
    const Limit l = find_limit(r, levels(), 0.f);
    CHECK(l.found);
    CHECK_FALSE(l.rose);
    CHECK(l.last_ok == 2);
}

TEST_CASE("vibration: fan peak at the rotation speed") {
    // Tachymètre à 6 000 tr/min (100 Hz), rotation réelle à 101,3 Hz : dans la fenêtre du pic
    const Tone t = tone(magnitudes(101.3f, 0.05f, 100.f, 0.002f));
    CHECK(t.detected);
    CHECK(t.amplitude == Approx(0.05f + 0.002f).epsilon(0.1));
    CHECK(t.noise < 0.01f);

    // Rien à la fréquence de rotation (ventilateur de la chambre, loin de la buse) : non capté
    const Tone none = tone(magnitudes(60.f, 0.05f, 140.f, 0.002f));
    CHECK_FALSE(none.detected);

    // Pic au niveau du bruit voisin : non capté
    const Tone weak = tone(magnitudes(100.f, 0.002f, 100.f, 0.004f));
    CHECK_FALSE(weak.detected);
}

TEST_CASE("vibration: tone bank, a phasor per bin, matches the direct transform") {
    // Une seconde de signal sur les trois axes : pic du ventilateur sur x, autres raies sur y et z
    const auto freq = bin_frequencies(100.f);
    const auto signal = [](int axis, double t) {
        const double w = 2.0 * std::numbers::pi * t;
        return float(axis == 0 ? 0.05 * std::sin(w * 101.3) : axis == 1 ? 0.02 * std::cos(w * 80.0 + 0.4) : 0.01 * std::sin(w * 117.0));
    };
    const int n = int(rate);
    ToneBank bank;
    CHECK(bank.magnitudes()[0] == 0.f); // sans échantillon
    bank.start(freq, 1.f / rate);
    for (int i = 0; i < n; ++i) {
        const double t = i / double(rate);
        bank.add(signal(0, t), signal(1, t), signal(2, t));
    }
    const auto got = bank.magnitudes();
    for (size_t k = 0; k < fan_bins; ++k) {
        double sum = 0;
        for (int axis = 0; axis < 3; ++axis) {
            double re = 0, im = 0;
            for (int i = 0; i < n; ++i) {
                const double t = i / double(rate);
                re += signal(axis, t) * std::cos(2.0 * std::numbers::pi * freq[k] * t);
                im += signal(axis, t) * std::sin(2.0 * std::numbers::pi * freq[k] * t);
            }
            sum += re * re + im * im;
        }
        CHECK(got[k] == Approx(2.0 * std::sqrt(sum) / n).epsilon(0.002).margin(1e-5));
    }
    CHECK(tone(got).detected);

    // Une nouvelle fenêtre repart de zéro
    bank.start(freq, 1.f / rate);
    bank.add(0.f, 0.f, 0.f);
    CHECK(bank.magnitudes()[peak_bins / 2] == 0.f);
}

TEST_CASE("vibration: fan compared with its reference") {
    const std::array<Tone, 3> first { Tone { 0.02f, 0.002f, true }, Tone { 0.03f, 0.002f, true }, Tone { 0.05f, 0.002f, true } };
    const FanLevel ref = fan_level(first);
    CHECK(ref.detected);
    CHECK(ref.value == Approx(std::sqrt((0.0004f + 0.0009f + 0.0025f) / 3.f)));

    // Sans référence : la première mesure captée le devient ; une valeur seule ne dit rien
    CHECK(compare(ref, std::nullopt) == Verdict::reference);

    // Gardée sur 7 bits, relue à 9 % près
    const uint8_t code = encode(ref.value);
    REQUIRE(decode(code));
    CHECK(*decode(code) == Approx(ref.value).epsilon(0.05));

    // Un peu plus haut : normal ; deux fois plus : balourd ou usure
    CHECK(compare(FanLevel { ref.value * 1.3f, true }, decode(code)) == Verdict::ok);
    CHECK(compare(FanLevel { ref.value * 2.5f, true }, decode(code)) == Verdict::worse);
    // Non capté : ni comparaison ni nouvelle référence
    CHECK(compare(FanLevel { ref.value * 3.f, false }, decode(code)) == Verdict::not_detected);
}

TEST_CASE("vibration: reference codes packed in the calibration values") {
    CHECK(encode(0.f) == 0);
    CHECK(encode(NAN) == 0);
    CHECK_FALSE(decode(0));
    CHECK(encode(1e-6f) == 1); // sous le bas de l'échelle : le plus petit code
    CHECK(encode(1e6f) == 127);

    const int16_t p = pack(encode(0.05f), encode(30.f));
    CHECK(p != od3d::calib_registry::no_value);
    CHECK(unpack(p, false) == encode(0.05f));
    CHECK(unpack(p, true) == encode(30.f));

    // Config_store neuve (no_value) : aucune référence
    CHECK(unpack(od3d::calib_registry::no_value, false) == 0);
    CHECK(unpack(od3d::calib_registry::no_value, true) == 0);

    // Version 1 du test : [2] gardait le régime (5 500 tr/min), [3] jamais écrit : pas pris pour des références
    CHECK(unpack_references(5500, od3d::calib_registry::no_value) == std::array<uint8_t, 4> {});
    // Écrites par M2001 V : [3] jamais no_value, même sans ventilateur de chambre
    const int16_t chamber = pack(0, 0);
    CHECK(chamber != od3d::calib_registry::no_value);
    CHECK(unpack_references(p, chamber) == std::array<uint8_t, 4> { encode(0.05f), encode(30.f), 0, 0 });
}

TEST_CASE("od3d accel mount: tilt of the gravity vector, either way up") {
    CHECK(tilt_degrees(0.f, 0.f, 9.81f) == Approx(0.f).margin(0.01f));
    CHECK(tilt_degrees(0.f, 0.f, -9.81f) == Approx(0.f).margin(0.01f)); // à l'envers : même inclinaison
    const float a = 15.f * pi / 180.f;
    CHECK(tilt_degrees(9.81f * std::sin(a), 0.f, 9.81f * std::cos(a)) == Approx(15.f).margin(0.05f));
    CHECK(tilt_degrees(9.81f, 0.f, 0.f) == Approx(90.f).margin(0.05f));
    CHECK(std::isnan(tilt_degrees(0.f, 0.f, 0.f)));
}

TEST_CASE("od3d accel mount: share of the horizontal plane, whatever the turn in it") {
    const std::array<float, 3> x_move { 8.f, 1.f, 1.f };
    CHECK(plane_share(x_move) == Approx(0.9f));
    // Le même mouvement vu par un accéléromètre tourné d'un quart de tour (photo du 24/11) : même verdict
    const std::array<float, 3> turned { 1.f, 8.f, 1.f };
    CHECK(plane_share(turned) == Approx(plane_share(x_move)));
    const std::array<float, 3> wobbling { 2.f, 1.f, 6.f }; // il bouge sur la buse : surtout en Z
    CHECK(plane_share(wobbling) == Approx(1.f / 3.f));
    const std::array<float, 3> zero { 0.f, 0.f, 0.f };
    CHECK(std::isnan(plane_share(zero)));
}

TEST_CASE("od3d accel mount: verdict, most serious first") {
    CHECK(mount_verdict(2.f, 0.8f, 0.85f) == Mount::ok);
    CHECK(mount_verdict(12.f, 0.8f, 0.85f) == Mount::tilted);
    CHECK(mount_verdict(12.f, 0.3f, 0.85f) == Mount::tilted);
    CHECK(mount_verdict(NAN, 0.8f, 0.85f) == Mount::tilted);
    CHECK(mount_verdict(3.f, 0.5f, 0.85f) == Mount::loose);
    CHECK(mount_verdict(3.f, 0.8f, NAN) == Mount::loose);
}

TEST_CASE("od3d shaper check: reduction with the noise removed") {
    // Bruit 1, sans filtre 11, avec 3 : 1 − 2/10 = 80 % retirés
    CHECK(shaper_reduction(3.f, 11.f, 1.f) == Approx(0.8f));
    // Avec le filtre dans le bruit : tout retiré
    CHECK(shaper_reduction(0.5f, 11.f, 1.f) == Approx(1.f));
    // Le filtre empire : borné à −100 %
    CHECK(shaper_reduction(40.f, 11.f, 1.f) == Approx(-1.f));
    // Même sans filtre dans le bruit : rien à juger
    CHECK(std::isnan(shaper_reduction(1.f, 1.5f, 1.f)));
    CHECK(std::isnan(shaper_reduction(NAN, 11.f, 1.f)));
}

TEST_CASE("od3d shaper check: verdict per axis and overall") {
    CHECK(shaper_check(true, 0.8f) == ShaperCheck::ok);
    CHECK(shaper_check(true, 0.1f) == ShaperCheck::weak);
    CHECK(shaper_check(true, NAN) == ShaperCheck::quiet);
    CHECK(shaper_check(false, 0.8f) == ShaperCheck::no_filter);

    const std::array both_ok { ShaperCheck::ok, ShaperCheck::quiet };
    CHECK(shaper_check_passed(both_ok));
    const std::array one_weak { ShaperCheck::ok, ShaperCheck::weak };
    CHECK_FALSE(shaper_check_passed(one_weak));
    const std::array no_filter { ShaperCheck::no_filter, ShaperCheck::no_filter };
    CHECK_FALSE(shaper_check_passed(no_filter)); // l'input shaper d'abord
    const std::array y_only { ShaperCheck::no_filter, ShaperCheck::ok };
    CHECK(shaper_check_passed(y_only));
}
