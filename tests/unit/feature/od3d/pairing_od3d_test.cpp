// OneDrop3D : association de PrintForge par un code à 6 chiffres.
#include <feature/od3d/pairing_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <string>
#include <random/random.h>
#include <timing.h> // déclarent ticks_ms et rand_u avec leur linkage : les doublures ci-dessous le reprennent

static uint32_t maintenant_ms = 0;
uint32_t ticks_ms() { return maintenant_ms; }

static uint32_t tirage = 12345;
uint32_t rand_u() { return tirage = tirage * 1103515245u + 12345u; }

static std::string texte(uint32_t code) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%06u", static_cast<unsigned>(code));
    return buf;
}

using namespace od3d::pairing;

TEST_CASE("association : le bon code ne sert qu'une fois") {
    const uint32_t code = start();
    CHECK(code >= 100000);
    CHECK(code <= 999999);
    CHECK(check(texte(code)));
    CHECK_FALSE(check(texte(code)));
}

TEST_CASE("association : rien sans fenêtre ouverte, ni après stop") {
    stop();
    CHECK_FALSE(check("123456"));
    const uint32_t code = start();
    stop();
    CHECK_FALSE(check(texte(code)));
}

TEST_CASE("association : la fenêtre expire au bout de 5 minutes") {
    maintenant_ms = 1000;
    const uint32_t code = start();
    maintenant_ms += 5 * 60 * 1000 + 1;
    CHECK_FALSE(check(texte(code)));
}

TEST_CASE("association : 5 mauvais codes ferment la fenêtre") {
    maintenant_ms = 0;
    const uint32_t code = start();
    const std::string faux = texte(code == 999999 ? 100000 : code + 1);
    for (int i = 0; i < 5; ++i) {
        CHECK_FALSE(check(faux));
    }
    CHECK_FALSE(check(texte(code)));
}

TEST_CASE("association : refuse ce qui n'est pas 6 chiffres") {
    const uint32_t code = start();
    CHECK_FALSE(check(texte(code).substr(0, 5)));
    CHECK_FALSE(check("12a456"));
    CHECK_FALSE(check(texte(code) + "0"));
    stop();
}
