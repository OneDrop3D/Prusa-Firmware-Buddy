// OneDrop3D : textes de l'interface G (titres des menus Prusa, coupe « ... »).
#include <gui/od3d/text_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <string>

namespace {

std::string titled(const char *s) {
    char buf[64];
    strcpy(buf, s);
    od3d::text::title_case(buf);
    return buf;
}

std::string cut(const char *s, size_t n) {
    char buf[64];
    strcpy(buf, s);
    od3d::text::ellipsize(buf, n);
    return buf;
}

} // namespace

TEST_CASE("titre : capitales Prusa → casse de la maquette") {
    CHECK(titled("RÉSEAU") == "Réseau");
    CHECK(titled("ÉTAT DU RÉSEAU") == "État du réseau");
    CHECK(titled("LANGUE & HEURE") == "Langue & heure");
    CHECK(titled("TÊTE D'IMPRESSION") == "Tête d'impression");
    CHECK(titled("RÉGLAGES WI-FI") == "Réglages Wi-Fi");
    CHECK(titled("PRUSA CONNECT") == "Prusa Connect");
    CHECK(titled("PRUSALINK") == "PrusaLink");
    CHECK(titled("MMU") == "MMU");
    CHECK(titled("SYSTÈME") == "Système");
    CHECK(titled("INFOS VERSION") == "Infos version");
}

TEST_CASE("titre : déjà en casse mixte, inchangé") {
    CHECK(titled("Fonctions OneDrop3D") == "Fonctions OneDrop3D");
    CHECK(titled("Températures") == "Températures");
    CHECK(titled("") == "");
}

TEST_CASE("coupe : n caractères affichés, « ... » compris") {
    CHECK(cut("Interface utilisateur", 21) == "Interface utilisateur");
    CHECK(cut("Interface utilisateur", 19) == "Interface utilis...");
    CHECK(cut("Réglages Wi-Fi", 8) == "Régla...");
    CHECK(cut("abc", 2) == "");
}
