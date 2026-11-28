// OneDrop3D : textes de l'interface G (titres des menus Prusa, coupe « ... »).
#include <gui/od3d/text_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <string>

namespace {

std::string titled(const char *s, bool nouns = false) {
    char buf[64];
    strcpy(buf, s);
    od3d::text::title_case(buf, nouns);
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
    CHECK(titled("DÉPLACEMENT DE L'AXE Z") == "Déplacement de l'axe Z");
    CHECK(titled("CALIBRATION XY") == "Calibration XY");
    CHECK(titled("AXE X") == "Axe X");
}

TEST_CASE("titre : capitales de Latin étendu-A (tchèque, polonais)") {
    CHECK(titled("VŠECHNY NÁSTROJE") == "Všechny nástroje");
    CHECK(titled("JAZYK A ČAS") == "Jazyk a čas");
    CHECK(titled("ČISTÍTKO TRYSKY") == "Čistítko trysky");
    CHECK(titled("WSZYSTKIE NARZĘDZIA") == "Wszystkie narzędzia");
    CHECK(titled("BŁĄD KOMUNIKACJI") == "Błąd komunikacji");
    CHECK(titled("ZAŻÓŁĆ GĘŚLĄ JAŹŃ ĽŔŤ ŐŰ") == "Zażółć gęślą jaźń ľŕť őű");
    CHECK(titled("AĿ") == "Aŀ"); // Ŀ (U+013F, C4 BF) → ŀ (U+0140, C5 80)
}

TEST_CASE("titre : en allemand, les noms gardent leur capitale") {
    CHECK(titled("PLATINEN INFO", true) == "Platinen Info");
    CHECK(titled("SPRACHE & ZEIT", true) == "Sprache & Zeit");
    CHECK(titled("TEMPERATUR & LÜFTER", true) == "Temperatur & Lüfter");
    CHECK(titled("KALIBRIERUNG DES TÜRSENSORS", true) == "Kalibrierung des Türsensors");
    CHECK(titled("Z-ACHSE BEWEGEN", true) == "Z-Achse bewegen");
    CHECK(titled("ERSTE-SCHICHT-KALIBRIERUNG", true) == "Erste-Schicht-Kalibrierung");
    CHECK(titled("WI-FI EINSTELLUNGEN", true) == "Wi-Fi Einstellungen");
    CHECK(titled("ÜBERSPRINGEN", true) == "Überspringen");
    CHECK(titled("DIE EINSTELLUNGEN", true) == "Die Einstellungen"); // 1er mot : capitale
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
