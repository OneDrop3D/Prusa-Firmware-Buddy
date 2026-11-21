// OneDrop3D : métadonnées des G-codes, durées et anneau de focus des écrans d'impression (interface G).
#include <gui/od3d/print_info_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <string>

using namespace od3d::print_info;

TEST_CASE("durée du trancheur") {
    CHECK(parse_duration("2h 14m 3s") == 2 * 3600 + 14 * 60 + 3);
    CHECK(parse_duration("1d 2h 3m 4s") == 86400 + 2 * 3600 + 3 * 60 + 4);
    CHECK(parse_duration("45s") == 45);
    CHECK(parse_duration("") == 0);

    char buf[16];
    format_hm(2 * 3600 + 14 * 60 + 59, buf, sizeof(buf));
    CHECK(std::string(buf) == "2 h 14");
    format_hm(32 * 60, buf, sizeof(buf));
    CHECK(std::string(buf) == "0 h 32");
}

TEST_CASE("métadonnées : grammes, outils utilisés, matières") {
    FileInfo info;
    parse_metadata(info, "filament used [g]", "3.72, 0.00, 1.50, 0.0001");
    parse_metadata(info, "filament used [mm]", "1000.0, 0, 250.5");
    parse_metadata(info, "filament_type", "PLA;PETG;PLA;TPU");
    parse_metadata(info, "estimated printing time (normal mode)", "1h 5m 0s");
    parse_metadata(info, "printer_model", "COREONE");

    CHECK(info.is_used(0));
    CHECK_FALSE(info.is_used(1));
    CHECK(info.is_used(2));
    CHECK_FALSE(info.is_used(3)); // 0,0001 g : la purge seule ne compte pas
    CHECK(info.total_grams() > 5.21f);
    CHECK(info.total_grams() < 5.23f);
    CHECK(info.millimeters > 1250.4f);
    CHECK(std::string(info.types[1].data()) == "PETG");
    CHECK(std::string(info.types[3].data()) == "TPU");
    CHECK(info.seconds == 3900);

    // Plus d'outils que d'emplacements : le reste est ignoré
    FileInfo many;
    parse_metadata(many, "filament used [g]", "1, 1, 1, 1, 1, 1, 1");
    CHECK(many.used == 0b11111);
}

TEST_CASE("anneau de focus : même règle que draw_rounded_rect") {
    // Rayon 4 sur 10 lignes : un pixel coupé sur la première et la dernière ligne seulement
    CHECK(corner_inset(0, 10, 4) == 1);
    CHECK(corner_inset(1, 10, 4) == 0);
    CHECK(corner_inset(5, 10, 4) == 0);
    CHECK(corner_inset(9, 10, 4) == 1);
    // Rayon 14 : retrait maximal en haut, nul au milieu, symétrique
    CHECK(corner_inset(0, 100, 14) == 14 - 6); // 6 colonnes avec x² + 13² < 14²
    CHECK(corner_inset(13, 100, 14) == 0);
    CHECK(corner_inset(99, 100, 14) == corner_inset(0, 100, 14));
}
