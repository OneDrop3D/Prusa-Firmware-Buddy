// OneDrop3D : métadonnées des G-codes, durées et anneau de focus des écrans d'impression (interface G).
#include <gui/od3d/print_info_od3d.hpp>

#include <algorithm>
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

TEST_CASE("métadonnées : température de buse par outil (F6)") {
    FileInfo info;
    CHECK(info.nozzle[0] == 0); // absente : pas d'alerte
    parse_metadata(info, "first_layer_temperature", "230,250");
    CHECK(info.nozzle[0] == 0);
    parse_metadata(info, "temperature", "215,240,0,220");
    CHECK(info.nozzle[0] == 215);
    CHECK(info.nozzle[1] == 240);
    CHECK(info.nozzle[2] == 0);
    CHECK(info.nozzle[3] == 220);
    CHECK(info.nozzle[4] == 0);
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

TEST_CASE("frise de l'écran Impression : un pas ne repeint que les colonnes qui changent") {
    constexpr int w = 464, h = 20;
    // Tout pixel hors des colonnes annoncées est le même avant et après
    const auto bytes = [&](int from, int to) {
        const auto cols = strip_changed(w, from, to);
        REQUIRE(cols.first >= 0);
        REQUIRE(cols.second <= w);
        for (int x = 0; x < w; ++x) {
            if (x >= cols.first && x < cols.second) {
                continue;
            }
            for (int y = 0; y < h; ++y) {
                INFO("de " << from << " à " << to << " %, x " << x << " y " << y);
                REQUIRE(strip_part(x, y, w, h, from) == strip_part(x, y, w, h, to));
            }
        }
        return (cols.second - cols.first) * h * 3; // octets envoyés à l'écran (RGB666)
    };
    int worst = 0;
    for (int p = 0; p < 100; ++p) {
        worst = std::max(worst, bytes(p, p + 1));
    }
    CHECK(worst < 1024); // ~0,8 Ko au lieu de 27,8 Ko pour la frise entière
    bytes(0, 100);
    bytes(100, 0); // réimpression : la frise revient au début
    bytes(37, 52);

    // Les parties : piste au début, orange à moitié, curseur au bout de l'orange
    CHECK(strip_part(w / 2, 10, w, h, 0) == StripPart::track);
    CHECK(strip_part(w / 4, 10, w, h, 50) == StripPart::done);
    CHECK(strip_part(w / 2, 10, w, h, 50) == StripPart::cursor);
    CHECK(strip_part(w / 2, 0, w, h, 0) == StripPart::outside); // au-dessus de la piste
}

TEST_CASE("od3d files: the folder window is borrowed before the thumbnails (P-04)") {
    constexpr size_t reserve = 32 * 1024, folder = 5200, infos = 1348, overhead = 560;

    // 45 000 o libres : l'ancien ordre prenait métadonnées et 8 Ko de vignettes, puis plus de place pour la fenêtre du
    // dossier (relu tous les deux crans) ; maintenant la fenêtre d'abord, et les vignettes avec ce qui reste
    CachePlan plan = plan_caches(45000, reserve, folder, infos, overhead);
    CHECK(plan.folder);
    CHECK(plan.infos);
    CHECK(plan.thumbs == 4 * 1024);

    plan = plan_caches(60000, reserve, folder, infos, overhead);
    CHECK((plan.folder && plan.infos && plan.thumbs == 16 * 1024));

    plan = plan_caches(48100, reserve, folder, infos, overhead); // 8 Ko dès 48 068 o
    CHECK((plan.folder && plan.infos && plan.thumbs == 8 * 1024));

    // Juste la fenêtre : les métadonnées ne laisseraient plus la réserve
    plan = plan_caches(folder + reserve, reserve, folder, infos, overhead);
    CHECK((plan.folder && !plan.infos && plan.thumbs == 0));

    // Rien sous la réserve : le réseau garde ses 32 Ko
    plan = plan_caches(reserve + 1000, reserve, folder, infos, overhead);
    CHECK((!plan.folder && !plan.infos && plan.thumbs == 0));

    // Déjà pris (taille 0) : seulement ce qui manque, au repos après un envoi
    plan = plan_caches(52000, reserve, 0, 0, overhead);
    CHECK((!plan.folder && !plan.infos && plan.thumbs == 16 * 1024));
    plan = plan_caches(60000, reserve, folder, 0, 0);
    CHECK((plan.folder && !plan.infos && plan.thumbs == 0));
}

TEST_CASE("métadonnées : G-code mono-outil ou multi-matériaux (préréglage d'imprimante de PrusaSlicer)") {
    FileInfo single; // préréglage sans MMU3 : un seul outil listé
    parse_metadata(single, "filament used [g]", "12.30");
    CHECK(single.single_tool());

    FileInfo slot3; // préréglage MMU3, objet sur l'extrudeur 3 : les cinq listés
    parse_metadata(slot3, "filament used [g]", "0.00, 0.00, 12.30, 0.00, 0.00");
    CHECK_FALSE(slot3.single_tool());
    FileInfo slot1;
    parse_metadata(slot1, "filament used [g]", "12.30, 0.00, 0.00, 0.00, 0.00");
    CHECK_FALSE(slot1.single_tool());
    CHECK_FALSE(FileInfo {}.single_tool()); // métadonnées absentes
}
