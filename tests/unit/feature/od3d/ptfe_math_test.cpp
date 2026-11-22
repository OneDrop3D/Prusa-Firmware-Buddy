// OneDrop3D (F41) : longueur du tube PTFE avant d'après la position de la poulie quand le capteur voit le filament.
#include <feature/od3d/ptfe_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace od3d::ptfe;

TEST_CASE("tube PTFE : marge de 20 mm, ou 5 % au-delà de 400 mm") {
    CHECK(margin_mm(360) == 20);
    CHECK(margin_mm(400) == 20);
    CHECK(margin_mm(600) == 30);
    CHECK(margin_mm(1000) == 50);
}

TEST_CASE("tube PTFE : longueur sous la distance mesurée") {
    CHECK(bowden_length(452) == 430); // 452 - 22
    CHECK(bowden_length(380) == 360); // la valeur d'origine de Prusa pour 380 mm mesurés
    CHECK(bowden_length(800) == 760);
    // Toujours strictement sous la distance mesurée tant qu'elle dépasse le minimum
    for (uint16_t d = min_mm + 20; d <= max_mm; ++d) {
        const auto len = bowden_length(d);
        REQUIRE(len.has_value());
        CHECK(*len < d);
        CHECK(*len >= min_mm);
    }
}

TEST_CASE("tube PTFE : bornes du MMU") {
    // Juste au-delà de l'avance rapide minimale : la marge ferait descendre sous 341, on garde le minimum
    CHECK(bowden_length(350) == min_mm);
    // Relevé un peu en retard alors que le capteur a vu le filament au début de l'avance lente
    CHECK(bowden_length(min_mm - late_sample_mm) == min_mm);
    CHECK(bowden_length(max_mm) == max_mm - margin_mm(max_mm));
}

TEST_CASE("tube PTFE : relevés invraisemblables") {
    // Capteur vu pendant l'avance rapide : tube plus court que le minimum du MMU
    CHECK_FALSE(bowden_length(min_mm - late_sample_mm - 1).has_value());
    CHECK_FALSE(bowden_length(0).has_value());
    // Position au-delà de ce que le MMU pousse avant d'abandonner
    CHECK_FALSE(bowden_length(max_mm + 1).has_value());
    CHECK_FALSE(bowden_length(65535).has_value()); // position négative relue en uint16
    // Mesure faite avec une autre avance rapide
    CHECK_FALSE(bowden_length(380, 400).has_value());
    CHECK(bowden_length(420, 400) == 399);
}
