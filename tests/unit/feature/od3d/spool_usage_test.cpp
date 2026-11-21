// OneDrop3D (F13) : suivi du poids des bobines — longueur → grammes, reste, alerte de fin, écritures en mémoire.
#include <feature/od3d/spool_usage_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <string>

using namespace od3d::spool_usage;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("densité selon la matière") {
    CHECK_THAT(density("PLA"), WithinAbs(1.24, 1e-6));
    CHECK_THAT(density("PETG"), WithinAbs(1.27, 1e-6));
    CHECK_THAT(density("ASA"), WithinAbs(1.07, 1e-6));
    CHECK_THAT(density("FLEX"), WithinAbs(1.21, 1e-6));
    // Le plus long préfixe : PCTG n'est pas du PC, PPS n'est pas du PP
    CHECK_THAT(density("PCTG"), WithinAbs(1.23, 1e-6));
    CHECK_THAT(density("PC"), WithinAbs(1.20, 1e-6));
    CHECK_THAT(density("PPS"), WithinAbs(1.35, 1e-6));
    CHECK_THAT(density("PP"), WithinAbs(0.90, 1e-6));
    // Variantes et casse
    CHECK_THAT(density("PETG-CF"), WithinAbs(1.27, 1e-6));
    CHECK_THAT(density("pla+"), WithinAbs(1.24, 1e-6));
    // Inconnue ou vide : densité par défaut
    CHECK_THAT(density("WOOD"), WithinAbs(default_density, 1e-6));
    CHECK_THAT(density(""), WithinAbs(default_density, 1e-6));
}

TEST_CASE("longueur vers grammes, filament 1,75 mm") {
    // Section π/4 × 1,75² = 2,4053 mm² ; PLA 1,24 g/cm³ → 2,983 g par mètre
    CHECK_THAT(grams_per_mm(1.24f) * 1000, WithinRel(2.9825, 1e-3));
    CHECK_THAT(grams(1000, 1.24f), WithinRel(2.9825, 1e-3));
    CHECK_THAT(grams(0, 1.24f), WithinAbs(0, 1e-9));
    // Une bobine de 1 kg de PLA fait environ 335 m ; de PETG, environ 327 m
    CHECK_THAT(meters(1000, 1.24f), WithinRel(335.3, 1e-3));
    CHECK_THAT(meters(1000, 1.27f), WithinRel(327.4, 1e-3));
    // Aller-retour
    CHECK_THAT(grams(meters(350, 1.07f) * 1000, 1.07f), WithinRel(350, 1e-4));
}

TEST_CASE("bobine neuve, poids net, consommation") {
    const Record unknown {};
    CHECK_FALSE(is_tracked(unknown));
    CHECK(unknown.alert_g == default_alert_g);

    // Changer la bobine : reste = poids neuf, compteurs remis à zéro, seuil gardé
    Record old = new_spool(unknown, 1000);
    old.alert_g = 80;
    old = consume(old, 300);
    old.last_print_g = 12;
    const Record fresh = new_spool(old, 750);
    CHECK(is_tracked(fresh));
    CHECK(fresh.net_g == 750);
    CHECK(fresh.remaining_g == 750);
    CHECK(fresh.used_g == 0);
    CHECK(fresh.last_print_g == 0);
    CHECK(fresh.alert_g == 80);

    // Consommation : reste et « depuis la pose » ; pas sous zéro ; rien pour une valeur nulle ou négative
    Record r = consume(fresh, 100.5f);
    CHECK_THAT(r.remaining_g, WithinAbs(649.5, 1e-4));
    CHECK_THAT(r.used_g, WithinAbs(100.5, 1e-4));
    r = consume(r, 1000);
    CHECK(r.remaining_g == 0);
    CHECK_THAT(r.used_g, WithinAbs(1100.5, 1e-3));
    CHECK(consume(r, -5) == r);
    CHECK(consume(r, NAN) == r);

    // Matière et couleur seule : reste = min(reste, net) ; une bobine jamais suivie part pleine
    Record half = consume(new_spool(unknown, 1000), 600);
    CHECK(set_net(half, 2000).remaining_g == 400);
    CHECK(set_net(half, 250).remaining_g == 250);
    CHECK(set_net(half, 250).used_g == half.used_g);
    CHECK(set_net(unknown, 500).remaining_g == 500);
    CHECK(set_net(unknown, 500).net_g == 500);
}

TEST_CASE("remplissage et alerte de fin") {
    CHECK(fill_percent(350, 1000) == 35);
    CHECK(fill_percent(1000, 1000) == 100);
    CHECK(fill_percent(1200, 1000) == 100); // pesée au-dessus du nominal
    CHECK(fill_percent(0, 1000) == 0);
    CHECK(fill_percent(-3, 1000) == 0);
    CHECK(fill_percent(500, 0) == 0);
    CHECK(fill_percent(4, 1000) == 0); // 0,4 % arrondi

    CHECK(is_low(50, 50));
    CHECK(is_low(10, 50));
    CHECK_FALSE(is_low(51, 50));

    // Une alerte au passage du seuil, pas avant, pas après
    CHECK(crossed_alert(60, 50, 50));
    CHECK(crossed_alert(60, 0, 50));
    CHECK_FALSE(crossed_alert(60, 55, 50));
    CHECK_FALSE(crossed_alert(40, 30, 50));
    CHECK_FALSE(crossed_alert(50, 30, 50));
    // Seuil à zéro : l'alerte vient quand la bobine est vide
    CHECK(crossed_alert(3, 0, 0));
}

TEST_CASE("assez de filament pour le fichier") {
    CHECK(not_enough(120, 80));
    CHECK_FALSE(not_enough(80, 80));
    CHECK_FALSE(not_enough(12, 350));
    // Valeur inconnue : pas d'alerte
    CHECK_FALSE(not_enough(NAN, 80));
    CHECK_FALSE(not_enough(120, NAN));
}

TEST_CASE("écritures en mémoire avec parcimonie") {
    // Trop peu : jamais
    CHECK_FALSE(should_commit(0, false, false, 1'000'000));
    CHECK_FALSE(should_commit(0.05f, false, false, 1'000'000));
    // Beaucoup : toujours (coupure de courant)
    CHECK(should_commit(flush_g, true, true, 0));
    // En impression, l'outil actif attend la fin ou flush_g
    CHECK_FALSE(should_commit(20, true, true, 1'000'000));
    // Changement d'outil : l'ancien outil est écrit s'il a assez consommé
    CHECK(should_commit(tool_change_g, true, false, 0));
    CHECK_FALSE(should_commit(tool_change_g - 0.5f, true, false, 1'000'000));
    // Hors impression : quand l'extrusion est arrêtée depuis idle_ms
    CHECK_FALSE(should_commit(2, false, true, idle_ms - 1));
    CHECK(should_commit(2, false, true, idle_ms));
    CHECK(should_commit(0.2f, false, false, idle_ms));
}

TEST_CASE("poids affichés") {
    char buf[16];
    format_weight(buf, sizeof(buf), 250);
    CHECK(std::string(buf) == "250 g");
    format_weight(buf, sizeof(buf), 1000);
    CHECK(std::string(buf) == "1 kg");
    format_weight(buf, sizeof(buf), 2000);
    CHECK(std::string(buf) == "2 kg");
    format_weight(buf, sizeof(buf), 1250);
    CHECK(std::string(buf) == "1250 g");
}
