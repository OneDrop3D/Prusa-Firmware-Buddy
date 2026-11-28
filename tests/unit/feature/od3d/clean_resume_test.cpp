// OneDrop3D (F30) : reprise sans cicatrice dans le remplissage. Marqueurs « ;TYPE:… », suivi des tracés de
// remplissage, choix du point de purge et du cordon, quantités de filament, replis vers la reprise de Prusa.
#include <feature/od3d/clean_resume_math_od3d.hpp>
#include <feature/od3d/feature_marks_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstdint>
#include <string>

using namespace od3d::clean_resume;
using Catch::Matchers::WithinAbs;

namespace {

// CORE One : X -1 à 252, Y -19 à 220
constexpr Box core_one { -1, 252, -19, 220 };
constexpr Settings on { .enabled = true, .purge_tenths = 10, .retract_tenths = 8 };
constexpr float epm = 0.04f; ///< filament par mm de cordon (0,45 × 0,2 mm, filament de 1,75 mm)

/// Un tracé qui extrude de a à b à la hauteur z ; rend E à la fin
float line(Tracker &t, Point a, Point b, float z, float e) {
    const float e1 = e + epm * std::hypot(b.x - a.x, b.y - a.y);
    t.move(a, z, e, b, z, e1);
    return e1;
}

/// Une couche sans remplissage (périmètre) à la hauteur z
void layer(Tracker &t, float z, float &e) {
    t.set_feature(od3d::feature_marks::other);
    e = line(t, { 10, 10 }, { 50, 10 }, z, e);
}

int parse(const char *after_semicolon) {
    od3d::feature_marks::LineParser p;
    for (const char *c = after_semicolon; *c; ++c) {
        p.feed(*c);
    }
    return p.finish();
}

float length(Point a, Point b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

} // namespace

TEST_CASE("reprise : réglages", "[od3d][resume]") {
    const Settings def {};
    CHECK_FALSE(def.enabled); // coupée par défaut
    CHECK(def.purge_tenths == 10); // 1 mm de filament, environ 2,4 mm³
    CHECK(def.retract_tenths == 8);
    const Settings wild { .enabled = true, .purge_tenths = 200, .retract_tenths = 99 };
    CHECK(wild.clamped().purge_tenths == Settings::purge_max);
    CHECK(wild.clamped().retract_tenths == Settings::retract_max);
}

TEST_CASE("reprise : marqueurs « ;TYPE: » du trancheur", "[od3d][resume]") {
    CHECK(parse("TYPE:Internal infill") == od3d::feature_marks::infill);
    CHECK(parse("TYPE:Internal infill\r") == od3d::feature_marks::infill);
    CHECK(parse("TYPE:Sparse infill") == od3d::feature_marks::infill); // Orca
    CHECK(parse("TYPE:FILL") == od3d::feature_marks::infill); // Cura
    CHECK(parse("TYPE:Solid infill") == od3d::feature_marks::other);
    CHECK(parse("TYPE:Top solid infill") == od3d::feature_marks::other);
    CHECK(parse("TYPE:Perimeter") == od3d::feature_marks::other);
    CHECK(parse("TYPE:Support material interface") == od3d::feature_marks::other); // nom long : un autre tracé
    CHECK(parse("TYPE:") == od3d::feature_marks::other);
    CHECK(parse("LAYER_CHANGE") == -1); // commentaire ordinaire : sauté comme chez Prusa
    CHECK(parse(" TYPE:Internal infill") == -1);
    CHECK(parse("TYP") == -1);

    char out[16];
    REQUIRE(od3d::feature_marks::write_command(od3d::feature_marks::infill, out, sizeof(out)) == 8);
    CHECK(std::string(out) == "M2011 S1");
    REQUIRE(od3d::feature_marks::write_command(od3d::feature_marks::other, out, sizeof(out)) == 8);
    CHECK(std::string(out) == "M2011 S0");
    CHECK(od3d::feature_marks::write_command(-1, out, sizeof(out)) == 0);
    char small[4];
    CHECK(od3d::feature_marks::write_command(od3d::feature_marks::infill, small, sizeof(small)) == 0);
}

TEST_CASE("reprise : suivi des tracés de remplissage", "[od3d][resume]") {
    Tracker t;
    float e = 0;

    SECTION("sans marqueur : aucun remplissage, mais les couches sont suivies") {
        e = line(t, { 0, 0 }, { 10, 0 }, 0.2f, e);
        e = line(t, { 0, 0 }, { 10, 0 }, 0.4f, e);
        CHECK(t.current().count == 0);
        CHECK(t.layer_z() == 0.4f);
        CHECK(t.prev_layer_z() == 0.2f);
    }

    SECTION("tracés enchaînés, puis déplacement, puis changement de type") {
        t.set_feature(od3d::feature_marks::infill);
        CHECK(t.move({ 20, 100 }, 0.4f, 0, { 40, 100 }, 0.4f, 0.8f)); // début d'une suite
        CHECK_FALSE(t.move({ 40, 100 }, 0.4f, 0.8f, { 40, 110 }, 0.4f, 1.2f)); // à la suite
        CHECK(t.current().count == 3);
        CHECK_THAT(t.current().total_mm, WithinAbs(30, 1e-4));
        CHECK_THAT(t.current().e_per_mm(), WithinAbs(0.04, 1e-5));
        // Rétraction, déplacement, mouvement de Z : ignorés
        CHECK_FALSE(t.move({ 40, 110 }, 0.4f, 1.2f, { 40, 110 }, 0.4f, 0.4f));
        CHECK_FALSE(t.move({ 40, 110 }, 0.4f, 0.4f, { 90, 150 }, 0.4f, 0.4f));
        CHECK_FALSE(t.move({ 90, 150 }, 0.4f, 0.4f, { 90, 150 }, 0.6f, 0.5f));
        CHECK(t.current().count == 3);
        // Un tracé qui ne part pas du dernier point : nouvelle suite, l'ancienne (30 mm) gardée
        CHECK(t.move({ 90, 150 }, 0.4f, 1.2f, { 95, 150 }, 0.4f, 1.4f));
        CHECK(t.last().count == 3);
        CHECK(t.current().count == 2);
        // Fin du remplissage : la suite de 5 mm, trop courte, n'écrase pas la précédente
        t.set_feature(od3d::feature_marks::other);
        CHECK(t.current().count == 0);
        CHECK(t.last().count == 3);
    }

    SECTION("suite longue : les 24 derniers points gardés") {
        t.set_feature(od3d::feature_marks::infill);
        for (int i = 0; i < 30; ++i) {
            e = line(t, { 20.f + i, 100 }, { 21.f + i, 100 }, 0.4f, e);
        }
        CHECK(t.current().count == Run::max_points);
        CHECK_THAT(t.current().at(0).x, WithinAbs(50 - (Run::max_points - 1), 1e-4));
        CHECK_THAT(t.current().at(Run::max_points - 1).x, WithinAbs(50, 1e-4));
        CHECK_THAT(t.current().total_mm, WithinAbs(30, 1e-3)); // toute la suite pour le débit
    }

    SECTION("remise à zéro au départ d'une impression") {
        layer(t, 0.2f, e);
        layer(t, 0.4f, e);
        t.set_feature(od3d::feature_marks::infill);
        e = line(t, { 20, 100 }, { 60, 100 }, 0.4f, e);
        t.set_feature(od3d::feature_marks::other);
        REQUIRE(t.last().count == 2);
        t.reset();
        CHECK(t.current().count == 0);
        CHECK(t.last().count == 0);
        CHECK(std::isnan(t.layer_z()));
        CHECK(std::isnan(t.prev_layer_z()));
        // Le type de tracé aussi : sans nouveau M2011, plus de remplissage suivi
        e = line(t, { 20, 100 }, { 60, 100 }, 0.6f, e);
        CHECK(t.current().count == 0);
    }

    SECTION("changement de tête (INDX) : le remplissage d'une autre buse oublié, les couches gardées") {
        layer(t, 0.2f, e);
        layer(t, 0.4f, e);
        t.set_feature(od3d::feature_marks::infill);
        e = line(t, { 20, 100 }, { 60, 100 }, 0.4f, e);
        e = line(t, { 60, 100 }, { 60, 140 }, 0.4f, e);
        REQUIRE(t.current().count == 3);
        t.forget_runs();
        CHECK(t.current().count == 0);
        CHECK(t.last().count == 0);
        CHECK(t.layer_z() == 0.4f);
        CHECK(t.prev_layer_z() == 0.2f);
        InfillPlan p;
        CHECK(plan_infill(on, t, 0.4f, 10, 9.2f, core_one, nullptr, p) == Refusal::no_infill); // la reprise de Prusa
        // La nouvelle tête suit son propre remplissage
        e = line(t, { 100, 100 }, { 140, 100 }, 0.4f, e);
        CHECK(t.current().count == 2);
    }

    SECTION("Z qui redescend (objets l'un après l'autre) : les couches repartent") {
        layer(t, 0.2f, e);
        layer(t, 5.0f, e);
        CHECK(t.prev_layer_z() == 0.2f);
        layer(t, 0.2f, e);
        CHECK(t.layer_z() == 0.2f);
        CHECK(std::isnan(t.prev_layer_z()));
    }
}

TEST_CASE("reprise : purge au milieu d'une ligne de remplissage", "[od3d][resume]") {
    Tracker t;
    float e = 0;
    layer(t, 0.2f, e);
    layer(t, 0.4f, e);
    t.set_feature(od3d::feature_marks::infill);
    e = line(t, { 20, 100 }, { 60, 100 }, 0.4f, e); // 40 mm
    layer(t, 0.4f, e); // pause pendant les périmètres de la même couche

    InfillPlan p;
    // Pause : E de la pièce 1234,5, rétracté de 0,8 mm par la pause
    REQUIRE(plan_infill(on, t, 0.4f, 1234.5f, 1233.7f, core_one, nullptr, p) == Refusal::none);
    CHECK_THAT(p.purge.x, WithinAbs(40, 1e-4)); // milieu, loin des périmètres
    CHECK_THAT(p.purge.y, WithinAbs(100, 1e-4));
    CHECK(p.z == 0.4f);
    CHECK_THAT(p.purge_e, WithinAbs(0.8 + 1.0, 1e-4)); // désrétraction de la pause + petite purge, en bas
    REQUIRE(p.line_count == 1);
    CHECK_THAT(p.line[0].x, WithinAbs(50, 1e-4)); // cordon de 10 mm le long de la ligne
    CHECK_THAT(p.line[0].y, WithinAbs(100, 1e-4));
    CHECK_THAT(p.e_per_mm, WithinAbs(epm, 1e-5)); // au débit du remplissage
    CHECK_THAT(p.retract, WithinAbs(0.8, 1e-6));
    CHECK_THAT(1234.5f - p.e_after, WithinAbs(0.8, 1e-4)); // Prusa désrétracte exactement la rétraction
}

TEST_CASE("reprise : remplissage en zigzag, purge sur une longue ligne", "[od3d][resume]") {
    Tracker t;
    float e = 0;
    layer(t, 0.2f, e);
    layer(t, 0.4f, e);
    t.set_feature(od3d::feature_marks::infill);
    e = line(t, { 20, 100 }, { 80, 100 }, 0.4f, e);
    e = line(t, { 80, 100 }, { 80, 103 }, 0.4f, e); // raccord le long d'un périmètre
    e = line(t, { 80, 103 }, { 20, 103 }, 0.4f, e);

    InfillPlan p;
    REQUIRE(plan_infill(on, t, 0.4f, 10, 10, core_one, nullptr, p) == Refusal::none);
    CHECK_THAT(p.purge.x, WithinAbs(50, 1e-3));
    CHECK((std::fabs(p.purge.y - 100) < 1e-3f || std::fabs(p.purge.y - 103) < 1e-3f));
    REQUIRE(p.line_count >= 1);
    CHECK_THAT(length(p.purge, p.line[p.line_count - 1]), WithinAbs(prime_line_mm, 1e-3));
    CHECK_THAT(p.purge_e, WithinAbs(1.0, 1e-5)); // pas de rétraction de pause : la petite purge seule
}

TEST_CASE("reprise : courbe faite de petits segments (gyroïde)", "[od3d][resume]") {
    Tracker t;
    float e = 0;
    layer(t, 0.2f, e);
    layer(t, 0.4f, e);
    t.set_feature(od3d::feature_marks::infill);
    Point a { 20, 100 };
    for (int i = 1; i <= 40; ++i) {
        const Point b { 20.f + i, 100.f + 3.f * std::sin(i / 4.f) };
        e = line(t, a, b, 0.4f, e);
        a = b;
    }

    InfillPlan p;
    REQUIRE(plan_infill(on, t, 0.4f, 10, 9.2f, core_one, nullptr, p) == Refusal::none);
    const Run &run = t.current();
    const float first_x = run.at(0).x;
    const float last_x = run.at(run.count - 1).x;
    // Loin des deux bouts gardés : 2 mm le long de la courbe, au moins 1,6 mm en X (pente au plus 0,75)
    CHECK(p.purge.x > first_x + 1.0f);
    CHECK(p.purge.x < last_x - 1.0f);
    REQUIRE(p.line_count > 1); // le cordon suit la courbe point par point
    float mm = length(p.purge, p.line[0]);
    for (size_t i = 1; i < p.line_count; ++i) {
        mm += length(p.line[i - 1], p.line[i]);
    }
    CHECK(mm <= prime_line_mm + 1e-3f);
    CHECK(mm >= min_line_mm);
    for (size_t i = 0; i < p.line_count; ++i) {
        CHECK(p.line[i].x > first_x + 1.0f); // jamais plus près d'un bout que la marge
        CHECK(p.line[i].x < last_x - 1.0f);
    }
}

TEST_CASE("reprise : replis vers la reprise de Prusa", "[od3d][resume]") {
    Tracker t;
    float e = 0;
    InfillPlan p;

    SECTION("option coupée") {
        layer(t, 0.2f, e);
        layer(t, 0.4f, e);
        t.set_feature(od3d::feature_marks::infill);
        e = line(t, { 20, 100 }, { 60, 100 }, 0.4f, e);
        CHECK(plan_infill(Settings {}, t, 0.4f, 10, 9.2f, core_one, nullptr, p) == Refusal::disabled);
        // Position de E incohérente : en avance sur la pièce, ou trop loin derrière
        CHECK(plan_infill(on, t, 0.4f, 10, 10.5f, core_one, nullptr, p) == Refusal::e_position);
        CHECK(plan_infill(on, t, 0.4f, 100, 80, core_one, nullptr, p) == Refusal::e_position);
        CHECK(plan_infill(on, t, 0.4f, NAN, 80, core_one, nullptr, p) == Refusal::e_position);
    }

    SECTION("première couche") {
        t.set_feature(od3d::feature_marks::infill);
        e = line(t, { 20, 100 }, { 60, 100 }, 0.2f, e);
        CHECK(plan_infill(on, t, 0.2f, 10, 9.2f, core_one, nullptr, p) == Refusal::first_layer);
    }

    SECTION("objet sans remplissage, mode vase, pause avant le premier remplissage") {
        layer(t, 0.2f, e);
        layer(t, 0.4f, e);
        CHECK(plan_infill(on, t, 0.4f, 10, 9.2f, core_one, nullptr, p) == Refusal::no_infill);
    }

    SECTION("suites trop courtes ; une courte en cours, la longue d'avant sert") {
        layer(t, 0.2f, e);
        layer(t, 0.4f, e);
        t.set_feature(od3d::feature_marks::infill);
        e = line(t, { 20, 100 }, { 24, 100 }, 0.4f, e); // 4 mm
        CHECK(plan_infill(on, t, 0.4f, 10, 9.2f, core_one, nullptr, p) == Refusal::no_infill);
        e = line(t, { 30, 120 }, { 70, 120 }, 0.4f, e); // 40 mm, ailleurs
        e = line(t, { 80, 140 }, { 83, 140 }, 0.4f, e); // 3 mm en cours
        REQUIRE(plan_infill(on, t, 0.4f, 10, 9.2f, core_one, nullptr, p) == Refusal::none);
        CHECK_THAT(p.purge.y, WithinAbs(120, 1e-4));
    }

    SECTION("couche en cours ou précédente seulement") {
        layer(t, 0.2f, e);
        layer(t, 0.4f, e);
        t.set_feature(od3d::feature_marks::infill);
        e = line(t, { 20, 100 }, { 60, 100 }, 0.4f, e);
        layer(t, 0.6f, e);
        // Couche précédente : la purge se fait à la hauteur de la reprise
        REQUIRE(plan_infill(on, t, 0.6f, 10, 9.2f, core_one, nullptr, p) == Refusal::none);
        CHECK(p.z == 0.6f);
        // Deux couches plus bas : trop loin (le haut de l'objet peut déjà être plein ou visible)
        layer(t, 0.8f, e);
        CHECK(plan_infill(on, t, 0.8f, 10, 9.2f, core_one, nullptr, p) == Refusal::too_far);
    }

    SECTION("reprise trop haut au-dessus du remplissage, ou en dessous") {
        layer(t, 0.2f, e);
        layer(t, 0.4f, e);
        t.set_feature(od3d::feature_marks::infill);
        e = line(t, { 20, 100 }, { 60, 100 }, 0.4f, e);
        CHECK(plan_infill(on, t, 1.6f, 10, 9.2f, core_one, nullptr, p) == Refusal::too_far);
        CHECK(plan_infill(on, t, 0.2f, 10, 9.2f, core_one, nullptr, p) == Refusal::too_far);
    }

    SECTION("objet annulé") {
        layer(t, 0.2f, e);
        layer(t, 0.4f, e);
        t.set_feature(od3d::feature_marks::infill);
        REQUIRE(t.move({ 20, 100 }, 0.4f, e, { 60, 100 }, 0.4f, e + 1.6f));
        t.set_object(3);
        CHECK(plan_infill(on, t, 0.4f, 10, 9.2f, core_one, +[](int16_t o) { return o == 3; }, p) == Refusal::cancelled);
        CHECK(plan_infill(on, t, 0.4f, 10, 9.2f, core_one, +[](int16_t o) { return o == 4; }, p) == Refusal::none);
    }

    SECTION("débit invraisemblable, hors des limites") {
        layer(t, 0.2f, e);
        layer(t, 0.4f, e);
        t.set_feature(od3d::feature_marks::infill);
        REQUIRE(t.move({ 20, 100 }, 0.4f, e, { 60, 100 }, 0.4f, e + 40)); // 1 mm de filament par mm
        CHECK(plan_infill(on, t, 0.4f, 10, 9.2f, core_one, nullptr, p) == Refusal::extrusion);
        const Box small { -1, 50, -19, 220 };
        Tracker t2;
        float e2 = 0;
        layer(t2, 0.2f, e2);
        layer(t2, 0.4f, e2);
        t2.set_feature(od3d::feature_marks::infill);
        e2 = line(t2, { 20, 100 }, { 60, 100 }, 0.4f, e2);
        CHECK(plan_infill(on, t2, 0.4f, 10, 9.2f, small, nullptr, p) == Refusal::limits);
    }
}
