// OneDrop3D (F41) : longueur du tube PTFE avant d'après l'avance lente du MMU jusqu'au capteur de l'extrudeur.
#include <feature/od3d/ptfe_math_od3d.hpp>

#include <algorithm>
#include <catch2/catch_test_macros.hpp>

using namespace od3d::ptfe;

namespace {

/// Un chargement comme le voit l'imprimante : la poulie part de base (position cumulée depuis l'allumage du MMU, au
/// hasard), avance vite jusqu'à base + fast, puis lentement de step mm par relevé jusqu'à base + distance. La
/// progression relue annonce l'avance lente un relevé en retard ; la position relue est celle d'avant la progression.
Tracker::Outcome load(uint16_t base, uint16_t distance, uint16_t step = 4, uint16_t fast = min_mm) {
    Tracker t;
    uint16_t pos = base;
    bool slow = false;
    uint16_t shown = base; ///< position relue (en retard d'un relevé)
    for (int i = 0; i < 1000; ++i) {
        const uint16_t travelled = uint16_t(pos - base);
        const bool sensor = travelled >= distance;
        t.sample(slow, shown, sensor);
        if (sensor) {
            break;
        }
        shown = pos;
        slow = travelled >= fast; // progression relue après le mouvement
        pos = uint16_t(pos + (travelled >= fast ? step : std::min<uint16_t>(30, uint16_t(fast - travelled)))); // l'avance rapide finit à fast
    }
    return t.outcome(fast);
}

} // namespace

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
    CHECK(bowden_length(350) == min_mm); // la marge ferait descendre sous 341 : le minimum
    CHECK(bowden_length(max_mm) == max_mm - margin_mm(max_mm));
    for (uint16_t d = min_mm + 20; d <= max_mm; ++d) {
        const auto len = bowden_length(d);
        REQUIRE(len.has_value());
        CHECK(*len < d);
        CHECK(*len >= min_mm);
    }
    CHECK_FALSE(bowden_length(min_mm - 1).has_value());
    CHECK_FALSE(bowden_length(max_mm + 1).has_value());
}

TEST_CASE("tube PTFE : la position cumulée de la poulie ne compte pas, seul son chemin dans l'avance lente") {
    // Même tube de 520 mm, poulie partie de n'importe où (y compris négative, relue en uint16, qui repasse par 0)
    for (const uint16_t base : { uint16_t(0), uint16_t(1234), uint16_t(65000), uint16_t(65535 - 400) }) {
        const Tracker::Outcome o = load(base, 520);
        CHECK(o.result == Result::saved);
        CHECK(o.measured_mm <= 520); // jamais trop long : sinon « capteur trop tôt » au chargement suivant
        CHECK(o.measured_mm >= 520 - 12); // un à trois relevés de 4 mm en moins
        CHECK(o.saved_mm == *bowden_length(o.measured_mm));
        CHECK(o.saved_mm < 520 - 20);
    }
}

TEST_CASE("tube PTFE : tubes longs, jusqu'au maximum du MMU") {
    const Tracker::Outcome o = load(500, 950);
    CHECK(o.result == Result::saved);
    CHECK(o.measured_mm <= 950);
    CHECK(o.measured_mm >= 950 - 12);
    // Capteur vu juste après le début de l'avance lente, avant tout relevé : la distance de l'avance rapide
    const Tracker::Outcome quick = load(7, min_mm + 2);
    CHECK(quick.result == Result::saved);
    CHECK(quick.measured_mm == min_mm);
    CHECK(quick.saved_mm == min_mm);
}

TEST_CASE("tube PTFE : chaque échec a sa raison") {
    // Capteur vu pendant l'avance rapide : tube plus court que 341 mm
    CHECK(load(100, 200).result == Result::too_short);

    // Avance lente sans capteur (le MMU abandonne à 1000 mm)
    {
        Tracker t;
        t.sample(false, 10, false);
        t.sample(true, 351, false);
        t.sample(true, 355, false);
        CHECK(t.outcome().result == Result::no_trigger);
    }
    // Rien : arrêté avant l'avance lente (erreur du MMU, arrêt)
    {
        Tracker t;
        t.sample(false, 10, false);
        CHECK(t.outcome().result == Result::interrupted);
        CHECK(Tracker {}.outcome().result == Result::interrupted);
    }
    // Relevés invraisemblables : au-delà du maximum du MMU
    {
        Tracker t;
        t.sample(true, 0, false);
        t.sample(true, 10, false);
        t.sample(true, 900, false);
        t.sample(true, 904, true);
        const Tracker::Outcome o = t.outcome();
        CHECK(o.result == Result::too_long);
        CHECK(o.measured_mm == min_mm + 890);
        CHECK(o.saved_mm == 0);
    }
}

TEST_CASE("tube PTFE : reprise du MMU, le départ est repris") {
    Tracker t;
    t.sample(true, 2000, false); // 1re avance lente, puis erreur et reprise : la poulie recule
    t.sample(true, 2010, false);
    t.sample(true, 2050, false);
    t.sample(false, 1500, false);
    t.sample(false, 1600, false);
    t.sample(true, 1841, false); // 2e avance lente : position peut-être d'avant, écartée
    t.sample(true, 1845, false);
    t.sample(true, 1900, false);
    t.sample(true, 1950, true);
    const Tracker::Outcome o = t.outcome();
    CHECK(o.result == Result::saved);
    CHECK(o.measured_mm == min_mm + 55); // 1845 → 1900
}

TEST_CASE("tube PTFE : longueurs que le MMU garde telles quelles") {
    // Hors de 341 à 1000 mm, le MMU prend 360 mm sans le dire : rien n'est écrit
    CHECK_FALSE(valid_length(0));
    CHECK_FALSE(valid_length(min_mm - 1));
    CHECK(valid_length(min_mm));
    CHECK(valid_length(max_mm));
    CHECK_FALSE(valid_length(max_mm + 1));
}

TEST_CASE("tube PTFE : avance rapide relue, pas supposée") {
    // Le MMU a gardé 492 mm au lieu des 341 écrits : la phase lente commence à 492, la distance reste juste
    const Tracker::Outcome o = load(3000, 560, 4, 492);
    CHECK(o.result == Result::saved);
    CHECK(o.measured_mm <= 560);
    CHECK(o.measured_mm >= 560 - 12);
    // Calculée avec 341 (l'ancienne hypothèse), elle aurait été trop courte de 151 mm : ralentissement 15 cm trop tôt
    Tracker t;
    t.sample(true, 100, false);
    t.sample(true, 104, false);
    t.sample(true, 150, false);
    t.sample(true, 154, true);
    CHECK(t.outcome(492).measured_mm == 492 + 46);
    CHECK(t.outcome(min_mm).measured_mm == min_mm + 46);
}
