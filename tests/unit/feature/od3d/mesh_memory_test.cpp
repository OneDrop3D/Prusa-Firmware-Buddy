// OneDrop3D (F44) : maillage du plateau gardé et repris, partie pure : conditions, couverture, points de contrôle
#include <feature/od3d/mesh_memory_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

using namespace od3d::mesh_memory;
using Catch::Matchers::WithinAbs;

namespace {

constexpr uint32_t hour = 3600;

/// Maillage gardé : plateau à 60 °C, chambre à 25 °C, PLA (palpage à 170 °C), buse de 0,4 mm, il y a une heure
Record stored_record() {
    Record r;
    r.version = Record::current_version;
    r.time = 1'800'000'000;
    r.bed_c10 = 600;
    r.chamber_c10 = 250;
    r.probe_c = 170;
    r.nozzle_um = 400;
    r.flags = 0b100;
    for (size_t iy = 1; iy <= 4; ++iy) {
        for (size_t ix = 1; ix <= 4; ++ix) {
            r.valid |= major_bit(index_of(ix, iy));
            r.z_um[index_of(ix, iy)] = int16_t(-100 + 20 * int(ix) - 10 * int(iy)); // plateau incliné
        }
    }
    return r;
}

Record now_record() {
    Record n = stored_record();
    n.time += hour;
    n.valid = 0;
    n.z_um = {};
    return n;
}

uint64_t square(size_t x0, size_t y0, size_t x1, size_t y1) {
    uint64_t m = 0;
    for (size_t iy = y0; iy <= y1; ++iy) {
        for (size_t ix = x0; ix <= x1; ++ix) {
            m |= major_bit(index_of(ix, iy));
        }
    }
    return m;
}

} // namespace

TEST_CASE("maillage gardé : conditions de reprise") {
    const Record s = stored_record();
    Record n = now_record();
    CHECK(compare(s, n, 24 * hour) == Refusal::none);
    CHECK(compare(s, n, 0) == Refusal::disabled);
    CHECK(compare(Record {}, n, 24 * hour) == Refusal::empty);
    CHECK(compare(s, n, hour - 1) == Refusal::too_old);

    n = now_record();
    n.time = 0; // heure pas encore reçue
    CHECK(compare(s, n, 24 * hour) == Refusal::clock);
    n.time = s.time - 10; // horloge revenue en arrière
    CHECK(compare(s, n, 24 * hour) == Refusal::too_old);

    n = now_record();
    n.bed_c10 = 650; // +5 °C : encore bon
    CHECK(compare(s, n, 24 * hour) == Refusal::none);
    n.bed_c10 = 651;
    CHECK(compare(s, n, 24 * hour) == Refusal::bed);
    n.bed_c10 = 850; // PETG
    CHECK(compare(s, n, 24 * hour) == Refusal::bed);

    n = now_record();
    n.chamber_c10 = 310;
    CHECK(compare(s, n, 24 * hour) == Refusal::chamber);
    n.chamber_c10 = no_temperature; // capteur disparu
    CHECK(compare(s, n, 24 * hour) == Refusal::chamber);

    n = now_record();
    n.probe_c = 176;
    CHECK(compare(s, n, 24 * hour) == Refusal::probe_temperature);
    n = now_record();
    n.nozzle_um = 600;
    CHECK(compare(s, n, 24 * hour) == Refusal::nozzle);
    n = now_record();
    n.flags = 0b101; // buse trempée
    CHECK(compare(s, n, 24 * hour) == Refusal::nozzle);
    n = now_record();
    n.flags = 0; // dilatation de la buse (F2) coupée depuis
    CHECK(compare(s, n, 24 * hour) == Refusal::nozzle);
    n = now_record();
    n.probe_offset_um = 20;
    CHECK(compare(s, n, 24 * hour) == Refusal::z_offset);
}

TEST_CASE("maillage gardé : couverture de la zone à palper") {
    const Record s = stored_record();
    CHECK(covers(s, square(1, 1, 4, 4)) == Refusal::none);
    CHECK(covers(s, square(2, 2, 3, 3)) == Refusal::none); // 4 points : 3 contrôlés, 1 gagné
    CHECK(covers(s, square(1, 1, 5, 4)) == Refusal::not_covered); // pièce plus à droite
    CHECK(covers(s, square(2, 2, 4, 2)) == Refusal::too_few_points); // 3 points : autant les palper
    CHECK(covers(s, 0) == Refusal::too_few_points);
}

TEST_CASE("maillage gardé : points de contrôle aux coins") {
    const uint64_t c = pick_checks(square(1, 1, 4, 4));
    CHECK(c == (major_bit(index_of(1, 1)) | major_bit(index_of(4, 4)) | major_bit(index_of(4, 1))));
    // Une seule rangée : les deux bouts et le milieu
    const uint64_t row = pick_checks(square(1, 3, 5, 3));
    CHECK(row == (major_bit(index_of(1, 3)) | major_bit(index_of(5, 3)) | major_bit(index_of(3, 3))));
    CHECK(pick_checks(0) == 0);
    CHECK(pick_checks(major_bit(index_of(2, 2))) == major_bit(index_of(2, 2)));
}

TEST_CASE("maillage gardé : palpage de contrôle") {
    const Record s = stored_record();
    const uint64_t checks = pick_checks(square(1, 1, 4, 4));
    std::array<float, majors> fresh;
    const auto probe = [&](float offset, float bump_at_44) {
        fresh.fill(NAN);
        for (size_t i = 0; i < majors; ++i) {
            if (checks & major_bit(i)) {
                fresh[i] = from_um(s.z_um[i]) + offset + (i == index_of(4, 4) ? bump_at_44 : 0.f);
            }
        }
        return evaluate(s, checks, fresh);
    };
    // Origine Z prise 12 µm plus bas : tout le maillage décalé d'autant, repris
    Check c = probe(0.012f, 0.f);
    CHECK(c.refusal == Refusal::none);
    CHECK_THAT(c.offset_mm, WithinAbs(0.012, 1e-5));
    CHECK_THAT(c.spread_mm, WithinAbs(0.0, 1e-5));
    // Un coin qui a bougé de 0,05 mm (plaque, plateau) : tout repalper
    c = probe(0.f, 0.05f);
    CHECK(c.refusal == Refusal::check_spread);
    CHECK(c.spread_mm > check_spread_mm);
    // Petit bruit de palpage : repris
    CHECK(probe(0.f, 0.015f).refusal == Refusal::none);
    // Tout plus haut de 0,08 mm (autre plaque) : tout repalper
    CHECK(probe(0.08f, 0.f).refusal == Refusal::check_offset);
    // Point de contrôle pas palpé (arrêt) : refus
    fresh.fill(NAN);
    CHECK(evaluate(s, checks, fresh).refusal == Refusal::probing);
}

TEST_CASE("maillage gardé : unités") {
    CHECK(to_um(0.1234f) == 123);
    CHECK(to_um(-0.0006f) == -1);
    CHECK(to_um(NAN) == 0);
    CHECK(to_um(100.f) == 32000);
    CHECK_THAT(from_um(-250), WithinAbs(-0.25, 1e-6));
    CHECK(to_c10(60.04f) == 600);
    CHECK(to_c10(NAN) == no_temperature);
    CHECK(Record {}.empty());
    CHECK_FALSE(stored_record().empty());
}

TEST_CASE("maillage gardé : proposé avant l'impression") {
    const Record s = stored_record();
    Record n = now_record();
    // Plateau froid, chambre et buse pas encore chauffées : vérifiés au G29, pas avant
    n.bed_c10 = 220;
    n.chamber_c10 = no_temperature;
    n.probe_c = 0;
    CHECK(compare_before_print(s, n, 24 * hour) == Refusal::none);
    CHECK(compare(s, n, 24 * hour) == Refusal::bed);
    // Le reste refuse dès « Avant l'impression » : la ligne n'est pas montrée
    CHECK(compare_before_print(s, n, 0) == Refusal::disabled);
    CHECK(compare_before_print(Record {}, n, 24 * hour) == Refusal::empty);
    CHECK(compare_before_print(s, n, hour - 1) == Refusal::too_old);
    n.nozzle_um = 600;
    CHECK(compare_before_print(s, n, 24 * hour) == Refusal::nozzle);
    n = now_record();
    n.sheet = 1; // autre plaque
    CHECK(compare_before_print(s, n, 24 * hour) == Refusal::nozzle);
    n = now_record();
    n.probe_offset_um = 20;
    CHECK(compare_before_print(s, n, 24 * hour) == Refusal::z_offset);
}

TEST_CASE("maillage gardé : le choix vaut pour l'impression demandée juste après") {
    PrintChoice c;
    c.on_print_request(); // sans « Avant l'impression » : le réglage
    CHECK_FALSE(c.current().has_value());

    c.choose(false); // « Maillage : nouveau » (plaque changée)
    CHECK_FALSE(c.current().has_value()); // rien avant la demande : un G29 en cours n'est pas touché
    c.on_print_request();
    CHECK(c.current() == false);
    CHECK(c.current() == false); // G29 P1 puis G29 P1 … C de la même impression
    c.on_print_request(); // impression suivante par PrusaLink, réimpression, Connect : le réglage
    CHECK_FALSE(c.current().has_value());

    c.choose(true); // « Maillage : repris »
    c.on_print_request();
    CHECK(c.current() == true);

    c.choose(false);
    c.choose(true); // l'interrupteur rebasculé avant « Imprimer »
    c.on_print_request();
    CHECK(c.current() == true);
}

TEST_CASE("maillage gardé : âge maximal selon le réglage et le choix") {
    // Sans choix : le réglage, désactivé = pas de reprise (comme avant)
    CHECK(max_age_s(0, std::nullopt) == 0);
    CHECK(max_age_s(24, std::nullopt) == 24 * hour);
    // « Nouveau » : palpage complet, même réglage actif
    CHECK(max_age_s(24, false) == 0);
    CHECK(max_age_s(0, false) == 0);
    // « Repris » : l'âge du réglage, ou le plus long proposé s'il est désactivé
    CHECK(max_age_s(6, true) == 6 * hour);
    CHECK(max_age_s(0, true) == uint32_t(chosen_max_age_hours) * hour);
    // Proposé avant l'impression réglage désactivé : maillage de 48 h repris si on le choisit
    Record n = now_record();
    n.time += 47 * hour;
    CHECK(compare_before_print(stored_record(), n, max_age_s(0, true)) == Refusal::none);
    CHECK(compare_before_print(stored_record(), n, max_age_s(24, true)) == Refusal::too_old);
}
