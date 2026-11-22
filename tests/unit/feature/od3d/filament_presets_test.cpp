// OneDrop3D (UI-13) : températures des préréglages de Prusa modifiées par l'utilisateur.
#include <feature/od3d/filament_presets_math_od3d.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace od3d::filament_presets;

namespace {
// PLA de Prusa sur CORE One : buse 215, préchauffage 170, plateau 60, chambre cible 20 (15 à 38)
constexpr Values pla { 215, 170, 60, 20, 15, 38 };
} // namespace

TEST_CASE("préréglages : sans modification, les valeurs de Prusa") {
    CHECK(apply(pla, Override {}) == pla);
    CHECK_FALSE(Override {}.any());
}

TEST_CASE("préréglages : chambre du PLA à 30 °C au lieu de 20") {
    const Override o = with(Override {}, chamber_target, 30, pla);
    CHECK(o.modified(chamber_target));
    CHECK(o.any());
    const Values v = apply(pla, o);
    CHECK(v[chamber_target] == 30);
    CHECK(v[nozzle] == 215);
    CHECK(v[chamber_max] == 38);
}

TEST_CASE("préréglages : la valeur de Prusa retire la modification") {
    Override o = with(Override {}, nozzle, 220, pla);
    CHECK(apply(pla, o)[nozzle] == 220);
    o = with(o, nozzle, 215, pla);
    CHECK_FALSE(o.any());
    o = without(with(o, bed, 65, pla), bed);
    CHECK_FALSE(o.any());
}

TEST_CASE("préréglages : chambre « Aucune » (0)") {
    const Override o = with(Override {}, chamber_target, 0, pla);
    CHECK(apply(pla, o)[chamber_target] == none);
    // Sans chambre chez Prusa, 0 n'est pas une modification
    constexpr Values no_chamber { 215, 170, 60, none, none, none };
    CHECK_FALSE(with(Override {}, chamber_target, 0, no_chamber).any());
    CHECK(apply(no_chamber, with(Override {}, chamber_target, 30, no_chamber))[chamber_target] == 30);
}

TEST_CASE("préréglages : la chambre reste dans l'ordre min <= cible <= max") {
    // Cible modifiée à 36, puis maximum ramené à 30 : la cible suit le maximum
    Override o = with(Override {}, chamber_target, 36, pla);
    o = with(o, chamber_max, 30, pla);
    Values v = apply(pla, o);
    CHECK(v[chamber_target] == 30);
    CHECK(v[chamber_max] == 30);
    // Minimum au-dessus du maximum : le maximum monte, la cible aussi
    o = with(Override {}, chamber_min, 45, pla);
    v = apply(pla, o);
    CHECK(v[chamber_min] == 45);
    CHECK(v[chamber_max] == 45);
    CHECK(v[chamber_target] == 45);
}
