/// @file
/// OneDrop3D : registre des calibrations de l'assistant (demande du 22/11 : « lorsque l'on installe le firmware
/// personnalisé il propose de faire tous les tests […] à chaque mise à jour qui modifie les calibrations il demande de
/// les refaire »). Pur : ni Marlin, ni interface, ni config_store (testé dans tests/unit/feature/od3d).
///
/// RÈGLE POUR LES AGENTS — la version d'une entrée (Entry::version) est la seule source de vérité :
/// - un commit qui change ce que mesure une calibration (algorithme, paramètres, format ou usage du résultat), au point
///   qu'il faille la refaire pour en profiter, augmente sa version de 1 dans `entries` ;
/// - au démarrage suivant, l'imprimante compare avec la version enregistrée (config_store « OD3D Calib Done ») et
///   propose de refaire seulement les calibrations dont la version a augmenté ;
/// - ne jamais baisser une version ni réutiliser la valeur d'un Test retiré ; une nouvelle calibration = une nouvelle
///   valeur de Test juste avant `count` (16 au plus), son entrée dans `entries` à sa place dans l'ordre d'exécution,
///   version 1, et son G-code dans step_gcode().
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>

namespace od3d::calib_registry {

/// Identifiant enregistré : indice dans le config_store, ne jamais renuméroter
enum class Test : uint8_t {
    input_shaper, ///< Prusa, M1959 (assistant input shaper, accéléromètre)
    phase_stepping, ///< Prusa, M1977 (accéléromètre)
    belts, ///< F14, M1992
    speed_map, ///< F22, M1993
    damping, ///< F33, M1997
    mpc, ///< F7, M306 T
    pa, ///< F4, M1994 par emplacement
    max_flow, ///< F5, M1995 par emplacement
    min_temp, ///< F6, M1996 par emplacement
    skew, ///< F15 : pièce imprimée à mesurer, à faire à la main
    mmu_ptfe, ///< F41, M1999 B1 : longueur du tube PTFE avant du MMU (une bobine préchargée, rien dans l'extrudeur)
    fan_response, ///< N3, M2001 : temps de montée du ventilateur pièce, règle l'avance de F26 (sans besoin)
    nozzle_expansion, ///< M2003 : dilatation de la buse palpée de 170 à 260 °C (buse vide)
    e_jump, ///< M2000 par emplacement : saut de vitesse de l'extrudeur, accélération max du filament
    mmu_speed, ///< N4, M2002 P : vitesse de la poulie du MMU (une bobine préchargée, rien dans l'extrudeur)
    count,
};

inline constexpr size_t max_tests = 16; ///< places réservées dans le config_store
static_assert(size_t(Test::count) <= max_tests);

/// Besoins d'une calibration (bits)
enum Need : uint8_t {
    accelerometer = 1 << 0, ///< accéléromètre fixé sur la buse (phase 1 de l'assistant)
    cold_nozzle = 1 << 1, ///< buse sous cold_nozzle_max avant de fixer l'accéléromètre et pendant la mesure
    filament = 1 << 2, ///< une mesure par emplacement qui a une bobine, bobine chargée dans la buse
    mmu = 1 << 3, ///< MMU activé
    empty_bed = 1 << 4, ///< imprime ou palpe : plateau vide et propre confirmé avant
    flow_limit = 1 << 5, ///< seulement avec « Limiter au débit mesuré » (le résultat ne sert qu'à lui)
    one_spool = 1 << 6, ///< une seule mesure, avec la bobine du premier emplacement qui en a une
};

struct Entry {
    Test test;
    uint8_t needs; ///< Need
    uint8_t minutes; ///< durée estimée (par emplacement avec Need::filament)
    uint8_t version; ///< version de l'algorithme : voir la règle en tête du fichier
    bool automatic; ///< false : à faire à la main depuis son écran (listée, jamais lancée)
};

/// Dans l'ordre d'exécution : d'abord l'accéléromètre (buse froide, une seule pose), puis le MPC (il part de la buse
/// froide), le tube PTFE et la dilatation de la buse (rien dans l'extrudeur : avant les chargements), la réponse du
/// ventilateur (sans besoin), la vitesse de la poulie du MMU (après le tube PTFE : sa longueur fait l'avance rapide),
/// puis par emplacement PA, débit, température minimale, accélération du filament (un seul chargement par bobine,
/// chauffe gardée ; l'ordre par emplacement est celui de cette liste ; build_steps() fait d'abord tous les tests sans
/// Need::filament)
inline constexpr std::array entries {
    Entry { Test::belts, accelerometer | cold_nozzle, 3, 1, true }, // courroies d'abord : elles changent les résonances
    Entry { Test::phase_stepping, accelerometer | cold_nozzle, 8, 1, true },
    Entry { Test::input_shaper, accelerometer | cold_nozzle, 6, 1, true },
    Entry { Test::damping, accelerometer | cold_nozzle, 4, 1, true },
    Entry { Test::speed_map, accelerometer | cold_nozzle, 6, 1, true }, // après le phase stepping (F34)
    Entry { Test::mpc, 0, 12, 1, true },
    Entry { Test::mmu_ptfe, mmu | one_spool, 3, 1, true },
    Entry { Test::nozzle_expansion, empty_bed, 10, 1, true }, // après le tube PTFE : avec le MMU, l'extrudeur est déjà vide
    Entry { Test::pa, filament | empty_bed, 7, 1, true },
    Entry { Test::max_flow, filament | empty_bed | flow_limit, 5, 1, true },
    Entry { Test::min_temp, filament | empty_bed, 6, 1, true },
    Entry { Test::skew, empty_bed, 0, 1, false },
    Entry { Test::fan_response, 0, 1, 1, true }, // sans besoin : avec les tests sans bobine (build_steps)
    Entry { Test::e_jump, filament, 3, 1, true }, // rien d'imprimé ni de palpé : purge à la position de Prusa
    Entry { Test::mmu_speed, mmu | one_spool, 4, 1, true }, // après le tube PTFE : sa longueur fait l'avance rapide
};
static_assert(entries.size() == size_t(Test::count), "une entrée par Test");

constexpr const Entry &entry(Test test) {
    for (const Entry &e : entries) {
        if (e.test == test) {
            return e;
        }
    }
    return entries[0]; // inatteignable : static_assert ci-dessus
}

constexpr uint32_t test_bit(Test test) {
    return uint32_t(1) << uint8_t(test);
}

/// Résultat du dernier essai
enum class Result : uint8_t {
    none, ///< jamais essayée
    ok,
    failed,
    skipped, ///< annulée, passée ou pas atteinte
};

/// Ce qui est gardé par calibration (config_store « OD3D Calib Done », indice = Test). Taille fixe : pour la changer,
/// créer un nouvel élément sous un autre nom.
struct Record {
    uint8_t version = 0; ///< dernière version menée à bien (0 : jamais)
    Result result = Result::none; ///< dernier essai
    uint16_t reserved = 0;
    uint32_t date = 0; ///< dernier essai [s depuis 1970], 0 si l'heure n'était pas connue

    constexpr bool operator==(const Record &) const = default;
};

/// Le réglage « Accéléromètre disponible »
enum class Accelerometer : uint8_t {
    unknown, ///< jamais répondu : l'assistant demande
    yes,
    no,
};

/// L'imprimante telle qu'elle est
struct Context {
    Accelerometer accelerometer = Accelerometer::unknown;
    bool mmu = false;
    bool flow_limit = false;
    uint8_t spools = 0; ///< bit n : l'emplacement n a une bobine (utilisable)
};

/// La calibration a un sens sur cette imprimante (sans accéléromètre, ses tests ne sont pas proposés)
bool applicable(const Entry &e, const Context &ctx);

/// Le test passe par le MMU sur cette imprimante (retour du 23/11 : calibrer pendant le flash du MMU) : il le pilote
/// (Need::mmu), ou il charge ses bobines par lui (Need::filament avec un MMU activé). Les autres tournent MMU muet.
constexpr bool uses_mmu(const Entry &e, const Context &ctx) {
    return (e.needs & mmu) || ((e.needs & filament) && ctx.mmu);
}

/// Jamais menée à bien dans cette version
constexpr bool outdated(const Entry &e, const Record &r) {
    return r.version < e.version;
}

/// Tests applicables (bits de Test) ; only_outdated : seulement ceux à refaire (nouveaux ou version changée) ;
/// manuels compris
uint32_t select(const Context &ctx, std::span<const Record> records, bool only_outdated);

/// Les bits des tests automatiques, et parmi eux ceux qui ont tel besoin
uint32_t automatic_mask();
uint32_t needs_mask(uint8_t needs);

/// Tests automatiques cochés d'office dans la proposition (retour du 23/11 : « pouvoir passer certaines calibrations
/// comme le PA ») : ceux de select(), sauf le PA jamais mesuré (1re calibration : il se mesure au début de la
/// 1re impression de chaque bobine). Chacun se coche ou se décoche ensuite à l'écran.
uint32_t proposed(const Context &ctx, std::span<const Record> records, bool only_outdated);

/// Proposer l'assistant au démarrage : un test coché d'office (proposed()), et ce registre n'a pas été écarté par
/// « Ne plus demander » (dismissed = signature() enregistrée à ce moment)
bool should_propose(const Context &ctx, std::span<const Record> records, uint32_t dismissed);

/// Empreinte des versions du registre : elle change dès qu'une version change
uint32_t signature();

/// Durée estimée des tests automatiques de mask [min]
uint16_t minutes(uint32_t mask, const Context &ctx);

/// Une étape de l'assistant : un G-code mis en file, puis le suivant (jamais imbriqués)
struct Step {
    Test test;
    uint8_t slot = 0; ///< emplacement, pour Need::filament
    bool deferred = false; ///< déjà remise à la fin de la série (MMU indisponible) : la fois suivante, reportée
};

/// L'étape i attend le MMU (indisponible : flash, M709 S2, pas encore connecté) : remise à la fin de la série, les
/// autres gardent leur ordre, marquée deferred. Si elle l'était déjà, rien ne bouge : l'assistant la reporte (passée,
/// à refaire) au lieu d'un échec.
void defer(std::span<Step> steps, size_t i);

/// Étapes des tests automatiques de mask, dans l'ordre de `entries`, les tests par emplacement groupés par bobine (un
/// chargement par bobine) ; nombre d'étapes écrites (out trop court : coupé)
size_t build_steps(uint32_t mask, const Context &ctx, std::span<Step> out);

/// G-code de l'étape (une ligne) ; false pour un test manuel ou out trop court
bool step_gcode(const Step &step, std::span<char> out);

/// G-code de la mesure du PA d'un emplacement : seul endroit qui écrit les arguments de M1994 (od3d/f4-compact les
/// change). M1994 P<n> charge la bobine lui-même (mise en file) si elle n'est pas dans la buse.
bool pa_gcode(uint8_t slot, std::span<char> out);

/// Résultat d'un test fait de plusieurs étapes : un échec l'emporte, puis une étape passée, puis réussi
constexpr Result fold(Result so_far, Result step) {
    if (so_far == Result::none) {
        return step;
    }
    if (so_far == Result::failed || step == Result::failed) {
        return Result::failed;
    }
    if (so_far == Result::skipped || step == Result::skipped) {
        return Result::skipped;
    }
    return Result::ok;
}

/// Enregistrement après un essai : la version n'avance que sur un succès (sinon le test reste à refaire)
constexpr Record after_run(Record r, const Entry &e, Result result, uint32_t date) {
    r.result = result;
    r.date = date;
    if (result == Result::ok) {
        r.version = e.version;
    }
    return r;
}

// ---- Chiffres du bilan (demande du 22/11 : « vitesse améliorée de x %, vibrations réduites de x % ») ----
// Seulement des valeurs mesurées ou calculées par les tests ; une prédiction est affichée « estimé ».

inline constexpr int16_t no_value = std::numeric_limits<int16_t>::min();

/// Chiffres d'un test, avant ou après la série. Par emplacement (Need::filament) : v[emplacement] ; sinon propres au
/// test (calib_assistant_od3d.hpp, numbers_before() / numbers_after())
using Numbers = std::array<int16_t, 8>;
inline constexpr Numbers no_numbers { no_value, no_value, no_value, no_value, no_value, no_value, no_value, no_value };

/// Gardés après un succès (config_store « OD3D Calib Values », indice = Test) : le « avant » de la fois suivante pour
/// les tests dont le résultat n'est enregistré nulle part ailleurs (courroies, amortissement, vitesses, phase stepping).
/// Taille fixe : pour la changer, créer un nouvel élément sous un autre nom.
struct Values {
    std::array<int16_t, 4> v { no_value, no_value, no_value, no_value };

    constexpr bool operator==(const Values &) const = default;
};

/// Baisse en % (arrondie, négative si ça augmente) de remaining par rapport à reference ; nullopt sans référence
std::optional<int16_t> drop_percent(float remaining, float reference);

/// Oscillations (périodes) pour que l'amplitude d'une résonance tombe à 5 % avec l'amortissement zeta : ln 20 / (2π zeta)
float ringing_cycles(float zeta);

/// Buse froide avant de fixer l'accéléromètre (consigne de l'utilisateur : en dessous de 40 °C), chauffe coupée
inline constexpr float cold_nozzle_max = 40.f;

constexpr bool nozzle_cold(float current, float target) {
    return current < cold_nozzle_max && target <= 0.f; // NaN : jamais froide
}

} // namespace od3d::calib_registry
