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
    // Longueur du PTFE avant du MMU (autre agent) : ajouter `mmu_ptfe` ici, son entrée dans `entries` (après min_temp,
    // needs mmu | filament ou non selon la mesure) et son G-code dans step_gcode()
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
};

struct Entry {
    Test test;
    uint8_t needs; ///< Need
    uint8_t minutes; ///< durée estimée (par emplacement avec Need::filament)
    uint8_t version; ///< version de l'algorithme : voir la règle en tête du fichier
    bool automatic; ///< false : à faire à la main depuis son écran (listée, jamais lancée)
};

/// Dans l'ordre d'exécution : d'abord l'accéléromètre (buse froide, une seule pose), puis le MPC (il part de la buse
/// froide), puis par emplacement PA, débit, température minimale (un seul chargement par bobine, chauffe gardée)
inline constexpr std::array entries {
    Entry { Test::belts, accelerometer | cold_nozzle, 3, 1, true }, // courroies d'abord : elles changent les résonances
    Entry { Test::phase_stepping, accelerometer | cold_nozzle, 8, 1, true },
    Entry { Test::input_shaper, accelerometer | cold_nozzle, 6, 1, true },
    Entry { Test::damping, accelerometer | cold_nozzle, 4, 1, true },
    Entry { Test::speed_map, accelerometer | cold_nozzle, 6, 1, true }, // après le phase stepping (F34)
    Entry { Test::mpc, 0, 12, 1, true },
    Entry { Test::pa, filament | empty_bed, 7, 1, true },
    Entry { Test::max_flow, filament | empty_bed | flow_limit, 5, 1, true },
    Entry { Test::min_temp, filament | empty_bed, 6, 1, true },
    Entry { Test::skew, empty_bed, 0, 1, false },
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

constexpr uint32_t bit(Test test) {
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

/// Proposer l'assistant au démarrage : un test automatique à faire, et ce registre n'a pas été écarté par « Ne plus
/// demander » (dismissed = signature() enregistrée à ce moment)
bool should_propose(const Context &ctx, std::span<const Record> records, uint32_t dismissed);

/// Empreinte des versions du registre : elle change dès qu'une version change
uint32_t signature();

/// Durée estimée des tests automatiques de mask [min]
uint16_t minutes(uint32_t mask, const Context &ctx);

/// Une étape de l'assistant : un G-code mis en file, puis le suivant (jamais imbriqués)
struct Step {
    Test test;
    uint8_t slot = 0; ///< emplacement, pour Need::filament
};

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

/// Buse froide avant de fixer l'accéléromètre (consigne de l'utilisateur : en dessous de 40 °C), chauffe coupée
inline constexpr float cold_nozzle_max = 40.f;

constexpr bool nozzle_cold(float current, float target) {
    return current < cold_nozzle_max && target <= 0.f; // NaN : jamais froide
}

} // namespace od3d::calib_registry
