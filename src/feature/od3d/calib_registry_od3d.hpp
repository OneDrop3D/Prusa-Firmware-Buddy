/// @file
/// OneDrop3D : registre des calibrations de l'assistant (demande du 22/11 : « lorsque l'on installe le firmware
/// personnalisé il propose de faire tous les tests […] à chaque mise à jour qui modifie les calibrations il demande de
/// les refaire »). Pur : ni Marlin, ni interface, ni config_store (testé dans tests/unit/feature/od3d).
///
/// RÈGLE POUR LES AGENTS — la version d'une entrée (Entry::version) est la seule source de vérité :
/// - un commit qui change ce que mesure une calibration (algorithme, paramètres, format ou usage du résultat), au point
///   qu'il faille la refaire pour en profiter, augmente sa version de 1 dans `entries` ;
/// - l'assistant compare avec la version enregistrée (config_store « OD3D Calib Done ») et coche d'office seulement les
///   calibrations dont la version a augmenté ; il s'ouvre seul **une fois** au premier démarrage d'un firmware dont
///   signature() a changé (demande du 24/11 : « au premier démarrage sur ce nouveau firmware il doit proposer les
///   nouvelles calibrations » ; le 23/11, la question à chaque démarrage avait été retirée) ;
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
    /// N3, M2001 V : temps de montée du ventilateur pièce, règle l'avance de F26 (sans besoin) ; version 2 : puis les
    /// vibrations de chaque ventilateur si l'accéléromètre répond (vibration_od3d.hpp ; pas de place pour un Test de
    /// plus : max_tests est atteint avec accel_limit)
    fan_response,
    nozzle_expansion, ///< M2003 : dilatation de la buse palpée de 170 à 260 °C (buse vide)
    e_jump, ///< M2000 par emplacement : saut de vitesse de l'extrudeur, accélération max du filament
    mmu_speed, ///< N4, M2002 P : vitesse de la poulie du MMU (une bobine préchargée, rien dans l'extrudeur)
    accel_limit, ///< M2004 : accélération max utile de X et Y (accéléromètre), une information (rien n'est appliqué)
    /// M2005 : pose de l'accéléromètre (inclinaison, axes, fixation), en premier de la série « Accéléromètre » (retour
    /// du 24/11 : « encore plus de calibrations avec l'accéléromètre »)
    accel_mount,
    shaper_check, ///< M2006 : vérification de l'input shaper (vibrations avec et sans filtre), après accel_limit
    count,
};

/// Places des tests : les 16 premières dans « OD3D Calib Done » / « OD3D Calib Values » ; le config_store est plein
/// (CS-1) : les suivantes n'ont que leur version et leur résultat, rangés dans Record::reserved des premières
/// (calib_assistant::stored_record()). 18 : la grille de l'assistant (3 × 6 tuiles)
inline constexpr size_t base_tests = 16;
inline constexpr size_t max_tests = 18;
static_assert(size_t(Test::count) <= max_tests && max_tests >= base_tests);

/// Besoins d'une calibration (bits)
enum Need : uint8_t {
    accelerometer = 1 << 0, ///< accéléromètre fixé sur la buse (phase 1 de l'assistant)
    cold_nozzle = 1 << 1, ///< buse sous cold_nozzle_max avant de fixer l'accéléromètre et pendant la mesure
    filament = 1 << 2, ///< une mesure par emplacement qui a une bobine, bobine chargée dans la buse
    mmu = 1 << 3, ///< MMU activé
    empty_bed = 1 << 4, ///< imprime ou palpe : plateau vide et propre confirmé avant
    flow_limit = 1 << 5, ///< seulement avec « Limiter au débit mesuré » (le résultat ne sert qu'à lui)
    one_spool = 1 << 6, ///< une seule mesure, avec la bobine du premier emplacement qui en a une
    /// Série « Chauffe » (retour du 23/11 : une série à froid avec l'accéléromètre, une autre qui chauffe) : le test
    /// chauffe, ou veut l'extrudeur vide (déchargement à chaud, partagé avec la dilatation de la buse). Sans ce bit :
    /// série « Accéléromètre », à froid du début à la fin
    heated = 1 << 7,
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
/// Need::filament). Deux séries lancées séparément (Need::heated, series_tests()) :
/// - « Accéléromètre », à froid : pose de l'accéléromètre (d'abord), courroies, phase stepping, input shaper,
///   amortissement, vitesses critiques, accélération max utile (après l'input shaper : mesurée avec le filtre en place),
///   vérification de l'input shaper, et les ventilateurs (temps de montée sans accéléromètre, vibrations avec lui s'il
///   répond : rien n'y chauffe) ;
/// - « Chauffe » : MPC, tube PTFE et vitesse de la poulie (à froid, mais l'extrudeur doit être vide : le déchargement à
///   chaud est partagé avec la dilatation de la buse ; build_steps() les groupe avant elle, quel que soit leur ordre
///   ici, et le plateau chauffe pour elle pendant ce temps), dilatation de la buse, puis les mesures par bobine.
inline constexpr std::array entries {
    // La pose de l'accéléromètre avant tout : tourné, de travers ou mal fixé, il fausserait toutes les mesures qui suivent
    Entry { Test::accel_mount, accelerometer | cold_nozzle, 1, 2, true }, // v2 : orientation à plat plus jugée
    Entry { Test::belts, accelerometer | cold_nozzle, 3, 1, true }, // courroies d'abord : elles changent les résonances
    Entry { Test::phase_stepping, accelerometer | cold_nozzle, 8, 1, true },
    Entry { Test::input_shaper, accelerometer | cold_nozzle, 6, 1, true },
    Entry { Test::damping, accelerometer | cold_nozzle, 4, 2, true }, // v2 : gain du plan horizontal
    Entry { Test::speed_map, accelerometer | cold_nozzle, 2, 2, true }, // après le phase stepping (F34) ; v2 : 20 à 80 mm/s, base locale (V-07)
    Entry { Test::accel_limit, accelerometer | cold_nozzle, 1, 2, true }, // après l'input shaper : avec le filtre en place ; v2 : rampe de l'axe (V-05)
    Entry { Test::shaper_check, accelerometer | cold_nozzle, 1, 2, true }, // le filtre choisi, vérifié sur la machine ; v2 : rampe de l'axe (V-05)
    Entry { Test::mpc, heated, 12, 1, true },
    Entry { Test::mmu_ptfe, mmu | one_spool | heated, 3, 1, true },
    Entry { Test::nozzle_expansion, empty_bed | heated, 10, 1, true }, // après le tube PTFE : avec le MMU, l'extrudeur est déjà vide
    Entry { Test::pa, filament | empty_bed | heated, 3, 1, true }, // chargement compris ; motif compact, plateau palpé une fois par série
    Entry { Test::max_flow, filament | empty_bed | flow_limit | heated, 3, 1, true },
    Entry { Test::min_temp, filament | empty_bed | heated, 7, 1, true },
    Entry { Test::skew, empty_bed | heated, 0, 1, false },
    Entry { Test::fan_response, 0, 2, 3, true }, // à froid, sans besoin : série « Accéléromètre » ; v2 : vibrations ; v3 : période étalonnée (V-08)
    Entry { Test::e_jump, filament | heated, 1, 1, true }, // rien d'imprimé ni de palpé : purge à la position de Prusa
    Entry { Test::mmu_speed, mmu | one_spool | heated, 4, 1, true }, // après le tube PTFE : sa longueur fait l'avance rapide
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
    uint16_t reserved = 0; ///< enregistrements 0 et suivants : version et résultat des tests base_tests et suivants (un octet chacun)
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
    /// Par test (indice Test), bit n : l'emplacement n a déjà une valeur pour la bobine en place (K du PA, débit maximal,
    /// température minimale, accélération du filament ; effacée au changement de bobine)
    std::array<uint8_t, max_tests> measured {};
    /// Ordre des bobines pour les tests par bobine (spool_order()) ; les emplacements absents de spools sont ignorés
    std::array<uint8_t, 8> order { 0, 1, 2, 3, 4, 5, 6, 7 };
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

/// Tests automatiques d'une série : « Chauffe » (heat) ou « Accéléromètre », à froid (voir entries)
uint32_t series_tests(bool heat);

/// Tests par bobine dont toutes les bobines sont à refaire : leur version a augmenté depuis la dernière série qui les
/// a menés à bien (une mesure faite à la main, version 0 ici, reste bonne)
uint32_t redo_mask(std::span<const Record> records);

/// Emplacements d'un test par bobine à mesurer : ceux qui n'ont pas encore de valeur pour la bobine en place, ou tous
/// avec redo
constexpr uint8_t spools_to_measure(Test test, const Context &ctx, bool redo) {
    return redo ? ctx.spools : uint8_t(ctx.spools & ~ctx.measured[size_t(test) % max_tests]);
}

/// Tests automatiques cochés d'office dans la proposition (retour du 23/11 : « j'avais déjà calibré le PA de
/// 2 filaments et il a essayé de refaire le PA de tous les filaments ») : un test machine jamais réussi dans cette
/// version ; un test par bobine si une bobine n'a pas encore de valeur (seulement elle sera mesurée) ou si sa version a
/// augmenté (redo_mask()). Ce qui est déjà fait n'est pas coché ; le cocher à l'écran le refait (toutes les bobines).
uint32_t proposed(const Context &ctx, std::span<const Record> records);

/// Ordre des bobines d'une série (accélérer : jamais refroidir le plateau entre deux bobines, qui prend plusieurs
/// minutes) : plateau, puis buse, de la plus froide à la plus chaude ; à égalité, la bobine déjà dans la buse d'abord.
/// bed, nozzle : consignes de la matière de chaque emplacement
std::array<uint8_t, 8> spool_order(std::span<const int16_t> bed, std::span<const int16_t> nozzle, std::optional<uint8_t> loaded);

/// Empreinte des versions du registre sur 16 bits, jamais 0 (0 : jamais proposée) : elle change dès qu'une version
/// change ou qu'un test arrive
uint16_t signature(std::span<const Entry> list = entries);

/// Durée estimée des tests automatiques de mask [min] ; redo : tests par bobine à refaire sur toutes les bobines
uint16_t minutes(uint32_t mask, const Context &ctx, uint32_t redo = 0);

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

/// Étapes des tests automatiques de mask, dans l'ordre de `entries` (qui est aussi celui de la grille de l'assistant),
/// sauf les tests du MMU à froid (bed_free()), groupés juste avant la dilatation de la buse ; les tests par emplacement
/// groupés par bobine (un chargement par bobine, bobines dans ctx.order), la température minimale en dernier pour
/// chaque bobine (elle finit sous la température de la matière : rien ne remonte après elle) ; par bobine, seulement
/// les emplacements sans valeur, sauf pour les tests de redo (spools_to_measure()). Nombre d'étapes écrites (out trop
/// court : coupé)
size_t build_steps(uint32_t mask, const Context &ctx, std::span<Step> out, uint32_t redo = 0);

/// Le test ne dépend ni du plateau ni de l'air de la chambre : le plateau peut chauffer pendant lui pour un test suivant
/// (tests du MMU à froid : une longueur et des vitesses de la poulie, que la chambre qui tiédit ne change pas). Pas le
/// MPC : il mesure les pertes de la buse dans l'air de la chambre
constexpr bool bed_free(Test test) {
    return (entry(test).needs & mmu) != 0;
}

/// Étape dont la consigne du plateau est posée dès le lancement de l'étape i (retour du 24/11 : « optimise la vitesse
/// de test ») : i elle-même si un chargement ou un déchargement la précède (loading : rien n'est mesuré pendant ce temps),
/// sinon, si i laisse le plateau libre (bed_free()), la prochaine étape qui ne l'est pas. Seulement une étape qui
/// imprime ou palpe (Need::empty_bed) : elle attend toujours son plateau elle-même, qui est simplement déjà chaud.
/// nullopt : rien à préchauffer
std::optional<size_t> bed_step(std::span<const Step> steps, size_t i, bool loading);

/// G-code de l'étape (une ligne) ; false pour un test manuel ou out trop court
bool step_gcode(const Step &step, std::span<char> out);

/// G-code de la mesure du PA d'un emplacement : seul endroit qui écrit les arguments de M1994 (od3d/f4-compact les
/// change) ; toujours le motif compact (V1), aussi précis à ±0,004 près et 3 à 4 fois plus court que le grand.
/// M1994 P<n> charge la bobine lui-même (mise en file) si elle n'est pas dans la buse.
bool pa_gcode(uint8_t slot, std::span<char> out);

/// Buse pour changer de bobine pendant la série (retour du 23/11 : « la température de la buse entre le passage du PLA
/// au PETG descend beaucoup ») : la plus haute de la matière dans la buse (à décharger) et de la nouvelle (à charger),
/// sans redescendre entre les deux ; loaded : température de la matière dans la buse, s'il y en a une
constexpr int16_t load_temperature(std::optional<int16_t> loaded, int16_t next) {
    return loaded && *loaded > next ? *loaded : next;
}

/// Chargement au-dessus du départ de la ligne de purge réservée au test sur le plateau, à z
struct Spot {
    float x;
    float y;
    float z;
};

/// Chargement de la bobine slot avant son test, mis en file d'injection par l'assistant (« Charger dans la buse » de
/// Prusa, jamais depuis la pile d'un test) : origine si besoin (G28 O I), buse à nozzle (load_temperature()), puis :
/// - test qui imprime (spot) : plateau palpé d'abord s'il ne l'est pas encore dans la série (M1989 L<slot>, avant que du
///   filament pende de la buse), tête au-dessus du départ de la purge (G27 … P1 : Z absolu, après l'origine), chargement
///   là (M701 s'y gare aussi s'il décharge d'abord une autre bobine) : ce qui sort de la buse est posé par la ligne de
///   purge du test, comme la purge du G-code de départ de PrusaSlicer pour la CORE One, au lieu de tomber dans le vide ;
/// - sinon : position de chargement de Prusa (G27 W2), comme avant.
/// filament : paramètre S de M701 ; false si out est trop court
bool load_gcode(uint8_t slot, const char *filament, int16_t nozzle, std::optional<Spot> spot, std::span<char> out);

/// Bilan : les chiffres d'un test sont montrés. Test par bobine : dès qu'une bobine en a, même si une autre a échoué
/// ou a été passée (retour du 23/11 : « il n'a pas retrouvé le PA d'un filament qu'il avait calibré » : l'échec d'une
/// bobine cachait les K de toutes)
bool numbers_shown(const Entry &e, Result result, std::span<const int16_t> after);

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
