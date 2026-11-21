/// @file
/// OneDrop3D (F5, F6) : calibrations de la bobine par la cellule de charge, partie pure (sans firmware), testée dans
/// tests/unit/feature/od3d avec des courbes synthétiques.
///
/// Les deux mesures font des paliers d'extrusion à débit constant ; chaque palier donne la force moyenne de la cellule
/// (relative à la tête au repos juste avant) et son écart type.
///
/// F5 — débit maximal : paliers de débit croissants à la température de la matière. Tant que la buse fond assez de
/// filament, la force suit une loi douce, force ≈ a·Q^n avec n < 1 (le plastique fondu coule plus facilement quand il
/// va plus vite). Quand elle ne fond plus assez, la force monte plus vite que cette loi : c'est le coude. Si
/// l'extrudeur saute des pas, la force chute ou devient très bruitée. La loi est ajustée sur les premiers paliers
/// (droite des moindres carrés en log-log) ; le coude est le débit où la force dépasse la loi de plus de `excess`
/// (interpolé entre deux paliers), confirmé par le palier suivant.
///
/// F6 — température minimale : à débit constant modéré, la température descend par paliers depuis celle de la
/// matière ; on garde la plus basse température essayée dont la force reste sous `max_ratio` × la force du premier
/// palier (température nominale).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace od3d::calib {

/// Un palier mesuré
struct StepPoint {
    float x; ///< débit [mm³/s] (F5) ou température [°C] (F6)
    float force_g; ///< force moyenne du palier, relative au repos [g] (signe selon le montage de la cellule)
    float noise_g; ///< écart type de la force sur le palier [g]
};

// ---- F5 : coude de la courbe force/débit ----

/// Seuils (valeurs de départ, à régler d'après les CSV de l'utilisateur)
struct KneeThresholds {
    uint8_t ref_points = 4; ///< premiers paliers qui servent à la loi de référence
    float excess = 0.25f; ///< dépassement relatif de la loi qui marque le coude
    float drop = 0.10f; ///< chute relative de la force d'un palier au suivant : l'extrudeur saute
    float max_noise = 0.25f; ///< écart type relatif au-delà duquel le palier est jugé « extrudeur qui saute »
    float min_force_g = 5.f; ///< plus forte force de la série en dessous : signal trop faible
    float min_exponent = 0.2f; ///< exposant n de la loi borné à [min_exponent, max_exponent]
    float max_exponent = 1.2f;
};

enum class KneeFit : uint8_t {
    found, ///< coude trouvé : flow est le débit maximal
    not_reached, ///< pas de coude dans la rampe : flow est le plus fort débit essayé (borne basse)
    too_few_points, ///< pas assez de paliers pour la loi de référence et un palier au-delà
    weak_signal, ///< force trop faible, ou nulle / de signe contraire sur les paliers de référence
};

struct Knee {
    KneeFit fit = KneeFit::too_few_points;
    float flow = 0; ///< débit maximal [mm³/s] (voir fit)
    float sign = 1; ///< signe de la force (montage de la cellule)
    float a = 0; ///< loi de référence : force (signe corrigé) = a·Q^n
    float n = 0;
    uint8_t index = 0; ///< premier palier au-delà du coude (found), sinon nombre de paliers
    bool skipping = false; ///< coude vu par une chute ou un bruit de la force (extrudeur qui saute)
    bool pending = false; ///< dépassement sur le dernier palier, à confirmer par le suivant (final faux)

    /// Force prévue par la loi de référence (signe corrigé) [g]
    float reference(float flow) const;
};

/// Cherche le coude dans les paliers (débits croissants). final faux pendant la rampe : un dépassement sur le dernier
/// palier attend sa confirmation (pending) ; final vrai à la fin : il est retenu.
Knee find_knee(std::span<const StepPoint> points, bool final, const KneeThresholds &th = {});

// ---- F6 : température minimale ----

struct MinTempThresholds {
    float max_ratio = 1.5f; ///< force acceptable jusqu'à max_ratio × la force à la température nominale
    float max_noise = 0.25f; ///< écart type relatif au-delà duquel le palier n'est pas acceptable
    float min_force_g = 3.f; ///< force à la température nominale en dessous : signal trop faible
};

enum class MinTempFit : uint8_t {
    found, ///< un palier dépasse le seuil : temperature est le palier précédent
    floor_reached, ///< tous acceptables jusqu'au plancher : temperature est le plancher (on pourrait descendre plus)
    not_reached, ///< série interrompue avant le plancher, sans dépassement
    too_few_points, ///< moins de 2 paliers
    weak_signal, ///< force trop faible à la température nominale
};

struct MinTemp {
    MinTempFit fit = MinTempFit::too_few_points;
    float temperature = 0; ///< plus basse température essayée acceptable [°C]
    float reference_g = 0; ///< force à la température nominale (signe corrigé) [g]
    float sign = 1;
    uint8_t index = 0; ///< premier palier refusé (found), sinon nombre de paliers
};

/// Plus basse température acceptable des paliers (températures décroissantes, le premier à la température nominale) ;
/// floor : plancher de la série [°C]
MinTemp find_min_temperature(std::span<const StepPoint> points, float floor, const MinTempThresholds &th = {});

/// Températures de la série : start, start − step… jusqu'au plancher compris (ajouté s'il ne tombe pas sur un pas) ;
/// rend le nombre écrit (0 si start ≤ floor ou step ≤ 0)
size_t temperature_steps(int16_t start, int16_t floor, int16_t step, std::span<int16_t> out);

// ---- Mémoire (config_store « OD3D Spool Calib », par emplacement) ----

inline constexpr uint16_t flow_none = 0xFFFF;

/// Débit en entier de 0,1 mm³/s (flow_none sans valeur)
uint16_t flow_to_e1(float flow);
float flow_from_e1(uint16_t e1);

/// Ce qui est gardé par emplacement, effacé au changement de bobine. Taille fixe : pour changer la structure, créer un
/// nouvel élément (nouveau nom) sans réutiliser celui-ci.
struct Record {
    enum Flag : uint8_t {
        flow_lower_bound = 1 << 0, ///< pas de coude dans la rampe : max_flow est le plus fort débit essayé
        temperature_floor = 1 << 1, ///< plancher atteint sans dépassement : on pourrait descendre plus bas
    };

    uint16_t max_flow_e1 = flow_none; ///< débit maximal [0,1 mm³/s]
    int16_t flow_temperature = 0; ///< consigne de la buse pendant la mesure du débit [°C]
    int16_t min_temperature = 0; ///< température minimale [°C] ; 0 : pas mesurée
    uint8_t flags = 0;
    uint8_t reserved = 0;

    bool has_flow() const { return max_flow_e1 != flow_none; }
    bool has_min_temperature() const { return min_temperature > 0; }
    bool operator==(const Record &) const = default;
};
static_assert(sizeof(Record) == 8);

/// Limite de vitesse du filament [mm/s] pour un débit maximal [mm³/s], bornée à [floor, ceiling] mm³/s ; 0 sans débit
float e_speed_limit(float max_flow, float floor, float ceiling, float filament_area_mm2);

} // namespace od3d::calib
