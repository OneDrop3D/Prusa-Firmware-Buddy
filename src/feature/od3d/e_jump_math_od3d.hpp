/// @file
/// OneDrop3D (accélération du filament, M2000) : calculs purs, sans Marlin (testés dans tests/unit/feature/od3d).
///
/// Quand le filament accélère de a, le pressure advance de Prusa pousse l'extrudeur de K·a en plus : un saut de vitesse
/// de K·a, lissé sur le temps de lissage du PA (0,04 s). Prusa borne a à 120 mm/s² (M204 F,
/// DEFAULT_MAX_FILAMENT_ACCELERATION) quel que soit K ; la vraie limite est le saut que l'extrudeur suit sans caler.
/// M2000 mesure ce saut : paliers croissants d'impulsions de l'extrudeur seul (palier = saut J sur le temps de lissage,
/// au-dessus d'un débit de base), calage vu par le détecteur de filament coincé de Prusa (cellule de force). On garde
/// margin × le dernier palier sûr ; à l'impression, F = saut gardé / K, borné.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace od3d::e_jump {

/// Paliers de saut [mm/s] : de first_jump à max_jump, ×step_ratio. Le calage est au plus 25 % au-dessus du dernier
/// palier sûr ; 12 paliers de 2 impulsions : au plus ~60 mm de filament (Prusa purge 40 mm après un chargement)
inline constexpr float first_jump = 4.f;
inline constexpr float step_ratio = 1.25f;
inline constexpr float max_jump = 50.f;
inline constexpr size_t max_steps = 12;
inline constexpr uint8_t pulses = 2; ///< impulsions par palier : 2 détections = calage (comme le seau de Prusa, 2 en 2 s)

/// Part gardée du dernier palier sûr. Le vrai calage est entre 1 et 1,25 fois ce palier ; une impression ajoute un
/// débit plus fort, des sauts plus longs et la traînée de la bobine : on garde 56 à 70 % du calage
inline constexpr float margin = 0.7f;

/// Bornes de F [mm/s²] : jamais sous Prusa (une mesure basse est plus souvent une fausse détection) ; au-dessus de
/// 1500, la limite d'accélération de E de Prusa (M201 E1500) borne déjà le filament
inline constexpr float prusa_filament_accel = 120.f;
inline constexpr float max_filament_accel = 1500.f;

/// Cordon de référence du bilan (buse 0,4 de la CORE One) et accélération XY maximale (HWLIMIT_NORMAL_MAX_ACCELERATION)
inline constexpr float ref_width = 0.45f;
inline constexpr float ref_height = 0.2f;
inline constexpr float xy_accel_cap = 10000.f;
inline constexpr float filament_area_mm2 = 2.4053f; ///< filament de 1,75 mm

/// Sauts des paliers, croissants ; nombre écrit (max_steps au plus)
size_t jump_steps(std::span<float> out);

/// Une impulsion de l'extrudeur seul : base [mm/s] pendant before_mm, saut à base + jump (montée et descente sur le
/// temps de lissage, palier d'un temps de lissage), puis base pendant after_mm
struct Pulse {
    float base_speed = 0;
    float peak_speed = 0;
    float accel = 0; ///< [mm/s²] : jump / smooth_time
    float before_mm = 0;
    float jump_mm = 0;
    float after_mm = 0;

    float total_mm() const { return before_mm + jump_mm + after_mm; }
};
inline constexpr float base_speed = 2.f; ///< ~5 mm³/s : la buse est déjà sous pression quand le saut arrive
Pulse pulse(float jump, float smooth_time);

/// Résultat d'un palier
struct Step {
    float jump = 0; ///< [mm/s]
    uint8_t stalls = 0; ///< impulsions où le détecteur a vu un calage (sur pulses)
};

enum class StepState : uint8_t {
    clean, ///< aucune détection
    doubtful, ///< une détection sur deux : peut-être un parasite
    stalled, ///< deux détections : calage
};
StepState state(const Step &s);

/// La rampe s'arrête : le dernier palier a calé, ou les deux derniers sont douteux
bool must_stop(std::span<const Step> steps);

enum class Fit : uint8_t {
    none, ///< mesure incomplète (arrêt)
    found, ///< calage trouvé, au-dessus d'au moins un palier sûr
    not_reached, ///< pas de calage jusqu'au dernier palier : le saut est au moins le dernier palier sûr
    too_low, ///< calage dès le premier palier (ou pas de palier sûr avant) : rien d'enregistré
};

struct Verdict {
    Fit fit = Fit::none;
    float safe_jump = 0; ///< dernier palier sûr [mm/s]
    uint8_t safe_index = 0xff;
    uint8_t stop_index = 0xff; ///< palier du calage (0xff : aucun)

    float kept_jump() const { return margin * safe_jump; }
};

/// Classement des paliers mesurés ; complete : tous les paliers prévus ont été faits
Verdict classify(std::span<const Step> steps, bool complete);

/// F = saut / K [mm/s²], borné à [prusa_filament_accel, max_filament_accel] ; NAN sans saut ou sans PA (K <= 0 : pas de
/// saut à limiter)
float filament_accel(float jump, float k);

/// Accélération de ligne [mm/s²] que permet F pour un cordon width × height (section du trancheur : rectangle aux bouts
/// arrondis, (width − height·(1 − π/4))·height), au plus xy_accel_cap : une estimation
float line_accel(float f, float width = ref_width, float height = ref_height);

/// Mémoire par emplacement (config_store « OD3D E Jump »), effacée au changement de bobine. Taille fixe : pour la
/// changer, créer un nouvel élément sous un autre nom.
struct Record {
    enum Flag : uint8_t {
        lower_bound = 1 << 0, ///< pas de calage jusqu'au dernier palier : le saut sûr est au moins celui-là
    };
    uint16_t jump_e1 = 0; ///< saut gardé (marge comprise) [0,1 mm/s] ; 0 : pas de mesure
    uint8_t flags = 0;
    uint8_t reserved = 0;

    bool has_jump() const { return jump_e1 > 0; }
    float jump() const { return float(jump_e1) / 10.f; }
    bool operator==(const Record &) const = default;
};
static_assert(sizeof(Record) == 4);

uint16_t jump_to_e1(float jump);

} // namespace od3d::e_jump
