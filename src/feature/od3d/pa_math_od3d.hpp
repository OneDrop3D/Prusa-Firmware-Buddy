/// @file
/// OneDrop3D (F4) : calibration du pressure advance (PA) par la cellule de charge, partie pure (sans firmware),
/// testée dans tests/unit/feature/od3d avec des courbes synthétiques.
///
/// Méthode : celle de PrusaPATuner (CNC Kitchen, https://github.com/CNCKitchen/PrusaPATuner, AGPL-3.0),
/// **réimplémentée ici d'après sa description, sans reprendre son code**. Elle-même s'inspire de la calibration
/// automatique des Bambu Lab A1, du calibrateur de débit du Snapmaker U1 (extrusion en créneaux dans le vide) et de
/// bd_pressure de markniu (analyse de la réponse à un échelon) ; lecture physique du guide d'Ellis (PA trop faible :
/// la force est en retard sur la commande ; trop fort : elle est en avance).
///
/// Principe : la buse extrude dans le vide en créneaux (débit lent, rapide, lent…) avec un déplacement en X (le PA
/// de Prusa n'agit que sur les mouvements avec XY), pour chaque K d'un balayage. La pression dans la buse se lit sur
/// la cellule de charge. Modèle du premier ordre : τ·dr/dt + r = u + K·du/dt (r : force normalisée, u : débit
/// commandé normalisé, τ : constante de temps de la buse). Le K idéal vaut τ : la force suit alors la commande.
/// Pour chaque K on mesure :
/// - le retard (« lag ») : aire entre commande et force, à la montée et à la descente, en secondes ; il vaut τ − K
///   dans le modèle (positif : PA trop faible, négatif : trop fort). Le K retenu est le zéro de la droite lag(K).
/// - le coût : intégrale du carré de l'écart force − commande ; minimal au bon K (contrôle croisé).
/// - le dépassement à la montée, le creux à la descente, l'amplitude du créneau et le bruit (seuils de validité).
#pragma once

#include "force_trace_od3d.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace od3d::pa {

inline constexpr float filament_area_mm2 = 2.4053f; ///< filament de 1,75 mm

/// Séquence de mesure d'un K (réglable par M1994) ; valeurs par défaut prudentes, à affiner sur la machine
struct Params {
    float slow_flow = 2.0f; ///< débit lent [mm³/s]
    float fast_flow = 15.0f; ///< débit rapide [mm³/s]
    float slow_s = 1.0f; ///< durée d'un palier lent [s] (la tête change de sens en son milieu)
    float fast_s = 0.3f; ///< durée d'un palier rapide [s]
    float e_per_mm = 0.08f; ///< filament par mm de déplacement XY [mm/mm] : fixe le rapport débit / vitesse
    uint8_t cycles = 5; ///< créneaux par K
};

/// Vitesses XY de la séquence [mm/s] et durées des paliers
struct Profile {
    float v_lo;
    float v_hi;
    float slow_s;
    float fast_s;
};

Profile profile(const Params &p);

/// Paramètres valides (débits, durées, rapport et nombre de créneaux dans des bornes raisonnables)
bool is_valid(const Params &p);

/// Durée d'un bloc (un K) : lancement lent (2 demi-paliers), créneaux, sans les rampes [s]
float block_duration_s(const Params &p);

/// Forme moyenne d'un créneau (pour l'écran) : force et commande normalisées (×64 : 64 = palier rapide), depuis
/// 0,1 s avant la montée
struct Waveform {
    static constexpr size_t points = 96;
    std::array<int8_t, points> force {};
    std::array<int8_t, points> command {};
    float step_s = 0; ///< pas de temps entre deux points ; 0 = vide
    float pre_s = 0.1f; ///< le premier point est pre_s avant la montée
};

/// Résultat de l'analyse d'un bloc (médianes sur les créneaux)
struct BlockResult {
    uint8_t cycles = 0; ///< créneaux exploitables
    float lag_s = 0; ///< retard équivalent (τ − K dans le modèle) [s]
    float cost = 0; ///< ∫(force − commande)² normalisé [s]
    float overshoot = 0; ///< dépassement à la montée, fraction du créneau
    float undershoot = 0; ///< creux sous le palier lent après la descente, fraction du créneau
    float amplitude_g = 0; ///< hauteur du créneau de force (signée : le sens dépend du montage de la cellule) [g]
    float noise = 0; ///< bruit relatif (écart type / amplitude) sur les paliers lents

    bool valid() const { return cycles > 0; }
};

/// Analyse la trace d'un bloc, échantillonnée au pas dt [s] ; wave (facultatif) reçoit la forme moyenne
BlockResult analyse_block(force::Trace t, float dt, const Profile &p, Waveform *wave = nullptr);

/// Seuils de validité et de choix (à régler d'après les enregistrements de l'utilisateur)
struct Thresholds {
    float min_amplitude_g = 3.0f; ///< créneau plus faible : signal trop faible
    float max_noise = 0.25f; ///< bruit relatif au-delà duquel le bloc est écarté
    float local_span = 0.04f; ///< second ajustement sur les K à ±local_span du premier zéro
    float min_slope = -3.0f; ///< pente lag(K) attendue vers −1 : hors de [min_slope, max_slope], refus
    float max_slope = -0.2f;
};

struct SweepPoint {
    float k;
    BlockResult r;
};

enum class Fit : uint8_t {
    ok,
    too_few_points, ///< moins de 3 blocs exploitables
    weak_signal, ///< créneau de force trop faible ou trop bruité
    no_crossing, ///< le retard ne dépend pas de K comme prévu (pente hors bornes)
    out_of_range, ///< le K trouvé sort de la plage balayée (k donné quand même)
};

struct Choice {
    Fit fit = Fit::too_few_points;
    float k = 0; ///< K retenu [s]
    uint8_t confidence = 0; ///< 0 à 100
    float slope = 0; ///< pente de lag(K)
    float k_lag = 0; ///< zéro de la droite lag(K)
    float k_cost = 0; ///< minimum du coût (interpolé)
    uint8_t used = 0; ///< blocs retenus
};

/// Choix de K d'après les blocs d'un balayage (dans n'importe quel ordre)
Choice choose_k(std::span<const SweepPoint> sweep, float k_min, float k_max, float k_step, const Thresholds &th = {});

/// Valeurs de K de lo à hi par pas step (hi compris à 1e-6 près) ; rend le nombre écrit
size_t sweep_values(float lo, float hi, float step, std::span<float> out);

/// count valeurs centrées sur center, espacées de step, bornées à [lo, hi] (sans doublon) ; rend le nombre écrit
size_t refine_values(float center, float step, uint8_t count, float lo, float hi, std::span<float> out);

/// Droite des moindres carrés y = a + b·x ; r2 : coefficient de détermination
struct LineFit {
    float a = 0;
    float b = 0;
    float r2 = 0;
    bool ok = false;
};
LineFit fit_line(std::span<const float> xs, std::span<const float> ys);

/// Abscisse du sommet de la parabole qui passe par 3 points (x distincts) ; x1 si les points sont alignés
float parabola_vertex(float x0, float y0, float x1, float y1, float x2, float y2);

/// Interpolation linéaire dans une table (xs croissants), bornée aux extrémités (PA adaptatif : K selon le débit)
float interpolate(std::span<const float> xs, std::span<const float> ys, float x);

/// Médiane (tableau modifié : trié) ; NAN si vide
float median(std::span<float> v);

/// K en entier de 1e-4 s pour la mémoire (0xFFFF = pas de valeur)
inline constexpr uint16_t k_none = 0xFFFF;
uint16_t k_to_e4(float k);
float k_from_e4(uint16_t e4);

/// Ce qui est gardé par emplacement (config_store, « OD3D PA Slot »). Taille fixe : pour changer la structure, créer
/// un nouvel élément (nouveau nom) sans réutiliser celui-ci.
struct Record {
    uint16_t k_e4 = k_none; ///< K mesuré [1e-4 s] ; k_none : à mesurer
    uint8_t confidence = 0; ///< confiance de la mesure, 0 à 100
    uint8_t reserved = 0;

    bool has_k() const { return k_e4 != k_none; }
    bool operator==(const Record &) const = default;
};
static_assert(sizeof(Record) == 4);

} // namespace od3d::pa
