/// @file
/// OneDrop3D (F4) : calibration du pressure advance (PA) par la cellule de charge, partie pure (sans firmware),
/// testée dans tests/unit/feature/od3d sur des signaux synthétiques et sur des CSV enregistrés (rejouables).
///
/// Méthode : celle de PrusaPATuner (CNC Kitchen, https://github.com/CNCKitchen/PrusaPATuner, AGPL-3.0), étudiée en
/// détail puis **réécrite ici sans reprendre son code** ; elle-même vient des Bambu Lab A1 (idée), du calibrateur de
/// débit du Snapmaker U1 (créneaux d'extrusion dans le vide) et de bd_pressure de markniu (analyse de la réponse à un
/// échelon) ; lecture physique du guide d'Ellis (PA trop faible : la force traîne derrière la commande ; trop fort :
/// elle la devance, avec dépassement et creux).
///
/// Protocole (valeurs par défaut de PrusaPATuner) : la buse extrude **dans le vide**, à une position de parcage sûre,
/// des créneaux asymétriques « lent 3 mm³/s pendant 2 s (6 mm³), rapide 14 mm³/s pendant 0,714 s (10 mm³) », avec un
/// petit aller-retour en X (1 mm : aller pendant le lent, retour pendant le rapide), sans lequel le PA de Prusa ne
/// s'applique pas (il n'agit que sur les mouvements avec XY) ; accélération de 500 mm/s². Une séquence par K :
/// lent de lancement, puis `cycles` × (rapide, lent). Le premier créneau de chaque K est écarté (il porte l'histoire
/// du K précédent). La force est la charge tarée de la cellule (`loadcell_value` des metrics de Prusa), lue ici à la
/// source (interruption de la cellule, ~107 points/s) avec la position X et celle du moteur E au même instant.
///
/// Analyse de chaque créneau bas-haut-bas (bd_pressure), après une tare locale (médiane du palier lent avant la
/// montée, qui retire la dérive lente) : retard et erreur de montée, dépassement, pente du palier, retard de descente,
/// creux, aire de la traîne, temps de retour au calme. Médianes par K, puis chaque critère est ramené à son échelle
/// sur le balayage (90e centile) et pondéré ; le K retenu est le minimum de ce coût composé (interpolé par une
/// parabole). Deux contrôles croisés : l'aire signée de la descente (positive : traîne, PA trop faible ; négative :
/// creux, PA trop fort) qui change de signe au bon K, et l'estimation par créneau K + aire / amplitude.
#pragma once

#include "force_trace_od3d.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace od3d::pa {

inline constexpr float filament_area_mm2 = 2.4053f; ///< filament de 1,75 mm

/// Séquence d'un K (réglable par M1994) : valeurs par défaut de PrusaPATuner
struct Params {
    float slow_flow = 3.0f; ///< débit lent [mm³/s]
    float fast_flow = 14.0f; ///< débit rapide [mm³/s]
    float slow_s = 2.0f; ///< durée d'un palier lent [s] (6 mm³)
    float fast_s = 0.714f; ///< durée d'un palier rapide [s] (10 mm³)
    float wiggle = 1.0f; ///< aller-retour en X [mm] : aller pendant le lent, retour pendant le rapide
    float accel = 500.f; ///< accélération des mouvements (et limite de E) [mm/s²]
    uint8_t cycles = 5; ///< créneaux par K (le premier est écarté)
};

/// Vitesses de la séquence
struct Profile {
    float e_slow; ///< filament [mm/s]
    float e_fast;
    float v_slow; ///< tête le long de X [mm/s]
    float v_fast;
    float slow_s;
    float fast_s;
};

Profile profile(const Params &p);

inline constexpr float max_block_s = 17.f; ///< durée d'une séquence au plus (mémoire de la trace)

/// Paramètres valides (débits, durées, rapport, aller-retour, accélération, créneaux, durée de la séquence)
bool is_valid(const Params &p);

/// Durée d'une séquence (un K) : lent de lancement, puis cycles × (rapide + lent), sans les rampes [s]
float block_duration_s(const Params &p);

/// Front de la commande : demi-tour de X (montée : lent → rapide, descente : rapide → lent)
struct Turn {
    uint16_t index;
    bool rise;
};

/// Demi-tours de X dans la trace (hystérésis de threshold mm), le premier étant une montée ; rend le nombre écrit
size_t find_turns(force::Trace t, float threshold_mm, std::span<Turn> out);

/// Débit commandé (sans le PA) au point i d'après les fronts [mm³/s] : lent avant la première montée
float command_flow(std::span<const Turn> turns, size_t i, const Params &p);

/// Critères de la réponse à un créneau (bd_pressure), pondérés dans le coût composé
enum Metric : uint8_t {
    rise_error, ///< ∫|palier − force| sur le palier rapide [g·s]
    overshoot, ///< pic au-dessus du palier rapide, moins le pic attendu du bruit [g]
    undershoot, ///< creux sous le palier lent après la descente, moins celui du bruit [g]
    tail, ///< ∫|force| sur la traîne après la descente [g·s]
    plateau_slope, ///< pente du palier rapide [g/s] (en valeur absolue dans le coût)
    rise_delay, ///< de la montée au pic [s]
    fall_delay, ///< du début de la descente au creux [s]
    settling, ///< de la descente au retour au calme (±3 σ pendant 0,1 s) [s]
    metric_count,
};

/// Poids de PrusaPATuner (BD_DEFAULT_WEIGHTS)
inline constexpr std::array<float, metric_count> weights { 1.f, 2.f, 2.f, 1.f, 0.5f, 1.f, 1.f, 0.5f };

/// Un créneau analysé
struct Segment {
    bool ok = false; ///< retenu (amplitude nette au-dessus du bruit, fenêtres complètes)
    std::array<float, metric_count> m {};
    float high_g = 0; ///< palier rapide au-dessus du palier lent [g]
    float noise_g = 0; ///< écart type sur le palier lent [g]
    float fall_area = 0; ///< ∫ force après la descente [g·s] : > 0 traîne (PA faible), < 0 creux (PA fort)
    float k_estimate = 0; ///< K + fall_area / high_g
};

/// Forme moyenne d'un créneau (pour l'écran) : force et commande normalisées (×64 : 64 = palier rapide), depuis
/// pre_s avant la montée
struct Waveform {
    static constexpr size_t points = 96;
    std::array<int8_t, points> force {};
    std::array<int8_t, points> command {};
    float step_s = 0; ///< pas de temps entre deux points ; 0 = vide
    float pre_s = 0.1f;
};

/// Résultat d'une séquence (un K) : médianes sur les créneaux retenus
struct BlockResult {
    uint8_t found = 0; ///< créneaux trouvés (demi-tours de X)
    uint8_t cycles = 0; ///< créneaux retenus (le premier est toujours écarté)
    std::array<float, metric_count> m {};
    float amplitude_g = 0; ///< hauteur du créneau de force, signée (le sens dépend de la cellule) [g]
    float noise_g = 0; ///< bruit sur le palier lent [g]
    float fall_area = 0;
    float k_estimate = 0;
    bool saturated = false; ///< plus de 1 % des points à la borne de la trace : signal écrêté, inexploitable

    bool valid() const { return cycles > 0; }
};

/// Analyse la trace d'une séquence échantillonnée au pas dt [s], pour le K appliqué k ; wave (facultatif) reçoit la
/// forme moyenne
BlockResult analyse_block(force::Trace t, float dt, const Params &p, float k, Waveform *wave = nullptr);

/// Seuils de validité et de choix (à régler d'après les enregistrements de l'utilisateur)
struct Thresholds {
    float min_amplitude_g = 10.f; ///< créneau plus faible : signal trop faible
    float max_noise = 0.3f; ///< bruit / amplitude au-delà duquel le K est écarté
    uint8_t min_segments = 3; ///< créneaux retenus au moins par K
    float min_depth = 0.15f; ///< creux du coût sous sa médiane, relatif : en dessous, le coût ne dépend pas de K
};

struct SweepPoint {
    float k;
    BlockResult r;
};

enum class Fit : uint8_t {
    ok,
    too_few_points, ///< moins de 4 K exploitables
    weak_signal, ///< créneau de force trop faible ou trop bruité
    no_crossing, ///< le coût ne dépend pas de K (« la force ne suit pas K »)
    out_of_range, ///< minimum au K le plus haut balayé (k donné quand même)
    saturated, ///< force écrêtée à la borne de la trace (relevé du 22/11 : −655 g sur toute la ligne)
};

struct Choice {
    Fit fit = Fit::too_few_points;
    float k = 0; ///< K retenu [s]
    uint8_t confidence = 0; ///< 0 à 100
    float k_cost = 0; ///< minimum du coût composé (interpolé)
    float k_segments = 0; ///< médiane des estimations par créneau (NAN sans)
    float k_signed = 0; ///< changement de signe de l'aire de la descente (NAN sans)
    float depth = 0; ///< creux relatif du coût
    uint8_t used = 0; ///< K retenus
};

/// Choix de K d'après les séquences d'un balayage (dans n'importe quel ordre) ; costs (facultatif, même ordre que
/// sweep) reçoit le coût composé de chaque K (NAN : écarté)
Choice choose_k(std::span<const SweepPoint> sweep, float k_step, const Thresholds &th = {}, std::span<float> costs = {});

/// Valeurs de K de lo à hi par pas step (hi compris à 1e-6 près) ; rend le nombre écrit
size_t sweep_values(float lo, float hi, float step, std::span<float> out);

/// count valeurs centrées sur center, espacées de step, bornées à [lo, hi] (sans doublon) ; rend le nombre écrit
size_t refine_values(float center, float step, uint8_t count, float lo, float hi, std::span<float> out);

/// Droite des moindres carrés y = a + b·x ; r2 : coefficient de détermination (F6)
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

// ---- CSV de la mesure (écrit sur la clé, relu par les tests pour rejouer les fichiers de l'utilisateur) ----

/// Ligne « P,slow_flow,slow_s,fast_flow,fast_s,wiggle,accel,cycles »
int format_params(char *buf, size_t size, const Params &p);
bool parse_params(const char *line, Params &p);

/// Ligne « D,k,t_s,x_mm,e_mm,force_g,cmd_mm3s » : un point de la trace brute et la commande reconstituée ; la relecture
/// accepte aussi l'ancien format « D,k,t_s,x_mm,force_g » (motif imprimé, jusqu'à la 8.0.2.21 : e_mm 0, commande NAN)
struct CsvPoint {
    float k;
    float t_s;
    float x_mm;
    float e_mm;
    float force_g;
    float command;
};
int format_point(char *buf, size_t size, const CsvPoint &p);
bool parse_point(const char *line, CsvPoint &p);

/// K en entier de 1e-4 s pour la mémoire (0xFFFF = pas de valeur)
inline constexpr uint16_t k_none = 0xFFFF;
uint16_t k_to_e4(float k);
float k_from_e4(uint16_t e4);

/// Ce qui est gardé par emplacement (config_store, « OD3D PA Slot »). Taille fixe : pour changer la structure, créer
/// un nouvel élément (nouveau nom) sans réutiliser celui-ci.
struct Record {
    enum Source : uint8_t {
        measured = 0, ///< mesure par la cellule de charge
        chosen = 1, ///< ligne du motif choisie à l'œil par l'utilisateur
    };

    uint16_t k_e4 = k_none; ///< K mesuré [1e-4 s] ; k_none : à mesurer
    uint8_t confidence = 0; ///< confiance de la mesure, 0 à 100 (100 pour une ligne choisie)
    uint8_t source = measured; ///< ancien octet de réserve, à 0 dans les mémoires d'avant : « mesuré »

    bool has_k() const { return k_e4 != k_none; }
    bool operator==(const Record &) const = default;
};
static_assert(sizeof(Record) == 4);

} // namespace od3d::pa
