/// @file
/// OneDrop3D (F4) : calibration du pressure advance (PA) par la cellule de charge, partie pure (sans firmware),
/// testée dans tests/unit/feature/od3d avec des courbes synthétiques.
///
/// Méthode : inspirée de PrusaPATuner (CNC Kitchen, https://github.com/CNCKitchen/PrusaPATuner, AGPL-3.0),
/// **réimplémentée ici d'après sa description, sans reprendre son code**, et des motifs PA des Bambu Lab / d'Orca
/// (lignes avec changements de vitesse) ; lecture physique du guide d'Ellis (PA trop faible : la force est en retard
/// sur la commande ; trop fort : elle est en avance). PrusaPATuner extrude dans le vide ; ici la buse **imprime** :
/// dans le vide la poussée de l'extrudeur et la contre-pression de la buse se compensent presque dans l'ensemble pesé
/// par la cellule de charge, alors que le plastique écrasé entre la buse et le plateau la repousse vers le haut avec
/// une force qui suit la pression dans la buse (retour de l'utilisateur, 22/11 : « la force ne suit pas K » dans le vide).
///
/// Principe : pour chaque K d'un balayage, une ligne à hauteur de première couche le long de X, imprimée lente,
/// rapide, lente… (lancement lent, puis `cycles` fois rapide + lent). La force verticale (positive vers le haut) se lit
/// sur la cellule de charge. Modèle : τ·dp/dt + p = débit commandé + K·d(débit)/dt (p : pression, τ : constante de temps
/// de la buse) ; la force a deux parts, la pression elle-même et l'écrasement du cordon (filament déposé par mm, qui ne
/// bouge qu'aux changements de vitesse si le PA est faux). Le K idéal vaut τ : les deux parts suivent alors la commande.
/// Pour chaque K on mesure :
/// - le retard (« lag ») : aire entre commande et force normalisées, à la montée et à la descente, en secondes ; il
///   s'annule au bon K (positif : PA trop faible, négatif : trop fort) et décroît avec K (pente −1 pour la pression
///   seule, plus raide avec l'écrasement). Le K retenu est le zéro de lag(K) : interpolé entre les deux K voisins qui
///   l'encadrent, sinon celui de la droite des moindres carrés.
/// - le coût : intégrale du carré de l'écart force − commande ; minimal au bon K (contrôle croisé).
/// - le dépassement à la montée, le creux à la descente, l'amplitude du créneau et le bruit (seuils de validité).
#pragma once

#include "force_trace_od3d.hpp"
#include "pattern_math_od3d.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace od3d::pa {

inline constexpr float filament_area_mm2 = pattern::filament_area_mm2; ///< filament de 1,75 mm

/// Ligne de mesure d'un K (réglable par M1994) ; valeurs par défaut prudentes, à affiner sur la machine
struct Params {
    float slow_flow = 1.5f; ///< débit lent [mm³/s]
    float fast_flow = 10.0f; ///< débit rapide [mm³/s]
    float slow_s = 0.8f; ///< durée d'un tronçon lent [s]
    float fast_s = 0.3f; ///< durée d'un tronçon rapide [s]
    pattern::Bead bead { 0.45f, 0.2f }; ///< cordon : largeur et hauteur de première couche [mm]
    uint8_t cycles = 3; ///< tronçons rapides par ligne
};

/// Vitesses XY de la séquence [mm/s] et durées des paliers
struct Profile {
    float v_lo;
    float v_hi;
    float slow_s;
    float fast_s;
};

Profile profile(const Params &p);

inline constexpr float max_line_length = 200.f; ///< [mm]
inline constexpr float max_line_speed = 300.f; ///< [mm/s]

/// Paramètres valides (cordon, débits, durées, rapport, nombre de tronçons, vitesse et longueur de la ligne)
bool is_valid(const Params &p);

/// Durée d'une ligne (un K) : lancement lent, puis cycles × (rapide + lent), sans les rampes [s]
float block_duration_s(const Params &p);

/// Longueur d'une ligne [mm]
float line_length(const Params &p);

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
    float min_slope = -8.0f; ///< pente lag(K) : −1 (pression seule) ou plus raide (écrasement du cordon) ; hors de
    float max_slope = -0.2f; ///< [min_slope, max_slope], refus
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

inline constexpr size_t air_lines = 2; ///< lignes sans force avant de conclure que la buse imprime en l'air

/// La buse imprime en l'air (retour du 24/11 : origine Z fausse, la mesure continuait sans toucher le plateau) : au
/// moins air_lines lignes, aucune avec un créneau de force (min_amplitude_g) ; dans le vide, la poussée de l'extrudeur
/// et la contre-pression de la buse se compensent
bool in_air(std::span<const SweepPoint> done, const Thresholds &th = {});

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

/// M-02 : clé de validité d'une mesure par emplacement (K, débit maximal, température minimale) : la buse (diamètre en
/// µm, haut débit) et la matière (son nom). 7 bits, jamais 0 : 0 est une mesure d'avant la clé (adopt()).
/// shortcut: empreinte de 7 bits (deux contextes sur 127 se confondent) ; une vraie clé demande de la place dans le
/// config_store, plein
constexpr uint8_t make_key(uint16_t nozzle_um, bool high_flow, std::string_view material) {
    uint32_t h = 2166136261u; // FNV-1a
    const auto mix = [&h](uint8_t byte) { h = (h ^ byte) * 16777619u; };
    mix(uint8_t(nozzle_um));
    mix(uint8_t(nozzle_um >> 8));
    mix(high_flow ? 1 : 0);
    for (const char c : material) {
        mix(uint8_t(c));
    }
    return uint8_t(h % 127u + 1u);
}

/// M-02 : r s'il a été mesuré avec la clé key, sinon un enregistrement vide : la valeur ne sert pas (pa::Record,
/// calib::Record)
template <class R>
constexpr R if_key(const R &r, uint8_t key) {
    return r.key == key ? r : R {};
}

/// Mesure d'avant la clé (clé 0 : 8.0.3.37 et avant) : adoptée une fois avec la clé de maintenant key (0 : matière
/// inconnue, rien n'est adopté) ; elle compte alors comme mesurée avec la configuration présente (retour du 25/11 :
/// « Remesurer le PA » après la mise à jour). has_data : r garde une valeur. true : r a pris la clé, à enregistrer
template <class R>
constexpr bool adopt(R &r, uint8_t key, bool has_data) {
    if (r.key != 0 || key == 0 || !has_data) {
        return false;
    }
    r.key = key;
    return true;
}

/// M-10 : un K ne remplace le PA du trancheur qu'à partir de cette confiance ; il est enregistré dès 30 (mesure validée
/// le 22/11) et montré « peu sûr » en dessous. shortcut: seuil à régler sur des mesures réelles (CSV)
inline constexpr uint8_t apply_confidence = 60;

/// Ce qui est gardé par emplacement (config_store, « OD3D PA Slot »). Taille fixe : pour changer la structure, créer
/// un nouvel élément (nouveau nom) sans réutiliser celui-ci.
struct Record {
    enum Source : uint8_t {
        measured = 0, ///< mesure par la cellule de charge
        chosen = 1, ///< ligne du motif choisie à l'œil par l'utilisateur
    };

    uint16_t k_e4 = k_none; ///< K mesuré [1e-4 s] ; k_none : à mesurer
    uint8_t confidence = 0; ///< confiance de la mesure, 0 à 100 (100 pour une ligne choisie)
    uint8_t source : 1 = measured; ///< bit 0 de l'ancien octet de réserve, à 0 dans les mémoires d'avant : « mesuré »
    uint8_t key : 7 = 0; ///< M-02 : make_key() de la mesure (bits libres du même octet) ; 0 : d'avant la clé

    bool has_k() const { return k_e4 != k_none; }
    /// Le K remplace celui du trancheur (une ligne choisie à l'œil a la confiance 100)
    bool applies() const { return has_k() && confidence >= apply_confidence; }
    bool operator==(const Record &) const = default;
};
static_assert(sizeof(Record) == 4);

// ---- Mesure automatique au départ d'une impression (comme la calibration de la dynamique du débit des Bambu) ----

/// Réglage « PA automatique » (config_store « OD3D PA Auto ») ; l'ancien « à chaque impression » (2) est lu comme
/// first_print : la règle ne mesure jamais une couleur qui a déjà un K
enum class AutoMode : uint8_t {
    off = 0,
    first_print = 1, ///< à la 1re impression d'une bobine (auto_choice)
};
inline constexpr uint8_t auto_mode_count = 2;

/// Règle de la mesure automatique (retour du 25/11 : « le PA automatique ne se fait que quand il n'y a qu'une couleur
/// qui n'est pas calibrée et que cette couleur est sur la première couche ») : used, settled : emplacements de
/// l'impression, et ceux qui n'attendent pas de mesure (K valable, ou flexible : jamais mesuré) ; first : emplacement
/// de la première couche (outil de départ), s'il est connu. La bobine mesurée reste dans la buse pour l'impression.
struct AutoChoice {
    uint8_t uncalibrated = 0; ///< emplacements de l'impression qui attendent une mesure (bit n : emplacement n)
    std::optional<uint8_t> slot; ///< à mesurer : la seule couleur à calibrer, si c'est celle de la première couche
};
AutoChoice auto_choice(AutoMode mode, uint8_t used, uint8_t settled, std::optional<uint8_t> first);

/// Où imprimer un motif de w × h avant l'impression sans la gêner : dans zone, à clearance au moins de la zone
/// d'impression print (M555), de la zone que le G-code de départ de PrusaSlicer pour la CORE One palpe près de sa
/// purge (G29 P1 X150 Y0 W100 H20 C ; l'origine Z y est aussi) et de chaque point que Prusa palpera autour de la pièce
/// (points majeurs xs × ys de la grille dans print agrandie de grow_x, grow_y : G29 P1). Le plus à l'avant possible,
/// contre le bord gauche, sinon le droit ; si rien ne tient contre un bord, le plus à l'avant puis le plus à gauche
/// ailleurs ; vide si rien ne tient.
std::optional<pattern::Rect> auto_spot(const pattern::Rect &zone, const pattern::Rect &print, float w, float h, std::span<const float> xs,
    std::span<const float> ys, float grow_x, float grow_y, float clearance = 5.f);

/// Place du bloc de la mesure automatique (w × h) : auto_spot(), sinon le bloc tourné de 90° (h × w, along_y : lignes
/// parallèles à Y) ; vide si rien ne tient
std::optional<pattern::Rect> auto_place(bool &along_y, const pattern::Rect &zone, const pattern::Rect &print, float w, float h,
    std::span<const float> xs, std::span<const float> ys, float grow_x, float grow_y, float clearance = 5.f);

// ---- Demande de mesure (M1994) ----

inline constexpr uint8_t max_lines = 15; ///< lignes du motif au plus (grossier + fin)

struct Request {
    uint8_t slot = 0;
    float k_min = 0.f;
    float k_max = 0.10f;
    float k_step = 0.02f;
    uint8_t refine = 5; ///< lignes du second balayage, au pas k_step / 4, autour du premier choix (0 : aucune)
    Params params;
    int16_t temperature = 0; ///< buse ; 0 : celle de la matière de l'emplacement
    int16_t bed_temperature = 0; ///< plateau ; 0 : celle de la matière
    float x = 20.f; ///< début des lignes (gauche)
    float y = 12.f; ///< première ligne, à l'avant du plateau (les suivantes vers l'arrière)
    float pitch = 5.f; ///< écart entre deux lignes [mm]
    bool csv = false; ///< écrit /usb/OD3D_PA<n>.CSV
    bool compact = false; ///< motif compact et recherche adaptative (sinon : grand motif, balayage complet, repères)
};

// ---- Motif compact (F4-b, demande du 22/11) ----
// Comme la calibration de la dynamique du débit de la Bambu X1 Carbon (3 lignes de 135 mm, à 2 mm d'écart, à l'avant
// du plateau) : lignes serrées à l'avant, sans repères, plus courtes (2 créneaux, paliers plus brefs) ; une ligne par
// K choisi au fur et à mesure (next_k()) au lieu du balayage complet. Le grand motif (repères, « Meilleure ligne »)
// reste le mode manuel.

inline constexpr float compact_pitch = 2.5f; ///< écart entre deux lignes [mm] (Bambu : 2 mm)
inline constexpr uint8_t compact_cap = 8; ///< lignes au plus (place réservée sur le plateau)
inline constexpr float default_prior = 0.04f; ///< K de départ sans mesure de l'emplacement ni de la matière

/// Passe la demande en motif compact : 2 créneaux, paliers de 0,6 s et 0,25 s (ligne de ~86 mm au lieu de 153),
/// écart compact_pitch, pas de second balayage
void make_compact(Request &rq);

/// Recherche adaptative de K (motif compact)
struct Search {
    float prior = default_prior; ///< K de départ : dernier K de l'emplacement, sinon de la matière
    float k_min = 0.f;
    float k_max = 0.10f;
    float step = 0.02f; ///< pas de la montée ; l'affinage se fait au demi-pas
    uint8_t cap = compact_cap; ///< lignes au plus
    uint8_t min_confidence = 30; ///< choix assez sûr pour s'arrêter (sinon : balayage élargi)
};

/// K de la ligne suivante d'après les lignes déjà imprimées (done, dans leur ordre) ; nullopt : fini (le choix se fait
/// ensuite par choose_k() sur toutes les lignes). Le retard dit de quel côté est le bon K (positif : K trop faible) :
/// - 1re ligne au K de départ (prior, sur la grille k_min + i·step) ;
/// - on avance d'un pas tant que le retard garde son signe (les lignes s'améliorent) ;
/// - dès qu'il change de signe, le bon K est encadré : une ligne d'un pas au-delà de la moins bonne des deux (coût)
///   confirme, puis une ligne au demi-pas dans l'encadrement affine ;
/// - bord de la plage atteint, ligne trop faible ou retard non monotone (bruit) : si choose_k() n'est pas assez sûr,
///   balayage élargi (points de la grille les plus proches pas encore imprimés), jusqu'à cap lignes.
/// Typiquement 4 à 6 lignes au lieu de 11.
std::optional<float> next_k(std::span<const SweepPoint> done, const Search &s, const Thresholds &th = {});

// ---- Chargement par le MMU : jamais depuis la pile de M1994 ----
// Le chargement appelé depuis M1994 a fait déborder la pile de Marlin (8.0.2.23, BSOD « STACK_OVF marlin » pendant
// « Déchargement vers la FINDA »). M1994 met donc en file les G-codes de Prusa et rend la main ; la mesure suit dans un
// autre M1994. Suites de G-codes séparés par '\n' ; filament : paramètre S codé de la matière (« #<n> », comme le menu
// de Prusa) ; false si out est trop court.

/// Chaque chargement ou déchargement est précédé de G28 O I (origine si besoin) et G27 W2 (position de chargement de
/// Prusa, buse levée) : jamais de purge ni de descente de Z avant l'origine.
///
/// En un clic, bobine pas dans la buse : plateau mis à chauffer (M140 S<bed>, si bed > 0) pendant le chargement,
/// « Charger dans la buse » de Prusa (M701 S"<matière>" W2 P<n> : l'autre filament est déchargé d'abord), puis la même
/// demande avec J1 (chargement tenté : pas de seconde tentative)
bool oneclick_sequence(std::span<char> out, const Request &rq, const char *filament, int bed = 0);

/// Mesure automatique de slot : son chargement comme ci-dessus si filament (sinon : déjà dans la buse), puis
/// M1994 P<n> A1. level_first : M1994 A2 avant le chargement, le plateau palpé d'abord, pour que le chargement se pose
/// sur une place que le palpage ne touchera plus
bool auto_step_sequence(std::span<char> out, uint8_t slot, const char *filament, bool level_first = false);

/// Après la mesure automatique, la buse reste chaude pendant le G-code de départ (retour du 24/11 : « passe après le PA
/// de 230 à 170 puis va remonter, c'est une grosse perte de temps ») : seules ses consignes de palpage et d'attente
/// (M109 R170, M104 S100 : de 1 à start_hold_below − 1 °C, sous la consigne actuelle) sont ignorées, et le filament
/// recule contre le suintement pendant le palpage chaud (décision du 24/11 : « au lieu de revenir a 170, il faudrait
/// juste rétracter le filament »). Toute autre demande (impression, 0) est suivie et lève le verrou (N-01, M-03).
inline constexpr int16_t start_hold_below = 180;
/// Recul en plus du G1 E-2 du G-code de départ : 8 mm en tout, la rétraction de parking de Prusa (STANDARD_RETRACT_LENGTH)
/// pour une buse qui attend chaude. shortcut: à régler sur la machine, d'après le suintement sur les points palpés
inline constexpr float start_hold_retract_mm = 6.f;

struct StartHold {
    bool active = false; ///< mesure automatique faite, chauffe d'impression pas encore demandée
    bool retracted = false; ///< filament reculé de start_hold_retract_mm
};

struct HoldStep {
    int16_t target; ///< consigne à poser
    float e_mm; ///< filament à bouger en plus [mm], sans changer sa position logique (négatif : recul)
};

/// Demande de consigne requested (M104 S, M109 S/R), consigne actuelle current ; can_retract : filament dans la buse,
/// buse assez chaude, pas un flexible. Le filament recule une fois, à une demande ignorée, et revient à la levée.
constexpr HoldStep start_hold_step(StartHold &hold, int16_t requested, int16_t current, bool can_retract) {
    if (!hold.active) {
        return { requested, 0.f };
    }
    if (requested > 0 && requested < start_hold_below && requested < current) {
        const bool retract = can_retract && !hold.retracted;
        hold.retracted = hold.retracted || retract;
        return { current, retract ? -start_hold_retract_mm : 0.f };
    }
    const float back = hold.retracted ? start_hold_retract_mm : 0.f;
    hold = {};
    return { requested, back };
}

} // namespace od3d::pa
