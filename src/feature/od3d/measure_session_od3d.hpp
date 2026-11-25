/// @file
/// OneDrop3D (F4, F5, F6, puis F9) : cadre commun des mesures par la cellule de charge, côté firmware (fil de Marlin).
///
/// Les mesures **impriment un motif** à hauteur de première couche à l'avant du plateau (pattern_math_od3d.hpp : dans
/// le vide, la force ne suit pas la pression dans la buse). Une Session :
/// - garde l'état d'avant (pressure advance, réglages du planificateur, mode furtif, consignes de la buse et du
///   plateau, maillage actif ou non), suspend la détection de filament coincé, la limite de débit (F5) et
///   l'accélération du filament mesurée (M2000, F de Prusa à la place) le temps de la mesure, et remet tout à sa destruction, même après un arrêt : petite rétraction, buse levée de 10 mm ;
/// - prépare le plateau comme le G-code de départ de PrusaSlicer pour la CORE One : plateau à la température de la
///   matière, buse à sa température de palpage (170 °C pour le PLA), origine, nettoyage de la buse par tapotements
///   (G29 P9), nivellement local de la zone du motif et de la purge (G29 P1, P3.2, P3.13, A), buse à la température
///   d'impression, ligne de purge au bord avant ;
/// - imprime des lignes (déplacements levés, rétraction).
/// Une seule session à la fois (les G-codes de mesure passent l'un après l'autre).
/// En impression (mesure automatique du PA au départ, in_print) : l'opération peut être passée depuis l'écran Impression
/// (« Passer » de Prusa, SkippableGCode), la détection de collision est suspendue, et la session s'arrête si
/// l'impression quitte l'état « en cours » (pause, arrêt).
#pragma once

#include "pattern_math_od3d.hpp"

#include <atomic>
#include <cstdint>
#include <optional>

namespace od3d::session {

inline constexpr uint8_t queue_depth = 5; ///< mouvements en file au plus : un arrêt prend effet en ~3 s
inline constexpr float edge_margin = 10.f; ///< mm gardés aux bords du plateau
inline constexpr float travel_speed_xy = 100.f;
inline constexpr float travel_speed_z = 10.f;
inline constexpr float hop = 1.f; ///< levée de la buse pour les déplacements au-dessus du motif [mm]
inline constexpr float end_lift = 10.f; ///< buse levée à la fin [mm]
inline constexpr int16_t max_temperature = 300; ///< demande bornée ; la protection de Prusa reste la seule limite réelle
inline constexpr int16_t max_bed_temperature = 110;
inline constexpr uint32_t heat_timeout_ms = 6 * 60 * 1000;
inline constexpr uint32_t bed_timeout_ms = 12 * 60 * 1000;

// Ligne de purge du G-code de départ de PrusaSlicer pour la CORE One (bord avant du plateau, de droite à gauche)
inline constexpr float purge_x = 249.f;
inline constexpr float purge_y = -2.5f;
inline constexpr float purge_z = 15.f; ///< hauteur de départ, au-dessus du début de la purge
inline constexpr float purge_length = 50.f; ///< longueur de la purge le long de X, essuyage compris

enum class Result : uint8_t {
    ok,
    stopped, ///< arrêt demandé, impression lancée ou arrêt de Marlin
    homing, ///< origine impossible
    no_room, ///< le motif ne tient pas dans la zone imprimable
    heating, ///< la buse ou le plateau n'a pas atteint la consigne à temps (ou consigne retirée)
    leveling, ///< palpage du plateau impossible
};

/// La mesure doit s'arrêter : arrêt demandé (session en cours), Marlin vide ses files ; hors impression : impression
/// lancée, imprimante occupée par autre chose ; en impression : « Passer », pause ou arrêt de l'impression
bool must_stop();

/// Attend ms millisecondes (marche de Marlin assurée) ; false s'il faut s'arrêter
bool wait_ms(uint32_t ms);

/// Attend la fin des mouvements en file ; false s'il faut s'arrêter
bool wait_moves();

/// Attend une place dans la file des mouvements ; false s'il faut s'arrêter
bool wait_queue(uint8_t depth = queue_depth);

/// Zone où un motif peut être imprimé : le plateau moins edge_margin, dans la course
pattern::Rect printable_zone();

// ---- Série de l'assistant de calibration (M1989 R) ----
// Retours du 23/11 : chaque bobine imprimait au même endroit, le plateau était palpé à chaque test, la purge du
// chargement tombait dans le vide, la buse redescendait entre deux matières. Pendant une série, en mémoire vive
// seulement (config_store plein) :
// - chaque motif et chaque ligne de purge a sa place sur le plateau (pattern::Shelf), sans chevauchement ;
// - le plateau est palpé une seule fois, en entier, avant le premier motif ; les tests suivants reprennent ce maillage
//   (origine refaite seulement si elle est perdue). shortcut: maillage gardé quelle que soit la température du plateau
//   de chaque matière (palpé à celle de la première) ; re-palper si les mesures l'exigent ;
// - la fin d'un test laisse le plateau chauffer, et la buse aussi tant qu'une bobine y est : le test suivant va
//   directement à sa température. La série coupe tout à sa fin.

/// Début de série ou plateau nettoyé (« Plateau plein » de l'assistant) : place libre partout, rien de palpé
void series_begin();
/// Fin de série : retour au fonctionnement d'avant
void series_end();
bool in_series();

/// Plateau plein : une place demandée depuis series_begin() n'a pas été trouvée
bool series_full();

/// Série : place pour un motif de w × h (series_full() si elle manque) ; hors série : nullopt
std::optional<pattern::Rect> claim(float w, float h);

/// Départ de la ligne de purge (Session::purge, le long de X dans le sens dir)
struct PurgeStart {
    float x;
    float y;
    float dir;
};

/// Hors série : le bord avant de Prusa. Série : la place réservée sur le plateau, gardée jusqu'à ce que la purge soit
/// faite (le chargement qui précède le test se fait au-dessus de son départ, load_spot()) ; nullopt : plateau plein
std::optional<PurgeStart> purge_start();

/// Où M701 et M702 (chargement, déchargement de Prusa) se font, au-dessus du plateau : ce qui sort de la buse (pointe
/// de l'ancien filament façonnée par le MMU, bout du nouveau) se pose sur une place libre au lieu de pendre de la buse
/// garée en hauteur (retour du 24/11 : « il purge en hauteur et du coup je dois enlever le filament à la main »)
struct LoadSpot {
    float x;
    float y;
    float z; ///< absolue (l'origine est faite)
};

/// Hauteur du chargement de la mesure automatique au départ d'une impression : le plastique (~70 mm³ au plus : ramming
/// de 20 mm de l'ancien filament, 10 mm du nouveau, poussés à l'arrêt par le MMU de Prusa) se pose sur le plateau, la
/// buse reste au-dessus du petit tas
inline constexpr float load_z = 5.f;

/// Recul laissé par la dernière session à sa fin [mm] (sa rétraction contre le suintement) : le PA automatique le remet
/// avec le sien, avant la purge du G-code de départ
float left_retracted_mm();

/// Série : au-dessus de la ligne de purge réservée, à purge_z. Sinon la place posée par set_load_spot(), rendue une
/// seule fois (le M701 ou M702 qui la prend). nullopt : position de chargement de Prusa
std::optional<LoadSpot> take_load_spot();

/// Mesure automatique au départ d'une impression : le prochain M701 ou M702 se fait là ; nullopt : retirée
void set_load_spot(std::optional<LoadSpot> spot);

/// Série : le maillage du plateau entier est en mémoire et l'origine est là ; le test va directement à sa température
/// d'impression (pas de palpage, donc pas de descente à la température de palpage)
bool mesh_ready();

/// Fil de Marlin, à chaque tour (marlin_server, aussi pendant un G-code) : en série, suit le plateau (bed_steady_ms())
void step();

/// Série : depuis combien de temps le plateau tient sa consigne actuelle à ±2 °C, sans interruption [ms] ; 0 : pas
/// encore, ou hors série. La dilatation de la buse compte son repos du plateau depuis là (préchauffé pendant les tests
/// du MMU, calib_registry::bed_step() : rien à réattendre)
uint32_t bed_steady_ms();

class Session {
public:
    /// stop_requested : drapeau d'arrêt de la mesure (posé par l'interface) ; in_print : au départ d'une impression
    explicit Session(const std::atomic<bool> &stop_requested, bool in_print = false);
    ~Session();
    Session(const Session &) = delete;
    Session &operator=(const Session &) = delete;

    /// Temps de lissage du PA d'avant (0,04 s s'il n'y en avait pas) : à garder quand on change K
    float smooth_time() const;

    /// Plateau à bed (et buse à nozzle_probe en même temps, sans l'attendre), puis attente du plateau
    Result heat_bed(int16_t bed, int16_t nozzle_probe);

    /// Buse à nozzle_probe (attendue), puis origine : Z refaite au premier appel de la session (jamais celle d'avant,
    /// peut-être fausse), X et Y si nécessaire ; peut se faire pendant que le plateau chauffe, comme le G-code de départ
    /// de la CORE One (M140, M109 R170, G28, puis M190)
    Result home(int16_t nozzle_probe);

    /// home(), puis nettoyage de la buse, nivellement local de area et de la zone de purge, maillage activé (G-code de
    /// départ de la CORE One ; plateau chaud d'abord). Un arrêt demandé pendant le palpage prend effet après.
    /// Série : le plateau entier la première fois (rien n'y est encore imprimé), ensuite le maillage repris.
    Result level(const pattern::Rect &area, int16_t nozzle_probe);

    /// Buse à la consigne, puis attente : à ±tolerance, et y rester hold_ms ; échec après timeout_ms ou si la
    /// consigne est retirée entre-temps. La consigne réelle peut être bornée par Prusa : on attend celle-là.
    /// extrude : la consigne doit permettre d'extruder (sinon : palpage, refroidissement).
    Result heat(int16_t temperature, float tolerance = 2.f, uint32_t hold_ms = 3000, uint32_t timeout_ms = heat_timeout_ms, bool extrude = true);

    /// Va au-dessus du départ de la ligne de purge (bord avant droit par défaut), à purge_z du plateau ; dir : sens de
    /// la purge qui suivra (sa fin doit rester dans la course) ; along_y : purge le long de Y
    Result go_to_purge(float x = purge_x, float y = purge_y, float dir = -1.f, bool along_y = false);

    /// Ligne de purge du G-code de départ de la CORE One depuis le départ x (à l'Y actuel), le long de X dans le sens
    /// dir (-1 : vers la gauche, comme Prusa au bord avant) ; false s'il faut s'arrêter. Série : sa place est libérée
    /// de la réservation (la suivante en prendra une autre). along_y (motif tourné de la mesure automatique) : départ
    /// y = x à l'X actuel, le long de Y
    bool purge(float x = purge_x, float dir = -1.f, bool along_y = false);

    /// Déplacement au-dessus du motif : rétraction, levée de hop au-dessus de z, (x, y), puis descente à z si lower
    bool travel_to(float x, float y, float z, bool lower = true);

    /// Descend (ou monte) à z sans bouger en XY
    bool move_z(float z);

    /// Ligne imprimée de la position actuelle à (x, y) : e_per_mm mm de filament par mm, à speed mm/s ; mise en file
    /// sans attendre la fin (attente d'une place) ; remet d'abord le filament rétracté. false s'il faut s'arrêter
    bool print_to(float x, float y, float e_per_mm, float speed);

    /// Rétraction contre le suintement (une seule fois tant que le filament n'est pas remis), de
    /// pattern::retract_mm() selon la matière dans la buse
    void retract();
    /// Rétraction de parking de Prusa (STANDARD_RETRACT_LENGTH en tout), avant un trajet et une attente buse chaude ;
    /// jamais pour un flexible
    void park_retract();
    /// Remet le filament rétracté (rien sinon)
    void deretract();
};

} // namespace od3d::session
