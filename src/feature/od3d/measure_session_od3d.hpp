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

    /// Buse à nozzle_probe (attendue), puis origine si nécessaire : peut se faire pendant que le plateau chauffe, comme
    /// le G-code de départ de la CORE One (M140, M109 R170, G28, puis M190)
    Result home(int16_t nozzle_probe);

    /// home(), puis nettoyage de la buse, nivellement local de area et de la zone de purge, maillage activé (G-code de
    /// départ de la CORE One ; plateau chaud d'abord). Un arrêt demandé pendant le palpage prend effet après.
    Result level(const pattern::Rect &area, int16_t nozzle_probe);

    /// Buse à la consigne, puis attente : à ±tolerance, et y rester hold_ms ; échec après timeout_ms ou si la
    /// consigne est retirée entre-temps. La consigne réelle peut être bornée par Prusa : on attend celle-là.
    /// extrude : la consigne doit permettre d'extruder (sinon : palpage, refroidissement).
    Result heat(int16_t temperature, float tolerance = 2.f, uint32_t hold_ms = 3000, uint32_t timeout_ms = heat_timeout_ms, bool extrude = true);

    /// Va au-dessus du départ de la ligne de purge (bord avant droit par défaut), à purge_z du plateau ; dir : sens de
    /// la purge qui suivra (sa fin doit rester dans la course)
    Result go_to_purge(float x = purge_x, float y = purge_y, float dir = -1.f);

    /// Ligne de purge du G-code de départ de la CORE One depuis le départ x (à l'Y actuel), le long de X dans le sens
    /// dir (-1 : vers la gauche, comme Prusa au bord avant) ; false s'il faut s'arrêter
    bool purge(float x = purge_x, float dir = -1.f);

    /// Déplacement au-dessus du motif : rétraction, levée de hop au-dessus de z, (x, y), puis descente à z si lower
    bool travel_to(float x, float y, float z, bool lower = true);

    /// Descend (ou monte) à z sans bouger en XY
    bool move_z(float z);

    /// Ligne imprimée de la position actuelle à (x, y) : e_per_mm mm de filament par mm, à speed mm/s ; mise en file
    /// sans attendre la fin (attente d'une place) ; remet d'abord le filament rétracté. false s'il faut s'arrêter
    bool print_to(float x, float y, float e_per_mm, float speed);

    /// Rétraction contre le suintement (une seule fois tant que le filament n'est pas remis)
    void retract();
    /// Remet le filament rétracté (rien sinon)
    void deretract();
};

} // namespace od3d::session
