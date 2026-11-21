/// @file
/// OneDrop3D (F4, F5, F6, puis F9) : cadre commun des mesures par la cellule de charge, côté firmware (fil de Marlin).
///
/// Une Session garde l'état d'avant (pressure advance, réglages du planificateur, mode furtif, consigne de la buse),
/// suspend la détection de filament coincé et la limite de débit (F5) le temps de la mesure, et remet tout à sa
/// destruction, même après un arrêt, après une petite rétraction contre le suintement. Elle sait aussi mettre la buse
/// au-dessus du plateau à hauteur sûre, la chauffer et purger. Une seule session à la fois (les G-codes de mesure
/// passent l'un après l'autre).
#pragma once

#include <atomic>
#include <cstdint>

namespace od3d::session {

inline constexpr uint8_t queue_depth = 5; ///< mouvements en file au plus : un arrêt prend effet en ~3 s
inline constexpr float edge_margin = 10.f; ///< mm gardés aux bords de la course
inline constexpr float travel_speed_xy = 100.f;
inline constexpr float travel_speed_z = 10.f;
inline constexpr int16_t max_temperature = 300; ///< demande bornée ; la protection de Prusa reste la seule limite réelle
inline constexpr uint32_t heat_timeout_ms = 6 * 60 * 1000;

enum class Result : uint8_t {
    ok,
    stopped, ///< arrêt demandé, impression lancée ou arrêt de Marlin
    homing, ///< origine impossible
    no_room, ///< la séquence ne tient pas dans la course
    heating, ///< la buse n'a pas atteint la consigne à temps (ou consigne retirée)
};

/// La mesure doit s'arrêter : arrêt demandé (session en cours), Marlin vide ses files, impression lancée, imprimante
/// occupée par autre chose
bool must_stop();

/// Attend ms millisecondes (marche de Marlin assurée) ; false s'il faut s'arrêter
bool wait_ms(uint32_t ms);

/// Attend la fin des mouvements en file ; false s'il faut s'arrêter
bool wait_moves();

/// Attend une place dans la file des mouvements ; false s'il faut s'arrêter
bool wait_queue(uint8_t depth = queue_depth);

class Session {
public:
    /// stop_requested : drapeau d'arrêt de la mesure (posé par l'interface)
    explicit Session(const std::atomic<bool> &stop_requested);
    ~Session();
    Session(const Session &) = delete;
    Session &operator=(const Session &) = delete;

    /// Temps de lissage du PA d'avant (0,04 s s'il n'y en avait pas) : à garder quand on change K
    float smooth_time() const;

    /// Origine si besoin, hauteur sûre (au moins z, et 10 mm au-dessus de ce qui a été imprimé), puis (x_start, y).
    /// La tête ira de x_lo à x_hi en X et de y à y + y_span en Y : refus si ça ne tient pas dans la course (marges de
    /// 10 mm) ; y est ramené dans la course.
    Result go_to(float x_lo, float x_hi, float x_start, float &y, float y_span, float z);

    /// Consigne de la buse, puis attente : à ±tolerance, et y rester hold_ms ; échec après timeout_ms ou si la
    /// consigne est retirée entre-temps. La consigne réelle peut être bornée par Prusa : on attend celle-là.
    Result heat(int16_t temperature, float tolerance = 2.f, uint32_t hold_ms = 3000, uint32_t timeout_ms = heat_timeout_ms);

    /// Purge sur place : mm de filament à speed mm/s, puis 0,5 s ; false s'il faut s'arrêter
    bool prime(float mm, float speed);
};

} // namespace od3d::session
