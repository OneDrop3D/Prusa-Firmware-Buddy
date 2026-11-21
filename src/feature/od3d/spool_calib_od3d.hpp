/// @file
/// OneDrop3D (F5, F6) : calibrations de la bobine par la cellule de charge, par emplacement du MMU.
///
/// - F5, débit maximal (M1995) : à la température de la matière, paliers de débit croissants ; la force de chaque
///   palier donne la courbe force/débit, dont le coude est le débit maximal (spool_calib_math_od3d.hpp). Option
///   « limiter au débit mesuré » (coupée par défaut) : le planificateur ralentit un mouvement d'impression dont le
///   filament irait plus vite (flow_limit_od3d.hpp) ; la limite est bornée à [5, 40] mm³/s.
/// - F6, température minimale (M1996) : à débit constant modéré, la température descend par paliers depuis celle de
///   la matière (jamais au-dessus), jamais sous la température minimale d'extrusion de Prusa (+10 °C) ; on garde la
///   plus basse température dont la force reste sous un seuil relatif. Purement informatif.
/// Les deux : refusées pendant une impression, bobine de l'emplacement chargée dans la buse, buse au-dessus de l'avant
/// du plateau à hauteur sûre, arrêtables (request_stop), réglages remis (session commune avec F4), CSV sur la clé.
/// Mémoire : config_store « OD3D Spool Calib » (à côté du K de F4), effacée au changement de bobine.
#pragma once

#include "spool_calib_math_od3d.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace od3d::calib {

/// Le débit appliqué reste dans ces bornes sûres [mm³/s] ; la rampe ne dépasse jamais flow_ceiling
inline constexpr float flow_limit_floor = 5.f;
inline constexpr float flow_ceiling = 40.f;

// ---- Mémoire ----

Record record(uint8_t slot);
void set_record(uint8_t slot, const Record &r);

/// Efface débit maximal et température minimale (bobine changée)
void clear(uint8_t slot);

/// « Limiter au débit mesuré »
bool flow_limit_enabled();
void set_flow_limit_enabled(bool set);

/// Recalcule la limite du planificateur d'après l'emplacement chargé et le réglage (changement d'outil, départ
/// d'impression, réglage, mesure, effacement)
void refresh_flow_limit();

/// Limite en vigueur [mm³/s] ; 0 : aucune
float applied_flow_limit();

// ---- Mesures ----

enum class Kind : uint8_t {
    flow, ///< F5, M1995
    temperature, ///< F6, M1996
};

enum class Phase : uint8_t {
    idle,
    preparing, ///< origine, position
    heating,
    priming,
    measuring, ///< palier d'extrusion
    settling, ///< F6 : la buse descend au palier suivant
    done,
    failed, ///< voir Error
};

enum class Error : uint8_t {
    none,
    printing, ///< refusé : impression en cours
    bad_request, ///< emplacement ou paramètres invalides
    not_loaded, ///< la bobine de l'emplacement n'est pas chargée dans la buse
    homing,
    no_room,
    heating, ///< la buse n'a pas atteint la consigne à temps
    stopped, ///< annulé
    weak_signal, ///< force trop faible
    too_few_points, ///< pas assez de paliers exploitables
};

struct Status {
    Kind kind = Kind::flow;
    Phase phase = Phase::idle;
    Error error = Error::none;
    uint8_t slot = 0;
    uint8_t step = 0; ///< palier en cours (1 à steps)
    uint8_t steps = 0;
    int16_t value = 0; ///< palier en cours : débit [0,1 mm³/s] (F5) ou température [°C] (F6)
    int16_t temperature = 0; ///< consigne de la buse
    bool csv = false; ///< le CSV est écrit sur la clé

    bool operator==(const Status &) const = default;
};

Status status();

/// Une mesure F5 ou F6 est en cours
bool is_running();

/// Nombre de M1995 / M1996 traités depuis le démarrage : l'interface sait quand sa demande a répondu
uint16_t run_count();

/// Demande l'arrêt (depuis l'interface)
void request_stop();

struct FlowRequest {
    uint8_t slot = 0;
    float flow_min = 2.f; ///< premier palier [mm³/s]
    float flow_max = 30.f; ///< dernier palier [mm³/s], borné à flow_ceiling
    float flow_step = 2.f;
    float step_s = 3.f; ///< durée d'un palier [s]
    float excess = 0.25f; ///< seuil du coude (dépassement relatif de la loi de référence)
    int16_t temperature = 0; ///< 0 : celle de la matière
    float x = NAN; ///< centre ; NAN : milieu du plateau
    float y = 10.f; ///< avant du plateau
    float z = 20.f; ///< hauteur minimale de la buse (et 10 mm au-dessus de ce qui a été imprimé)
    bool csv = false; ///< /usb/OD3D_FL<n>.CSV
};

/// Corps de M1995 : bloque jusqu'à la fin ou l'arrêt
void measure_flow(const FlowRequest &request);

struct TempRequest {
    uint8_t slot = 0;
    int16_t start = 0; ///< température nominale ; 0 : celle de la matière (jamais au-dessus)
    int16_t floor = 0; ///< plancher ; 0 : start − 60 ; jamais sous EXTRUDE_MINTEMP + 10
    int16_t step = 5; ///< [°C]
    float flow = 5.f; ///< débit constant [mm³/s]
    float max_ratio = 1.5f; ///< force acceptable jusqu'à max_ratio × la force à la température nominale
    float step_s = 3.f; ///< durée d'un palier d'extrusion [s]
    float hold_s = 5.f; ///< stabilisation à ±1 °C avant chaque palier [s]
    float x = NAN;
    float y = 10.f;
    float z = 20.f;
    bool csv = false; ///< /usb/OD3D_TM<n>.CSV
};

/// Corps de M1996 : bloque jusqu'à la fin ou l'arrêt
void measure_temperature(const TempRequest &request);

/// Dernière mesure (mémoire vive seulement), pour l'écran : paliers et résultat. Lire quand is_running() est faux.
struct LastRun {
    static constexpr size_t max_points = 24;
    Kind kind = Kind::flow;
    uint8_t slot = 0xff; ///< 0xff : pas de mesure depuis le démarrage
    uint8_t points = 0;
    std::array<StepPoint, max_points> point {};
    Knee knee; ///< F5
    MinTemp min_temp; ///< F6
    float max_ratio = 1.5f; ///< F6 : seuil utilisé
};
const LastRun &last_run();

} // namespace od3d::calib
