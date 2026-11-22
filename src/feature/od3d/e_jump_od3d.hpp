/// @file
/// OneDrop3D : accélération du filament par emplacement (M2000), principe dans e_jump_math_od3d.hpp.
///
/// - Mesure (M2000 S<emplacement>) : bobine chargée dans la buse, buse à la température de la matière, tête à la position
///   de purge de Prusa (là où il purge après un chargement), PA coupé ; par palier, deux impulsions de l'extrudeur seul
///   (débit de base, saut, débit de base) ; le détecteur de filament coincé de Prusa (cellule de force, réglé comme pour
///   le chargement du MMU) dit s'il a calé. On garde margin × le dernier palier sûr. Refusée pendant une impression,
///   arrêtable, réglages remis à la fin (session commune de F4 à F6). Pas de plateau : rien n'est imprimé.
/// - Mémoire : un saut par emplacement (config_store « OD3D E Jump ») plutôt que par matière : le K du PA est déjà par
///   emplacement (F4), le bilan et l'assistant aussi, et deux bobines d'une même matière n'ont pas la même traînée ;
///   effacé au changement de bobine.
/// - Application (refresh()) : accélération max du filament de Prusa (M204 F) posée à F = saut / K, bornée, à chaque
///   changement de K (pressure_advance::set_axis_e_config, donc M572 / M900, le PA de l'emplacement, la reprise après
///   coupure), d'outil et au départ d'une impression. planner.apply_settings ne vide pas les files (comme M204) : jamais
///   d'arrêt de la tête. Sans mesure (ou sans PA), 120 comme Prusa ; un M204 F posé par quelqu'un d'autre est gardé tant
///   que l'emplacement n'a pas de mesure. Pendant une mesure par la cellule (F4 à F6, M2000), F de Prusa.
#pragma once

#include "e_jump_math_od3d.hpp"

#include <array>
#include <cstdint>

namespace od3d::e_jump {

// ---- Mémoire (tous les fils) ----

Record record(uint8_t slot);
void set_record(uint8_t slot, const Record &r);

/// Efface la mesure (bobine changée) ; F suit au prochain changement d'outil ou de K
void clear(uint8_t slot);

/// F pour l'emplacement [mm/s²] : son saut gardé et son K (le K mesuré de l'emplacement, sinon celui en vigueur) ;
/// 120 comme Prusa sans mesure ou sans PA
float filament_accel_for(uint8_t slot);

// ---- Application (fil de Marlin) ----

/// Pose M204 F d'après l'outil en vigueur et K ; sans arrêt de la tête
void refresh();

// ---- Mesure (M2000) ----

enum class Phase : uint8_t {
    idle,
    preparing,
    homing, ///< origine si besoin, tête à la position de purge
    heating,
    measuring,
    done,
    failed, ///< voir Error
};

enum class Error : uint8_t {
    none,
    printing, ///< refusé : impression en cours
    bad_request,
    not_loaded, ///< la bobine de l'emplacement n'est pas chargée dans la buse
    homing,
    heating,
    stopped,
    too_low, ///< calage dès le premier palier : rien d'enregistré
};

struct Status {
    Phase phase = Phase::idle;
    Error error = Error::none;
    uint8_t slot = 0;
    uint8_t step = 0; ///< palier en cours (1 à steps)
    uint8_t steps = 0;
    uint16_t jump_e1 = 0; ///< saut du palier en cours, puis saut gardé [0,1 mm/s]
    int16_t temperature = 0;

    bool operator==(const Status &) const = default;
};

Status status();
bool is_running();

/// Nombre de M2000 traités depuis le démarrage : l'interface sait quand sa demande a répondu
uint16_t run_count();

/// Change à chaque palier fini : l'écran redessine ses barres
uint16_t revision();

void request_stop();

/// Corps de M2000 : bloque jusqu'à la fin ou l'arrêt
void calibrate(uint8_t slot);

/// Dernière mesure (mémoire vive) : paliers faits et classement
struct LastRun {
    uint8_t slot = 0xff; ///< 0xff : pas de mesure depuis le démarrage
    uint8_t planned = 0;
    uint8_t points = 0;
    std::array<Step, max_steps> step {};
    Verdict verdict;
};
const LastRun &last_run();

} // namespace od3d::e_jump
