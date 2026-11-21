/// @file
/// OneDrop3D (F4, première étape) : pressure advance par emplacement du MMU, mesuré par la cellule de charge.
///
/// - Mémoire : un K par emplacement (config_store « OD3D PA Slot »), effacé quand on change la bobine dans
///   l'assistant de l'Atelier ; réglage général « le PA de l'imprimante prime » (« OD3D PA Printer Wins », coupé par
///   défaut) : K est appliqué à chaque changement d'outil et le M572 / M900 du trancheur est alors ignoré pour un
///   emplacement qui a un K (gardé pour un emplacement sans K).
/// - Mesure : M1992 (calibrate()), séquence décrite dans pa_math_od3d.hpp (méthode PrusaPATuner réimplémentée).
///   Buse à la température de la matière, au-dessus de l'avant du plateau à hauteur sûre, refusée pendant une
///   impression, arrêtable à tout moment (request_stop()). Trace et résultats exportables en CSV sur la clé.
/// La mesure automatique au début de la 1re impression d'une bobine neuve et le PA adaptatif sont pour plus tard.
#pragma once

#include "pa_math_od3d.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace od3d::pa {

// ---- Mémoire ----

Record record(uint8_t slot);

/// K mesuré de l'emplacement, s'il y en a un
std::optional<float> k_for_slot(uint8_t slot);

void set_record(uint8_t slot, const Record &r);

/// Efface le K (bobine changée)
void clear(uint8_t slot);

/// « Le PA de l'imprimante prime »
bool printer_wins();
void set_printer_wins(bool set);

/// Le réglage est actif et l'emplacement n'a pas de K : « PA à mesurer »
bool needs_measure(uint8_t slot);

// ---- Application (fil de Marlin) ----

/// Après un changement d'outil du MMU (tool_change) : applique le K du nouvel emplacement si le réglage est actif ;
/// sans K, pendant une impression, la dernière valeur demandée par le trancheur
void on_tool_change();

/// Au départ d'une impression : oublie la valeur du trancheur (le K est posé par le T du début du G-code, ou à la
/// place du premier M572)
void on_print_start();

/// Appelée par M572 / M900 (M572_internal) : retient la valeur demandée et rend le K à utiliser à sa place si le
/// réglage est actif, pendant une impression, avec un emplacement chargé qui a un K
std::optional<float> override_slicer_pa(float requested);

// ---- Mesure (M1992) ----

enum class Phase : uint8_t {
    idle, ///< rien depuis le démarrage
    preparing, ///< origine, position, préparation
    heating,
    priming,
    measuring,
    done, ///< K mesuré et enregistré
    failed, ///< voir Error
};

enum class Error : uint8_t {
    none,
    printing, ///< refusé : impression en cours
    bad_request, ///< emplacement ou paramètres invalides
    not_loaded, ///< la bobine de l'emplacement n'est pas chargée dans la buse
    homing, ///< origine impossible
    no_room, ///< la séquence ne tient pas dans la course
    heating, ///< la buse n'a pas chauffé à temps
    stopped, ///< annulé
    weak_signal, ///< créneau de force trop faible ou trop bruité
    no_crossing, ///< la force ne répond pas à K comme prévu
    too_few_points, ///< pas assez de blocs exploitables
    out_of_range, ///< K hors de la plage balayée
    low_confidence, ///< mesure trop peu sûre pour être enregistrée
};

struct Status {
    Phase phase = Phase::idle;
    Error error = Error::none;
    uint8_t slot = 0;
    uint8_t step = 0; ///< bloc en cours (1 à steps)
    uint8_t steps = 0;
    uint16_t k_e4 = k_none; ///< K en cours de mesure, puis K retenu
    int16_t temperature = 0; ///< consigne de la buse
    bool csv = false; ///< le CSV est écrit sur la clé

    bool operator==(const Status &) const = default;
};

Status status();

/// Une mesure est en cours (M1992)
bool is_running();

/// Nombre de M1992 traités (mesurés ou refusés) depuis le démarrage : l'interface sait quand sa demande a répondu
uint16_t run_count();

/// Demande l'arrêt (depuis l'interface) : les mouvements déjà en file finissent (3 s au plus)
void request_stop();

struct Request {
    uint8_t slot = 0;
    float k_min = 0.f;
    float k_max = 0.10f;
    float k_step = 0.02f;
    uint8_t refine = 5; ///< points du second balayage, au pas k_step / 4, autour du premier choix (0 : aucun)
    Params params;
    int16_t temperature = 0; ///< 0 : celle de la matière de l'emplacement
    float x = NAN; ///< centre de la séquence ; NAN : milieu du plateau
    float y = 10.f; ///< à l'avant du plateau (le filament tombe hors de la zone des pièces)
    float z = 20.f; ///< hauteur minimale de la buse (et 10 mm au-dessus de ce qui a été imprimé)
    bool csv = false; ///< écrit /usb/OD3D_PA<n>.CSV
};

/// Corps de M1992 : mesure le K de l'emplacement et l'enregistre ; bloque jusqu'à la fin ou l'arrêt
void calibrate(const Request &request);

/// Dernière mesure (en mémoire vive seulement), pour l'écran : K par bloc, retard, coût, et forme moyenne d'un
/// créneau au K le plus bas, au meilleur coût et au K le plus haut. Lire quand is_running() est faux.
struct LastRun {
    static constexpr size_t max_points = 24;
    uint8_t slot = 0xff; ///< 0xff : pas de mesure depuis le démarrage
    uint8_t points = 0;
    std::array<float, max_points> k {};
    std::array<float, max_points> lag {};
    std::array<float, max_points> cost {};
    Choice choice;
    Waveform low, best, high;
    float k_low = 0, k_best = 0, k_high = 0;
};
const LastRun &last_run();

} // namespace od3d::pa
