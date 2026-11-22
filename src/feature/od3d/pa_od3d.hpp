/// @file
/// OneDrop3D (F4) : pressure advance par emplacement du MMU, mesuré par la cellule de charge.
///
/// - Mémoire : un K par emplacement (config_store « OD3D PA Slot »), effacé quand on change la bobine dans
///   l'assistant de l'Atelier ; réglage général « le PA de l'imprimante prime » (« OD3D PA Printer Wins », coupé par
///   défaut) : K est appliqué à chaque changement d'outil et le M572 / M900 du trancheur est alors ignoré pour un
///   emplacement qui a un K (gardé pour un emplacement sans K).
/// - Mesure : M1994 (calibrate()), **dans le vide, à la manière de PrusaPATuner** (pa_math_od3d.hpp) : buse à la
///   température de la matière, origine si nécessaire, position de purge de Prusa à 50 mm au moins du plateau,
///   chargement de l'emplacement par le MMU s'il n'est pas dans la buse, purge lente, puis une séquence de créneaux par
///   K (balayage grossier puis fin autour du minimum). Trace brute, commande et résultats exportables en CSV sur la clé.
/// - Mesure automatique au début d'une impression (réglage « OD3D PA Auto », activé par défaut) : on_print_init()
///   met en file un M1994 A par emplacement utilisé par le fichier qui n'a pas de K ; l'impression reprend ensuite
///   normalement (le G-code de départ chauffe, palpe et charge l'outil). « Passer » (SkippableGCode de Prusa) ou un
///   échec : l'impression continue avec la valeur du trancheur.
/// - Repli visuel : M1994 V1 imprime une ligne par K à l'avant du plateau (maillage appliqué) et l'utilisateur choisit
///   la plus régulière (choose_line()).
/// Le PA adaptatif (K selon le débit) est pour plus tard.
#pragma once

#include "pa_math_od3d.hpp"
#include "pattern_math_od3d.hpp"

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

// ---- Mesure (M1994) ----

enum class Phase : uint8_t {
    idle, ///< rien depuis le démarrage
    preparing, ///< vérifications, réglages
    heating, ///< la buse chauffe à la température de la matière
    parking, ///< origine, position de purge (mesure dans le vide)
    loading, ///< le MMU charge l'emplacement dans la buse
    priming, ///< purge lente avant la mesure (ou ligne de purge du motif)
    measuring, ///< séquences de créneaux (ou lignes du motif)
    bed, ///< motif : le plateau chauffe
    leveling, ///< motif : origine, nettoyage de la buse, palpage de la zone
    done, ///< K mesuré et enregistré (motif : lignes imprimées)
    failed, ///< voir Error
};

enum class Error : uint8_t {
    none,
    printing, ///< refusé : impression en cours
    bad_request, ///< emplacement ou paramètres invalides
    not_loaded, ///< l'emplacement n'a pas pu être chargé dans la buse
    homing, ///< origine impossible
    no_room, ///< le motif ne tient pas dans la zone imprimable
    heating, ///< la buse ou le plateau n'a pas chauffé à temps
    leveling, ///< palpage du plateau impossible
    stopped, ///< annulé
    weak_signal, ///< créneau de force trop faible ou trop bruité
    no_crossing, ///< la force ne dépend pas de K
    too_few_points, ///< pas assez de séquences exploitables
    out_of_range, ///< K au bord haut de la plage balayée
    low_confidence, ///< mesure trop peu sûre pour être enregistrée
    skipped, ///< « Passer » au début d'une impression
    saturated, ///< force écrêtée à la borne de la trace
};

struct Status {
    Phase phase = Phase::idle;
    Error error = Error::none;
    uint8_t slot = 0;
    uint8_t step = 0; ///< séquence (ou ligne) en cours (1 à steps)
    uint8_t steps = 0;
    uint16_t k_e4 = k_none; ///< K en cours de mesure, puis K retenu
    int16_t temperature = 0; ///< consigne de la buse
    bool csv = false; ///< le CSV est écrit sur la clé
    bool automatic = false; ///< mesure au début d'une impression
    bool pattern = false; ///< motif imprimé (repli visuel)

    bool operator==(const Status &) const = default;
};

Status status();

/// Une mesure est en cours (M1994)
bool is_running();

/// Nombre de M1994 traités (mesurés ou refusés) depuis le démarrage : l'interface sait quand sa demande a répondu
uint16_t run_count();

/// Demande l'arrêt (depuis l'interface) : les mouvements déjà en file finissent (3 s au plus)
void request_stop();

/// Mesure automatique en cours : « Passer » (l'impression continue avec la valeur du trancheur, et sans mesurer les
/// autres emplacements de cette impression)
void request_skip();

/// Numéro (non nul) de la mesure automatique en cours, 0 sans : l'écran d'impression ouvre l'écran PA une fois par mesure
uint16_t auto_run();

/// « Mesurer le PA des bobines inconnues au début de l'impression » (activé par défaut)
bool auto_measure();
void set_auto_measure(bool set);

/// Au départ d'une impression (fil de Marlin, avant les lignes du fichier) : met en file une mesure (M1994 A) par
/// emplacement utilisé par le fichier (association des outils comprise) qui n'a pas de K
void on_print_init();

inline constexpr uint8_t max_lines = 15; ///< lignes du motif au plus

struct Request {
    uint8_t slot = 0;
    float k_min = 0.f;
    float k_max = 0.10f;
    float k_step = 0.01f;
    uint8_t refine = 4; ///< séquences du second balayage, au pas k_step / 2, autour du premier choix (0 : aucune)
    Params params; ///< créneaux (PrusaPATuner)
    int16_t temperature = 0; ///< buse ; 0 : celle de la matière de l'emplacement
    float park_z = 50.f; ///< hauteur de la buse au-dessus du plateau pendant la mesure dans le vide [mm]
    bool csv = false; ///< écrit /usb/OD3D_PA<n>.CSV
    bool automatic = false; ///< au début d'une impression (M1994 A)
    // Motif imprimé (repli visuel, M1994 V1)
    bool visual = false;
    pattern::PaLine line;
    int16_t bed_temperature = 0; ///< plateau ; 0 : celle de la matière
    float x = 20.f; ///< début des lignes (gauche)
    float y = 12.f; ///< première ligne, à l'avant du plateau (les suivantes vers l'arrière)
    float pitch = 5.f; ///< écart entre deux lignes [mm]
};

/// Corps de M1994 : mesure le K de l'emplacement et l'enregistre (ou imprime le motif) ; bloque jusqu'à la fin
void calibrate(const Request &request);

/// Dernière mesure (en mémoire vive seulement), pour l'écran : K de chaque séquence (motif : de chaque ligne, dans
/// l'ordre des lignes, ligne 1 à l'avant), coût composé et amplitude, forme moyenne d'un créneau au K le plus bas, au
/// meilleur coût et au K le plus haut. Lire quand is_running() est faux (l'écran montre les courbes en cours de route).
struct LastRun {
    static constexpr size_t max_points = 24;
    uint8_t slot = 0xff; ///< 0xff : pas de mesure depuis le démarrage
    uint8_t points = 0; ///< séquences (ou lignes) faites en entier
    bool pattern = false; ///< lignes imprimées : le choix à l'œil est possible
    std::array<float, max_points> k {};
    std::array<float, max_points> cost {};
    std::array<float, max_points> amplitude {};
    Choice choice;
    Waveform low, best, high;
    float k_low = 0, k_best = 0, k_high = 0;
};
const LastRun &last_run();

/// Repli visuel : enregistre pour l'emplacement le K de la ligne line (0 : la première, à l'avant) du dernier motif
/// imprimé de cet emplacement ; false sans motif de cet emplacement, ligne non imprimée ou mesure en cours
bool choose_line(uint8_t slot, uint8_t line);

} // namespace od3d::pa
