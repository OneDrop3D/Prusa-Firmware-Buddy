/// @file
/// OneDrop3D (F4, première étape) : pressure advance par emplacement du MMU, mesuré par la cellule de charge.
///
/// - Mémoire : un K par emplacement (config_store « OD3D PA Slot »), effacé quand on change la bobine dans
///   l'assistant de l'Atelier ; réglage général « le PA de l'imprimante prime » (« OD3D PA Printer Wins », coupé par
///   défaut) : K est appliqué à chaque changement d'outil et le M572 / M900 du trancheur est alors ignoré pour un
///   emplacement qui a un K (gardé pour un emplacement sans K).
/// - Mesure : M1994 (calibrate()), principe dans pa_math_od3d.hpp. **Motif imprimé** à l'avant du plateau (la mesure
///   dans le vide ne marchait pas, retour du 22/11) : plateau à la température de la matière, origine, nivellement
///   local de la zone (comme le G-code de départ de la CORE One), ligne de purge au bord avant, puis une ligne par K à
///   hauteur de première couche (lente, rapide, lente…) ; chaque ligne porte à sa droite des traits qui donnent son
///   numéro (un sur cinq plus long). La force verticale pendant les changements de vitesse donne le K ; à défaut,
///   l'utilisateur choisit la meilleure ligne à l'œil (choose_line()), comme le test PA d'Orca.
///   Refusée pendant une impression, arrêtable à tout moment (request_stop()), état remis à la fin (session commune).
///   Trace et résultats exportables en CSV sur la clé.
/// - En un clic (retour du 22/11) : si la bobine n'est pas dans la buse, M1994 met en file « Charger dans la buse » de
///   Prusa (G28 O I et G27 W2 d'abord : origine si besoin, buse levée à la position de chargement ; puis M701 S W2 P :
///   l'autre filament est déchargé d'abord, question de la couleur comprise) puis lui-même avec
///   J1, et rend la main : le MMU n'est jamais piloté depuis la pile de M1994 (débordement de la pile de Marlin en
///   8.0.2.23). Le filament reste chargé à la fin.
/// - Automatique (réglage « PA automatique », comme la calibration de la dynamique du débit des Bambu) : M1994 A, mis
///   en file au départ de l'impression (PrintInit), choisit les emplacements utilisés sans K (ou tous) et une bande libre
///   à côté de la pièce (auto_spot()) ; puis, une étape à la fois par la file d'injection (avant les lignes du fichier) :
///   « Charger dans la buse » de Prusa, M1994 P<n> A1 (mesure courte, 5 lignes) ; enfin le filament remis comme avant
///   (M702 W2 en général). Le G-code de départ fait ensuite tout comme d'habitude. Un échec ou un « Passer » ne bloque
///   jamais l'impression : le PA du trancheur reste, et un message le dit.
/// - Deux motifs (F4-b, demande du 22/11) : **compact** (par défaut, et toujours pour la mesure automatique) : lignes
///   courtes et serrées sans repères, K cherché ligne après ligne (next_k() : 4 à 6 lignes au lieu de 11) ; **grand**
///   (mode manuel, réglage « Grand motif PA ») : balayage complet, repères, choix à l'œil.
/// - Départ rapide (F4-b) : plateau et buse (température de palpage) chauffent ensemble, origine pendant que le plateau
///   chauffe, nettoyage et palpage une fois le plateau chaud (comme le G-code de départ de la CORE One) ; la purge
///   commence à 5 °C de la consigne (20 °C jusqu'à la 8.0.2.27), les lignes une fois la température stable (K dépend
///   de la viscosité). En automatique, les températures de l'impression (fichier) ; en un clic, celles de la matière,
///   et le plateau chauffe pendant le chargement par le MMU.
/// Le PA adaptatif est pour plus tard.
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
/// place du premier M572) ; from_file : impression d'un fichier, la mesure automatique (M1994 A) est mise en file si
/// le réglage est actif
void on_print_start(bool from_file);

/// Appelée par M572 / M900 (M572_internal) : retient la valeur demandée et rend le K à utiliser à sa place si le
/// réglage est actif, pendant une impression, avec un emplacement chargé qui a un K
std::optional<float> override_slicer_pa(float requested);

// ---- Mesure (M1994) ----

enum class Phase : uint8_t {
    idle, ///< rien depuis le démarrage
    preparing, ///< vérifications, réglages
    bed, ///< le plateau chauffe (la buse va à sa température de palpage)
    leveling, ///< origine, nettoyage de la buse, palpage de la zone du motif
    heating, ///< la buse chauffe à la température d'impression
    loading, ///< le MMU charge la bobine dans la buse
    priming, ///< ligne de purge
    measuring, ///< lignes du motif
    done, ///< K mesuré et enregistré
    failed, ///< voir Error
};

enum class Error : uint8_t {
    none,
    printing, ///< refusé : impression en cours
    bad_request, ///< emplacement ou paramètres invalides
    not_loaded, ///< la bobine de l'emplacement n'est pas chargée dans la buse
    homing, ///< origine impossible
    no_room, ///< le motif ne tient pas dans la zone imprimable
    heating, ///< la buse ou le plateau n'a pas chauffé à temps
    leveling, ///< palpage du plateau impossible
    stopped, ///< annulé
    weak_signal, ///< créneau de force trop faible ou trop bruité
    no_crossing, ///< la force ne répond pas à K comme prévu
    too_few_points, ///< pas assez de blocs exploitables
    out_of_range, ///< K hors de la plage balayée
    low_confidence, ///< mesure trop peu sûre pour être enregistrée
    mmu, ///< le MMU n'est pas prêt (absent ou en erreur)
    no_area, ///< mesure automatique : l'impression ne déclare pas sa zone (M555)
    load_failed, ///< le chargement de Prusa n'a pas mis la bobine dans la buse (pas de seconde tentative)
};

struct Status {
    Phase phase = Phase::idle;
    Error error = Error::none;
    uint8_t slot = 0;
    uint8_t step = 0; ///< ligne en cours (1 à steps)
    uint8_t steps = 0;
    uint16_t k_e4 = k_none; ///< K en cours de mesure, puis K retenu
    int16_t temperature = 0; ///< consigne de la buse
    bool csv = false; ///< le CSV est écrit sur la clé

    bool operator==(const Status &) const = default;
};

Status status();

/// Une mesure est en cours (M1994), ou attend son chargement par Prusa (en un clic)
bool is_running();

/// Nombre de M1994 traités (mesurés ou refusés) depuis le démarrage : l'interface sait quand sa demande a répondu
uint16_t run_count();

/// Demande l'arrêt (depuis l'interface) : les mouvements déjà en file finissent (3 s au plus)
void request_stop();

/// Corps de M1994 : mesure le K de l'emplacement et l'enregistre ; bloque jusqu'à la fin ou l'arrêt. Bobine pas dans
/// la buse : met en file le chargement de Prusa puis le même M1994 avec J1 (load_tried), et rend la main aussitôt ;
/// avec load_tried, une bobine toujours pas dans la buse est un échec (load_failed), sans seconde tentative
void calibrate(const Request &request, bool load_tried = false);

/// Réglage « Grand motif PA » (F4-b) : true = grand motif à repères (mode manuel) en un clic ; false (par défaut) =
/// motif compact. La mesure automatique est toujours compacte.
bool large_pattern();
void set_large_pattern(bool set);

/// Réglage « PA automatique » ; un mode actif allume aussi « le PA de l'imprimante prime » (sinon le K mesuré ne
/// servirait pas)
AutoMode auto_mode();
void set_auto_mode(AutoMode mode);

/// Corps de M1994 A, au départ d'une impression : choisit les emplacements à mesurer et leur place, puis met en file,
/// avant les lignes du fichier, une étape par emplacement (chargement de Prusa, puis M1994 P<n> A1) ; jamais bloquant
void calibrate_auto();

/// Corps de M1994 P<n> A1 : mesure courte de l'emplacement chargé par l'étape, puis met en file l'étape suivante, ou
/// à la fin le filament remis comme au départ ; un échec ou « Passer » laisse le PA du trancheur, l'impression continue
void calibrate_auto_step(uint8_t slot);

/// Dernière mesure (en mémoire vive seulement), pour l'écran : K de chaque ligne imprimée **dans l'ordre des lignes**
/// (ligne 1 à l'avant), retard, coût, et forme moyenne d'un créneau au K le plus bas, au meilleur coût et au K le plus
/// haut. Lire quand is_running() est faux.
struct LastRun {
    static constexpr size_t max_points = 24;
    uint8_t slot = 0xff; ///< 0xff : pas de mesure depuis le démarrage
    uint8_t points = 0; ///< lignes imprimées en entier (repères compris)
    bool compact = false; ///< motif compact : lignes dans l'ordre des essais, sans repères
    std::array<float, max_points> k {};
    std::array<float, max_points> lag {};
    std::array<float, max_points> cost {};
    Choice choice;
    Waveform low, best, high;
    float k_low = 0, k_best = 0, k_high = 0;
};
const LastRun &last_run();

/// Repli visuel : enregistre pour l'emplacement le K de la ligne line (0 : la première, à l'avant) de la dernière
/// mesure de cet emplacement ; false sans mesure de cet emplacement, ligne non imprimée ou mesure en cours
bool choose_line(uint8_t slot, uint8_t line);

} // namespace od3d::pa
