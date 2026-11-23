/// @file
/// OneDrop3D : assistant de calibration, côté Marlin (M1989) et mémoire. Le registre (tests, besoins, versions) est
/// dans calib_registry_od3d.hpp ; l'écran dans gui/od3d/screen_calib_assistant_od3d.hpp.
///
/// Déroulement d'une série (M1989 R<bits des tests>) : les étapes sont construites par le registre, puis chacune est
/// mise en file seule, suivie de « M1989 N<i> ». Quand Marlin arrive à ce repère, l'étape i est finie (ses suites mises
/// en file d'injection par elle-même, comme le chargement de M1994, passent avant) : son résultat est lu, puis l'étape
/// suivante est mise en file. Rien n'est imbriqué (pile de Marlin de 5 440 o) ; la série continue même si l'écran est
/// fermé, la pastille de l'en-tête la montre. Les résultats sont enregistrés à la fin de la série.
#pragma once

#include "calib_registry_od3d.hpp"

#include <array>
#include <cstdint>

namespace od3d::calib_assistant {

using calib_registry::Accelerometer;
using calib_registry::Context;
using calib_registry::Numbers;
using calib_registry::Record;
using calib_registry::Result;
using calib_registry::Step;
using calib_registry::Test;

// ---- Mémoire et réglages (lisibles depuis tous les fils) ----

Context context();
Record record(Test test);
std::array<Record, calib_registry::max_tests> records();

/// « Accéléromètre disponible » (Réglages › Fonctions OneDrop3D, et question de l'assistant)
Accelerometer accelerometer();
void set_accelerometer(Accelerometer value);

/// Calibration faite hors de l'assistant (équerrage, manuel)
void record_done(Test test);

// ---- Série en cours ----

struct Progress {
    bool running = false;
    uint8_t index = 0; ///< étape en cours (0 à count - 1)
    uint8_t count = 0;
    Step step {};
    uint16_t runs = 0; ///< séries finies depuis le démarrage : l'écran sait quand la sienne est finie
    bool bed_full = false; ///< série en pause : plus de place sur le plateau pour l'étape index ; nettoyé : M1989 C
};
Progress progress();

/// Résultat du test dans la dernière série qui l'a contenu depuis le démarrage (none : pas encore)
Result result(Test test);

/// Interface, hors série : les chiffres « après » d'un test réussi relevés à nouveau et gardés (courroies réglées en
/// direct après leur mesure : le bilan et la fois suivante voient les fréquences finales)
void recapture(Test test);

/// Tests de la dernière série reportés parce que le MMU ne répondait pas (bits de Test) : comptés « passés »
uint32_t postponed();

/// Pour l'écran du flash du MMU par l'USB avant (F42) : vrai si l'étape en cours de la série pilote le MMU, charge ou
/// décharge par lui. Ne pas lancer M709 S2 tant que c'est vrai ; le reste du temps, la série continue sans le MMU
/// (ses étapes qui en ont besoin passent à la fin, ou sont reportées si M709 S2 dure encore)
bool mmu_in_use();

/// « Annuler » (interface) : l'étape en cours est arrêtée, les suivantes ne sont pas lancées
void request_cancel();

/// L'étape en cours peut être passée sans risque (toutes, sauf le tube PTFE qui ne s'arrête pas en route)
bool can_skip();

/// « Passer » (interface, ou arrêt depuis l'écran de la mesure) : l'étape en cours s'arrête si elle tourne, comptée
/// « passée » (pas un échec) ; la série continue. index : l'étape vue par l'interface (Progress::index), rien si la
/// série est déjà à la suivante. À redemander tant que l'étape n'a pas commencé (chargement)
void request_skip(uint8_t index);

/// Chiffres du bilan de la dernière série (no_value : pas de valeur), relevés avant l'étape et après son succès :
/// - input_shaper (écrits par M1959 A) : [0..3] type et fréquence ×10 de X puis Y ; après : [4], [5] baisse estimée des
///   vibrations restantes de X, Y par rapport à sans filtre (%) ; [6], [7] par rapport au réglage d'avant (%) ;
/// - phase_stepping (écrits par M1977) : [0], [1] baisse des vibrations des moteurs X, Y mesurée par Prusa (%) ;
/// - belts : [0], [1] fréquence ×10 des courroies A, B ; damping : [0], [1] ζ ×1000 de X, Y ;
/// - speed_map : [0] nombre de vitesses critiques, [1] sur X, [2] sur Y ;
/// - mpc : [0] P ×10 (W), [1] C ×10 (J/K), [2] A ×1000 (W/K), [3] F ×1000 (W/K) ;
/// - mmu_ptfe : [0] longueur enregistrée (mm), après : [1] longueur relevée (mm) ;
/// - fan_response : [0] temps de montée (ms), [1] avance de F26 (dixièmes de s), [2] régime final (tr/min) ; avant :
///   le temps de montée du dernier succès et l'avance réglée ;
/// - nozzle_expansion : [0] pente (nm/°C ; avant : no_value pour le modèle de Prusa), après : [1] pente de Prusa
///   (nm/°C), [2] écart avec Prusa à 250 °C (0,1 µm), [3] retour − aller (0,1 µm) ;
/// - mmu_speed : [0] vitesse de la poulie (mm/s) ; après : [1], [2] durée mesurée d'un cycle chargement + déchargement
///   à la vitesse de Prusa puis à la vitesse gardée (0,1 s), [3] dernier palier passé (mm/s) ;
/// - pa, max_flow, min_temp : [emplacement] K ×10 000, débit ×10 (mm³/s), température (°C) ;
/// - e_jump : [emplacement] accélération max du filament (mm/s²) avec le K de l'emplacement.
/// « Avant » des courroies, de l'amortissement, des vitesses, du phase stepping et du ventilateur : le dernier succès
/// (OD3D Calib Values).
const Numbers &numbers_before(Test test);
const Numbers &numbers_after(Test test);

/// Fil de Marlin : chiffre écrit par un assistant de Prusa (M1959, M1977)
void set_number(Test test, bool after, uint8_t index, int16_t value);

/// Vérification avant les tests à l'accéléromètre (M1989 A)
enum class Check : uint8_t {
    none,
    ok,
    no_accelerometer, ///< il ne répond pas (od3d::accel::ready())
    not_homed, ///< origine perdue (coupure des moteurs) : la refaire sans l'accéléromètre
    hot, ///< buse pas froide, ou chauffe allumée
    printing,
};
Check check_result();
uint16_t check_count();

// ---- Corps de M1989 (fil de Marlin) ----

/// M1989 H : chauffe de la buse coupée, ventilateur à fond tant qu'elle est chaude, origine si besoin, puis buse au
/// centre à mi-hauteur, comme le début de l'assistant input shaper de Prusa (avant de fixer l'accéléromètre)
void prepare();

/// M1989 A
void check();

/// M1989 R<mask> S<redo> : redo, tests par bobine à refaire sur toutes les bobines (sinon seulement celles qui n'ont
/// pas encore de valeur, calib_registry::build_steps())
void run(uint32_t mask, uint32_t redo = 0);

/// M1989 N<i>
void step_end(uint8_t index);

/// M1989 C : plateau nettoyé après « Plateau plein » (Progress::bed_full) : tout le plateau est de nouveau libre (et
/// palpé de nouveau), l'étape en attente est relancée ; aussi pour finir la série après « Annuler » pendant l'attente
void clean_done();

/// M1989 L<emplacement> (mis en file avant le chargement d'un test qui imprime, calib_registry::load_gcode) : plateau
/// entier palpé pour la série s'il ne l'est pas encore, avant que du filament pende de la buse
void level_bed(uint8_t slot);

} // namespace od3d::calib_assistant
