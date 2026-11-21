/// @file
/// OneDrop3D (F7) : branchement du MPC (mpc_od3d.hpp) dans le firmware.
///
/// - HotendHook : appelé par LocalHotend::manage() (régulation de Prusa), dans le même garde de température que le
///   PID et avant le même écrêtage de la sortie. Il rend la sortie du MPC si le MPC est choisi et calibré, la
///   commande de la calibration pendant M306 T, ou laisse la main au PID de Prusa. Toutes les protections
///   (BaseHotend::manage, LocalHotend::isr_on_readings_ready, set_nozzle_target_temp, ventilateur du heatbreak)
///   restent dans le code de Prusa et s'appliquent de la même façon dans les deux modes.
/// - Mode PID / MPC (config_store od3d_mpc_enabled, PID par défaut) et paramètres (od3d_mpc_params).
/// - Calibration M306 T, son état pour l'écran et son « Annuler ».
///
/// F8 (température selon le débit), dans HotendHook::step, seulement quand le MPC régule et si le réglage est mis :
/// - débit des ~500 ms à venir lu dans la file du planificateur (lookahead_od3d.hpp) ;
/// - (a) puissance anticipée : le terme « filament » des pertes compensées par le MPC prend ce débit à venir ;
/// - (b) hausse de consigne : le MPC vise consigne + hausse (flow_boost_request, FlowBoost : rampe de descente),
///   toujours bornée sous la consigne maximale de Prusa (boosted_target).
/// La consigne de Prusa (nozzle_target_temp(), affichée et vue par les protections) ne change jamais. Ainsi :
/// - MAXTEMP (BaseHotend::manage) : inchangé ; la consigne visée reste <= max_nozzle_temp − HEATER_MAXTEMP_SAFETY_MARGIN,
///   comme toute consigne posée par set_nozzle_target_temp, et le MPC coupe au-delà de cette borne + 5 K ;
/// - MINTEMP : inchangé ;
/// - emballement (thermal_runaway_) : il compare la mesure à la consigne de Prusa, qui ne bouge pas, donc il n'est
///   jamais réarmé par la hausse (un réarmement à chaque pas le neutraliserait) ; la hausse met la buse au-dessus de
///   la consigne, du côté sûr de la règle « pas plus de 15 °C sous la consigne pendant plus de 80 s », qui reste la
///   même ; la hausse est plafonnée à 15 °C (flow_max_boost_range), l'hystérésis de cette règle ;
/// - surveillance de la chauffe (heater_watch_) : armée par un changement de la consigne de Prusa, seul cas où elle
///   travaille ; la hausse ne l'arme pas et ne change pas sa règle (+2 °C en 20 s tant que la buse est à plus de
///   6 °C sous la consigne de Prusa) ; viser plus haut ne peut que chauffer plus fort ;
/// - garde de la régulation (current < max_nozzle_temp) et contrôle des valeurs brutes dans l'ISR : inchangés.
#pragma once

#include "mpc_od3d.hpp"

#include <cstdint>

namespace od3d::mpc {

// ---- Régulation (fil de Marlin) ----

/// Le crochet de la buse locale (un par LocalHotend)
class HotendHook {
public:
    struct Args {
        float current_temp; ///< [°C]
        int16_t target_temp; ///< consigne de la buse [°C], <= 0 : coupée
        int16_t max_target_temp; ///< consigne maximale : max_nozzle_temp − HEATER_MAXTEMP_SAFETY_MARGIN
        uint8_t fan_speed; ///< ventilateur pièce demandé, 0 à 255
        uint8_t applied_pwm; ///< sortie appliquée depuis le pas précédent, 0 à 255
        bool is_current_tool; ///< la vitesse du filament ne compte que pour l'outil actif
    };

    struct Result {
        bool handled; ///< false : le PID de Prusa régule ce pas
        float pwm; ///< sortie si handled, 0 à PID_MAX
        bool reset_pid; ///< le PID reprend la main : le remettre à zéro (reset_pid de HotendRegulatorArgs)
    };

    Result step(const Args &args);

private:
    enum class Source : uint8_t {
        pid,
        mpc,
        tune, ///< calibration : chauffe coupée ou pleine puissance
    };

    Model model_;
    FlowBoost boost_; ///< F8 : hausse de consigne en cours
    uint32_t params_version_ = 0;
    uint32_t last_ms_ = 0;
    int32_t last_e_steps_ = 0;
    bool has_last_ = false;
    Source last_source_ = Source::pid;
};

/// Vitesse du filament au-delà de laquelle ce n'est plus de l'extrusion (chargement, déchargement, saut de position) :
/// 20 mm/s de filament de 1,75 mm = 48 mm³/s, plus que la buse ne peut fondre
inline constexpr float max_melt_e_speed = 20.0f;

// ---- Réglages ----

/// Au démarrage (settings_load) : paramètres et mode enregistrés
void load();

/// Calibration valable enregistrée (lisible depuis tous les fils)
bool has_valid_calibration();

/// MPC choisi dans les réglages (lisible depuis tous les fils)
bool is_mpc_selected();

enum class ModeResult : uint8_t {
    ok,
    printing, ///< refusé pendant une impression (pause comprise)
    not_calibrated, ///< MPC refusé sans calibration valable
    tuning, ///< refusé pendant la calibration
};

/// Choisit PID (false) ou MPC (true) et l'enregistre. printing : marlin_client::is_printing() depuis l'interface,
/// marlin_server::is_printing() depuis Marlin.
ModeResult set_mode(bool mpc, bool printing);

/// Nouveaux paramètres (M306 P C R A F H, ou fin de calibration) : enregistrés et pris par le régulateur.
/// Des paramètres non valables repassent la régulation en PID. Fil de Marlin.
void set_params(const Params &params);

/// Paramètres en service (fil de Marlin)
const Params &params();

// ---- F8 : température selon le débit (lisible depuis tous les fils ; réglée depuis l'interface) ----

/// Réglage marche / arrêt (coupé par défaut). Sans effet tant que le MPC ne régule pas.
bool flow_temp_enabled();
void set_flow_temp_enabled(bool enabled);

/// Seuil [mm³/s], ramené dans flow_threshold_range
uint8_t flow_temp_threshold();
void set_flow_temp_threshold(uint8_t mm3s);

/// Hausse maximale [°C], ramenée dans flow_max_boost_range
uint8_t flow_temp_max_boost();
void set_flow_temp_max_boost(uint8_t celsius);

/// Hausse appliquée maintenant [°C, arrondie] (0 si le MPC ne régule pas)
uint8_t flow_temp_current_boost();

/// Écrit les paramètres, la calibration et le mode sur la console (M306 sans paramètre). Fil de Marlin.
void report();

// ---- Calibration (M306 T) ----

/// Corps de M306 T : bloque jusqu'à la fin de la calibration (≈ 5 à 15 min), ou jusqu'à request_cancel().
/// Fil de Marlin.
void run_autotune(float tune_temp);

/// « Annuler » (depuis l'interface)
void request_cancel();

struct TuneStatus {
    Phase phase; ///< Phase::idle : jamais lancée depuis le démarrage
    Error error;
    uint8_t progress; ///< 0 à 100
    bool running;
    uint16_t runs; ///< nombre de M306 T traités (l'écran sait quand sa demande a répondu)
};

TuneStatus tune_status();

} // namespace od3d::mpc
