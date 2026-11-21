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
/// F8 (température selon le débit) se branchera dans HotendHook::step : consigne relevée d'après le débit des
/// mouvements à venir (planificateur) et puissance anticipée, seulement quand le MPC régule.
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
