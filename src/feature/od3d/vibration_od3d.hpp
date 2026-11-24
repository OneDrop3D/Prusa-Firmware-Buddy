/// @file
/// OneDrop3D : deux mesures de la série « Accéléromètre » de l'assistant (à froid, accéléromètre fixé sur la buse),
/// calculs dans vibration_math_od3d.hpp. Comme les autres tests à l'accéléromètre (accelerometer_od3d.hpp) : refusées
/// sans rien bouger si l'accéléromètre ne répond pas, si la buse dépasse 50 °C ou pendant une impression ; rien ne
/// chauffe. Les résultats restent en mémoire jusqu'au redémarrage ; mesures brutes ajoutées sur la clé (CSV).
///
/// - M2004, accélération max utile : à la position de l'assistant input shaper de Prusa (M1959 : centre du plateau,
///   mi-hauteur), X puis Y font quatre déplacements courts (aller, retour, aller, retour) à chaque palier de 2 000 à
///   10 000 mm/s², input shaper en place, mode furtif coupé ; le résidu est mesuré dans les 250 ms qui suivent l'arrêt.
///   Une information, montrée au bilan de l'assistant : rien n'est appliqué, ni M201 ni plafond (V-01 : le plafond
///   d'avant limitait chaque M201 des impressions sur une mesure non validée). /usb/OD3D_ACCEL.CSV.
/// - M2001 V, vibrations des ventilateurs (après le temps de montée de M2001) : chaque ventilateur seul (les autres
///   arrêtés par les commandes du test des ventilateurs de Prusa, M1978), à 50, 75 et 100 %, moteurs à l'arrêt ;
///   régime lu au tachymètre, amplitude à la fréquence de rotation par la transformée en un point de M958
///   (FourierSeries3d), à la période d'échantillonnage étalonnée (V-08 : étalonnée d'abord, ~15 s, si rien ne l'a fait
///   depuis le démarrage). La première mesure captée de chaque ventilateur devient sa référence, gardée sur 7 bits dans
///   « OD3D Calib Values » (case du ventilateur, [2] et [3], vibration::calibrated_mark). /usb/OD3D_FANS.CSV.
#pragma once

#include "vibration_math_od3d.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace od3d::vibration {

enum class Status : uint8_t {
    idle,
    running,
    done,
    stopped, ///< arrêtée depuis l'écran (ou « Passer » de l'assistant)
    printing, ///< refusé : impression en cours
    hot_nozzle, ///< refusé : buse au-dessus de 50 °C (l'accéléromètre est sur la buse)
    no_accelerometer, ///< refusé sans bouger : l'accéléromètre ne répond pas (ACC-1)
    failed, ///< l'accéléromètre a cessé de répondre
    not_homed, ///< M2004 : pas d'origine (jamais prise accéléromètre fixé)
};

inline constexpr float max_nozzle_temperature = 50.f;

} // namespace od3d::vibration

namespace od3d::accel_limit {

using vibration::Status;

inline constexpr uint8_t axis_count = 2; ///< X, Y

struct Measurement {
    std::array<std::array<float, vibration::accel_levels>, axis_count> residual {}; ///< variance moyenne (m²/s⁴), NAN : pas mesuré
    float noise = NAN; ///< variance au repos
    std::array<uint16_t, axis_count> top {}; ///< plus haut palier essayé (limite matérielle de Prusa), 0 : pas d'axe
    std::array<vibration::Limit, axis_count> limits {};
};

/// Corps de M2004 : bloque jusqu'à la fin (~1 min) ou request_stop()
void run();
void request_stop();
bool is_running();
Status status();
uint8_t progress_percent();
const Measurement &last();

/// Accélération max utile d'un axe (mm/s²) à proposer pour M201 ; nullopt : non concluant (résidu dans le bruit)
std::optional<uint16_t> proposal(uint8_t axis);

} // namespace od3d::accel_limit

namespace od3d::fan_vibration {

using vibration::Status;

/// Les références gardées, dans cet ordre (deux codes par case de « OD3D Calib Values »)
enum class Fan : uint8_t {
    print, ///< ventilateur pièce
    hotend, ///< ventilateur de la tête (barrière thermique)
    chamber_1, ///< chambre (xBuddy Extension) : premier ventilateur, ou celui de filtration
    chamber_2,
};
inline constexpr size_t fan_count = 4;

struct FanResult {
    bool present = false;
    std::array<uint16_t, vibration::fan_pwm.size()> rpm {}; ///< par régime, au tachymètre
    std::array<vibration::Tone, vibration::fan_pwm.size()> tones {};
    vibration::FanLevel level {};
    std::optional<float> reference; ///< m/s², avant cette mesure
    vibration::Verdict verdict = vibration::Verdict::not_detected;
};

struct Measurement {
    std::array<FanResult, fan_count> fans {};
};

/// Corps de M2001 V après le temps de montée : bloque jusqu'à la fin (~1 min) ou request_stop()
void run();
void request_stop();
bool is_running();
Status status();
uint8_t progress_percent();
const Measurement &last();

/// Les références des ventilateurs telles que gardées (deux int16 de « OD3D Calib Values », case du ventilateur)
std::array<int16_t, 2> stored_references();

} // namespace od3d::fan_vibration

/// M2005, pose de l'accéléromètre (série « Accéléromètre », en premier ; retour du 24/11 : « calibration automatique »
/// de l'accéléromètre) : au centre du plateau à mi-hauteur, tête au repos, la pesanteur donne l'inclinaison et le
/// bruit ; puis un aller-retour court sur X et sur Y (mount_accel) : la part du mouvement dans le plan horizontal. Un
/// accéléromètre clipsé de travers ou qui bouge sur la buse fausserait toutes les mesures qui suivent : le test échoue
/// alors avec la raison. Son orientation à plat n'est pas jugée (vibration_math_od3d.hpp). Rien n'est réglé.
/// /usb/OD3D_MOUNT.CSV.
namespace od3d::accel_mount {

using vibration::Status;

struct Measurement {
    std::array<float, 3> gravity { NAN, NAN, NAN }; ///< m/s², moyenne au repos (axes de l'imprimante)
    float tilt_deg = NAN;
    float noise_rms = NAN; ///< m/s², au repos
    std::array<std::array<float, 3>, 2> sums {}; ///< somme des |a| par axe pendant le mouvement sur X, sur Y
    std::array<float, 2> share { NAN, NAN }; ///< part du plan horizontal pendant le mouvement sur X, sur Y
    vibration::Mount verdict = vibration::Mount::ok;
};

/// Corps de M2005 : bloque jusqu'à la fin (~20 s) ou request_stop()
void run();
void request_stop();
bool is_running();
Status status();
uint8_t progress_percent();
const Measurement &last();

/// Le test est réussi (mesure finie et pose correcte)
bool passed();

} // namespace od3d::accel_mount

/// M2006, vérification de l'input shaper (série « Accéléromètre », après l'accélération max utile) : à la position de
/// M1959, à check_accel (bornée par la limite matérielle), X puis Y font quatre déplacements courts avec le filtre de
/// l'axe en place, puis quatre sans (filtre coupé le temps de la mesure, toujours remis) : la part des vibrations après
/// l'arrêt que le filtre retire. Sous check_min_reduction sur un axe : à refaire. Rien n'est réglé.
/// /usb/OD3D_SHAPER.CSV.
namespace od3d::shaper_check {

using vibration::Status;

inline constexpr uint8_t axis_count = 2;

struct Measurement {
    float noise = NAN; ///< variance au repos (m²/s⁴)
    uint16_t accel = 0; ///< mm/s² utilisée
    std::array<float, axis_count> ramp { NAN, NAN }; ///< s, rampe de l'axe (vibration::ramp_for(), V-05)
    std::array<bool, axis_count> has_filter {};
    std::array<float, axis_count> with_filter { NAN, NAN }; ///< variance moyenne après l'arrêt
    std::array<float, axis_count> without_filter { NAN, NAN };
    std::array<float, axis_count> reduction { NAN, NAN }; ///< 0 à 1 (négatif : le filtre empire)
    std::array<vibration::ShaperCheck, axis_count> verdict { vibration::ShaperCheck::no_filter, vibration::ShaperCheck::no_filter };
};

/// Corps de M2006 : bloque jusqu'à la fin (~1 min) ou request_stop()
void run();
void request_stop();
bool is_running();
Status status();
uint8_t progress_percent();
const Measurement &last();

/// Le test est réussi (mesure finie, au moins un axe filtré, aucun axe filtré faible)
bool passed();

} // namespace od3d::shaper_check
