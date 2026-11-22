/// @file
/// OneDrop3D (F38, document Vecteurs v3 n° 20) : sécurité collision / warping, partie pure (sans firmware), testée.
///
/// Signal : la cellule de charge de la buse, toujours présente. L'accéléromètre de l'input shaper est sur la buse et
/// souvent débranché : il ne sert que de confirmation quand il répond, jamais de condition.
/// Force « vers le haut » positive, comme au palpage : un coin qui se soulève ou une pièce heurtée repousse la buse.
///
/// Un choc, c'est :
/// 1. une impulsion : la force monte au-dessus de sa moyenne lente (0,5 s) de plus que le seuil de journal, puis
///    redescend sous la moitié de ce seuil en 120 ms au plus. Une marche (débit d'extrusion qui change, dérive de la
///    cellule) dure plus longtemps : la moyenne repart de la nouvelle valeur et rien n'est compté ;
/// 2. un pic au-dessus du seuil de la sensibilité **et** de 8 fois le bruit de fond de l'impression en cours (moyenne
///    des écarts sur 2 s) : « bien au-dessus de la force normale d'impression » ;
/// 3. hors des mouvements rapides de l'extrudeur et des 100 ms qui suivent (rétraction, désrétraction, essuyage
///    pendant la rétraction : |vE| > 20 mm/s), au-dessus de Z = 1 mm (purge, première couche écrasée), hors des
///    phases exclues (palpage, origine, changement d'outil, chargement : collision_od3d.hpp), et 1 s après le réarmement.
/// Alerte : assez de chocs à moins de 5 mm les uns des autres en 15 min (au moins 2 chocs distincts ; un choc
/// confirmé par l'accéléromètre compte double). Deux impulsions à moins de 0,5 s au même endroit font un seul choc.
///
/// Seuils de départ, prudents (aucun relevé réel encore) : Prusa palpe à 125 g (seuil statique de la cellule) et à
/// 40 g en continu ; le palpage XY déclenche à 40 g. Le plus petit choc compté (250 g, sensibilité élevée) vaut le
/// double du palpage. Toutes les impulsions dès la moitié du seuil vont dans /usb/OD3D_COLL.CSV pour les régler.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace od3d::collision {

enum class Sensitivity : uint8_t {
    low,
    normal,
    high,
};

enum class Action : uint8_t {
    pause, ///< « Mettre en pause » (par défaut) : la pause de Prusa, puis le message
    warn, ///< « Prévenir seulement »
};

struct Params {
    float threshold_g; ///< pic minimal d'un choc [g]
    uint8_t hits; ///< chocs au même endroit pour l'alerte (un choc confirmé compte double)
};

/// Faible : 600 g, 4 chocs ; normale : 400 g, 3 chocs ; élevée : 250 g, 2 chocs
Params params_for(Sensitivity s);

inline constexpr float log_fraction = 0.5f; ///< impulsions relevées dès cette fraction du seuil (CSV)
inline constexpr float noise_factor = 8.f; ///< un choc dépasse aussi 8 fois le bruit de fond
inline constexpr uint32_t baseline_tau_us = 500'000; ///< moyenne lente de la force
inline constexpr uint32_t noise_tau_us = 2'000'000; ///< bruit de fond (moyenne des écarts)
inline constexpr uint32_t max_pulse_us = 120'000; ///< plus long : une marche, pas un choc
inline constexpr uint32_t settle_us = 1'000'000; ///< après le réarmement, la moyenne se refait
inline constexpr float fast_e_mm_s = 20.f; ///< extrudeur plus rapide : rétraction ou désrétraction
inline constexpr uint32_t fast_e_guard_us = 100'000;
inline constexpr float min_z_mm = 1.f; ///< en dessous : purge et premières couches
inline constexpr float radius_mm = 5.f; ///< « au même endroit »
inline constexpr uint32_t window_ms = 15 * 60'000; ///< chocs oubliés après 15 min
inline constexpr uint32_t rebound_ms = 500; ///< deux impulsions plus proches au même endroit : un seul choc
inline constexpr float accel_confirm_g = 0.5f; ///< secousse verticale de l'accéléromètre qui confirme un choc
inline constexpr float accel_alpha = 0.002f; ///< moyenne de l'accélération verticale (≈ 0,4 s à 1 344 Hz)

/// Impulsion repérée (tout ce qui dépasse le seuil de journal), pour le comptage et le CSV. Son heure est prise par le
/// fil de Marlin (ticks_ms) : celle de l'interruption, en µs, reboucle toutes les 71 min.
struct Pulse {
    float peak_g = 0; ///< pic au-dessus de la moyenne lente [g]
    float noise_g = 0; ///< bruit de fond au début de l'impulsion [g]
    float x = NAN, y = NAN, z = NAN; ///< position au pic [mm]
    float accel_g = NAN; ///< plus forte secousse verticale pendant l'impulsion [g] ; NAN sans accéléromètre
    uint16_t duration_ms = 0;
};

/// Repère les impulsions, échantillon par échantillon (interruption de la cellule : mémoire fixe, quelques calculs)
class Detector {
public:
    /// Un échantillon de l'accéléromètre (axe Z de l'imprimante [g]), entre deux échantillons de la cellule
    void accel(float az_g);

    /// Un échantillon de la cellule. armed : phase autorisée ; log_g : seuil des impulsions [g].
    /// Une position non finie reprend la dernière connue. Rend vrai quand une impulsion vient de finir (pulse()).
    bool sample(uint32_t time_us, float up_g, float x, float y, float z, float e, bool armed, float log_g);

    const Pulse &pulse() const { return done; }

private:
    void restart(uint32_t time_us, float up_g);

    bool have_base = false;
    bool in_pulse = false;
    bool fast_e_seen = false;
    uint32_t armed_us = 0;
    uint32_t last_us = 0;
    uint32_t last_fast_e_us = 0;
    uint32_t pulse_start_us = 0;
    float base_g = 0;
    float noise = 0;
    float last_x = NAN, last_y = NAN, last_z = NAN, last_e = NAN;
    float accel_base = NAN;
    float accel_peak = NAN; ///< depuis l'échantillon précédent de la cellule
    Pulse current;
    Pulse done;
};

/// L'impulsion est-elle un choc ? (seuil de la sensibilité et bruit de fond)
bool counts(const Pulse &p, float threshold_g);

/// Confirmée par l'accéléromètre ?
bool confirmed(const Pulse &p);

/// Chocs récents (fil de Marlin) ; alerte quand il y en a assez au même endroit
class HitMap {
public:
    struct Result {
        uint8_t near = 0; ///< chocs distincts à moins de radius_mm de celui-ci (lui compris)
        uint8_t weight = 0; ///< leur poids (confirmé = 2)
        bool alert = false; ///< la carte est vidée : il faut de nouveaux chocs pour une autre alerte
    };

    Result add(uint32_t time_ms, float x, float y, bool confirmed, uint8_t hits_needed);
    void clear() { count = 0; }

private:
    struct Hit {
        uint32_t time_ms;
        float x, y;
        uint8_t weight;
    };
    std::array<Hit, 8> hits {};
    uint8_t count = 0;
};

} // namespace od3d::collision
