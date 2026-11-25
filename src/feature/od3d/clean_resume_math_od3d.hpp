/// @file
/// OneDrop3D (F30) : reprise sans cicatrice, partie sans matériel (testée en unitaire).
///
/// Demande de l'utilisateur (25/11) : « pour cette purge elle doit être plus petite et à l'intérieur du remplissage de
/// l'objet et l'imprimante doit commencer par le remplissage comme ça on limite les traces ».
///
/// Le firmware ne peut pas réordonner le G-code : « commencer par le remplissage » veut dire amorcer d'abord dans le
/// remplissage. À la reprise d'une pause : descente, filament rétracté, sur un point du remplissage intérieur déjà
/// imprimé (couche en cours ou précédente) ; désrétraction et petite purge une seule fois en bas ; court cordon le long
/// de ce remplissage ; rétraction, remontée, puis le retour de Prusa au point de reprise (sa désrétraction pousse
/// exactement la rétraction faite ici). Les défauts de la reprise (goutte, sous-extrusion) restent dans l'objet.
///
/// Le remplissage vient du trancheur : chaque « ;TYPE:… » devient M2011 à la lecture du fichier (feature_marks_od3d.hpp)
/// et Tracker garde les derniers tracés de remplissage enchaînés (G1 qui extrudent dans le plan). Sans remplissage connu
/// (première couche, mode vase, objet sans remplissage, pause avant le premier remplissage, impression par le port
/// série, objet annulé, suite trop courte, couche trop ancienne) : la reprise de Prusa, telle quelle.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace od3d::clean_resume {

struct Settings {
    bool enabled = false; ///< coupé par défaut : la reprise de Prusa reste telle quelle
    uint8_t purge_tenths = 10; ///< petite purge en bas, en dixièmes de mm de filament (1 mm ≈ 2,4 mm³)
    uint8_t retract_tenths = 8; ///< rétraction avant le retour au point de reprise (désrétractée par Prusa)

    static constexpr uint8_t purge_max = 30; ///< 3 mm
    static constexpr uint8_t retract_max = 20; ///< 2 mm

    Settings clamped() const;
};

/// Désrétraction de la pause prise en compte au plus (au-delà, la position de E n'est pas crédible)
inline constexpr float max_unretract_mm = 10.0f;
/// Suite de remplissage utilisable : au moins cette longueur gardée
inline constexpr float min_run_mm = 6.0f;
/// Purge et bout du cordon au moins à cette distance (le long du tracé) des bouts gardés : loin des périmètres
inline constexpr float edge_margin_mm = 2.0f;
/// Court cordon d'amorce après la purge ; plus court que min_line_mm : purge seule
inline constexpr float prime_line_mm = 10.0f;
inline constexpr float min_line_mm = 1.0f;
inline constexpr float prime_line_mm_s = 20.0f;
/// Reprise au plus à cette hauteur au-dessus du remplissage utilisé
inline constexpr float max_layer_gap_mm = 1.0f;
/// Filament par mm de cordon du remplissage mesuré : bornes de vraisemblance
inline constexpr float min_e_per_mm = 0.005f;
inline constexpr float max_e_per_mm = 0.2f;
/// Écart de Z qui fait une autre couche, et écart XY qui casse l'enchaînement de deux tracés
inline constexpr float z_eps = 0.005f;
inline constexpr float join_mm = 0.05f;

struct Point {
    float x = 0;
    float y = 0;
};

/// Tracés de remplissage enchaînés, à une même hauteur ; les max_points derniers points gardés
struct Run {
    static constexpr size_t max_points = 24;
    std::array<Point, max_points> ring {};
    uint8_t first = 0;
    uint8_t count = 0;
    float z = NAN;
    float total_mm = 0; ///< toute la suite, points oubliés compris
    float total_e = 0;
    int16_t object = -1; ///< objet (M486) au début de la suite, -1 inconnu

    const Point &at(size_t i) const { return ring[(first + i) % max_points]; }
    void push(Point p);
    /// Longueur des points gardés [mm]
    float kept_mm() const;
    float e_per_mm() const { return total_mm > 0 ? total_e / total_mm : NAN; }
};

/// Suit le type de tracé (M2011) et les mouvements G0/G1 traités. Fil de Marlin.
class Tracker {
public:
    /// M2011 S : 1 remplissage intérieur, 0 autre tracé ; un changement termine la suite en cours
    void set_feature(uint8_t feature);
    /// Mouvement traité (coordonnées natives, avant/après). \returns true si une suite de remplissage commence :
    /// l'appelant peut y poser l'objet en cours (set_object)
    bool move(Point from, float z0, float e0, Point to, float z1, float e1);
    void set_object(int16_t object) { current_.object = object; }

    /// Z des deux dernières couches imprimées (mouvements qui extrudent dans le plan) ; NAN : pas encore
    float layer_z() const { return layer_z_; }
    float prev_layer_z() const { return prev_layer_z_; }
    const Run &current() const { return current_; }
    const Run &last() const { return last_; }

private:
    void close_run();

    uint8_t feature_ = 0;
    Run current_;
    Run last_; ///< dernière suite terminée assez longue
    float layer_z_ = NAN;
    float prev_layer_z_ = NAN;
};

/// Ce que fait la reprise : descente sur purge, désrétraction et purge (purge_e) en bas, cordon par line (filament :
/// longueur × e_per_mm), rétraction de retract, puis E posé à e_after pour la désrétraction de Prusa
struct InfillPlan {
    static constexpr size_t max_line = Run::max_points + 1;
    Point purge;
    float z = NAN;
    float purge_e = 0;
    std::array<Point, max_line> line {};
    uint8_t line_count = 0;
    float e_per_mm = 0;
    float retract = 0;
    float e_after = NAN;
};

/// Pourquoi la reprise de Prusa est gardée
enum class Refusal : uint8_t {
    none, ///< plan fait
    disabled,
    e_position, ///< E incohérent (en avance sur la pièce, ou trop loin derrière)
    first_layer, ///< une seule couche imprimée
    no_infill, ///< aucune suite de remplissage assez longue
    too_far, ///< remplissage ni de la couche en cours ni de la précédente, ou trop bas sous la reprise
    cancelled, ///< objet annulé
    extrusion, ///< filament par mm invraisemblable
    limits, ///< point hors des limites de la machine
    short_run, ///< aucun point assez loin des bouts du tracé
};

struct Box {
    float x_min, x_max, y_min, y_max;
};

/// Plan de la reprise après pause. resume_z, resume_e : position de la pièce à la pause ; current_e : E maintenant
/// (rétracté par la pause). cancelled(objet) : vrai si l'objet est annulé (nullptr : jamais).
/// Point de purge : sur la suite (en cours si assez longue, sinon la précédente), le point de tracé le plus proche de
/// son barycentre, à edge_margin_mm au moins des bouts ; cordon le long du tracé, du côté le plus long.
Refusal plan_infill(const Settings &settings, const Tracker &tracker, float resume_z, float resume_e, float current_e,
    const Box &limits, bool (*cancelled)(int16_t), InfillPlan &out);

} // namespace od3d::clean_resume
