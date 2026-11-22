/// @file
/// OneDrop3D (F44) : maillage du plateau gardé et repris, côté firmware (fil de Marlin). Principe, conditions et
/// contrôle : mesh_memory_math_od3d.hpp. Branché dans le G29 P1 de Prusa (ubl_G29.cpp) :
/// - G29 P1 sans C : restore() remet les points gardés dans le maillage et rend les 3 points de contrôle, laissés vides ;
///   G29 les palpe comme ses autres points (probe_major_points), puis verify() corrige le maillage de leur écart moyen,
///   ou l'efface : palpage complet, comme Prusa ;
/// - après le palpage, on_probed() garde le maillage d'une impression (G29 P1 sans zone : zone M555 du fichier) et son
///   extension près de la purge (G29 P1 … C). Un palpage à zone donnée (mesures OneDrop3D, G-code tapé) peut reprendre
///   le maillage gardé mais ne le remplace pas.
/// Désactivé par défaut (réglage « Réutiliser le maillage »).
#pragma once

#include <array>
#include <cstdint>

#include <feature/print_area.h>

namespace od3d::mesh_memory {

/// Choix du réglage « Réutiliser le maillage » : âge maximal [h] ; 0 = désactivé (par défaut)
inline constexpr std::array<uint8_t, 4> age_choices { 0, 6, 24, 72 };

uint8_t max_age_hours();
void set_max_age_hours(uint8_t hours);

/// « Refaire le maillage » : oublie le maillage gardé, la prochaine impression palpe tout
void forget();

/// Un maillage est gardé
bool has_mesh();

// ---- Pour G29 P1 (fil de Marlin) ----

/// G29 P1 sans C : si le maillage gardé peut servir pour probe_area, le remet dans le maillage (le reste vide) et rend
/// le masque des points de contrôle, laissés vides pour que G29 les palpe ; 0 : rien de repris, maillage inchangé
uint64_t restore(const PrintArea::rect_t &probe_area);

/// Après le palpage des points de contrôle : true, maillage corrigé de leur écart moyen ; false, maillage effacé (à
/// palper en entier)
bool verify(uint64_t checks);

/// Après G29 P1 réussi ou non. extend : G29 P1 … C ; print_area : zone de l'impression (M555), pas de zone donnée ;
/// reused : maillage repris par restore() et verify()
void on_probed(bool extend, bool print_area, bool reused);

} // namespace od3d::mesh_memory
