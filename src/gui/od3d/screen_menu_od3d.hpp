/// @file
/// OneDrop3D : le groupe « Fonctions OneDrop3D », en tête des réglages (SPEC_G.md §3.22), et ses écrans
/// « Température de la buse » (§3.23 : régulation PID / MPC et sa calibration (F7), température selon le débit (F8)
/// (screen_mpc_od3d.*), compensation de
/// dilatation (F2), sa mesure (M2003) dans screen_nozzle_expansion_od3d.*) et « Relais de bobine » (§3.24) ;
/// « Ventilation anticipée » (F26) ;
/// « Équerrage XY » (F15) est dans screen_skew_od3d.*, « Tension des courroies » (F14) dans screen_belts_od3d.*,
/// « Vitesses critiques » (F22) dans screen_speed_map_od3d.*, « Amortissement des axes » (F33) dans screen_damping_od3d.*, « Climat de la chambre » (F29) dans screen_climate_od3d.*,
/// « Reprise sans cicatrice » (F30) dans screen_resume_od3d.*, « Sécurité collision » (F38) dans screen_collision_od3d.*.
/// Chaque fonction a une ligne d'explication sous son nom (libellé sur deux lignes, séparées par « \n »).
#pragma once

#include "screen_calib_assistant_od3d.hpp"
#include "screen_climate_od3d.hpp"
#include "screen_collision_od3d.hpp"
#include "screen_mmu_usb_od3d.hpp"
#include "screen_mpc_od3d.hpp"
#include "screen_resume_od3d.hpp"
#include "screen_skew_od3d.hpp"
#include "menu_items_od3d.hpp"

#include <basic_screen_menu.hpp>
#include <MItem_menus.hpp>
#include <WindowMenuItems.hpp>
#include <WindowItemFormatableLabel.hpp>
#include <gui/menu_item/menu_item_select_menu.hpp>
#include <feature/od3d/mesh_memory_od3d.hpp>
#include <option/has_attachable_accelerometer.h>
#include <option/has_indx.h>
#include <option/has_loadcell.h>
#include <option/has_mmu2.h>
#include <option/has_nozzle_thermal_compensation.h>
#include <option/has_od3d_ui.h>
#include <tool_index.hpp>

static_assert(HAS_OD3D_UI(), "Fonctions OneDrop3D : CORE One et CORE One L, avec ou sans INDX");
#if HAS_MMU2()
static_assert(VirtualToolIndex::count == 5, "Un relais par emplacement du MMU");
#endif

class ScreenMenuInfo; // menu Info de Prusa

namespace od3d {

/// Ouvre « Calibrage de l'imprimante » (« Fonctions OneDrop3D » avant le 25/11)
class MI_FEATURES final : public IWindowMenuItem {
public:
    MI_FEATURES();
    void click(IWindowMenu &) override;
};

/// « Infos de l'imprimante » : le menu Info de Prusa, deuxième ligne des réglages
using MI_PRINTER_INFO = MI_SCREEN<N_("Printer info"), ::ScreenMenuInfo, &img::info_16x16>;

/// Ouvre « Température de la buse »
class MI_NOZZLE_TEMPERATURE final : public IWindowMenuItem {
public:
    MI_NOZZLE_TEMPERATURE();
    void click(IWindowMenu &) override;
};

/// F26 : « Ventilation anticipée » : une hausse du ventilateur pièce (M106) est appliquée jusqu'à 1 s plus tôt ;
/// « Arrêt » (par défaut) : comme Prusa. Un clic ouvre l'éditeur de valeur.
class MI_FAN_LEAD final : public WI_LAMBDA_LABEL_t {
public:
    MI_FAN_LEAD();
    void click(IWindowMenu &) override;
};

/// Ouvre « Relais de bobine »
class MI_SPOOL_JOIN_SLOTS final : public IWindowMenuItem {
public:
    MI_SPOOL_JOIN_SLOTS();
    void click(IWindowMenu &) override;
};

/// F2 : compensation de la dilatation de la buse
class MI_NOZZLE_THERMAL_COMPENSATION final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_NOZZLE_THERMAL_COMPENSATION();
    void OnChange(size_t) override;

protected:
    void Loop() override; ///< grisé pendant une impression seulement : réévalué à chaque LOOP
};

/// F2 : correction appliquée maintenant (cible de la buse), en lecture seule
class MI_NOZZLE_THERMAL_CORRECTION final : public MenuItemAutoUpdatingLabel<float> {
public:
    MI_NOZZLE_THERMAL_CORRECTION();
};

/// F4 : « le PA de l'imprimante prime » : le K mesuré de l'emplacement remplace le M572 du trancheur
class MI_PA_PRINTER_WINS final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_PA_PRINTER_WINS();
    void OnChange(size_t) override;
};

/// F4 : « PA automatique » : mesure courte avant l'impression, à côté de la pièce (désactivé, bobine neuve : à la 1re
/// impression d'une bobine, à chaque impression) ; un mode actif allume « le PA de l'imprimante prime »
class MI_PA_AUTO final : public MenuItemSelectMenu {
public:
    MI_PA_AUTO();
    int item_count() const override;
    string_view_utf8 build_item_text(int index, StringViewUtf8ParamBase &params) const override;

protected:
    bool on_item_selected(const OnItemSelectedArgs &args) override;
};

/// F4-b : « Grand motif PA » : la mesure en un clic imprime le grand motif à repères (mode manuel, choix à l'œil) au
/// lieu du motif compact ; la mesure automatique reste compacte
class MI_PA_LARGE final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_PA_LARGE();
    void OnChange(size_t) override;
};

/// F44 : « Réutiliser le maillage » : le maillage du plateau d'une impression est repris aux suivantes si rien n'a
/// changé (3 points de contrôle) ; désactivé (par défaut), ou âge maximal 6 h, 24 h, 72 h
class MI_MESH_REUSE final : public MenuItemSelectMenu {
public:
    MI_MESH_REUSE();
    int item_count() const override;
    string_view_utf8 build_item_text(int index, StringViewUtf8ParamBase &params) const override;

protected:
    bool on_item_selected(const OnItemSelectedArgs &args) override;
};

/// F44 : « Refaire le maillage » : oublie le maillage gardé, la prochaine impression palpe tout
class MI_MESH_REDO final : public IWindowMenuItem {
public:
    MI_MESH_REDO();
    void click(IWindowMenu &) override;
};

/// F5 : « limiter au débit mesuré » : le planificateur ralentit ce qui dépasserait le débit maximal de la bobine
class MI_FLOW_LIMIT final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_FLOW_LIMIT();
    void OnChange(size_t) override;
};

/// F3 : relais de bobine permanents, interrupteur général
class MI_SPOOL_JOIN final : public WI_ICON_SWITCH_OFF_ON_t {
public:
    MI_SPOOL_JOIN();
    void OnChange(size_t) override;
};

/// F3 : « Quand n est vide » → emplacement k ; l'élément 0 désactive le relais de cet emplacement.
/// Un clic passe à la valeur suivante (SPEC_G.md §3.24).
class MI_SPOOL_JOIN_TARGET : public MenuItemSelectMenu {
public:
    explicit MI_SPOOL_JOIN_TARGET(uint8_t slot);
    int item_count() const override;
    string_view_utf8 build_item_text(int index, StringViewUtf8ParamBase &params) const override;

protected:
    bool on_item_selected(const OnItemSelectedArgs &args) override;

private:
    uint8_t slot;
    StringViewUtf8Parameters<2> label_params;
};

template <uint8_t slot_>
class MI_SPOOL_JOIN_TARGET_N final : public MI_SPOOL_JOIN_TARGET {
public:
    MI_SPOOL_JOIN_TARGET_N()
        : MI_SPOOL_JOIN_TARGET(slot_) {}
};

/// Association de PrintForge : affiche un code à 6 chiffres, valable tant que le message est ouvert (5 minutes au plus)
class MI_PAIR_PRINTFORGE final : public IWindowMenuItem {
public:
    MI_PAIR_PRINTFORGE();
    void click(IWindowMenu &) override;
};

/// Lot UI-0 : « Mesure de l'écran sur la clé » : M2010 ajoute les derniers dessins de l'écran à /usb/OD3D_UI.CSV
class MI_UI_MEASURE final : public IWindowMenuItem {
public:
    MI_UI_MEASURE();
    void click(IWindowMenu &) override;
};

// Selon l'imprimante : l'INDX n'a ni MMU (mesures par emplacement, assistant, relais, MMU par l'USB) ni régulation MPC,
// compensation de dilatation ou équerrage ; l'accéléromètre de la CORE One L et de l'INDX est intégré (rien à demander) ;
// le maillage gardé est celui de la CORE One (OD3D_MESH_MEMORY())
class ScreenMenuFeatures final : public BasicScreenMenu<
#if HAS_LOADCELL() && HAS_MMU2()
                                     MI_CALIB_ASSISTANT,
    #if HAS_ATTACHABLE_ACCELEROMETER()
                                     MI_ACCELEROMETER,
    #endif
#endif
#if !HAS_INDX()
                                     MI_NOZZLE_TEMPERATURE,
#endif
                                     MI_FAN_LEAD,
#if !HAS_INDX()
                                     MI_FAN_RESPONSE, // le ventilateur pièce de l'INDX est sur la tête : pas de tachymètre lu ici
#endif
                                     MI_CHAMBER_CLIMATE, MI_CLEAN_RESUME,
#if HAS_LOADCELL() && HAS_MMU2()
                                     MI_COLLISION, MI_SPOOL_JOIN_SLOTS, MI_PA_PRINTER_WINS, MI_PA_AUTO, MI_PA_LARGE,
#endif
#if OD3D_MESH_MEMORY()
                                     MI_MESH_REUSE, MI_MESH_REDO,
#endif
#if HAS_LOADCELL() && HAS_MMU2()
                                     MI_FLOW_LIMIT, MI_E_JUMP,
#endif
#if !HAS_INDX()
                                     MI_SKEW_XY,
#endif
                                     MI_BELT_TENSION, MI_SPEED_MAP, MI_DAMPING,
#if HAS_MMU2()
                                     MI_MMU_USB,
#endif
                                     MI_PAIR_PRINTFORGE, MI_UI_MEASURE> {
public:
    ScreenMenuFeatures();
};

#if !HAS_INDX() // régulation MPC (OD3D_MPCTEMP) et compensation de dilatation (F2) : CORE One et CORE One L
class ScreenNozzleTemperature final : public BasicScreenMenu<MI_NOZZLE_CONTROL, MI_MPC_CALIBRATE, MI_FLOW_TEMP, MI_FLOW_TEMP_THRESHOLD, MI_FLOW_TEMP_MAX_BOOST, MI_FLOW_TEMP_BOOST,
                                       MI_NOZZLE_THERMAL_COMPENSATION, MI_NOZZLE_THERMAL_CORRECTION, MI_NOZZLE_EXPANSION> {
public:
    ScreenNozzleTemperature();
};
#endif

#if HAS_MMU2()
class ScreenSpoolJoin final : public BasicScreenMenu<MI_SPOOL_JOIN,
                                  MI_SPOOL_JOIN_TARGET_N<0>, MI_SPOOL_JOIN_TARGET_N<1>, MI_SPOOL_JOIN_TARGET_N<2>, MI_SPOOL_JOIN_TARGET_N<3>, MI_SPOOL_JOIN_TARGET_N<4>> {
public:
    ScreenSpoolJoin();
};
#endif

} // namespace od3d
