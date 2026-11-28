/// @file
#include "screen_calib_assistant_od3d.hpp"

#include "menu_od3d.hpp" // section_tab
#include "screen_belts_od3d.hpp"
#include "screen_damping_od3d.hpp"
#include "screen_fan_response_od3d.hpp"
#include "screen_speed_map_od3d.hpp"
#include <option/has_indx.h>
#include <option/has_loadcell.h>
#include <option/has_mmu2.h>
#if HAS_LOADCELL() && HAS_MMU2() // série « Chauffe » : pas sur l'INDX
    #include "screen_e_jump_od3d.hpp"
    #include "screen_mmu_speed_od3d.hpp"
    #include "screen_mpc_od3d.hpp"
    #include "screen_nozzle_expansion_od3d.hpp"
    #include "screen_pa_od3d.hpp"
    #include "screen_ptfe_od3d.hpp"
    #include "screen_skew_od3d.hpp"
    #include "screen_spool_calib_od3d.hpp"
    #include <feature/od3d/e_jump_od3d.hpp>
    #include <feature/od3d/mmu_speed_od3d.hpp>
    #include <feature/od3d/mpc_control_od3d.hpp>
    #include <feature/od3d/nozzle_expansion_od3d.hpp>
    #include <feature/od3d/pa_od3d.hpp>
    #include <feature/od3d/spool_calib_od3d.hpp>
#endif
#if HAS_INDX()
    #include <selftest_snake_config.hpp> // docks et décalages des outils du serpent de Prusa
#endif

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <span>
#include <utility>
#include <client_response.hpp>
#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <feature/od3d/belt_tension_od3d.hpp>
#include <feature/od3d/calib_assistant_od3d.hpp>
#include <feature/od3d/damping_od3d.hpp>
#include <feature/od3d/fan_response_od3d.hpp>
#include <feature/od3d/live_od3d.hpp>
#include <feature/od3d/sleep_od3d.hpp>
#include <feature/od3d/speed_map_od3d.hpp>
#include <feature/od3d/vibration_od3d.hpp>
#include <filament.hpp>
#include <Marlin/src/feature/input_shaper/input_shaper_config.hpp>
#include <option/has_attachable_accelerometer.h>
#include <option/has_hotend_type_support.h>
#include <tool_index.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <ScreenHandler.hpp>
#include <window_msgbox.hpp>

namespace od3d {

namespace assistant = calib_assistant;
using calib_registry::Accelerometer;
using calib_registry::Entry;
using calib_registry::Result;
using calib_registry::Test;
using calib_registry::test_bit;
using calib_registry::entries;

namespace {

    // Géométrie (contenu de y 44 à 274) : texte sur une ligne, 3 colonnes de 6 tuiles alignées sur les 3 boutons, de
    // quoi loger tous les tests possibles du registre (calib_registry::max_tests). Retour du 24/11 (« pas pratique à
    // cliquer au doigt ») : l'intro tient en une ligne (le bilan complet passe par « Bilan »), les tuiles prennent la
    // place (corps 148 × 24 au lieu de 19) et les boutons font 40 px. Marges de 2 px jointives (aucun pixel entre deux
    // tuiles n'est à personne : la zone de toucher fait 152 × 28), nom à gauche et pastille d'état à droite, tous deux
    // hors des coins arrondis : 13 caractères au plus pour le nom (« Équerrage XY », « Accél. fil. »)
    constexpr size_t tile_columns = 3;
    constexpr size_t max_tiles = 18;
    constexpr uint8_t tile_radius = style::radius_small;
    constexpr int16_t tile_dot = 8; ///< diamètre de la pastille d'état
    constexpr Rect16 info_rect = Rect16(style::margin, style::content.Top(), 464, height(Font::small));
    constexpr Rect16 tile_body(size_t i) {
        return Rect16(10 + 156 * int(i % tile_columns), 62 + 28 * int(i / tile_columns), 148, 24);
    }
    constexpr Rect16 button_body(int i) {
        return Rect16(10 + 156 * i, 232, 148, 40);
    }
    /// Consignes (fixer l'accéléromètre, refroidir, plateau…) : le texte prend la place des tuiles, masquées
    constexpr Rect16 info_wide_rect = Rect16::fromLTRB(style::margin, style::content.Top(), style::margin + 464, style::with_ring(button_body(0)).Top() - 2);
    static_assert(max_tiles >= calib_registry::max_tests, "une tuile pour chaque test possible du registre");
    static_assert(entries.size() <= max_tiles);
    static_assert(style::with_ring(tile_body(0)).Top() > info_rect.Bottom());
    static_assert(style::with_ring(tile_body(tile_columns)).Top() == style::with_ring(tile_body(0)).Bottom() + 1, "rangées jointives");
    static_assert(style::with_ring(tile_body(1)).Left() > style::with_ring(tile_body(0)).Right());
    static_assert(tile_body(0).Height() >= height(Font::small) + 2 && 2 * tile_radius <= tile_body(0).Height());
    static_assert(style::inside(style::content, style::with_ring(tile_body(max_tiles - 1))));
    static_assert(style::with_ring(button_body(0)).Top() > style::with_ring(tile_body(max_tiles - 1)).Bottom());
    static_assert(style::inside(style::content, style::with_ring(button_body(2))));
    static_assert(info_wide_rect.Bottom() < style::with_ring(button_body(0)).Top() && info_wide_rect.Height() >= 8 * height(Font::small));

    constexpr uint8_t tag_a = 100;
    constexpr uint8_t tag_b = 101;
    constexpr uint8_t tag_c = 102;

    enum class Stage : uint8_t {
        idle,
        propose, ///< calibrations à faire, durée (depuis le menu, ou une fois au démarrage : flow.boot)
        ask_accel, ///< « Avez-vous un accéléromètre ? »
        cooling, ///< buse sous 40 °C (ventilateur), origine, buse au centre (M1989 H)
        sock_off, ///< « Retirez la chaussette en silicone » (buse avec chaussette seulement)
        attach, ///< « Fixez l'accéléromètre » ; Continuer le vérifie (M1989 A)
        checking,
        rehome, ///< origine perdue en attendant : retirer l'accéléromètre, la refaire
        accel_run, ///< série des tests à l'accéléromètre
        belts_adjust, ///< courroies hors de la plage : réglage en direct proposé, accéléromètre encore fixé (retour du 23/11)
        detach, ///< « Retirez l'accéléromètre » (et remettez la chaussette) : rien ne chauffe avant « C'est fait »
        bed, ///< « Plateau vide et propre ? »
        cell_run, ///< MPC et mesures par bobine
        bed_full, ///< série en pause : plus de place sur le plateau, à nettoyer (M1989 C)
        summary,
    };

    /// L'assistant, gardé hors de l'écran : il survit à sa fermeture (sous-écran, onglet, pastille)
    struct Flow {
        Stage stage = Stage::idle;
        bool boot = false; ///< ouvert seul au premier démarrage d'un firmware aux calibrations nouvelles : sa ligne de texte
        bool cancelled = false;
        bool check_failed = false; ///< l'accéléromètre n'a pas répondu à la dernière vérification
        uint32_t session = 0; ///< tests de cette session (affichés au bilan ; décochés : « passé »)
        uint32_t mask = 0; ///< tests encore à lancer ; dans la proposition : les tests cochés
        uint32_t offered = 0; ///< proposition : les tests cochés d'office
        uint32_t started = 0; ///< tests envoyés à Marlin (leur résultat compte)
        uint16_t runs_at_start = 0;
        uint16_t checks_at_request = 0;
        bool belts_offered = false; ///< réglage en direct des courroies déjà proposé dans cette session
        uint8_t skip_index = 0xff; ///< étape passée (« Passer »), redemandé tant qu'elle n'a pas commencé
        uint16_t skip_runs = 0; ///< série de cette étape
    };
    Flow flow;

    uint32_t accel_tests() {
        // INDX : les ventilateurs ne mesurent que leurs vibrations, par l'accéléromètre de la tête prise (M2001 V)
        return (calib_registry::needs_mask(calib_registry::accelerometer) | (HAS_INDX() ? test_bit(Test::fan_response) : 0u)) & calib_registry::automatic_mask();
    }

    uint32_t bed_tests() {
        return calib_registry::needs_mask(calib_registry::empty_bed) & calib_registry::automatic_mask();
    }

    /// Les deux séries lancées séparément (retour du 23/11) : « Accéléromètre », à froid, et « Chauffe »
    uint32_t cold_tests() {
        return calib_registry::series_tests(false);
    }
    uint32_t heat_tests() {
        return calib_registry::series_tests(true);
    }

    uint32_t tests_to_offer() {
        const auto records = assistant::records();
        return calib_registry::proposed(assistant::context(), records);
    }

    /// Tests par bobine à refaire sur toutes les bobines : cochés à la main alors que tout était fait, ou version
    /// augmentée ; les autres ne mesurent que les bobines sans valeur
    uint32_t redo_tests() {
        const auto records = assistant::records();
        return (flow.mask & ~flow.offered) | calib_registry::redo_mask(records);
    }

    const char *name_of(Test test) {
        switch (test) {
        case Test::input_shaper:
            return N_("Input Shaper");
        case Test::phase_stepping:
            return N_("Phase stepping");
        case Test::belts:
            return N_("Belts");
        case Test::speed_map:
            return N_("Speeds");
        case Test::damping:
            return N_("Damping");
        case Test::mpc:
            return "MPC";
        case Test::pa:
            return "PA";
        case Test::max_flow:
            return N_("Flow");
        case Test::min_temp:
            return N_("Min temp");
        case Test::skew:
            return N_("XY skew");
        case Test::mmu_ptfe:
            return N_("Front PTFE");
        case Test::fan_response:
            return N_("Fans");
        case Test::nozzle_expansion:
            return N_("Expansion");
        case Test::e_jump:
            return N_("Filament accel.");
        case Test::mmu_speed:
            return N_("MMU speed");
        case Test::accel_limit:
            return N_("Max accel.");
        case Test::accel_mount:
            return N_("Accel. mount");
        case Test::shaper_check:
            return N_("Shaper check");
        case Test::count:
            break;
        }
        return "";
    }

    int16_t nozzle() {
        return int16_t(std::lround(marlin_vars().active_hotend().temp_nozzle.get()));
    }

    bool cold() {
        auto &hotend = marlin_vars().active_hotend();
        return calib_registry::nozzle_cold(hotend.temp_nozzle.get(), hotend.target_nozzle.get());
    }

    bool run_finished() {
        const assistant::Progress p = assistant::progress();
        return !p.running && p.runs != flow.runs_at_start;
    }

    /// La buse a une chaussette en silicone (réglage de Prusa « Chaussette en silicone » de la tête)
    bool has_sock() {
#if HAS_HOTEND_TYPE_SUPPORT()
        return config_store().hotend_type.get(0) == HotendType::stock_with_sock;
#else
        return false;
#endif
    }

    /// L'assistant de Prusa en cours (input shaper ou phase stepping, lancé par la série), s'il est en haut
    struct PrusaWizard {
        ClientFSM fsm;
        PhaseUnderlyingType phase;
        fsm::PhaseData data;
    };
    std::optional<PrusaWizard> prusa_wizard() {
        const auto top = marlin_vars().peek_fsm_states([](const auto &states) { return states.get_top(); });
        if (!top || (top->fsm_type != ClientFSM::InputShaperCalibration && top->fsm_type != ClientFSM::PhaseSteppingCalibration)) {
            return std::nullopt;
        }
        return PrusaWizard { top->fsm_type, top->data.GetPhase(), top->data.GetData() };
    }

    /// Ce qu'il fait, en direct (fréquence, axe, calcul) ; vide sinon
    void prusa_live(char *out, size_t size) {
        out[0] = '\0';
        const auto w = prusa_wizard();
        if (!w) {
            return;
        }
        char format[32];
        if (w->fsm == ClientFSM::InputShaperCalibration) {
            switch (GetEnumFromPhaseIndex<PhasesInputShaperCalibration>(w->phase)) {
            case PhasesInputShaperCalibration::measuring_x_axis:
            case PhasesInputShaperCalibration::measuring_y_axis: {
                const bool x = GetEnumFromPhaseIndex<PhasesInputShaperCalibration>(w->phase) == PhasesInputShaperCalibration::measuring_x_axis;
                if (w->data[3]) {
                    _("calibrating %d%%").copyToRAM(format, sizeof(format));
                    snprintf(out, size, format, int(w->data[2] * 100 / 255));
                } else {
                    _("%c axis, %d Hz").copyToRAM(format, sizeof(format));
                    snprintf(out, size, format, x ? 'X' : 'Y', int(w->data[2]));
                }
                break;
            }
            case PhasesInputShaperCalibration::computing:
                _("computing %d%%").copyToRAM(format, sizeof(format));
                snprintf(out, size, format, int(w->data[0] * 100 / 255));
                break;
            default:
                break;
            }
        } else {
            const auto phase = GetEnumFromPhaseIndex<PhasesPhaseStepping>(w->phase);
            if (phase == PhasesPhaseStepping::calib_x || phase == PhasesPhaseStepping::calib_y) {
                _("%c axis").copyToRAM(format, sizeof(format));
                snprintf(out, size, format, phase == PhasesPhaseStepping::calib_x ? 'X' : 'Y');
            }
        }
    }

    /// « Annuler » de la série pendant un assistant de Prusa : son « Abandonner », dans les phases qui l'acceptent
    void abort_prusa_wizard() {
        const auto w = prusa_wizard();
        if (!w) {
            return;
        }
        bool abortable = false;
        if (w->fsm == ClientFSM::InputShaperCalibration) {
            switch (GetEnumFromPhaseIndex<PhasesInputShaperCalibration>(w->phase)) {
#if HAS_ATTACHABLE_ACCELEROMETER() // Prusa n'attend la buse froide que pour un accéléromètre à fixer
            case PhasesInputShaperCalibration::wait_for_extruder_temperature:
#endif
            case PhasesInputShaperCalibration::measuring_x_axis:
            case PhasesInputShaperCalibration::measuring_y_axis:
            case PhasesInputShaperCalibration::computing:
                abortable = true;
                break;
            default:
                break;
            }
        } else {
            switch (GetEnumFromPhaseIndex<PhasesPhaseStepping>(w->phase)) {
#if HAS_ATTACHABLE_ACCELEROMETER()
            case PhasesPhaseStepping::wait_for_extruder_temperature:
#endif
            case PhasesPhaseStepping::calib_x:
            case PhasesPhaseStepping::calib_y:
                abortable = true;
                break;
            default:
                break;
            }
        }
        if (abortable) {
            marlin_client::FSM_response(FSMAndPhase(w->fsm, w->phase), Response::Abort);
        }
    }

    /// L'écran d'une mesure de la série, avec ses courbes en direct ; vide : pas d'écran à nous (l'assistant la montre)
    ScreenFactory::Creator step_screen(const calib_registry::Step &s) {
        switch (s.test) {
        case Test::belts:
            return ScreenFactory::Screen<ScreenBelts>;
        case Test::speed_map:
            return ScreenFactory::Screen<ScreenSpeedMap>;
        case Test::damping:
            return ScreenFactory::Screen<ScreenDamping>;
#if HAS_LOADCELL() && HAS_MMU2()
        case Test::nozzle_expansion:
            return ScreenFactory::Screen<ScreenNozzleExpansion>;
        case Test::mpc:
            return mpc_calibration_screen();
        case Test::pa:
            return ScreenFactory::ScreenWithArg<ScreenPa>(s.slot);
        case Test::max_flow:
        case Test::min_temp:
            return ScreenFactory::ScreenWithArg<ScreenSpoolCalib>(s.slot);
        case Test::e_jump:
            return ScreenFactory::ScreenWithArg<ScreenEJump>(s.slot);
        case Test::mmu_speed:
            return ScreenFactory::Screen<ScreenMmuSpeed>;
#endif
#if !HAS_INDX() // INDX : son écran mesure le temps de montée (refusé, IC-7) ; l'assistant montre les vibrations
        case Test::fan_response:
            return ScreenFactory::Screen<ScreenFanResponse>;
#endif
        default:
            return {};
        }
    }

    void enter_cooling() {
        marlin_client::gcode("M1989 H"); // chauffe coupée, ventilateur, origine (sans l'accéléromètre), buse au centre
        flow.stage = Stage::cooling;
    }

    /// « Continuer » une fois l'accéléromètre fixé : M1989 A le vérifie
    void start_check() {
        flow.checks_at_request = assistant::check_count();
        marlin_client::gcode("M1989 A");
        flow.stage = Stage::checking;
    }

    /// Accéléromètre à fixer le temps des tests (CORE One) : chaussette, pose, retrait. Celui de la CORE One L est dans
    /// la tête : ni question, ni étape de pose ou de retrait
    constexpr bool attachable = HAS_ATTACHABLE_ACCELEROMETER();

    /// Lance les tests restants de la série à froid, accéléromètre fixé (accel : la réponse du ventilateur y passe aussi,
    /// avant « Retirez l'accéléromètre »), ou tous les autres
    void run_part(bool accel) {
        uint32_t part = flow.mask & (accel ? cold_tests() : ~accel_tests()) & ~flow.started;
        if (part & test_bit(Test::belts)) {
            part = test_bit(Test::belts); // seules d'abord : hors de la plage, réglées avant les tests qui en dépendent
        }
        flow.started |= part;
        flow.runs_at_start = assistant::progress().runs;
        char gcode[40];
        snprintf(gcode, sizeof(gcode), "M1989 R%lu S%lu", static_cast<unsigned long>(part), static_cast<unsigned long>(part & redo_tests()));
        marlin_client::gcode(gcode);
        flow.stage = accel ? Stage::accel_run : Stage::cell_run;
    }

    /// Les courroies viennent d'être mesurées hors de la plage de Prusa (conseil : tourner les vis), pas encore proposé
    bool belts_out_of_range() {
        const belts::Measurement &m = belts::last();
        return belts::advice_validated && !flow.belts_offered && (flow.started & test_bit(Test::belts)) && assistant::result(Test::belts) == Result::ok && m.peaks[0].found
            && m.peaks[1].found && belts::advise(m.peaks[0].frequency, m.peaks[1].frequency, belts::targets()).verdict == belts::Verdict::adjust;
    }

    void next_after_accel();

    /// Fin d'une série à l'accéléromètre : réglage en direct des courroies proposé, puis les autres tests à
    /// l'accéléromètre, puis « Retirez l'accéléromètre » (l'accélération max utile n'est qu'une information : V-01)
    void after_accel_part() {
        if (!flow.cancelled && belts_out_of_range()) {
            flow.belts_offered = true;
            flow.stage = Stage::belts_adjust;
        } else if (!flow.cancelled && (flow.mask & cold_tests() & ~flow.started)) {
            run_part(true);
        } else if (attachable) {
            flow.stage = Stage::detach; // aussi après « Annuler » : l'accéléromètre est encore sur la buse
        } else {
            next_after_accel();
        }
    }

    /// Après l'accéléromètre (fait, passé ou absent) : plateau, puis le reste ; sinon le bilan
    void next_after_accel() {
        const uint32_t rest = flow.mask & ~accel_tests() & ~flow.started;
        if (flow.cancelled || !rest) {
            flow.stage = Stage::summary;
        } else if (rest & bed_tests()) {
            flow.stage = Stage::bed;
        } else {
            run_part(false);
        }
    }

    /// session : tests montrés au bilan (cochés d'office et décochés compris) ; mask : ceux à lancer
    void start_session(uint32_t session, uint32_t mask, bool ask) {
        if (marlin_client::is_printing()) {
            MsgBoxWarning(_("Not while printing."), Responses_Ok);
            return;
        }
        if (marlin_vars().is_processing.get()) {
            // ECR-1 : la série attendrait derrière un G-code long (musique, MPC, chargement) sans pouvoir s'annuler avant
            MsgBoxWarning(_("Printer is busy. Please try repeating the action later."), Responses_Ok);
            return;
        }
#if HAS_INDX()
        // Une tête prise pour toute la série : docks et décalages des outils réussis d'abord (serpent de Prusa) ; rien ne bouge
        if (SelftestSnake::get_test_result(SelftestSnake::Action::DockCalibration, AllTools {}) != TestResult::passed
            || SelftestSnake::get_test_result(SelftestSnake::Action::ToolOffsetsCalibration, AllTools {}) != TestResult::passed) {
            MsgBoxWarning(_("Calibrate the docks and the tool offsets first: Control > Calibrations & Tests."), Responses_Ok);
            return;
        }
#endif
        flow.session = session | mask;
        flow.mask = mask;
        flow.skip_index = 0xff;
        flow.belts_offered = false;
        flow.started = 0;
        flow.cancelled = flow.check_failed = false;
        if (!(mask & accel_tests())) {
            next_after_accel();
        } else if (attachable && (ask || assistant::accelerometer() != Accelerometer::yes)) {
            flow.stage = Stage::ask_accel;
        } else {
            enter_cooling();
        }
    }

    void cancel() {
        if ((flow.stage == Stage::accel_run || flow.stage == Stage::cell_run)
            && MsgBoxQuestion(_("Stop the calibration series? The current measurement stops and the next ones are not done."), Responses_YesNo, 1) != Response::Yes) {
            return;
        }
        flow.cancelled = true;
        switch (flow.stage) {
        case Stage::accel_run:
        case Stage::cell_run:
            assistant::request_cancel(); // la série finit l'étape en cours sans lancer la suivante
            abort_prusa_wizard();
            break;
        case Stage::bed_full:
            assistant::request_cancel();
            marlin_client::gcode("M1989 C"); // la série en pause passe les étapes restantes et finit
            flow.stage = Stage::cell_run;
            break;
        case Stage::cooling:
        case Stage::sock_off:
            marlin_client::gcode("M107");
            flow.stage = Stage::summary;
            break;
        case Stage::attach:
        case Stage::checking:
        case Stage::rehome:
        case Stage::belts_adjust:
            // Chaussette retirée, accéléromètre peut-être fixé : les remettre en ordre avant toute chauffe
            marlin_client::gcode("M107");
            flow.stage = attachable ? Stage::detach : Stage::summary;
            break;
        default:
            flow.stage = Stage::summary;
            break;
        }
    }

    /// Ce que montre une tuile
    /// Pastilles d'état (le mot est au détail, au toucher)
    constexpr Color dot_ok = style::filament_green; ///< vert de l'atelier
    constexpr Color dot_measured = style::curve_blue; ///< mesuré, mais la mesure n'est pas validée : ni réussi ni échoué

    /// Un test réussi dont la mesure n'est pas encore validée sur la machine (V-03 : courroies, advice_validated)
    bool unvalidated(Test test) {
        return test == Test::belts && !belts::advice_validated;
    }

    struct TileView {
        std::optional<Color> dot; ///< pas de pastille : test sans objet ici
        Button::Look look = Button::Look::card; ///< alert : échec ; secondary : en cours
        bool active = false; ///< touchable : refaire un test du bilan, ouvrir l'écran d'un test manuel
    };

    TileView tile_view(const Entry &e, const calib_registry::Context &ctx, const calib_registry::Record &record, const assistant::Progress &p) {
        const uint32_t b = test_bit(e.test);
        const bool open_stage = flow.stage == Stage::propose || flow.stage == Stage::summary;
        if (!calib_registry::applicable(e, ctx)) {
            return {};
        }
        if (!e.automatic) { // à faire à la main : jaune ; fait : vert
            return { calib_registry::outdated(e, record) ? style::warning : dot_ok, Button::Look::card, open_stage };
        }
        if (flow.stage == Stage::propose) { // coché : orange ; décoché : vert si déjà fait, gris sinon ; le toucher coche ou décoche
            const bool done = (e.needs & calib_registry::filament) ? !calib_registry::spools_to_measure(e.test, ctx, false) : !calib_registry::outdated(e, record);
            return { (flow.mask & b) ? style::accent : (done ? (unvalidated(e.test) ? dot_measured : dot_ok) : style::text_muted), Button::Look::card, true };
        }
        if (!(flow.session & b)) { // à faire : orange ; à jour : gris
            return { calib_registry::outdated(e, record) ? style::accent : style::text_muted };
        }
        const bool summary = flow.stage == Stage::summary;
        if (flow.started & b) {
            switch (assistant::result(e.test)) {
            case Result::ok: // bilan : ses chiffres
                return { unvalidated(e.test) ? dot_measured : dot_ok, Button::Look::card, summary };
            case Result::failed:
                return { style::danger, Button::Look::alert, summary };
            case Result::skipped:
                return { style::warning, Button::Look::card, summary };
            case Result::none:
                if (p.running && p.step.test == e.test) {
                    return { style::accent, Button::Look::secondary, step_screen(p.step).func != nullptr }; // ses courbes
                }
                break;
            }
        }
        return summary ? TileView { style::warning, Button::Look::card, true } : TileView { style::accent };
    }

    // ---- Chiffres du bilan (seulement mesurés ou calculés ; prédictions marquées « estimé ») ----

    constexpr int16_t none = calib_registry::no_value;

    /// Texte écrit dans un tampon, coupé s'il est trop long
    class Writer {
    public:
        explicit Writer(std::span<char> out)
            : out(out) {
            out[0] = '\0';
        }

        [[gnu::format(printf, 2, 3)]] void add(const char *format, ...) {
            if (n + 1 >= out.size()) {
                return;
            }
            va_list args;
            va_start(args, format);
            const int w = vsnprintf(out.data() + n, out.size() - n, format, args);
            va_end(args);
            n = w > 0 ? std::min(out.size() - 1, n + size_t(w)) : n;
        }

        /// Texte traduit tel quel, copié directement dans la place qui reste (un tampon de 96 coupait les phrases
        /// françaises longues : « … comme le montre Pru », photo du 24/11)
        void text(const char *msgid) {
            if (n + 1 < out.size()) {
                n += _(msgid).copyToRAM(out.data() + n, out.size() - n);
            }
        }

        /// Libellé traduit en tête de ligne
        void label(const char *msgid) {
            if (n) {
                add("\n");
            }
            text(msgid);
            add(" ");
        }

        /// v / div avec ses décimales et son unité ; « -- » sans valeur
        void scaled(int16_t v, int div, const char *unit = "") {
            if (v == none) {
                add("--");
            } else if (div == 1) {
                add("%d%s", int(v), unit);
            } else {
                add(div == 10 ? "%.1f%s" : (div == 1000 ? "%.3f%s" : "%.4f%s"), double(v) / div, unit);
            }
        }

        /// « X -62 % » d'une baisse en % ; « X -- » sans valeur
        void drop(char axis, int16_t v) {
            add("%c ", axis);
            if (v == none) {
                add("--");
            } else {
                add("%+d %%", -int(v));
            }
        }

    private:
        std::span<char> out;
        size_t n = 0;
    };

    bool any(const calib_registry::Numbers &n, size_t from, size_t to) {
        return std::any_of(n.begin() + from, n.begin() + to, [](int16_t v) { return v != none; });
    }

    /// Vibrations des ventilateurs (M2001 V) : par ventilateur, amplitude à sa fréquence de rotation et comparaison à
    /// sa référence (une valeur seule ne dit rien)
    void fan_vibration_details(Writer &w) {
        static constexpr const char *names[fan_vibration::fan_count] = { N_("part"), N_("hotend"), N_("chamber 1"), N_("chamber 2") };
        w.label(N_("Fan vibration at its speed:"));
        if (fan_vibration::status() != fan_vibration::Status::done) {
            w.text(fan_vibration::status() == fan_vibration::Status::no_accelerometer ? N_("not measured, no accelerometer") : N_("not measured"));
            return;
        }
        for (uint8_t i = 0; i < fan_vibration::fan_count; ++i) {
            const fan_vibration::FanResult &f = fan_vibration::last().fans[i];
            if (!f.present) {
                continue;
            }
            w.add("\n ");
            w.text(names[i]);
            w.add(": ");
            switch (f.verdict) {
            case vibration::Verdict::not_detected:
                w.text(N_("not picked up from the nozzle"));
                break;
            case vibration::Verdict::reference:
                w.add("%.3f m/s2, ", double(f.level.value));
                w.text(N_("now its reference"));
                break;
            case vibration::Verdict::ok:
            case vibration::Verdict::worse:
                w.add("%.3f m/s2, x%.1f ", double(f.level.value), double(f.level.value / *f.reference));
                w.text(f.verdict == vibration::Verdict::worse ? N_("vs reference: imbalance or wear?") : N_("vs reference"));
                break;
            }
        }
    }

    /// Pose de l'accéléromètre (M2005) jugée mauvaise : la raison et ce qu'il faut faire ; false sans mesure finie
    bool mount_reason(Writer &w) {
        const accel_mount::Measurement &m = accel_mount::last();
        if (accel_mount::status() != vibration::Status::done || m.verdict == vibration::Mount::ok) {
            return false;
        }
        char format[112];
        switch (m.verdict) {
        case vibration::Mount::tilted:
            _("The accelerometer is clipped askew (tilt %d deg). Clip it flat on the nozzle.").copyToRAM(format, sizeof(format));
            w.add(format, std::isfinite(m.tilt_deg) ? int(std::lround(m.tilt_deg)) : 90);
            break;
        case vibration::Mount::loose:
            w.text(N_("The accelerometer moves on the nozzle: it barely follows the moves. Clip it firmly."));
            break;
        case vibration::Mount::ok:
            break;
        }
        return true;
    }

    /// Vérification de l'input shaper (M2006) en échec : l'axe dont le filtre ne retire presque plus rien
    bool shaper_reason(Writer &w) {
        const shaper_check::Measurement &m = shaper_check::last();
        if (shaper_check::status() != vibration::Status::done) {
            return false;
        }
        bool any_filter = false, written = false;
        char format[112];
        for (uint8_t i = 0; i < shaper_check::axis_count; ++i) {
            any_filter = any_filter || m.has_filter[i];
            if (m.verdict[i] == vibration::ShaperCheck::weak) {
                _("Axis %c: the filter removes only %d%% of the ringing. Redo the input shaper.").copyToRAM(format, sizeof(format));
                if (written) {
                    w.add("\n");
                }
                w.add(format, 'X' + i, int(std::lround(100.f * m.reduction[i])));
                written = true;
            }
        }
        if (!any_filter) {
            w.text(N_("No input shaper filter set: do the input shaper first."));
            written = true;
        }
        return written;
    }

    /// Ventilateurs (M2001 V) en échec (retour du 24/11 : « le test n'est pas allé au bout », sans dire pourquoi) : la
    /// partie qui a échoué et sa raison ; false sans raison connue
    bool fan_reason(Writer &w) {
        bool written = false;
        const auto line = [&](const char *msgid) {
            if (written) {
                w.add("\n");
            }
            w.text(msgid);
            written = true;
        };
        switch (fan_response::status()) {
        case fan_response::Status::no_signal: {
            char format[160];
            _("Part fan: no speed read at 100%% (%d RPM). Check that it spins and its cable.").copyToRAM(format, sizeof(format));
            w.add(format, int(std::lround(fan_response::result().rise.rpm)));
            written = true;
            break;
        }
        case fan_response::Status::stopped:
            line(N_("Part fan rise time: stopped before the end."));
            break;
        case fan_response::Status::hot_nozzle:
            line(N_("Let the nozzle cool below 50°C, then start again."));
            break;
        default:
            break;
        }
        switch (fan_vibration::status()) {
        case vibration::Status::failed:
            line(N_("Fan vibration: the accelerometer stopped answering or lost samples. Check its cable, then redo Fans."));
            break;
        case vibration::Status::stopped:
            line(N_("Fan vibration: stopped before the end."));
            break;
        default:
            break;
        }
        return written;
    }

    /// Détail d'un test du bilan : ses chiffres, d'avant et d'après
    void build_details(Test test, Writer &w) {
        const calib_registry::Numbers &a = assistant::numbers_after(test);
        const calib_registry::Numbers &b = assistant::numbers_before(test);
        // Test par bobine : les chiffres des bobines réussies, même si une autre a échoué ou a été passée ; « -- » pour celle-là
        const bool shown = calib_registry::numbers_shown(calib_registry::entry(test), assistant::result(test), a);
        if (!(flow.started & test_bit(test)) || (assistant::result(test) == Result::skipped && !shown)) {
            w.text(N_("Skipped: still to do next time."));
            return;
        }
        if (assistant::result(test) == Result::failed
            && ((test == Test::accel_mount && mount_reason(w)) || (test == Test::shaper_check && shaper_reason(w)) || (test == Test::fan_response && fan_reason(w)))) {
            return;
        }
        if (!shown) {
            w.text(N_("No numbers: the test did not finish."));
            return;
        }
        const auto pair = [&](const calib_registry::Numbers &n, const char *names, int div, const char *unit) {
            for (uint8_t i = 0; i < 2; ++i) {
                w.add(i ? ", %c " : "%c ", names[i]);
                w.scaled(n[i], div, unit);
            }
        };
        switch (test) {
        case Test::input_shaper: {
            const auto filters = [&](const calib_registry::Numbers &n) {
                for (uint8_t i = 0; i < 2; ++i) {
                    w.add(i ? ", %c " : "%c ", i ? 'Y' : 'X');
                    if (n[2 * i + 1] == none) {
                        w.add("--");
                    } else {
                        w.add("%s ", input_shaper::to_string(input_shaper::Type(n[2 * i])));
                        w.scaled(n[2 * i + 1], 10, " Hz");
                    }
                }
            };
            w.label(N_("Filters:"));
            filters(a);
            if (any(b, 0, 4)) {
                w.label(N_("Before:"));
                filters(b);
            }
            w.label(N_("Vibrations vs no filter (estimated):"));
            w.drop('X', a[4]);
            w.add(", ");
            w.drop('Y', a[5]);
            if (any(a, 6, 8)) {
                w.label(N_("Vibrations vs before (estimated):"));
                w.drop('X', a[6]);
                w.add(", ");
                w.drop('Y', a[7]);
            }
            break;
        }
        case Test::phase_stepping:
            w.label(N_("Motor vibrations (measured by Prusa):"));
            w.drop('X', a[0]);
            w.add(", ");
            w.drop('Y', a[1]);
            if (any(b, 0, 2)) {
                w.label(N_("Before:"));
                w.drop('X', b[0]);
                w.add(", ");
                w.drop('Y', b[1]);
            }
            break;
        case Test::belts: {
            if (belts::compared()) { // V-03 : les pics de chaque mode, ceux de la mesure habituelle compris
                // Titre et quatre modes, en français avec des pics à trois chiffres : bâtis dans le tampon partagé,
                // aussitôt copiés dans le texte
                auto &peaks = scratch_as<std::array<char, 288>>();
                belt_peaks_text(peaks);
                w.add("%s", peaks.data());
            } else {
                w.label(N_("Belts:"));
                pair(a, "AB", 10, " Hz");
                if (a[0] != none && a[1] != none) {
                    w.add(", A-B %.1f Hz", std::abs(a[0] - a[1]) / 10.0);
                }
                if (any(b, 0, 2)) {
                    w.label(N_("Before:"));
                    pair(b, "AB", 10, " Hz");
                }
            }
            if (belts::advice_validated) {
                const belts::Targets t = belts::targets();
                w.label(N_("Target:"));
                w.add("%.0f-%.0f Hz", double(t.result_min), double(t.result_max));
            } else {
                w.add("\n");
                w.text(N_("Measurement being validated: adjust with the Prusa app or the Prusa wizard (M961)."));
            }
            break;
        }
        case Test::damping:
            w.label(N_("Damping:"));
            pair(a, "XY", 1000, "");
            w.label(N_("Cycles to damp ringing to 5% (estimated):"));
            for (uint8_t i = 0; i < 2; ++i) {
                w.add(i ? ", %c " : "%c ", "XY"[i]);
                if (a[i] == none) {
                    w.add("--");
                } else {
                    w.add("%.1f", double(calib_registry::ringing_cycles(a[i] / 1000.f)));
                }
            }
            w.add(" (zeta 0.1: %.1f)", double(calib_registry::ringing_cycles(0.1f)));
            if (any(b, 0, 2)) {
                w.label(N_("Before:"));
                pair(b, "XY", 1000, "");
            }
            break;
        case Test::speed_map:
            w.label(N_("Critical speeds:"));
            w.add("%d (X %d, Y %d), %d-%d mm/s", int(a[0]), int(a[1]), int(a[2]), int(speed_map::speed_start), int(speed_map::speed_end));
            if (b[0] != none) {
                w.label(N_("Before:"));
                w.add("%d", int(b[0]));
            }
            break;
#if HAS_LOADCELL() && HAS_MMU2()
        case Test::mpc: {
            const auto model = [&](const calib_registry::Numbers &n) {
                w.add("P ");
                w.scaled(n[0], 10, " W, C ");
                w.scaled(n[1], 10, " J/K, A ");
                w.scaled(n[2], 1000, " W/K, F ");
                w.scaled(n[3], 1000, " W/K");
            };
            w.label(N_("MPC model:"));
            model(a);
            if (any(b, 0, 4)) {
                w.label(N_("Before:"));
                model(b);
            }
            break;
        }
        case Test::mmu_ptfe:
            w.label(N_("Front PTFE length:"));
            w.scaled(a[0], 1, " mm");
            w.label(N_("Measured:"));
            w.scaled(a[1], 1, " mm");
            if (b[0] != none) {
                w.label(N_("Before:"));
                w.scaled(b[0], 1, " mm");
            }
            break;
#endif
        case Test::fan_response: {
#if !HAS_INDX() // INDX : ni temps de montée ni avance (M2001, IC-7), les vibrations seules
            const auto lead = [&](int16_t ds) {
                if (ds == 0) {
                    w.text(N_("Off"));
                } else {
                    w.scaled(ds, 10, " s");
                }
            };
            w.label(N_("Part fan rise time:"));
            w.scaled(a[0], 1000, " s");
            w.add(" (%d RPM)", int(std::lround(fan_response::result().rise.rpm)));
            w.label(N_("Fan lead applied:"));
            lead(a[1]);
            if (b[1] != none) {
                w.label(N_("Before:"));
                lead(b[1]);
                if (b[0] != none) {
                    w.add(", ");
                    w.scaled(b[0], 1000, " s");
                }
            }
#endif
            fan_vibration_details(w);
            break;
        }
        case Test::accel_limit: {
            w.label(N_("Max useful accel., mm/s2:"));
            for (uint8_t i = 0; i < 2; ++i) {
                w.add(i ? ", %c " : "%c ", "XY"[i]);
                w.scaled(a[i], 1);
                if (a[i] != none && a[i] == int16_t(accel_limit::last().top[i])) {
                    w.add("+");
                }
            }
            w.label(N_("M201 before:"));
            pair(b, "XY", 1, "");
            w.add("\n");
            w.text(N_("+: no rise up to the top level. --: in the noise, nothing to change."));
            break;
        }
        case Test::accel_mount:
            w.label(N_("Tilt:"));
            w.scaled(a[0], 10, "\xC2\xB0");
            w.label(N_("Share of the moved axis:"));
            for (uint8_t i = 0; i < 2; ++i) {
                w.add(i ? ", %c " : "%c ", "XY"[i]);
                w.scaled(a[1 + i], 1, " %");
            }
            w.label(N_("Noise at rest:"));
            w.scaled(a[3], 1000, " m/s2");
            if (b[0] != none) {
                w.label(N_("Before:"));
                w.scaled(b[0], 10, "\xC2\xB0");
            }
            break;
        case Test::shaper_check:
            w.label(N_("Ringing removed by the filter:"));
            for (uint8_t i = 0; i < 2; ++i) {
                w.add(i ? ", %c " : "%c ", "XY"[i]);
                w.scaled(a[i], 1, " %");
            }
            if (a[2] != none) {
                w.add(" (%d mm/s2)", int(a[2]));
            }
            if (any(b, 0, 2)) {
                w.label(N_("Before:"));
                for (uint8_t i = 0; i < 2; ++i) {
                    w.add(i ? ", %c " : "%c ", "XY"[i]);
                    w.scaled(b[i], 1, " %");
                }
            }
            w.add("\n");
            w.text(N_("--: no filter on this axis, or no ringing even without it."));
            break;
#if HAS_LOADCELL() && HAS_MMU2()
        case Test::nozzle_expansion: {
            constexpr const char *unit = " um/\xC2\xB0""C";
            w.label(N_("Measured slope:"));
            w.scaled(a[0], 1000, unit);
            w.label(N_("Prusa model:"));
            w.scaled(a[1], 1000, unit);
            w.label(N_("At 250\xC2\xB0""C vs Prusa:"));
            w.add("%+.1f um", a[2] == none ? 0.0 : a[2] / 10.0);
            w.label(N_("Hysteresis:"));
            w.scaled(a[3], 10, " um");
            w.label(N_("Before:"));
            if (b[0] == none) {
                w.text(N_("Prusa model"));
            } else {
                w.scaled(b[0], 1000, unit);
            }
            break;
        }
        case Test::mmu_speed:
            w.label(N_("Pulley speed:"));
            w.scaled(a[0], 1, " mm/s");
            if (b[0] != none) {
                w.add(" (");
                w.text(N_("before"));
                w.add(" %d)", int(b[0]));
            }
            w.label(N_("Load + unload cycle (measured):"));
            w.scaled(a[1], 10, " s");
            w.add(" -> ");
            w.scaled(a[2], 10, " s");
            w.label(N_("Last speed without error:"));
            w.scaled(a[3], 1, " mm/s");
            break;
        case Test::pa:
        case Test::max_flow:
        case Test::min_temp:
            w.label(test == Test::pa ? N_("K per spool (before):") : (test == Test::max_flow ? N_("Max flow per spool, mm3/s (before):") : N_("Min temperature per spool, \xC2\xB0""C (material):")));
            for (uint8_t slot = 0; slot < VirtualToolIndex::count; ++slot) {
                if (a[slot] == none && b[slot] == none && !(assistant::context().spools & (1u << slot))) {
                    continue;
                }
                w.add("\n %d: ", slot + 1);
                const int div = test == Test::pa ? 10000 : (test == Test::max_flow ? 10 : 1);
                w.scaled(a[slot], div);
                w.add(" (");
                if (test == Test::min_temp) {
                    w.add("%d", int(FilamentType::for_tool(VirtualToolIndex::from_raw(slot)).parameters().nozzle_temperature));
                } else {
                    w.scaled(b[slot], div);
                }
                w.add(")");
            }
            break;
        case Test::e_jump: {
            // F avant → après avec le K de chaque emplacement, puis la ligne 0,45 × 0,2 qu'elle permet (calculée)
            const auto line = [](int16_t f) {
                return f == none ? none : int16_t(std::lround(e_jump::line_accel(float(f))));
            };
            w.label(N_("Max filament accel. per spool, mm/s2 (before):"));
            for (uint8_t slot = 0; slot < VirtualToolIndex::count; ++slot) {
                if (a[slot] != none || b[slot] != none) {
                    w.add("\n %d: ", slot + 1);
                    w.scaled(a[slot], 1);
                    w.add(" (");
                    w.scaled(b[slot], 1);
                    w.add(")");
                }
            }
            w.label(N_("Line 0.45 x 0.2 mm, mm/s2 (estimated):"));
            for (uint8_t slot = 0; slot < VirtualToolIndex::count; ++slot) {
                if (a[slot] != none) {
                    w.add("\n %d: ", slot + 1);
                    w.scaled(line(a[slot]), 1);
                    w.add(" (");
                    w.scaled(line(b[slot]), 1);
                    w.add(")");
                }
            }
            break;
        }
#else
        default: // INDX : série « Chauffe » absente
            break;
#endif
        case Test::skew:
        case Test::count:
            break;
        }
    }

    /// Bilan : réussites, échecs, puis les gains principaux en une ligne. brief : la seule ligne de compte, pour la
    /// ligne de texte de l'écran ; le bilan complet s'ouvre par « Bilan »
    void build_summary(Writer &w, bool brief = false) {
        int ok = 0, failed = 0, skipped = 0, measured = 0;
        for (const Entry &e : entries) {
            if (!(flow.session & test_bit(e.test)) || !e.automatic) {
                continue;
            }
            const Result r = (flow.started & test_bit(e.test)) ? assistant::result(e.test) : Result::skipped; // décoché
            const bool neutral = r == Result::ok && unvalidated(e.test); // V-03 : ni réussi ni échoué
            ok += r == Result::ok && !neutral;
            measured += neutral;
            failed += r == Result::failed;
            skipped += r == Result::skipped || r == Result::none;
        }
        char format[96];
        const auto add_measured = [&] {
            if (measured) {
                _("%d measured, not validated.").copyToRAM(format, sizeof(format));
                w.add(" ");
                w.add(format, measured);
            }
        };
        if (brief) {
            _("Finished: %d OK, %d failed, %d skipped.").copyToRAM(format, sizeof(format));
            w.add(format, ok, failed, skipped);
            add_measured();
            return;
        }
        _("Finished: %d OK, %d failed, %d skipped. Touch a test for its numbers.").copyToRAM(format, sizeof(format));
        w.add(format, ok, failed, skipped);
        add_measured();
        bool first = true;
        const auto next = [&](const char *msgid) {
            w.add(first ? "\n" : ", ");
            first = false;
            w.text(msgid);
        };
        const auto done = [&](Test t) {
            return (flow.started & test_bit(t)) && assistant::result(t) == Result::ok;
        };
        if (const auto &a = assistant::numbers_after(Test::input_shaper); done(Test::input_shaper) && a[4] != none && a[5] != none) {
            next(N_("Vibrations"));
            w.add(" X %+d %%, Y %+d %% ", -int(a[4]), -int(a[5]));
            w.text(N_("(estimated)"));
        }
        if (const auto &a = assistant::numbers_after(Test::phase_stepping); done(Test::phase_stepping) && a[0] != none) {
            next(N_("motors"));
            w.add(" X -%d %%, Y -%d %%", int(a[0]), int(a[1] == none ? 0 : a[1]));
        }
        if (const auto &a = assistant::numbers_after(Test::belts); done(Test::belts) && a[0] != none && a[1] != none) {
            next(N_("belts"));
            w.add(" %.1f/%.1f Hz", a[0] / 10.0, a[1] / 10.0);
            if (unvalidated(Test::belts)) {
                w.add(" ");
                w.text(N_("(not validated)"));
            }
        }
        if (done(Test::max_flow)) {
            const auto &a = assistant::numbers_after(Test::max_flow);
            const int16_t best = *std::max_element(a.begin(), a.end());
            if (best != none) {
                next(N_("max flow"));
                w.add(" %.1f mm3/s", best / 10.0);
            }
        }
        if (const auto &a = assistant::numbers_after(Test::fan_response); done(Test::fan_response) && a[0] != none) {
            next(N_("part fan"));
            w.add(" %.2f s, ", a[0] / 1000.0);
            w.text(N_("lead"));
            w.add(" %.1f s", a[1] / 10.0);
        }
        if (const auto &a = assistant::numbers_after(Test::accel_limit); done(Test::accel_limit) && (a[0] != none || a[1] != none)) {
            next(N_("max accel."));
            w.add(" X %d, Y %d mm/s2", int(a[0] == none ? assistant::numbers_before(Test::accel_limit)[0] : a[0]),
                int(a[1] == none ? assistant::numbers_before(Test::accel_limit)[1] : a[1]));
        }
        if (const auto &a = assistant::numbers_after(Test::shaper_check); done(Test::shaper_check) && (a[0] != none || a[1] != none)) {
            next(N_("filter removes"));
            for (uint8_t i = 0; i < 2; ++i) {
                w.add(i ? ", %c " : " %c ", "XY"[i]);
                w.scaled(a[i], 1, " %");
            }
        }
        // Pose de l'accéléromètre, vérification du filtre : la raison d'un échec, jamais un « échec » seul
        if ((flow.started & test_bit(Test::accel_mount)) && assistant::result(Test::accel_mount) == Result::failed) {
            w.label(N_("Accelerometer:"));
            if (!mount_reason(w)) {
                w.text(N_("not measured"));
            }
        }
        if ((flow.started & test_bit(Test::shaper_check)) && assistant::result(Test::shaper_check) == Result::failed) {
            w.label(N_("Filter check:"));
            if (!shaper_reason(w)) {
                w.text(N_("not measured"));
            }
        }
        if ((flow.started & test_bit(Test::fan_response)) && assistant::result(Test::fan_response) == Result::failed) {
            w.label(N_("Fans:"));
            if (!fan_reason(w)) {
                w.text(N_("not measured"));
            }
        }
        if (done(Test::fan_response)
            && std::any_of(fan_vibration::last().fans.begin(), fan_vibration::last().fans.end(), [](const auto &f) { return f.verdict == vibration::Verdict::worse; })) {
            w.add("\n");
            w.text(N_("A fan vibrates much more than its reference: imbalance or wear? Touch Fans."));
        }
        if (const auto &a = assistant::numbers_after(Test::nozzle_expansion); done(Test::nozzle_expansion) && a[2] != none) {
            next(N_("nozzle at 250\xC2\xB0""C"));
            w.add(" %+.1f um", a[2] / 10.0);
        }
        // Courroies : pic hors de la plage balayée ou trop peu marqué, jamais un conseil (retour du 23/11)
        if ((flow.started & test_bit(Test::belts)) && assistant::result(Test::belts) == Result::failed && belts::status() == belts::Status::done) {
            w.add("\n");
            w.text(N_("Belts: inconclusive, peak out of range (too tight or too loose?)."));
        }
        // MMU muet pendant la série (flash par l'USB avant...) : ses tests sont reportés, pas en échec
        if (const int later = std::popcount(assistant::postponed() & flow.started); later > 0) {
            char line[96];
            _("MMU not answering: %d tests postponed, still to do.").copyToRAM(line, sizeof(line));
            w.add("\n");
            w.add(line, later);
        }
#if HAS_LOADCELL() && HAS_MMU2()
        // Dilatation de la buse : la raison d'un échec, comme le tube PTFE
        if ((flow.started & test_bit(Test::nozzle_expansion)) && assistant::result(Test::nozzle_expansion) == Result::failed) {
            w.label(N_("Expansion:"));
            w.text(nozzle_expansion_result_text(nozzle_expansion::status()));
        }
        if (done(Test::e_jump)) {
            // L'emplacement qui gagne le plus : F avant → après, et la ligne 0,45 × 0,2 permise (calculée)
            const auto &a = assistant::numbers_after(Test::e_jump);
            const auto &b = assistant::numbers_before(Test::e_jump);
            const size_t best = size_t(std::max_element(a.begin(), a.end()) - a.begin());
            if (a[best] != none) {
                const int before = b[best] == none ? int(e_jump::prusa_filament_accel) : int(b[best]);
                next(N_("filament accel."));
                w.add(" %d->%d mm/s2, ", before, int(a[best]));
                w.text(N_("lines"));
                w.add(" %ld->%ld mm/s2 ", std::lround(e_jump::line_accel(float(before))), std::lround(e_jump::line_accel(float(a[best]))));
                w.text(N_("(estimated)"));
            }
        }
        // Vitesse de la poulie du MMU (N4) : gain mesuré d'un changement d'outil
        if (const auto &a = assistant::numbers_after(Test::mmu_speed); done(Test::mmu_speed) && a[1] != none && a[2] != none && a[2] < a[1]) {
            next(N_("MMU change"));
            w.add(" %+.1f s", (a[2] - a[1]) / 10.0);
        }
        if ((flow.started & test_bit(Test::mmu_speed)) && assistant::result(Test::mmu_speed) == Result::failed) {
            if (const char *why = mmu_speed_result_text(mmu_speed::last().result)) {
                w.label(N_("MMU speed:"));
                w.text(why);
            }
        }
        // Tube PTFE avant (F41) : la raison de l'échec, jamais un « échec » seul
        if ((flow.started & test_bit(Test::mmu_ptfe)) && assistant::result(Test::mmu_ptfe) == Result::failed) {
            w.label(N_("Front PTFE length:"));
            w.text(ptfe_result_text(ptfe::status().result));
        }
#endif
    }

    /// Avancement de la mesure de l'étape (les mêmes chiffres que son écran et la pastille), s'il est connu
    std::optional<uint8_t> step_percent(const calib_registry::Step &s) {
        const auto ratio = [](unsigned step, unsigned steps) -> std::optional<uint8_t> {
            return steps ? std::optional<uint8_t>(uint8_t(std::min(100u, step * 100u / steps))) : std::nullopt;
        };
        switch (s.test) {
        case Test::belts:
            return belts::is_running() ? std::optional<uint8_t>(belts::progress_percent()) : std::nullopt;
        case Test::speed_map:
            return speed_map::is_running() ? std::optional<uint8_t>(speed_map::progress_percent()) : std::nullopt;
        case Test::damping:
            return damping::is_running() ? std::optional<uint8_t>(damping::progress_percent()) : std::nullopt;
#if HAS_LOADCELL() && HAS_MMU2()
        case Test::nozzle_expansion:
            return nozzle_expansion::is_running() ? std::optional<uint8_t>(nozzle_expansion::progress_percent()) : std::nullopt;
#endif
        case Test::fan_response:
            if (fan_vibration::is_running()) {
                return fan_vibration::progress_percent();
            }
            return fan_response::is_running() ? std::optional<uint8_t>(fan_response::progress_percent()) : std::nullopt;
        case Test::accel_limit:
            return accel_limit::is_running() ? std::optional<uint8_t>(accel_limit::progress_percent()) : std::nullopt;
        case Test::accel_mount:
            return accel_mount::is_running() ? std::optional<uint8_t>(accel_mount::progress_percent()) : std::nullopt;
        case Test::shaper_check:
            return shaper_check::is_running() ? std::optional<uint8_t>(shaper_check::progress_percent()) : std::nullopt;
#if HAS_LOADCELL() && HAS_MMU2()
        case Test::mpc:
            if (const mpc::TuneStatus st = mpc::tune_status(); st.running) {
                return st.progress;
            }
            return std::nullopt;
        case Test::pa:
            if (const pa::Status st = pa::status(); pa::is_running() && st.phase == pa::Phase::measuring) {
                return ratio(st.step, st.steps);
            }
            return std::nullopt;
        case Test::max_flow:
        case Test::min_temp:
            if (const calib::Status st = calib::status(); calib::is_running() && st.phase == calib::Phase::measuring) {
                return ratio(st.step, st.steps);
            }
            return std::nullopt;
        case Test::e_jump:
            if (const e_jump::Status st = e_jump::status(); e_jump::is_running() && st.phase == e_jump::Phase::measuring) {
                return ratio(st.step, st.steps);
            }
            return std::nullopt;
        case Test::mmu_speed:
            return mmu_speed::is_running() ? ratio(mmu_speed::last().step, mmu_speed::cycles) : std::nullopt;
#endif
        default:
            return std::nullopt;
        }
    }

    /// Une phrase publiée par la mesure, traduite, avec ses nombres
    void add_line(Writer &w, const live::Line &l) {
        char format[96];
        _(l.msgid).copyToRAM(format, sizeof(format));
        w.add(format, int(l.a), int(l.b), int(l.c));
    }

    /// Pendant la série (retour du 23/11 : « voir plus en détail ce que l'imprimante fait ») : l'étape et son
    /// avancement, ce que fait l'imprimante en clair, puis la valeur mesurée dès qu'elle existe
    void build_running(const assistant::Progress &p, Writer &w) {
        // Une ligne (retour du 24/11 : les tuiles ont pris la place) : « 3/8 Courroies 45 % · ce qu'il fait · valeur »
        w.add("%d/%d ", p.index + 1, int(p.count));
        w.text(name_of(p.step.test));
        if (calib_registry::entry(p.step.test).needs & (calib_registry::filament | calib_registry::one_spool)) {
            w.add(" %d", p.step.slot + 1); // l'emplacement
        }
        if (const auto percent = step_percent(p.step)) {
            w.add(" %d%%", int(*percent));
        }
        w.add(" \xC2\xB7 ");
        char wizard[32];
        prusa_live(wizard, sizeof(wizard)); // input shaper, phase stepping : l'assistant de Prusa
        if (wizard[0]) {
            w.add("%s", wizard);
        } else if (const live::Line l = live::now(); l.msgid) {
            add_line(w, l);
        } else {
            w.text(N_("You may leave this screen: the pill shows the progress."));
        }
        if (const live::Line v = live::value(); v.msgid) {
            w.add(" \xC2\xB7 ");
            add_line(w, v);
        }
    }

    /// Le détail d'un test du bilan ; vrai pour le refaire
    bool show_details(Test test) {
        const std::span<char> text = message_text(); // la boîte le lit tant qu'elle est ouverte ; ventilateurs : une ligne chacun
        Writer w(text);
        build_details(test, w);
        const bool can_retry = assistant::result(test) != Result::ok;
        const PhaseResponses responses = can_retry ? PhaseResponses { Response::Retry, Response::Ok, Response::_none, Response::_none } : Responses_Ok;
        return MsgBoxInfo(string_view_utf8::MakeRAM(text.data()), responses, can_retry ? 1 : 0) == Response::Retry; // « OK » par défaut
    }

    template <size_t... I>
    std::array<CalibTile, sizeof...(I)> make_tiles(window_t *parent, std::index_sequence<I...>) {
        return { CalibTile(parent, tile_body(I), uint8_t(I))... };
    }

    /// Retour de l'en-tête : la proposition et le bilan se referment (« Plus tard »), une session en cours continue
    void leave() {
        if (flow.stage == Stage::propose || flow.stage == Stage::summary) {
            flow.stage = Stage::idle;
        }
        Screens::Access()->close_screen();
    }

} // namespace

bool calibration_prompt_at_boot() {
    const uint16_t signature = calib_registry::signature();
    if (assistant::offered_signature() == signature) {
        return false; // déjà proposée pour ces versions : plus jamais avant la prochaine mise à jour qui en change une
    }
    const uint32_t mask = calib_registry::proposed(assistant::context(), assistant::records());
    if (!mask) {
        return false;
    }
    assistant::set_offered_signature(signature); // une seule fois, quoi qu'on réponde (« Fermer » : plus tard, depuis Réglages)
    flow = Flow { .stage = Stage::propose, .boot = true, .mask = mask, .offered = mask };
    return true;
}

bool calibration_assistant_active() {
    return flow.stage != Stage::idle && flow.stage != Stage::propose && flow.stage != Stage::summary;
}

bool calibration_series_running() {
    return assistant::progress().running;
}

namespace {
    ScreenFactory::Creator curves_opened; ///< écran de la mesure ouvert depuis la tuile de l'étape en cours
} // namespace

void calibration_follow() {
    // L'assistant montre lui-même chaque étape en clair (retour du 23/11) ; l'écran d'une mesure, ouvert depuis sa
    // tuile pour ses courbes, revient à l'assistant quand l'étape change ou que la série finit
    static uint32_t followed_step = 0; ///< (séries << 8 | étape) + 1 de l'étape en cours ; 0 : pas de série
    const assistant::Progress p = assistant::progress();
    const uint32_t step = p.running ? (uint32_t(p.runs) << 8 | p.index) + 1 : 0;
    if (step == followed_step) {
        return;
    }
    followed_step = step;
    Screens &screens = *Screens::Access();
    if (curves_opened.func && screens.IsScreenOpened(curves_opened)) {
        screens.close_screen();
    }
    curves_opened = {};
}

bool confirm_stop_measure() {
    if (!calibration_series_running()) {
        return true;
    }
    const assistant::Progress p = assistant::progress();
    if (MsgBoxQuestion(_("Stop this measurement? The assistant goes on with the next one."), Responses_YesNo, 1) != Response::Yes) {
        return false;
    }
    assistant::request_skip(p.index); // « passée » au bilan, pas un échec
    return true;
}

string_view_utf8 calibration_assistant_name() {
    const assistant::Progress p = assistant::progress();
    return p.running ? _(name_of(p.step.test)) : _("Calibration");
}

// ---- Tuile ----

CalibTile::CalibTile(window_t *parent, Rect16 body, uint8_t tag)
    : Button(parent, body, tag, Look::card, string_view_utf8::MakeNULLSTR(), nullptr, style::screen, tile_radius) {
    set_font(Font::small);
}

void CalibTile::show(const string_view_utf8 &name, std::optional<Color> new_dot, Look new_look, bool active) {
    if (name.is_same_ref(label) && new_dot == dot && new_look == look && active == IsEnabled()) {
        return;
    }
    label = name;
    dot = new_dot;
    look = new_look;
    set_enabled(active);
    Invalidate();
}

void CalibTile::unconditionalDraw() {
    const Colors c = colors();
    draw_frame(body(), c); // corps arrondi et contour d'une passe, marge comprise
    // Nom et pastille repeignent leur rect au fond du corps : ils restent hors des carrés des coins (tile_radius de
    // chaque bord), pour ne jamais poser un rectangle sur l'arrondi anticrénelé
    const Rect16 b = body();
    constexpr int16_t h = height(Font::small);
    const int16_t dot_x = b.Left() + b.Width() - tile_radius - tile_dot;
    const int16_t name_x = b.Left() + tile_radius;
    const Color fg = !dot ? style::text_faint : (IsEnabled() ? c.fg : style::text); // lisible même pas touchable
    draw_text(Rect16(name_x, b.Top() + (b.Height() - h) / 2, dot_x - 4 - name_x, h), label, Font::small, fg, c.fill);
    if (dot) {
        // Disque anticrénelé (contour de la même couleur), comme les coins des cartes
        draw_rounded_frame(Rect16(dot_x, b.Top() + (b.Height() - tile_dot) / 2, tile_dot, tile_dot), c.fill, *dot, *dot, tile_dot / 2, 1);
    }
}

// ---- Écran ----

ScreenCalibAssistant::ScreenCalibAssistant()
    : Screen(true)
    , header(this, _("Calibration"), true, leave)
    , info(this, info_rect, is_multiline::yes)
    , tiles(make_tiles(this, std::make_index_sequence<entries.size()>()))
    , button_a(this, button_body(0), tag_a, Button::Look::primary, string_view_utf8::MakeNULLSTR(), nullptr, style::screen)
    , button_b(this, button_body(1), tag_b, Button::Look::secondary, string_view_utf8::MakeNULLSTR(), nullptr, style::screen)
    , button_c(this, button_body(2), tag_c, Button::Look::outline, string_view_utf8::MakeNULLSTR(), nullptr, style::screen)
    , tabs(this, section_tab()) {
    ClrMenuTimeoutClose(); // l'assistant attend l'utilisateur ou une longue série
    if (flow.stage == Stage::idle) {
        const uint32_t mask = tests_to_offer(); // depuis le menu : ce qui n'est pas encore fait
        flow = Flow { .stage = Stage::propose, .mask = mask, .offered = mask };
    }
    info.set_font(Font::small);
    info.SetAlignment(Align_t::LeftCenter());
    info.SetTextColor(style::text_soft);
    info.SetBackColor(style::screen);
    info.SetPadding({ 0, 0, 0, 0 });
    for (Button *b : { &button_a, &button_b, &button_c }) {
        b->set_font(Font::small);
    }
    update();
}

ScreenCalibAssistant::~ScreenCalibAssistant() {
    if (motors_held) {
        sleep::hold_motors(false);
    }
}

void ScreenCalibAssistant::update() {
    // Origine prise avant de demander de fixer l'accéléromètre (M1989 H) : les moteurs restent alimentés jusqu'à la
    // fin de ses tests, sinon elle se perdrait et serait refaite accéléromètre fixé (retour du 23/11)
    const bool hold = flow.stage == Stage::cooling || flow.stage == Stage::sock_off || flow.stage == Stage::attach || flow.stage == Stage::checking
        || flow.stage == Stage::accel_run || flow.stage == Stage::belts_adjust;
    if (hold != motors_held) {
        motors_held = hold;
        sleep::hold_motors(hold);
    }
    switch (flow.stage) {
    case Stage::checking:
        if (assistant::check_count() != flow.checks_at_request) {
            switch (assistant::check_result()) {
            case assistant::Check::ok:
                flow.check_failed = false;
                run_part(true);
                break;
            case assistant::Check::no_accelerometer:
                flow.check_failed = true;
                flow.stage = Stage::attach;
                break;
            case assistant::Check::not_homed:
                flow.stage = Stage::rehome;
                break;
            case assistant::Check::hot:
                enter_cooling();
                break;
            default:
                flow.stage = Stage::summary;
                break;
            }
        }
        break;
    case Stage::accel_run:
        if (run_finished()) {
            after_accel_part();
        }
        break;
    case Stage::cell_run:
        if (run_finished()) {
            flow.stage = Stage::summary;
        } else if (!flow.cancelled && assistant::progress().bed_full) {
            flow.stage = Stage::bed_full;
        }
        break;
    case Stage::bed_full:
        if (!assistant::progress().bed_full) {
            flow.stage = Stage::cell_run; // M1989 C lu par Marlin : l'étape repart
        }
        break;
    default:
        break;
    }

    const bool running_stage = flow.stage == Stage::accel_run || flow.stage == Stage::cell_run;
    const assistant::Progress p = assistant::progress();
    const bool skipping = running_stage && p.running && flow.skip_index == p.index && flow.skip_runs == p.runs;
    if (skipping) {
        assistant::request_skip(p.index); // l'étape pas encore commencée (chargement) s'arrêtera dès son début
    }
    if (running_stage && (flow.cancelled || skipping)) {
        if (flow.cancelled && p.running) {
            // À chaque tour (ECR-1) : annulé avant le départ de M1989 R (Marlin occupé) ou pendant un chargement, la
            // série l'apprend dès qu'elle tourne, la mesure dès qu'elle commence
            assistant::request_cancel();
        }
        abort_prusa_wizard(); // à chaque tour : une phase sans « Abandonner » passe vite
    }

    // Redessiné seulement si ce qui est affiché change
    const auto wizard = running_stage ? prusa_wizard() : std::nullopt;
    const bool cooling = flow.stage == Stage::cooling; // la température n'est affichée que là
    // Pendant la série : ce que publie la mesure (phrase, valeur) et son avancement
    const live::Line now = running_stage ? live::now() : live::Line {};
    const live::Line value = running_stage ? live::value() : live::Line {};
    const uint32_t percent = running_stage && p.running ? step_percent(p.step).value_or(0xff) : 0u;
    uint32_t key = 2166136261u;
    for (const uint32_t v : { uint32_t(flow.stage), uint32_t(flow.cancelled), uint32_t(flow.check_failed), uint32_t(cooling ? uint16_t(nozzle()) : 0), uint32_t(cooling && cold()),
             uint32_t(cooling && marlin_vars().is_processing.get()), uint32_t(p.running), uint32_t(p.index), uint32_t(p.runs), flow.mask, flow.started, uint32_t(skipping),
             uint32_t(reinterpret_cast<uintptr_t>(now.msgid)), uint32_t(now.a), uint32_t(now.b), uint32_t(now.c), uint32_t(reinterpret_cast<uintptr_t>(value.msgid)),
             uint32_t(value.a), uint32_t(value.b), uint32_t(value.c), percent,
             wizard ? uint32_t(wizard->phase) | uint32_t(wizard->data[0]) << 8 | uint32_t(wizard->data[2]) << 16 | uint32_t(wizard->data[3]) << 24 : 0u }) {
        key = (key ^ v) * 16777619u;
    }
    if (key != shown_key) {
        shown_key = key;
        show();
    }
}

void ScreenCalibAssistant::show() {
    const assistant::Progress p = assistant::progress();
    const calib_registry::Context ctx = assistant::context();
    const auto records = assistant::records();

    // Une ligne au-dessus des tuiles pour la proposition, la série et le bilan ; les consignes, sur toute la hauteur
    const bool wide = !(flow.stage == Stage::idle || flow.stage == Stage::propose || flow.stage == Stage::accel_run || flow.stage == Stage::cell_run
        || flow.stage == Stage::summary);
    if (wide != info_wide) {
        info_wide = wide;
        info.SetRect(wide ? info_wide_rect : info_rect);
        info.SetAlignment(wide ? Align_t::LeftTop() : Align_t::LeftCenter());
        for (CalibTile &tile : tiles) {
            tile.set_visible(!wide);
        }
        Invalidate(); // tout l'écran se repeint d'une passe (le fond entre les éléments)
    }

    // Tuiles
    for (size_t i = 0; i < entries.size(); ++i) {
        const Entry &e = entries[i];
        const TileView v = tile_view(e, ctx, records[size_t(e.test)], p);
        tiles[i].show(_(name_of(e.test)), v.dot, v.look, v.active);
    }

    // Texte
    string_view_utf8 text;
    switch (flow.stage) {
    case Stage::idle:
    case Stage::propose: {
        // Une ligne (retour du 24/11) : les deux séries et leur durée ; les K déjà mesurés sont au détail du PA
        [[maybe_unused]] const uint32_t cold = flow.mask & cold_tests(), heat = flow.mask & heat_tests(); // INDX : pas de « heat »
        text = !flow.mask ? _("Nothing to calibrate. Touch a test to add it.")
            : flow.boot   ? _("New in this firmware: %d tests, ~%d min").formatted(info_params, std::popcount(flow.mask), int(calib_registry::minutes(flow.mask, ctx, redo_tests())))
#if HAS_LOADCELL() && HAS_MMU2()
                          : _("Accel. %d tests ~%d min \xC2\xB7 Heating %d tests ~%d min")
                                .formatted(info_params, std::popcount(cold), int(calib_registry::minutes(cold, ctx)), std::popcount(heat), int(calib_registry::minutes(heat, ctx, redo_tests())));
#else
                          : _("Accel. %d tests ~%d min").formatted(info_params, std::popcount(cold), int(calib_registry::minutes(cold, ctx))); // INDX : pas de série « Chauffe »
#endif
        break;
    }
    case Stage::ask_accel:
        text = _("Do you have the accelerometer (input shaper kit)? It is clipped on the nozzle only during its tests.");
        break;
    case Stage::cooling:
        text = !cold() ? (attachable ? _("Nozzle %d\xC2\xB0""C: cooling below 40\xC2\xB0""C before attaching the accelerometer (fan on).")
                                     : _("Nozzle %d\xC2\xB0""C: cooling below 40\xC2\xB0""C before the accelerometer tests (fan on)."))
                             .formatted(info_params, int(nozzle()))
            : marlin_vars().is_processing.get() ? _("Nozzle %d\xC2\xB0""C. Homing, please wait.").formatted(info_params, int(nozzle()))
                                                : _("Nozzle %d\xC2\xB0""C, below 40\xC2\xB0""C. Continue.").formatted(info_params, int(nozzle()));
        break;
    case Stage::sock_off:
        text = _("Remove the silicone sock from the nozzle.");
        break;
    case Stage::attach:
        text = !attachable       ? _("The accelerometer in the printhead does not answer. Continue to try again, or Skip its tests.")
            : flow.check_failed ? _("The accelerometer does not answer. Check it is attached and plugged in, then Continue.")
                                 : _("Firmly attach the accelerometer to the extruder and plug it into the board, then Continue.");
        break;
    case Stage::checking:
        text = _("Checking the accelerometer...");
        break;
    case Stage::rehome:
        text = attachable ? _("Home position lost. Remove the accelerometer, then Continue: the head homes again.")
                          : _("Home position lost. Continue: the head homes again.");
        break;
    case Stage::accel_run:
    case Stage::cell_run:
        if (flow.cancelled) {
            text = _("Stopping...");
        } else if (!p.running) {
            text = _("Starting...");
        } else {
            static char running[220]; // la fenêtre de texte le lit jusqu'au prochain show()
            Writer w(running);
            build_running(p, w);
            text = string_view_utf8::MakeRAM(running);
        }
        break;
    case Stage::belts_adjust: {
        const belts::Measurement &m = belts::last();
        const belts::Targets t = belts::targets();
        char a[8], b[8];
        snprintf(a, sizeof(a), "%.1f", double(m.peaks[0].frequency));
        snprintf(b, sizeof(b), "%.1f", double(m.peaks[1].frequency));
        text = (attachable ? _("Belts out of range: A %s Hz, B %s Hz (target %d-%d Hz). Adjust them live, accelerometer still on, then Continue.")
                           : _("Belts out of range: A %s Hz, B %s Hz (target %d-%d Hz). Adjust them live, then Continue."))
                   .formatted(info_params, a, b, int(t.result_min), int(t.result_max));
        break;
    }
    case Stage::detach:
        // La buse est encore froide : le bon moment ; rien ne chauffe avant « C'est fait »
        text = has_sock() ? _("Remove the accelerometer from the nozzle, then put the silicone sock back on it. The nozzle heats only after that.")
                          : _("Remove the accelerometer from the nozzle.");
        break;
    case Stage::bed:
        text = _("Is the bed empty and clean? Each test prints in its own place on the bed.");
        break;
    case Stage::bed_full:
        text = _("The bed is full of test prints. Remove them and put the clean sheet back, then Continue.");
        break;
    case Stage::summary: {
        static char brief[112]; // la ligne de texte le lit jusqu'au prochain show() ; le bilan complet : « Bilan »
        Writer w(brief);
        build_summary(w, true);
        text = string_view_utf8::MakeRAM(brief);
        break;
    }
    }
    info.SetText(text);
    info.Invalidate();

    // Boutons (nullptr : caché)
    struct Buttons {
        const char *a = nullptr;
        const char *b = nullptr;
        const char *c = nullptr;
        bool a_enabled = true;
        bool b_enabled = true;
    };
    Buttons s;
    switch (flow.stage) {
    case Stage::idle:
    case Stage::propose:
        // Deux séries lancées chacune par son bouton, avec ses tests cochés
#if HAS_LOADCELL() && HAS_MMU2()
        s = { N_("Accelerometer"), N_("Heating"), N_("Close"), (flow.mask & cold_tests()) != 0, (flow.mask & heat_tests()) != 0 };
#else
        s = { N_("Accelerometer"), nullptr, N_("Close"), (flow.mask & cold_tests()) != 0 }; // INDX : pas de série « Chauffe »
#endif
        break;
    case Stage::ask_accel:
        s = { N_("Yes"), N_("No"), N_("Cancel") };
        break;
    case Stage::cooling:
        s = { N_("Continue"), nullptr, N_("Cancel"), cold() && !marlin_vars().is_processing.get() };
        break;
    case Stage::sock_off:
        s = { N_("It's done"), nullptr, N_("Cancel") };
        break;
    case Stage::attach:
        s = { N_("Continue"), N_("Skip"), N_("Cancel") };
        break;
    case Stage::checking:
        s = { N_("Continue"), nullptr, N_("Cancel"), false };
        break;
    case Stage::rehome:
    case Stage::bed:
    case Stage::bed_full:
        s = { N_("Continue"), nullptr, N_("Cancel") };
        break;
    case Stage::accel_run:
    case Stage::cell_run:
        s = { nullptr, N_("Skip this test"), N_("Cancel") };
        break;
    case Stage::belts_adjust:
        s = { N_("Adjust live"), N_("Continue"), N_("Cancel") };
        break;
    case Stage::detach:
        s = { N_("It's done") };
        break;
    case Stage::summary:
        s = { N_("Close"), N_("Summary") };
        break;
    }
    // Libellés posés au changement d'étape seulement (set() redessine le bouton)
    if (const uint8_t stage_key = uint8_t(flow.stage); stage_key != buttons_stage) {
        buttons_stage = stage_key;
        const std::pair<Button *, const char *> buttons[] = { { &button_a, s.a }, { &button_b, s.b }, { &button_c, s.c } };
        for (const auto &[button, label] : buttons) {
            if (label) {
                button->set(button == &button_a ? Button::Look::primary : (button == &button_b ? Button::Look::secondary : Button::Look::outline), _(label));
                button->Show();
            } else {
                button->Hide();
            }
        }
    }
    button_a.enable(s.a && s.a_enabled);
    const bool run = flow.stage == Stage::accel_run || flow.stage == Stage::cell_run;
    button_b.enable(run ? (!flow.cancelled && p.running && assistant::can_skip() && flow.skip_index != p.index) : (s.b && s.b_enabled));
    button_c.enable(!(flow.cancelled && run));
    // La molette part du bouton utile de l'étape ; « Continuer » le reprend dès qu'il devient possible
    if (const uint8_t want = uint8_t(buttons_stage ^ (s.a && s.a_enabled ? 0x40 : 0)); focus_stage != want) {
        focus_stage = want;
        (s.a && s.a_enabled ? button_a : (!run && s.b && s.b_enabled && flow.stage == Stage::propose ? button_b : (s.c ? button_c : button_b))).SetFocus();
    }
}

void ScreenCalibAssistant::clicked(uint8_t tag) {
    if (tag < entries.size()) {
        const Entry &e = entries[tag];
        if (!e.automatic) {
#if HAS_LOADCELL() && HAS_MMU2() // INDX : aucun test à la main (tous de la série « Chauffe »)
            // Tests à la main : l'écran de la mesure, lancée par l'utilisateur (accélération du filament N-04, avec la
            // première bobine ; vitesse de la poulie U-03), sinon la pièce imprimée à mesurer (équerrage)
            const uint8_t spools = assistant::context().spools;
            if (const ScreenFactory::Creator screen = step_screen({ e.test, spools ? uint8_t(std::countr_zero(unsigned(spools))) : uint8_t(0) }); screen.func) {
                Screens::Access()->Open(screen);
            } else {
                open_screen<ScreenSkew>();
            }
#endif
        } else if (flow.stage == Stage::propose) {
            flow.mask ^= test_bit(e.test); // cocher ou décocher (passer avant le lancement)
        } else if (flow.stage == Stage::summary && show_details(e.test)) {
            start_session(test_bit(e.test), test_bit(e.test), false); // refaire ce test seul
        } else if (const assistant::Progress p = assistant::progress(); p.running && p.step.test == e.test) {
            curves_opened = step_screen(p.step); // l'écran de la mesure en cours, avec ses courbes
            if (curves_opened.func) {
                Screens::Access()->Open(curves_opened);
            }
        }
        return;
    }
    switch (flow.stage) {
    case Stage::idle:
    case Stage::propose:
        if (tag == tag_a || tag == tag_b) {
            const uint32_t series = tag == tag_a ? cold_tests() : heat_tests();
            start_session(flow.offered & series, flow.mask & series, true);
        } else if (tag == tag_c) {
            flow.stage = Stage::idle;
            Screens::Access()->close_screen();
        }
        break;
    case Stage::ask_accel:
        if (tag == tag_a) {
            assistant::set_accelerometer(Accelerometer::yes);
            enter_cooling();
        } else if (tag == tag_b) {
            assistant::set_accelerometer(Accelerometer::no);
            flow.mask &= ~accel_tests();
            flow.session &= ~accel_tests();
            next_after_accel();
        } else {
            cancel();
        }
        break;
    case Stage::cooling:
        if (tag == tag_a && cold() && !marlin_vars().is_processing.get()) {
            marlin_client::gcode("M107"); // ventilateur de M1989 H
            if (!attachable) {
                start_check();
            } else {
                flow.stage = has_sock() ? Stage::sock_off : Stage::attach;
            }
        } else if (tag == tag_c) {
            cancel();
        }
        break;
    case Stage::sock_off:
        if (tag == tag_a) {
            flow.stage = Stage::attach;
        } else {
            cancel();
        }
        break;
    case Stage::attach:
        if (tag == tag_a) {
            start_check();
        } else if (tag == tag_b) {
            flow.mask &= ~accel_tests(); // passés : ils restent à faire pour la prochaine fois
            next_after_accel();
        } else {
            cancel();
        }
        break;
    case Stage::rehome:
        if (tag == tag_a) {
            enter_cooling();
        } else {
            cancel();
        }
        break;
    case Stage::belts_adjust:
        if (tag == tag_a) {
            open_belts_live(); // l'écran des courroies, en direct ; « Continuer » ici quand c'est réglé
        } else if (tag == tag_b) {
            assistant::recapture(Test::belts); // le bilan et la prochaine fois : les fréquences après réglage
            after_accel_part();
        } else {
            cancel();
        }
        break;
    case Stage::detach:
        next_after_accel();
        break;
    case Stage::bed:
        if (tag == tag_a) {
            run_part(false);
        } else {
            cancel();
        }
        break;
    case Stage::bed_full:
        if (tag == tag_a) {
            marlin_client::gcode("M1989 C"); // plateau nettoyé : l'étape en attente repart (update() revient à la série)
        } else {
            cancel();
        }
        break;
    case Stage::accel_run:
    case Stage::cell_run:
        if (tag == tag_b && !flow.cancelled && assistant::can_skip()) {
            const assistant::Progress p = assistant::progress();
            flow.skip_index = p.index;
            flow.skip_runs = p.runs;
            assistant::request_skip(p.index);
        } else if (tag == tag_c) {
            cancel();
        }
        break;
    case Stage::checking:
        if (tag == tag_c) {
            cancel();
        }
        break;
    case Stage::summary:
        if (tag == tag_b) {
            // Le bilan complet : gains, raisons des échecs, alertes (la ligne de l'écran n'en montre que le compte)
            const std::span<char> summary = message_text(); // + la raison d'un échec du tube PTFE et de la dilatation de la buse, l'alerte des ventilateurs
            Writer w(summary);
            build_summary(w);
            MsgBoxInfo(string_view_utf8::MakeRAM(summary.data()), Responses_Ok);
        } else {
            flow.stage = Stage::idle;
            Screens::Access()->close_screen();
        }
        break;
    }
}

void ScreenCalibAssistant::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;
    case GUI_event_t::CHILD_CLICK:
        clicked(static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param)));
        update();
        return;
    case GUI_event_t::HELD_RELEASED:
        if (!GetFirstDialog()) {
            leave();
            return;
        }
        break;
    default:
        break;
    }
    Screen::windowEvent(sender, event, param);
}

// ---- Réglages › Fonctions OneDrop3D ----

MI_CALIB_ASSISTANT::MI_CALIB_ASSISTANT()
    : IWindowMenuItem(_("Calibration assistant\nAll tests in a row, by themselves"), &img::wizard_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_CALIB_ASSISTANT::click(IWindowMenu &) {
    if (flow.stage == Stage::propose || flow.stage == Stage::summary) {
        flow.stage = Stage::idle; // l'écran repart de la proposition complète
    }
    open_screen<ScreenCalibAssistant>();
}

namespace {
    constexpr const char *yes_no_items[] = { N_("No"), N_("Yes") };
}

MI_ACCELEROMETER::MI_ACCELEROMETER()
    : MenuItemSwitch(_("Accelerometer available\nIts tests are offered"), yes_no_items, assistant::accelerometer() == Accelerometer::yes ? 1 : 0) {
    SetIconId(&img::wizard_16x16);
}

bool MI_ACCELEROMETER::on_item_selected(const OnItemSelectedArgs &args) {
    assistant::set_accelerometer(args.new_index == 1 ? Accelerometer::yes : Accelerometer::no);
    return true;
}

} // namespace od3d
