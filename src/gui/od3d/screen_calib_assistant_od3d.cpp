/// @file
#include "screen_calib_assistant_od3d.hpp"

#include "menu_od3d.hpp" // section_tab
#include "screen_belts_od3d.hpp"
#include "screen_damping_od3d.hpp"
#include "screen_fan_response_od3d.hpp"
#include "screen_e_jump_od3d.hpp"
#include "screen_mmu_speed_od3d.hpp"
#include "screen_mpc_od3d.hpp"
#include "screen_nozzle_expansion_od3d.hpp"
#include "screen_pa_od3d.hpp"
#include "screen_ptfe_od3d.hpp"
#include "screen_skew_od3d.hpp"
#include "screen_speed_map_od3d.hpp"
#include "screen_spool_calib_od3d.hpp"

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
#include <feature/od3d/e_jump_od3d.hpp>
#include <feature/od3d/fan_response_od3d.hpp>
#include <feature/od3d/live_od3d.hpp>
#include <feature/od3d/mmu_speed_od3d.hpp>
#include <feature/od3d/mpc_control_od3d.hpp>
#include <feature/od3d/nozzle_expansion_od3d.hpp>
#include <feature/od3d/pa_od3d.hpp>
#include <feature/od3d/sleep_od3d.hpp>
#include <feature/od3d/speed_map_od3d.hpp>
#include <feature/od3d/spool_calib_od3d.hpp>
#include <filament.hpp>
#include <Marlin/src/feature/input_shaper/input_shaper_config.hpp>
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

    // Géométrie (contenu de y 46 à 272) : texte sur 3 lignes, 3 colonnes de 6 tuiles alignées sur les 3 boutons, de
    // quoi loger tous les tests possibles du registre (calib_registry::max_tests). Tuile : corps 148 × 19, marges de
    // 2 px jointives (aucun pixel entre deux tuiles n'est à personne), nom à gauche et pastille d'état à droite, tous
    // deux hors des coins arrondis : 13 caractères au plus pour le nom (« Équerrage XY », « Accél. fil. »)
    constexpr size_t tile_columns = 3;
    constexpr size_t max_tiles = 18;
    constexpr uint8_t tile_radius = style::radius_small;
    constexpr int16_t tile_dot = 8; ///< diamètre de la pastille d'état
    constexpr Rect16 info_rect = Rect16(style::margin, style::top, 464, 48);
    constexpr Rect16 tile_body(size_t i) {
        return Rect16(10 + 156 * int(i % tile_columns), 96 + 23 * int(i / tile_columns), 148, 19);
    }
    constexpr Rect16 button_body(int i) {
        return Rect16(10 + 156 * i, 236, 148, 36); // deux lignes de police small
    }
    static_assert(max_tiles >= calib_registry::max_tests, "une tuile pour chaque test possible du registre");
    static_assert(entries.size() <= max_tiles);
    static_assert(style::with_ring(tile_body(0)).Top() > info_rect.Bottom());
    static_assert(style::with_ring(tile_body(tile_columns)).Top() == style::with_ring(tile_body(0)).Bottom() + 1, "rangées jointives");
    static_assert(style::with_ring(tile_body(1)).Left() > style::with_ring(tile_body(0)).Right());
    static_assert(tile_body(0).Height() >= height(Font::small) + 2 && 2 * tile_radius <= tile_body(0).Height());
    static_assert(style::inside(style::content, style::with_ring(tile_body(max_tiles - 1))));
    static_assert(style::with_ring(button_body(0)).Top() > style::with_ring(tile_body(max_tiles - 1)).Bottom());
    static_assert(style::inside(style::content, style::with_ring(button_body(2))));

    constexpr uint8_t tag_a = 100;
    constexpr uint8_t tag_b = 101;
    constexpr uint8_t tag_c = 102;

    enum class Stage : uint8_t {
        idle,
        propose, ///< calibrations à faire, durée ; au démarrage ou depuis le menu
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
        summary,
    };

    /// L'assistant, gardé hors de l'écran : il survit à sa fermeture (sous-écran, onglet, pastille)
    struct Flow {
        Stage stage = Stage::idle;
        bool boot = false; ///< proposé au démarrage : « Plus tard », « Ne plus demander »
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
        return calib_registry::needs_mask(calib_registry::accelerometer) & calib_registry::automatic_mask();
    }

    uint32_t bed_tests() {
        return calib_registry::needs_mask(calib_registry::empty_bed) & calib_registry::automatic_mask();
    }

    uint32_t tests_to_offer(bool only_outdated) {
        const auto records = assistant::records();
        return calib_registry::proposed(assistant::context(), records, only_outdated);
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
            return N_("Part fan");
        case Test::nozzle_expansion:
            return N_("Expansion");
        case Test::e_jump:
            return N_("Filament accel.");
        case Test::mmu_speed:
            return N_("MMU speed");
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
            case PhasesInputShaperCalibration::wait_for_extruder_temperature:
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
            case PhasesPhaseStepping::wait_for_extruder_temperature:
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
        case Test::nozzle_expansion:
            return ScreenFactory::Screen<ScreenNozzleExpansion>;
        case Test::mpc:
            return mpc_calibration_screen();
        case Test::pa:
            return ScreenFactory::ScreenWithArg<ScreenPa>(s.slot);
        case Test::max_flow:
        case Test::min_temp:
            return ScreenFactory::ScreenWithArg<ScreenSpoolCalib>(s.slot);
        case Test::fan_response:
            return ScreenFactory::Screen<ScreenFanResponse>;
        case Test::e_jump:
            return ScreenFactory::ScreenWithArg<ScreenEJump>(s.slot);
        case Test::mmu_speed:
            return ScreenFactory::Screen<ScreenMmuSpeed>;
        default:
            return {};
        }
    }

    void enter_cooling() {
        marlin_client::gcode("M1989 H"); // chauffe coupée, ventilateur, origine (sans l'accéléromètre), buse au centre
        flow.stage = Stage::cooling;
    }

    /// Lance la série des tests restants à l'accéléromètre (accel) ou des autres
    void run_part(bool accel) {
        uint32_t part = flow.mask & (accel ? accel_tests() : ~accel_tests()) & ~flow.started;
        if (part & test_bit(Test::belts)) {
            part = test_bit(Test::belts); // seules d'abord : hors de la plage, réglées avant les tests qui en dépendent
        }
        flow.started |= part;
        flow.runs_at_start = assistant::progress().runs;
        char gcode[24];
        snprintf(gcode, sizeof(gcode), "M1989 R%lu", static_cast<unsigned long>(part));
        marlin_client::gcode(gcode);
        flow.stage = accel ? Stage::accel_run : Stage::cell_run;
    }

    /// Les courroies viennent d'être mesurées hors de la plage de Prusa (conseil : tourner les vis), pas encore proposé
    bool belts_out_of_range() {
        const belts::Measurement &m = belts::last();
        return !flow.belts_offered && (flow.started & test_bit(Test::belts)) && assistant::result(Test::belts) == Result::ok && m.peaks[0].found
            && m.peaks[1].found && belts::advise(m.peaks[0].frequency, m.peaks[1].frequency, belts::targets()).verdict == belts::Verdict::adjust;
    }

    /// Fin d'une série à l'accéléromètre : réglage en direct des courroies proposé, puis les autres tests à
    /// l'accéléromètre, puis « Retirez l'accéléromètre »
    void after_accel_part() {
        if (!flow.cancelled && belts_out_of_range()) {
            flow.belts_offered = true;
            flow.stage = Stage::belts_adjust;
        } else if (!flow.cancelled && (flow.mask & accel_tests() & ~flow.started)) {
            run_part(true);
        } else {
            flow.stage = Stage::detach; // aussi après « Annuler » : l'accéléromètre est encore sur la buse
        }
    }

    /// Après l'accéléromètre (fait, passé ou absent) : plateau, puis le reste ; sinon le bilan
    void next_after_accel() {
        const uint32_t rest = flow.mask & ~accel_tests();
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
        flow.session = session | mask;
        flow.mask = mask;
        flow.skip_index = 0xff;
        flow.belts_offered = false;
        flow.started = 0;
        flow.cancelled = flow.check_failed = false;
        if (!(mask & accel_tests())) {
            next_after_accel();
        } else if (ask || assistant::accelerometer() != Accelerometer::yes) {
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
            flow.stage = Stage::detach;
            break;
        default:
            flow.stage = Stage::summary;
            break;
        }
    }

    /// Ce que montre une tuile
    /// Pastilles d'état (le mot est au détail, au toucher)
    constexpr Color dot_ok = Color::from_raw(0x2E9E5B); ///< vert de l'atelier
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
        if (flow.stage == Stage::propose) { // coché : orange ; décoché : gris ; le toucher coche ou décoche
            return { (flow.mask & b) ? style::accent : style::text_muted, Button::Look::card, true };
        }
        if (!(flow.session & b)) { // à faire : orange ; à jour : gris
            return { calib_registry::outdated(e, record) ? style::accent : style::text_muted };
        }
        const bool summary = flow.stage == Stage::summary;
        if (flow.started & b) {
            switch (assistant::result(e.test)) {
            case Result::ok:
                return { dot_ok, Button::Look::card, summary }; // bilan : ses chiffres
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

        /// Texte traduit tel quel
        void text(const char *msgid) {
            char t[96];
            _(msgid).copyToRAM(t, sizeof(t));
            add("%s", t);
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

    /// Détail d'un test du bilan : ses chiffres, d'avant et d'après
    void build_details(Test test, Writer &w) {
        const calib_registry::Numbers &a = assistant::numbers_after(test);
        const calib_registry::Numbers &b = assistant::numbers_before(test);
        if (!(flow.started & test_bit(test)) || assistant::result(test) == Result::skipped) {
            w.text(N_("Skipped: still to do next time."));
            return;
        }
        if (assistant::result(test) != Result::ok || !any(a, 0, a.size())) {
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
            w.label(N_("Belts:"));
            pair(a, "AB", 10, " Hz");
            if (a[0] != none && a[1] != none) {
                w.add(", A-B %.1f Hz", std::abs(a[0] - a[1]) / 10.0);
            }
            const belts::Targets t = belts::targets();
            w.label(N_("Target:"));
            w.add("%.0f-%.0f Hz", double(t.result_min), double(t.result_max));
            if (any(b, 0, 2)) {
                w.label(N_("Before:"));
                pair(b, "AB", 10, " Hz");
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
            w.add("%d (X %d, Y %d)", int(a[0]), int(a[1]), int(a[2]));
            if (b[0] != none) {
                w.label(N_("Before:"));
                w.add("%d", int(b[0]));
            }
            break;
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
        case Test::fan_response: {
            const auto lead = [&](int16_t ds) {
                if (ds == 0) {
                    w.text(N_("Off"));
                } else {
                    w.scaled(ds, 10, " s");
                }
            };
            w.label(N_("Part fan rise time:"));
            w.scaled(a[0], 1000, " s");
            w.add(" (%d RPM)", int(a[2]));
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
            break;
        }
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
                if (a[slot] == none && b[slot] == none) {
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
        case Test::skew:
        case Test::count:
            break;
        }
    }

    /// Bilan : réussites, échecs, puis les gains principaux en une ligne
    void build_summary(Writer &w) {
        int ok = 0, failed = 0, skipped = 0;
        for (const Entry &e : entries) {
            if (!(flow.session & test_bit(e.test)) || !e.automatic) {
                continue;
            }
            const Result r = (flow.started & test_bit(e.test)) ? assistant::result(e.test) : Result::skipped; // décoché
            ok += r == Result::ok;
            failed += r == Result::failed;
            skipped += r == Result::skipped || r == Result::none;
        }
        char format[96];
        _("Finished: %d OK, %d failed, %d skipped. Touch a test for its numbers.").copyToRAM(format, sizeof(format));
        w.add(format, ok, failed, skipped);
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
        case Test::nozzle_expansion:
            return nozzle_expansion::is_running() ? std::optional<uint8_t>(nozzle_expansion::progress_percent()) : std::nullopt;
        case Test::fan_response:
            return fan_response::is_running() ? std::optional<uint8_t>(fan_response::progress_percent()) : std::nullopt;
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
        char format[48];
        _("Step %d/%d:").copyToRAM(format, sizeof(format));
        w.add(format, p.index + 1, int(p.count));
        w.add(" ");
        w.text(name_of(p.step.test));
        if (calib_registry::entry(p.step.test).needs & (calib_registry::filament | calib_registry::one_spool)) {
            _("slot %d").copyToRAM(format, sizeof(format));
            w.add(", ");
            w.add(format, p.step.slot + 1);
        }
        if (const auto percent = step_percent(p.step)) {
            w.add(" - %d%%", int(*percent));
        }
        w.add("\n");
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
            w.add("\n");
            add_line(w, v);
        }
    }

    /// Le détail d'un test du bilan ; vrai pour le refaire
    bool show_details(Test test) {
        static char text[360]; // la boîte le lit tant qu'elle est ouverte
        Writer w(text);
        build_details(test, w);
        const bool can_retry = assistant::result(test) != Result::ok;
        const PhaseResponses responses = can_retry ? PhaseResponses { Response::Retry, Response::Ok, Response::_none, Response::_none } : Responses_Ok;
        return MsgBoxInfo(string_view_utf8::MakeRAM(text), responses, can_retry ? 1 : 0) == Response::Retry;
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
    const auto records = assistant::records();
    const calib_registry::Context ctx = assistant::context();
    if (!calib_registry::should_propose(ctx, records, assistant::dismissed())) {
        return false;
    }
    const uint32_t mask = calib_registry::proposed(ctx, records, true);
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
    , button_a(this, button_body(0), tag_a, Button::Look::primary, string_view_utf8::MakeNULLSTR(), nullptr, style::screen, 10)
    , button_b(this, button_body(1), tag_b, Button::Look::secondary, string_view_utf8::MakeNULLSTR(), nullptr, style::screen, 10)
    , button_c(this, button_body(2), tag_c, Button::Look::outline, string_view_utf8::MakeNULLSTR(), nullptr, style::screen, 10)
    , tabs(this, section_tab()) {
    ClrMenuTimeoutClose(); // l'assistant attend l'utilisateur ou une longue série
    if (flow.stage == Stage::idle) {
        const uint32_t mask = tests_to_offer(false); // depuis le menu : toutes, sauf le PA jamais mesuré
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
        const int count = std::popcount(flow.mask);
        const int minutes = calib_registry::minutes(flow.mask, ctx);
        text = !count ? _("Nothing to calibrate. Touch a test to add it.")
            : flow.boot ? _("%d calibrations are new or changed in this firmware (about %d min). Touch a test to add or skip it; the tests then run by themselves.").formatted(info_params, count, minutes)
                        : _("Full calibration: %d tests, about %d min. Touch a test to add or skip it; the tests then run by themselves.").formatted(info_params, count, minutes);
        break;
    }
    case Stage::ask_accel:
        text = _("Do you have the accelerometer (input shaper kit)? It is clipped on the nozzle only during its tests.");
        break;
    case Stage::cooling:
        text = !cold() ? _("Nozzle %d\xC2\xB0""C: cooling below 40\xC2\xB0""C before attaching the accelerometer (fan on).").formatted(info_params, int(nozzle()))
            : marlin_vars().is_processing.get() ? _("Nozzle %d\xC2\xB0""C. Homing, please wait.").formatted(info_params, int(nozzle()))
                                                : _("Nozzle %d\xC2\xB0""C, below 40\xC2\xB0""C. Continue.").formatted(info_params, int(nozzle()));
        break;
    case Stage::sock_off:
        text = _("Remove the silicone sock from the nozzle.");
        break;
    case Stage::attach:
        text = flow.check_failed ? _("The accelerometer does not answer. Check it is attached and plugged in, then Continue.")
                                 : _("Firmly attach the accelerometer to the extruder and plug it into the board, then Continue.");
        break;
    case Stage::checking:
        text = _("Checking the accelerometer...");
        break;
    case Stage::rehome:
        text = _("Home position lost. Remove the accelerometer, then Continue: the head homes again.");
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
        text = _("Belts out of range: A %s Hz, B %s Hz (target %d-%d Hz). Adjust them live, accelerometer still on, then Continue.")
                   .formatted(info_params, a, b, int(t.result_min), int(t.result_max));
        break;
    }
    case Stage::detach:
        // La buse est encore froide : le bon moment ; rien ne chauffe avant « C'est fait »
        text = has_sock() ? _("Remove the accelerometer from the nozzle, then put the silicone sock back on it. The nozzle heats only after that.")
                          : _("Remove the accelerometer from the nozzle.");
        break;
    case Stage::bed:
        text = _("Is the bed empty and clean? Test lines will be printed at the front of the bed.");
        break;
    case Stage::summary: {
        static char summary[360]; // + la raison d'un échec du tube PTFE et de la dilatation de la buse
        Writer w(summary);
        build_summary(w);
        text = string_view_utf8::MakeRAM(summary);
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
    };
    Buttons s;
    switch (flow.stage) {
    case Stage::idle:
    case Stage::propose:
        s = flow.boot ? Buttons { N_("Calibrate now"), N_("Later"), N_("Don't ask again"), flow.mask != 0 }
                      : Buttons { N_("Start"), N_("Close"), nullptr, flow.mask != 0 };
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
        s = { N_("Close") };
        break;
    }
    // Libellés posés au changement d'étape seulement (set() redessine le bouton)
    if (const uint8_t stage_key = uint8_t(uint8_t(flow.stage) | (flow.boot ? 0x80 : 0)); stage_key != buttons_stage) {
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
    button_b.enable(!run || (!flow.cancelled && p.running && assistant::can_skip() && flow.skip_index != p.index));
    button_c.enable(!(flow.cancelled && run));
    // La molette part du bouton utile de l'étape ; « Continuer » le reprend dès qu'il devient possible
    if (const uint8_t want = uint8_t(buttons_stage ^ (s.a && s.a_enabled ? 0x40 : 0)); focus_stage != want) {
        focus_stage = want;
        (s.a && s.a_enabled ? button_a : (s.c ? button_c : button_b)).SetFocus();
    }
}

void ScreenCalibAssistant::clicked(uint8_t tag) {
    if (tag < entries.size()) {
        const Entry &e = entries[tag];
        if (!e.automatic) {
            open_screen<ScreenSkew>(); // seul test manuel : la pièce imprimée à mesurer
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
        if (tag == tag_a) {
            start_session(flow.offered, flow.mask, true);
        } else if (tag == tag_b || tag == tag_c) {
            if (tag == tag_c) {
                assistant::dismiss(); // jusqu'à la prochaine version qui change une calibration
            }
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
            flow.stage = has_sock() ? Stage::sock_off : Stage::attach;
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
            flow.checks_at_request = assistant::check_count();
            marlin_client::gcode("M1989 A");
            flow.stage = Stage::checking;
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
        flow.stage = Stage::idle;
        Screens::Access()->close_screen();
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
