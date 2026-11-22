/// @file
#include "screen_calib_assistant_od3d.hpp"

#include "menu_od3d.hpp" // section_tab
#include "screen_skew_od3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <utility>
#include <common/marlin_vars.hpp>
#include <feature/od3d/calib_assistant_od3d.hpp>
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

    // Géométrie (contenu de y 46 à 272) : texte sur 3 lignes, 2 colonnes de 6 tuiles (place pour 2 tests de plus),
    // 3 boutons
    constexpr size_t max_tiles = 12;
    constexpr Rect16 info_rect = Rect16(style::margin, style::top, 464, 48);
    constexpr Rect16 tile_body(size_t i) {
        return Rect16(10 + 234 * int(i % 2), 97 + 23 * int(i / 2), 226, 20);
    }
    constexpr Rect16 button_body(int i) {
        return Rect16(10 + 156 * i, 236, 148, 36); // deux lignes de police small
    }
    static_assert(entries.size() <= max_tiles, "2 colonnes de 6 tuiles : agrandir la grille");
    static_assert(style::with_ring(tile_body(0)).Top() > info_rect.Bottom());
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
        attach, ///< « Fixez l'accéléromètre » ; Continuer le vérifie (M1989 A)
        checking,
        rehome, ///< origine perdue en attendant : retirer l'accéléromètre, la refaire
        accel_run, ///< série des tests à l'accéléromètre
        detach, ///< « Retirez l'accéléromètre »
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
        uint32_t session = 0; ///< tests de cette session (affichés au bilan)
        uint32_t mask = 0; ///< tests encore à lancer
        uint32_t started = 0; ///< tests envoyés à Marlin (leur résultat compte)
        uint16_t runs_at_start = 0;
        uint16_t checks_at_request = 0;
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
        return calib_registry::select(assistant::context(), records, only_outdated) & calib_registry::automatic_mask();
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

    void enter_cooling() {
        marlin_client::gcode("M1989 H"); // chauffe coupée, ventilateur, origine (sans l'accéléromètre), buse au centre
        flow.stage = Stage::cooling;
    }

    /// Lance la série des tests restants à l'accéléromètre (accel) ou des autres
    void run_part(bool accel) {
        const uint32_t part = flow.mask & (accel ? accel_tests() : ~accel_tests());
        flow.started |= part;
        flow.runs_at_start = assistant::progress().runs;
        char gcode[24];
        snprintf(gcode, sizeof(gcode), "M1989 R%lu", static_cast<unsigned long>(part));
        marlin_client::gcode(gcode);
        flow.stage = accel ? Stage::accel_run : Stage::cell_run;
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

    void start_session(uint32_t mask, bool ask) {
        if (marlin_client::is_printing()) {
            MsgBoxWarning(_("Not while printing."), Responses_Ok);
            return;
        }
        flow.session = flow.mask = mask;
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
        flow.cancelled = true;
        switch (flow.stage) {
        case Stage::accel_run:
        case Stage::cell_run:
            assistant::request_cancel(); // la série finit l'étape en cours sans lancer la suivante
            break;
        case Stage::cooling:
        case Stage::attach:
        case Stage::checking:
        case Stage::rehome:
            marlin_client::gcode("M107");
            flow.stage = Stage::summary;
            break;
        default:
            flow.stage = Stage::summary;
            break;
        }
    }

    /// Ce que montre une tuile
    struct TileView {
        const char *status;
        Color color;
        bool alert = false;
        bool active = false; ///< touchable : refaire un test du bilan, ouvrir l'écran d'un test manuel
    };

    TileView tile_view(const Entry &e, const calib_registry::Context &ctx, const calib_registry::Record &record, const assistant::Progress &p) {
        const uint32_t b = test_bit(e.test);
        const bool open_stage = flow.stage == Stage::propose || flow.stage == Stage::summary;
        if (!calib_registry::applicable(e, ctx)) {
            return { "-", style::text_faint };
        }
        if (!e.automatic) {
            return calib_registry::outdated(e, record) ? TileView { N_("by hand"), style::warning, false, open_stage }
                                                       : TileView { "OK", style::text_strong, false, open_stage };
        }
        if (flow.stage == Stage::propose || !(flow.session & b)) {
            return (flow.stage == Stage::propose ? (flow.mask & b) : calib_registry::outdated(e, record))
                ? TileView { N_("to do"), style::accent }
                : TileView { N_("up to date"), style::text_muted };
        }
        const bool summary = flow.stage == Stage::summary;
        if (flow.started & b) {
            switch (assistant::result(e.test)) {
            case Result::ok:
                return { "OK", style::text_strong };
            case Result::failed:
                return { N_("failed"), style::danger, true, summary };
            case Result::skipped:
                return { N_("skipped"), style::warning, false, summary };
            case Result::none:
                if (p.running && p.step.test == e.test) {
                    return { N_("running"), style::accent };
                }
                break;
            }
        }
        return summary ? TileView { N_("skipped"), style::warning, false, true } : TileView { N_("to do"), style::text_soft };
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
    flow = Flow { .stage = Stage::propose, .boot = true, .mask = calib_registry::select(ctx, records, true) & calib_registry::automatic_mask() };
    return true;
}

bool calibration_assistant_active() {
    return flow.stage != Stage::idle && flow.stage != Stage::propose && flow.stage != Stage::summary;
}

string_view_utf8 calibration_assistant_name() {
    const assistant::Progress p = assistant::progress();
    return p.running ? _(name_of(p.step.test)) : _("Calibration");
}

// ---- Tuile ----

CalibTile::CalibTile(window_t *parent, Rect16 body, uint8_t tag)
    : Button(parent, body, tag, Look::card, string_view_utf8::MakeNULLSTR(), nullptr, style::screen, 8)
    , status(string_view_utf8::MakeNULLSTR())
    , status_color(style::text_muted) {
    set_font(Font::small);
}

void CalibTile::show(const string_view_utf8 &name, const string_view_utf8 &new_status, Color color, bool alert, bool active) {
    const Look new_look = alert ? Look::alert : Look::card;
    if (name.is_same_ref(label) && new_status.is_same_ref(status) && color == status_color && new_look == look && active == IsEnabled()) {
        return;
    }
    label = name;
    status = new_status;
    status_color = color;
    look = new_look;
    set_enabled(active);
    Invalidate();
}

void CalibTile::unconditionalDraw() {
    Colors c = colors();
    if (!IsEnabled()) {
        c.fg = style::text; // un résultat reste lisible même quand la tuile n'est pas touchable
    }
    const Rect16 b = draw_frame(body(), c);
    constexpr uint16_t w = width(Font::small);
    constexpr uint16_t h = height(Font::small);
    const int16_t y = b.Top() + (b.Height() - h) / 2;
    const uint16_t status_w = std::min<uint16_t>(status.isNULLSTR() ? 0 : status.computeNumUtf8Chars() * w, b.Width() - 12);
    const int16_t right = b.Right() - 5;
    draw_text(Rect16(right - status_w, y, status_w, h), status, Font::small, status_color, c.fill, Align_t::RightCenter());
    draw_text(Rect16(b.Left() + 6, y, std::max(0, right - status_w - 4 - (b.Left() + 6)), h), label, Font::small, c.fg, c.fill);
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
        flow = Flow { .stage = Stage::propose, .mask = tests_to_offer(false) }; // depuis le menu : toutes
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

void ScreenCalibAssistant::update() {
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
            flow.stage = Stage::detach; // aussi après « Annuler » : l'accéléromètre est encore sur la buse
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

    // Redessiné seulement si ce qui est affiché change
    const assistant::Progress p = assistant::progress();
    const bool cooling = flow.stage == Stage::cooling; // la température n'est affichée que là
    uint32_t key = 2166136261u;
    for (const uint32_t v : { uint32_t(flow.stage), uint32_t(flow.cancelled), uint32_t(flow.check_failed), uint32_t(cooling ? uint16_t(nozzle()) : 0), uint32_t(cooling && cold()),
             uint32_t(cooling && marlin_vars().is_processing.get()), uint32_t(p.running), uint32_t(p.index), uint32_t(p.runs), flow.mask, flow.started }) {
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
        tiles[i].show(_(name_of(e.test)), _(v.status), v.color, v.alert, v.active);
    }

    // Texte
    string_view_utf8 text;
    char name[24] = "";
    switch (flow.stage) {
    case Stage::idle:
    case Stage::propose: {
        const int count = std::popcount(flow.mask);
        const int minutes = calib_registry::minutes(flow.mask, ctx);
        text = !count ? _("Nothing to calibrate.")
            : flow.boot ? _("%d calibrations are new or changed in this firmware (about %d min). Once started, the tests run by themselves.").formatted(info_params, count, minutes)
                        : _("Full calibration: %d tests, about %d min. Once started, the tests run by themselves.").formatted(info_params, count, minutes);
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
    case Stage::attach:
        text = flow.check_failed ? _("The accelerometer does not answer. Check it is attached and plugged in, then Continue.")
                                 : _("Attach the accelerometer to the nozzle and plug it into the board, then Continue.");
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
            _(name_of(p.step.test)).copyToRAM(name, sizeof(name));
            text = (calib_registry::entry(p.step.test).needs & calib_registry::filament)
                ? _("Step %d/%d: %s, slot %d. You may leave this screen: the pill shows the progress.").formatted(info_params, p.index + 1, int(p.count), name, p.step.slot + 1)
                : _("Step %d/%d: %s. You may leave this screen: the pill shows the progress.").formatted(info_params, p.index + 1, int(p.count), name);
        }
        break;
    case Stage::detach:
        text = _("Remove the accelerometer from the nozzle, then Continue.");
        break;
    case Stage::bed:
        text = _("Is the bed empty and clean? Test lines will be printed at the front of the bed.");
        break;
    case Stage::summary: {
        int ok = 0, failed = 0;
        for (const Entry &e : entries) {
            if ((flow.session & test_bit(e.test)) && (flow.started & test_bit(e.test))) {
                ok += assistant::result(e.test) == Result::ok;
                failed += assistant::result(e.test) == Result::failed;
            }
        }
        text = _("Finished: %d OK, %d failed. Touch a failed test to run it again.").formatted(info_params, ok, failed);
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
        s = { nullptr, nullptr, N_("Cancel") };
        break;
    case Stage::detach:
        s = { N_("Continue") };
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
    button_c.enable(!(flow.cancelled && (flow.stage == Stage::accel_run || flow.stage == Stage::cell_run)));
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
        } else if (flow.stage == Stage::summary) {
            start_session(test_bit(e.test), false); // refaire ce test seul
        }
        return;
    }
    switch (flow.stage) {
    case Stage::idle:
    case Stage::propose:
        if (tag == tag_a) {
            start_session(flow.mask, true);
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
            flow.stage = Stage::attach;
        } else if (tag == tag_c) {
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
    case Stage::checking:
    case Stage::accel_run:
    case Stage::cell_run:
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
