/// @file
#include "screen_progress_od3d.hpp"
#include "frost_od3d.hpp"

#include "island_od3d.hpp"

#include "print_info_od3d.hpp"
#include "spinner_od3d.hpp"
#include "thumbnail_od3d.hpp"

#include <algorithm>
#include <client_response_texts.hpp>
#include <common/error_code_mangle.hpp>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <span>
#include <utility>
#include <display.hpp>
#include <display_helper.h>
#include <filament_to_load.hpp>
#include <find_error.hpp>
#include <fsm_loadunload_type.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <mmu2/mmu2_error_converter.h>
#include <option/has_anfc.h>
#include <option/has_auto_retract.h>
#include <option/has_extruder_fsensor.h>
#include <option/has_loadcell.h>
#include <option/has_mmu2.h>
#include <option/has_nozzle_cleaner.h>
#include <option/has_side_fsensor.h>
#include <sound.hpp>
#include <timing.h>

namespace od3d {

namespace {

    using Phase = PhasesLoadUnload;
    using atelier::Op;

    // Géométrie du dialogue (SPEC_G.md §1.9, capture chargement.png), sous l'en-tête de 40 px : 6 px moins haut
    constexpr Rect16 box_rect = { 28, style::top, 424, 226 };
    constexpr Rect16 body_rect = { 40, 54, 400, 166 }; ///< sous les coins arrondis du cadre
    constexpr Rect16 bar_rect = { 46, 222, 388, 6 };
    constexpr int16_t buttons_y = 231;
    constexpr uint16_t buttons_h = 36; ///< deux lignes pour les textes longs de Prusa (« DÉSACTIVER MMU »)
    constexpr int16_t content_x = 46;
    constexpr uint16_t content_w = 388;
    constexpr int16_t text_x = 102; ///< titre et sous-titre, à droite de la bobine Ø44
    constexpr Rect16 mark_rect = { content_x, 56, 48, 48 }; ///< bobine Ø44 (46,58) ou icône d'erreur 48 × 48
    constexpr uint8_t spinner_d = 44; ///< anneau d'attente (spinner_od3d.hpp : un tour en 1,6 s, SPEC_G.md §3.4)
    constexpr uint8_t box_radius = style::radius_big; ///< coins de toutes nos grandes cartes
    constexpr Color veil = style::veil; ///< coins du cadre : la couleur du fond givré autour

    static_assert(style::inside(box_rect, body_rect) && style::inside(box_rect, bar_rect) && style::inside(body_rect, mark_rect));
    static_assert(style::inside(box_rect, style::with_ring(Rect16(content_x, buttons_y, content_w, buttons_h))));
    static_assert(body_rect.Bottom() < bar_rect.Top() && bar_rect.Bottom() < buttons_y - style::ring);

    /// Texte de chaque phase : les mêmes chaînes que ScreenFilamentChange (déjà traduites)
    const char *phase_text(Phase phase) {
        switch (phase) {
        case Phase::initial:
            return N_("Finishing buffered gcodes");
        case Phase::ChangingTool:
            return N_("Changing tool");
        case Phase::Parking_stoppable:
        case Phase::Parking_unstoppable:
            return N_("Parking");
        case Phase::WaitingTemp_stoppable:
        case Phase::WaitingTemp_unstoppable:
            return N_("Waiting for temperature");
        case Phase::Ramming_stoppable:
        case Phase::Ramming_unstoppable:
            return N_("Ramming");
        case Phase::Unloading_stoppable:
        case Phase::Unloading_unstoppable:
            return N_("Unloading");
        case Phase::IsFilamentUnloaded:
            return N_("Was filament unload successful?");
        case Phase::FilamentNotInFS:
            return N_("Please remove filament from filament sensor.");
        case Phase::ManualUnload_continuable:
        case Phase::ManualUnload_uncontinuable:
            return N_("Please open idler and remove filament manually");
        case Phase::UserPush_stoppable:
        case Phase::UserPush_unstoppable:
            return N_("Push the filament into the extruder and then, while still pushing, press CONTINUE.");
        case Phase::MakeSureInserted_stoppable:
        case Phase::MakeSureInserted_unstoppable:
        case Phase::AwaitingFilament_stoppable:
        case Phase::AwaitingFilament_unstoppable:
            return N_("Make sure the filament is inserted through the sensor.");
        case Phase::Inserting_stoppable:
        case Phase::Inserting_unstoppable:
        case Phase::LoadingToGears_stoppable:
        case Phase::LoadingToGears_unstoppable:
            return N_("Inserting");
        case Phase::IsFilamentInGear:
            return N_("Is filament in extruder gear?");
        case Phase::Ejecting_stoppable:
        case Phase::Ejecting_unstoppable:
            return N_("Ejecting");
        case Phase::Loading_stoppable:
        case Phase::Loading_unstoppable:
            return N_("Loading to nozzle");
        case Phase::Purging_stoppable:
        case Phase::Purging_unstoppable:
            return N_("Purging");
        case Phase::IsColor:
        case Phase::IsColorPurge:
            return N_("Is color correct?");
        case Phase::Unparking:
            return N_("Unparking");
#if HAS_NOZZLE_CLEANER()
        case Phase::UnloadNozzleCleaning:
        case Phase::LoadNozzleCleaning:
            return N_("Cleaning nozzle");
#endif
#if HAS_AUTO_RETRACT()
        case Phase::AutoRetracting:
            return N_("Auto-retracting filament");
#endif
#if HAS_MMU2()
        case Phase::LoadFilamentIntoMMU:
            return N_("Press CONTINUE and push filament into MMU.");
        case Phase::MMU_EngagingIdler:
        case Phase::MMU_ERREngagingIdler:
            return N_("Engaging idler");
        case Phase::MMU_DisengagingIdler:
        case Phase::MMU_ERRDisengagingIdler:
            return N_("Disengaging idler");
        case Phase::MMU_UnloadingToFinda:
            return N_("Unloading to FINDA");
        case Phase::MMU_UnloadingToPulley:
            return N_("Unloading to pulley");
        case Phase::MMU_FeedingToFinda:
            return N_("Feeding to FINDA");
        case Phase::MMU_FeedingToBondtech:
            return N_("Feeding to drive gear");
        case Phase::MMU_FeedingToNozzle:
            return N_("Feeding to nozzle");
        case Phase::MMU_AvoidingGrind:
            return N_("Avoiding grind");
        case Phase::MMU_FinishingMoves:
            return N_("Finishing moves");
        case Phase::MMU_ERRInternal:
            return N_("ERR Internal");
        case Phase::MMU_ERRHelpingFilament:
            return N_("ERR Helping filament");
        case Phase::MMU_ERRTMCFailed:
            return N_("ERR TMC failed");
        case Phase::MMU_UnloadingFilament:
            return N_("Unloading filament");
        case Phase::MMU_LoadingFilament:
            return N_("Loading filament");
        case Phase::MMU_SelectingFilamentSlot:
            return N_("Selecting filament slot");
        case Phase::MMU_PreparingBlade:
            return N_("Preparing blade");
        case Phase::MMU_PushingFilament:
            return N_("Pushing filament");
        case Phase::MMU_PerformingCut:
            return N_("Performing cut");
        case Phase::MMU_ReturningSelector:
            return N_("Returning selector");
        case Phase::MMU_ParkingSelector:
            return N_("Parking selector");
        case Phase::MMU_EjectingFilament:
            return N_("Ejecting filament");
        case Phase::MMU_RetractingFromFinda:
            return N_("Retracting from FINDA");
        case Phase::MMU_Homing:
            return N_("Homing");
        case Phase::MMU_MovingSelector:
            return N_("Moving selector");
        case Phase::MMU_FeedingToFSensor:
            return N_("Feeding to fsensor");
        case Phase::MMU_HWTestBegin:
            return N_("HW test begin");
        case Phase::MMU_HWTestIdler:
            return N_("HW test idler");
        case Phase::MMU_HWTestSelector:
            return N_("HW test selector");
        case Phase::MMU_HWTestPulley:
            return N_("HW test pulley");
        case Phase::MMU_HWTestCleanup:
            return N_("HW test cleanup");
        case Phase::MMU_HWTestExec:
            return N_("HW test exec");
        case Phase::MMU_HWTestDisplay:
            return N_("HW test display");
        case Phase::MMU_ErrHwTestFailed:
            return N_("ERR HW test failed");
#endif
        default:
            return nullptr; // début d'une opération du MMU, ou phase affichée en erreur
        }
    }

    /// Nom de l'opération quand elle ne vient pas de nos écrans (mêmes chaînes que ScreenFilamentChange)
    const char *mode_name(LoadUnloadMode mode) {
        switch (mode) {
        case LoadUnloadMode::Change:
            return N_("Changing filament");
        case LoadUnloadMode::Load:
            return N_("Loading filament");
        case LoadUnloadMode::Unload:
            return N_("Unloading filament");
        case LoadUnloadMode::FilamentStuck:
            return N_("Reloading filament");
        case LoadUnloadMode::Purge:
            return N_("Purging filament");
        case LoadUnloadMode::Test:
            return N_("Testing filament");
        case LoadUnloadMode::Cut:
            return N_("Cutting filament");
        case LoadUnloadMode::Eject:
            return N_("Ejecting filament");
        }
        return N_("Loading filament");
    }

    /// Titre d'une opération lancée depuis l'atelier, avec son emplacement
    const char *request_title(Op op) {
        switch (op) {
        case Op::load:
            return N_("Loading slot %d");
        case Op::unload:
            return N_("Unloading slot %d");
        case Op::preload:
            return N_("Preloading slot %d");
        case Op::preload_all:
            return N_("Preloading the MMU");
        case Op::eject:
            return N_("Ejecting slot %d");
        case Op::cut:
            return N_("Cutting filament %d");
        }
        return N_("Loading slot %d");
    }

    /// Étapes prévues d'une opération lancée depuis l'atelier : tables de Prusa (src/mmu2/mmu2_progress.cpp) et
    /// séquences du firmware du MMU (logic/*.cpp), filament hors de la tête (sinon l'atelier ne les propose pas)
    std::span<const Phase> plan_for(Op op) {
        static constexpr Phase load[] = { Phase::WaitingTemp_unstoppable, Phase::MMU_FeedingToFinda, Phase::MMU_FeedingToBondtech,
            Phase::MMU_FeedingToFSensor, Phase::MMU_DisengagingIdler, Phase::Loading_unstoppable };
        static constexpr Phase unload[] = { Phase::WaitingTemp_unstoppable, Phase::Ramming_unstoppable, Phase::MMU_UnloadingToFinda, Phase::MMU_DisengagingIdler };
        static constexpr Phase preload[] = { Phase::MMU_EngagingIdler, Phase::MMU_FeedingToFinda, Phase::MMU_RetractingFromFinda, Phase::MMU_DisengagingIdler };
        static constexpr Phase eject[] = { Phase::MMU_ParkingSelector, Phase::MMU_EngagingIdler, Phase::MMU_EjectingFilament };
        static constexpr Phase cut[] = { Phase::MMU_SelectingFilamentSlot, Phase::MMU_FeedingToFinda, Phase::MMU_RetractingFromFinda, Phase::MMU_PreparingBlade,
            Phase::MMU_PushingFilament, Phase::MMU_DisengagingIdler, Phase::MMU_PerformingCut, Phase::MMU_Homing, Phase::MMU_ReturningSelector };
        switch (op) {
        case Op::load:
            return load;
        case Op::unload:
            return unload;
        case Op::preload:
        case Op::preload_all:
            return preload;
        case Op::eject:
            return eject;
        case Op::cut:
            return cut;
        }
        return {};
    }

    /// Même étape à l'écran (les variantes « arrêtable » et « non arrêtable » ont le même texte)
    bool same_step(Phase a, Phase b) {
        const char *ta = phase_text(a);
        const char *tb = phase_text(b);
        return ta && tb && strcmp(ta, tb) == 0;
    }

    enum class Mark : uint8_t {
        done,
        current,
        upcoming,
    };

    /// Repère d'une étape (SPEC_G.md §3.4) : ✓ orange (le caractère n'est pas dans la police), ● blanc, ○ gris
    void draw_mark(int16_t x, int16_t y, Mark mark) {
        switch (mark) {
        case Mark::done:
            for (int16_t t = 0; t < 2; ++t) { // trait de 2 px
                display::draw_line(point_ui16(x, y + 5 + t), point_ui16(x + 3, y + 8 + t), style::accent);
                display::draw_line(point_ui16(x + 3, y + 8 + t), point_ui16(x + 9, y + 2 + t), style::accent);
            }
            break;
        case Mark::current:
            display::draw_rounded_rect(Rect16(x + 1, y + 2, 8, 8), style::surface, style::text, 4, MIC_ALL_CORNERS);
            break;
        case Mark::upcoming:
            display::draw_rounded_rect(Rect16(x + 1, y + 2, 8, 8), style::surface, style::text_faint, 4, MIC_ALL_CORNERS);
            display::draw_rounded_rect(Rect16(x + 3, y + 4, 4, 4), style::text_faint, style::surface, 2, MIC_ALL_CORNERS);
            break;
        }
    }

    /// Sons de ScreenFilamentChange : un bip quand Prusa attend l'utilisateur
    void play_phase_sound(Phase phase, LoadUnloadMode mode) {
        switch (phase) {
        case Phase::IsFilamentUnloaded:
            sound::play(SoundType::single_beep);
            if (mode == LoadUnloadMode::Change || mode == LoadUnloadMode::FilamentStuck) {
                sound::play(SoundType::waiting_beep); // M600 ou fin de bobine : on appelle l'utilisateur
            }
            break;
        case Phase::FilamentNotInFS:
        case Phase::UserPush_stoppable:
        case Phase::UserPush_unstoppable:
        case Phase::MakeSureInserted_stoppable:
        case Phase::MakeSureInserted_unstoppable:
        case Phase::AwaitingFilament_stoppable:
        case Phase::AwaitingFilament_unstoppable:
        case Phase::IsColor:
        case Phase::IsColorPurge:
            sound::play(SoundType::single_beep);
            break;
        default:
            break;
        }
    }

    bool valid_mode(LoadUnloadMode mode) {
        return std::to_underlying(mode) <= std::to_underlying(LoadUnloadMode::Eject);
    }

} // namespace

// ---- Cadre, contenu, barre ----

ScreenLoadUnload::Box::Box(ScreenLoadUnload &owner)
    : window_frame_t(&owner, box_rect)
    , owner(owner) {
    SetBackColor(style::surface); // ce qu'un enfant caché laisse derrière lui
}

Rect16 ScreenLoadUnload::Box::get_rect_for_touch() const {
    return Rect16(0, 0, style::screen_w, style::screen_h);
}

void ScreenLoadUnload::Box::unconditionalDraw() {
    display::draw_rounded_rect(GetRect(), veil, style::surface, box_radius, MIC_ALL_CORNERS);
}

void ScreenLoadUnload::Box::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::TOUCH_CLICK) {
        const point_ui16_t at = event_conversion_union { .pvoid = param }.point;
        if (!GetRect().Contain(at)) {
            if (style::back_touch.Contain(at)) {
                owner.minimize(); // la flèche de l'en-tête (seulement pendant une étape : minimize() le vérifie)
            }
            return;
        }
    }
    window_frame_t::windowEvent(sender, event, param);
}

ScreenLoadUnload::Body::Body(window_t *parent, const ScreenLoadUnload &owner)
    : window_t(parent, body_rect)
    , owner(owner) {}

void ScreenLoadUnload::Body::draw_header(Rect16 r) const {
    // Titre : l'opération (et son emplacement si elle vient de l'atelier), sinon le nom de Prusa
    StringViewUtf8Parameters<4> params;
    string_view_utf8 title;
    char subtitle[24] = "";
    if (owner.request) {
        title = _(request_title(owner.request->op)).formatted(params, owner.request->slot + 1);
        atelier::spool_name(subtitle, FilamentType::for_tool(VirtualToolIndex::from_raw(owner.request->slot)));
    } else {
        title = _(mode_name(owner.mode));
        if (owner.mode == LoadUnloadMode::Load) {
            atelier::spool_name(subtitle, filament::get_type_to_load());
        }
    }

    const uint16_t w = r.Left() + r.Width() - text_x;
    const Font font = title.computeNumUtf8Chars() * width(Font::normal) <= w ? Font::normal : Font::small;
    draw_text(Rect16(text_x, r.Top() + 7, w, height(Font::normal)), title, font, style::text_strong, style::surface);
    draw_text(Rect16(text_x, r.Top() + 28, w, height(Font::small)), string_view_utf8::MakeRAM(subtitle), Font::small, style::text_muted, style::surface);
}

void ScreenLoadUnload::Body::unconditionalDraw() {
    const Rect16 r = GetRect();
    // Le fond, sauf la case de la bobine (SpoolMark, dessinée par-dessus : pas de clignotement)
    const Rect16 m = mark_rect;
    display::fill_rect(Rect16(r.Left(), r.Top(), r.Width(), m.Top() - r.Top()), style::surface);
    display::fill_rect(Rect16(r.Left(), m.Top(), m.Left() - r.Left(), m.Height()), style::surface);
    display::fill_rect(Rect16(m.Left() + m.Width(), m.Top(), r.Left() + r.Width() - m.Left() - m.Width(), m.Height()), style::surface);
    display::fill_rect(Rect16(r.Left(), m.Top() + m.Height(), r.Width(), r.Top() + r.Height() - m.Top() - m.Height()), style::surface);

    if (owner.kind == Kind::notice) {
        const uint16_t w = r.Left() + r.Width() - text_x;
        if (owner.notice_title) {
            render_text_align(Rect16(text_x, r.Top() + 2, w, 2 * height(Font::normal)), _(owner.notice_title), Font::normal, style::surface, style::text_strong,
                { 0, 0, 0, 0 }, text_flags(Align_t::LeftCenter(), is_multiline::yes, check_overflow::no));
        }
        if (owner.notice_text) {
            render_text_align(Rect16(content_x, r.Top() + 56, content_w, 5 * height(Font::small)), _(owner.notice_text), Font::small, style::surface, style::text,
                { 0, 0, 0, 0 }, text_flags(Align_t::LeftTop(), is_multiline::yes, check_overflow::no));
        }
        if (owner.notice_code) {
            // Le lien d'aide de Prusa (TextErrorUrlWindow), sans le QR
            char url[24];
            snprintf(url, sizeof(url), "prusa.io/%05u", unsigned(map_error_code(ErrCode(owner.notice_code))));
            draw_text(Rect16(content_x, r.Top() + 144, content_w, height(Font::small)), string_view_utf8::MakeRAM(url), Font::small, style::text_muted, style::surface);
        }
        return;
    }

    draw_header(r);

    if (owner.kind == Kind::prompt) {
        if (const char *text = phase_text(owner.phase)) {
            render_text_align(Rect16(content_x, 108, content_w, 6 * height(Font::small)), _(text), Font::small, style::surface, style::text,
                { 0, 0, 0, 0 }, text_flags(Align_t::LeftTop(), is_multiline::yes, check_overflow::no));
        }
        return;
    }

    // Étapes annoncées par Prusa (faites, la dernière en cours), puis celles qui restent au plan de l'opération
    constexpr int lines = 6;
    const std::span<const Phase> plan = owner.request ? plan_for(owner.request->op) : std::span<const Phase> {};
    int reached = -1; ///< dernière étape du plan déjà vue
    for (int h = owner.history_count - 1; h >= 0 && reached < 0; --h) {
        for (int i = int(plan.size()) - 1; i >= 0; --i) {
            if (same_step(plan[i], owner.history[h])) {
                reached = i;
                break;
            }
        }
    }
    const int remaining = int(plan.size()) - (reached + 1);
    const int upcoming = std::min(remaining, lines - std::min<int>(owner.history_count, lines / 2));
    const int past = std::min<int>(owner.history_count, lines - upcoming);
    const uint16_t text_w = r.Left() + r.Width() - 68;
    int16_t y = 108;
    for (int i = owner.history_count - past; i < owner.history_count; ++i, y += 19) {
        const bool current = i == owner.history_count - 1;
        draw_mark(content_x + 2, y, current ? Mark::current : Mark::done);
        draw_text(Rect16(68, y, text_w, height(Font::small)), _(phase_text(owner.history[i])), Font::small, current ? style::text : style::text_muted, style::surface);
    }
    for (int i = 0; i < upcoming; ++i, y += 19) {
        draw_mark(content_x + 2, y, Mark::upcoming);
        draw_text(Rect16(68, y, text_w, height(Font::small)), _(phase_text(plan[reached + 1 + i])), Font::small, style::text_faint, style::surface);
    }
}

ScreenLoadUnload::SpoolMark::SpoolMark(window_t *parent, const ScreenLoadUnload &owner)
    : window_t(parent, mark_rect)
    , owner(owner) {}

bool ScreenLoadUnload::SpoolMark::spinning() const {
    return owner.kind != Kind::notice && !owner.request;
}

void ScreenLoadUnload::SpoolMark::unconditionalDraw() {
    const Rect16 r = GetRect();
    if (owner.kind == Kind::notice) {
        const img::Resource *icon = owner.notice_type == std::to_underlying(ErrType::ERROR) ? &img::error_48x48
            : (owner.notice_type == std::to_underlying(ErrType::WARNING) ? &img::warning_48x48 : &img::info_48x48);
        draw_icon({ r.Left(), r.Top() }, *icon, style::surface);
        drawn = std::nullopt;
        return;
    }
    // Bobine Ø44, 2 px sous le haut de la case de 48 px, ou anneau d'attente de 4 px (spinner_od3d.hpp)
    if (owner.request) {
        const spool::Disc disc(Slot::read(owner.request->slot).look, spinner_d);
        draw_pixels(r, [&](int x, int y) { return disc.pixel(x, y - 2, style::surface); });
        drawn = std::nullopt;
        return;
    }
    // Case entière à l'ouverture ou après autre chose ; ensuite, 25 fois par seconde, les deux bouts de l'arc seulement
    if (!HasValidBackground() || !drawn) {
        display::fill_rect(Rect16(r.Left(), r.Top(), r.Width(), 2), style::surface);
        display::fill_rect(Rect16(r.Left(), r.Top() + 2 + spinner_d, r.Width(), r.Height() - 2 - spinner_d), style::surface);
        display::fill_rect(Rect16(r.Left() + spinner_d, r.Top() + 2, r.Width() - spinner_d, spinner_d), style::surface);
        drawn = std::nullopt;
    }
    const uint8_t step = spinner::step_at(ticks_ms());
    draw_spinner(Rect16(r.Left(), r.Top() + 2, spinner_d, spinner_d), 4, step, style::surface, drawn);
    drawn = step;
}

void ScreenLoadUnload::SpoolMark::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::TEXT_ROLL && spinning() && drawn != spinner::step_at(ticks_ms())) {
        const bool background_valid = HasValidBackground(); // comme Card::invalidate_content : l'arc seul
        Invalidate();
        if (background_valid) {
            ValidateBackground();
        }
    }
    window_t::windowEvent(sender, event, param);
}

ScreenLoadUnload::Bar::Bar(window_t *parent, const ScreenLoadUnload &owner)
    : window_t(parent, bar_rect)
    , owner(owner) {}

void ScreenLoadUnload::Bar::unconditionalDraw() {
    // Piste et partie faite d'une passe (la piste se repeignait sous la partie faite à chaque pas)
    const Rect16 r = GetRect();
    const int w = r.Width(), h = r.Height();
    const int done = w * std::min<uint8_t>(owner.progress, 100) / 100;
    draw_pixels(r, [&](int x, int y) {
        const int inset = print_info::corner_inset(y, h, 3);
        if (x < inset || x >= w - inset) {
            return style::surface;
        }
        return done >= 6 && x < done - inset ? style::accent : style::line;
    });
}

// ---- Écran ----

namespace {

    void minimize_current() {
        if (auto *screen = Screens::Access()->get<ScreenLoadUnload>()) {
            screen->minimize();
        }
    }

    bool is_notice_phase(Phase phase) {
        switch (phase) {
        case Phase::MMU_ERRWaitingForUser:
        case Phase::FilamentStuck:
        case Phase::LoadingObstruction_stoppable:
        case Phase::LoadingObstruction_unstoppable:
#if HAS_ANFC()
        case Phase::OPT_UncommitedUsage:
#endif
            return true;
        default:
            return false;
        }
    }

} // namespace

bool ScreenLoadUnload::needs_user(fsm::BaseData data) {
    const Phase phase = GetEnumFromPhaseIndex<Phase>(data.GetPhase());
    if (is_notice_phase(phase)) {
        return true;
    }
    // Comme enter_phase() : une question dès qu'une réponse n'est pas « Stop »
    const PhaseResponses &available = ClientResponses::get_available_responses(FSMAndPhase(phase));
    return std::ranges::any_of(available, [](Response r) { return r != Response::_none && r != Response::Stop; });
}

void ScreenLoadUnload::minimize() {
    if (kind != Kind::progress || !has_data) {
        return; // une question ou une erreur attend une réponse : elle reste à l'écran
    }
    std::optional<island::Origin> origin;
    if (request) {
        origin = island::Origin { uint8_t(request->op), request->slot };
    }
    island::minimize(data, origin);
    Screens::Access()->close_screen(); // la machine continue ; DialogHandler alimente la pastille
}

ScreenLoadUnload::ScreenLoadUnload()
    : request(atelier::take_request())
    , header(this, _("Filament"), true, minimize_current) // la flèche réduit l'opération, elle ne ferme rien
    , box(*this)
    , body(&box, *this)
    , spool_mark(&box, *this)
    , bar(&box, *this)
    , buttons {
        { &box, Rect16(content_x, buttons_y, content_w, buttons_h), 0, Button::Look::outline, {} },
        { &box, Rect16(content_x, buttons_y, content_w, buttons_h), 1, Button::Look::outline, {} },
        { &box, Rect16(content_x, buttons_y, content_w, buttons_h), 2, Button::Look::outline, {} },
    } {
    SetBackColor(style::screen); // ce que laisse un élément caché de l'en-tête (le voile est sous l'en-tête seulement)
    ClrMenuTimeoutClose(); // comme les écrans FSM de Prusa
    // Molette et toucher vont au dialogue : rien ne s'ouvre par-dessus la machine à états (son écran ne recevrait plus
    // ses phases). La flèche de l'en-tête passe par Box::windowEvent, « Continuer en arrière-plan » est dans le cadre.
    CaptureNormalWindow(box);
    setup_buttons(); // la 1re phase arrive aussitôt par Change()
}

ScreenLoadUnload::~ScreenLoadUnload() {
    frost::stop(*this);
    sound::stop();
    atelier::release_request();
}

void ScreenLoadUnload::Change(fsm::BaseData new_data) {
    const Phase new_phase = GetEnumFromPhaseIndex<Phase>(new_data.GetPhase());
    // Une autre erreur du MMU garde la même phase : seules ses données (l'erreur) changent
    const bool phase_changed = !has_data || new_phase != phase || (new_phase == Phase::MMU_ERRWaitingForUser && new_data.GetData() != data.GetData());
    data = new_data;
    has_data = true;

    // MMU_ERRWaitingForUser porte un pointeur vers l'erreur, toutes les autres phases FSMLoadUnloadData
    bool redraw_header = false;
    if (new_phase != Phase::MMU_ERRWaitingForUser) {
        const auto d = fsm::deserialize_data<FSMLoadUnloadData>(new_data.GetData());
        if (valid_mode(d.mode) && d.mode != mode) {
            mode = d.mode;
            redraw_header = true;
        }
        if (d.progress != progress) {
            progress = d.progress;
            bar.Invalidate();
        }
    }

    if (phase_changed) {
        phase = new_phase;
        enter_phase();
    } else if (redraw_header) {
        body.Invalidate();
    }
}

void ScreenLoadUnload::enter_phase() {
    // Comme les cadres de Prusa : le son de la phase précédente s'arrête
    sound::stop();
    notice_title = notice_text = nullptr;
    notice_code = 0;
    notice_type = std::to_underlying(ErrType::WARNING);
    responses = {};

    const PhaseResponses &available = ClientResponses::get_available_responses(FSMAndPhase(phase));
    for (size_t i = 0, n = 0; i < available.size() && n < responses.size(); ++i) {
        if (available[i] != Response::_none) {
            responses[n++] = available[i];
        }
    }

    switch (phase) {
#if HAS_MMU2()
    case Phase::MMU_ERRWaitingForUser:
        if (has_data) {
            // L'erreur du MMU et ses 3 boutons, comme FrameMMUNotice
            const auto *desc = fsm::deserialize_data<const MMU2::MMUErrDesc *>(data.GetData());
            notice_title = desc->err_title;
            notice_text = desc->err_text;
            notice_code = std::to_underlying(desc->err_code);
            notice_type = std::to_underlying(desc->type);
            // Les boutons du MMU sont à leur place (gauche, milieu, droite) : une erreur à un bouton (« Filament
            // éjecté » : Continuer) l'a au milieu. On les tasse, sinon setup_buttons() n'en voyait aucun et
            // l'écran restait bloqué derrière un « Annuler » éteint.
            size_t n = 0;
            for (const auto button : desc->buttons) {
                if (const Response r = MMU2::ButtonOperationToResponse(button); r != Response::_none && n < responses.size()) {
                    responses[n++] = r;
                }
            }
        }
        kind = Kind::notice;
        break;
#endif
#if HAS_LOADCELL() && HAS_EXTRUDER_FSENSOR()
    case Phase::FilamentStuck: {
        const auto &err = find_error(ErrCode::ERR_MECHANICAL_STUCK_FILAMENT_DETECTED);
        notice_title = err.err_title;
        notice_text = err.err_text;
        notice_code = std::to_underlying(err.err_code);
        notice_type = std::to_underlying(err.type);
        kind = Kind::notice;
        break;
    }
#endif
#if HAS_SIDE_FSENSOR() && HAS_EXTRUDER_FSENSOR()
    case Phase::LoadingObstruction_stoppable:
    case Phase::LoadingObstruction_unstoppable: {
        const auto &err = find_error(ErrCode::ERR_MECHANICAL_LOADING_OBSTRUCTION);
        notice_title = err.err_title;
        notice_text = err.err_text;
        notice_code = std::to_underlying(err.err_code);
        notice_type = std::to_underlying(err.type);
        kind = Kind::notice;
        break;
    }
#endif
#if HAS_ANFC()
    case Phase::OPT_UncommitedUsage:
        notice_title = N_("OpenPrintTag Pending Write");
        notice_text = N_("There is filament usage to be written to the OpenPrintTag. Make sure the spool is close to the NFC reader.");
        notice_type = std::to_underlying(ErrType::USER_ACTION);
        kind = Kind::notice;
        break;
#endif
    default: {
        // Une question si Prusa attend autre chose que « Stop » ; sinon une étape de plus
        const bool question = std::ranges::any_of(responses, [](Response r) { return r != Response::_none && r != Response::Stop; });
        kind = question ? Kind::prompt : Kind::progress;
        if (!question && phase_text(phase) && (!history_count || history[history_count - 1] != phase)) {
            if (history_count == history.size()) {
                std::copy(history.begin() + 1, history.end(), history.begin());
                --history_count;
            }
            history[history_count++] = phase;
        }
        break;
    }
    }

    play_phase_sound(phase, mode);
    bar.set_visible(kind == Kind::progress);
    header.show_back(kind == Kind::progress); // réduire seulement pendant une étape, pas devant une question
    setup_buttons(); // repeint les boutons, et tout le cadre si leur nombre change
    // Le contenu seulement : le cadre (424 × 226) était repeint à chaque phase ; la barre cachée est effacée par lui
    body.Invalidate();
    spool_mark.Invalidate();
}

void ScreenLoadUnload::setup_buttons() {
    size_t count = 0;
    while (count < responses.size() && responses[count] != Response::_none) {
        ++count;
    }
    // Sans réponse, un « Annuler » éteint : Prusa ne permet pas d'arrêter cette étape (chargement par le MMU…). Pendant
    // une étape, « Continuer en arrière-plan » à droite : on peut aller ailleurs, la pastille de l'en-tête ramène ici
    const size_t answers = std::max<size_t>(count, 1);
    const bool background = kind == Kind::progress && has_data && answers < std::size(buttons);
    background_button = background ? uint8_t(answers) : 0xff;
    const size_t shown = answers + background;
    if (shown != shown_buttons) {
        shown_buttons = shown;
        box.Invalidate(); // les intervalles entre les boutons bougent : seul le cadre les repeint
    }
    constexpr int16_t gap = 6;
    const uint16_t w = (content_w - gap * (shown - 1)) / shown;
    bool primary_used = false;
    for (size_t i = 0; i < std::size(buttons); ++i) {
        Button &b = buttons[i];
        if (i >= shown) {
            b.set_visible(false);
            b.set_enabled(false);
            continue;
        }
        b.SetRect(style::with_ring(Rect16(content_x + i * (w + gap), buttons_y, w, buttons_h)));
        b.set_visible(true);
        if (i == background_button) {
            b.set_enabled(true);
            b.set(Button::Look::secondary, _("Continue in background"));
            continue;
        }
        const Response r = i < count ? responses[i] : Response::Stop;
        b.set_enabled(i < count);
        if (r == Response::Stop) {
            b.set(Button::Look::outline, _("Cancel"));
        } else {
            b.set(primary_used ? Button::Look::outline : Button::Look::primary, _(get_response_text(r)));
            primary_used = true;
        }
    }
    // Le premier bouton actif ; sans bouton actif, le cadre garde le focus (pas l'en-tête, qui n'est pas atteignable ici)
    if (const auto b = std::ranges::find_if(buttons, [](const Button &x) { return x.IsEnabled(); }); b != std::end(buttons)) {
        b->SetFocus();
    } else {
        box.SetFocus();
    }
}

void ScreenLoadUnload::respond(Response response) {
    if (response != Response::_none) {
        marlin_client::FSM_response(FSMAndPhase(phase), response);
    }
}

void ScreenLoadUnload::unconditionalDraw() {
    // L'en-tête garde son fond : ses éléments, peints sur #1B1C1E, faisaient des pavés clairs sur le voile
    display::fill_rect(style::header, style::screen);
    if (!std::exchange(frosted, true)) {
        // 1er dessin : l'écran d'où l'on vient reste autour du cadre, flouté et assombri par tranches (retour du 22/11 :
        // « il n'y a pas de fond ») ; les coins du cadre sont peints de la couleur qu'y prend le fond uni
        frost::start(*this, style::below_header, box_rect);
        return;
    }
    // Redessiné en entier : ce qui était autour n'est plus sûr, voile uni
    frost::stop(*this);
    display::fill_rect(Rect16::fromLTRB(0, style::header_line.Top(), style::screen_w, style::screen_h), veil);
}

void ScreenLoadUnload::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::CHILD_CLICK: {
        const size_t i = reinterpret_cast<uintptr_t>(param);
        if (i == background_button) {
            minimize();
        } else if (i < responses.size()) {
            respond(responses[i]);
        }
        return;
    }
    case GUI_event_t::HELD_RELEASED:
        // L'appui long annule, là où Prusa le permet (SPEC_G.md §3.4)
        if (std::ranges::find(responses, Response::Stop) != responses.end()) {
            respond(Response::Stop);
        }
        return;
    default:
        break;
    }
    screen_t::windowEvent(sender, event, param);
}

void ScreenLoadUnload::InitState(screen_init_variant var) {
    if (auto fsm_data = var.GetFsmBaseData()) {
        Change(*fsm_data);
    }
}

screen_init_variant ScreenLoadUnload::GetCurrentState() const {
    screen_init_variant var;
    var.SetFsmBaseData(data);
    return var;
}

} // namespace od3d
