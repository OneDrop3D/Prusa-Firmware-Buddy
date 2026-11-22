/// @file
#include "spool_sheet_od3d.hpp"

#include "atelier_od3d.hpp"
#include "frost_od3d.hpp"
#include "gesture_od3d.hpp"
#include "screen_pa_od3d.hpp"
#include "screen_spool_od3d.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <feature/od3d/spool_usage_od3d.hpp>
#include <feature/od3d/tip_od3d.hpp>
#include <i18n.h>
#include <IDialog.hpp>
#include <img_resources.hpp>
#include <knob_event.hpp>
#include <option/has_touch.h>
#include <ScreenHandler.hpp>
#include <utility>
#if HAS_TOUCH()
    #include <gui.hpp>
    #include <hw/touchscreen/touchscreen.hpp>
    #include <timing.h>
#endif

namespace od3d {

namespace {

    using atelier::Op;

    // Géométrie de la fiche (SPEC_G.md §3.3), resserrée : elle commence sous la bande des bobines de l'accueil (jusqu'à
    // y 111 avec sa marge) au lieu de la recouvrir de 2 px, ce qui coupait leurs coins du bas
    constexpr int16_t top = 114;
    constexpr Rect16 band = { 0, top, style::screen_w, style::screen_h - top };
    constexpr Rect16 head_rect = { 0, top, 393, 55 };
    constexpr uint8_t band_radius = style::radius; ///< arrondi normé (18 avant)
    // Le grand bouton (charger / décharger) partage sa ligne avec « Calibrer le PA » (F4, en un clic)
    constexpr Rect16 main_body = { 16, top + 60, 288, 42 };
    constexpr Rect16 calibrate_body = { 312, top + 60, 152, 42 };
    constexpr Rect16 close_body = { 397, top + 16, 67, 28 };
    /// Où un doigt saisit la fiche pour la tirer vers le bas : la poignée et le haut, sauf « Fermer »
    constexpr Rect16 grab_rect = { 0, top, style::screen_w, 56 };
    /// Tirée au plus jusque-là : son bord arrondi reste sur l'écran
    constexpr int16_t max_pull = band.Height() - style::radius;

    enum Tag : uint8_t {
        tag_main,
        tag_calibrate,
        tag_change,
        tag_material,
        tag_usage,
        tag_preload,
        tag_eject,
        tag_cut,
        tag_pa,
        tag_close,
    };

    enum class Choice : uint8_t {
        none,
        load,
        unload,
        insert,
        change,
        material,
        usage,
        preload,
        eject,
        cut,
        pa,
        calibrate,
    };

    constexpr Rect16 secondary_body(int i) {
        return Rect16(16 + 152 * i, top + 110, 144, 38);
    }
    constexpr Rect16 chip_body(int i) {
        return Rect16(16 + 114 * i, top + 156, 106, 32);
    }
    static_assert(style::inside(band, style::with_ring(chip_body(3))) && style::inside(band, style::with_ring(secondary_body(2))));
    static_assert(head_rect.Bottom() < style::with_ring(main_body).Top() && style::with_ring(main_body).Bottom() < style::with_ring(secondary_body(0)).Top());
    static_assert(style::with_ring(main_body).Right() < style::with_ring(calibrate_body).Left() && style::inside(band, style::with_ring(calibrate_body)));
    static_assert(style::with_ring(secondary_body(0)).Bottom() < style::with_ring(chip_body(0)).Top());

    /// Poignée (« tige ») de la fiche dont le haut est en y
    void draw_handle(int16_t y) {
        display::draw_rounded_rect(Rect16(222, y + 6, 36, 4), style::surface, Color::from_raw(0x5B5B5B), 2, MIC_ALL_CORNERS);
    }

    /// Haut de la fiche : poignée, bobine, « Emplacement n · matière », état ; rien d'actif
    class Head : public window_t {
    public:
        Head(window_t *parent, uint8_t index)
            : window_t(parent, head_rect)
            , index(index) {}

    protected:
        void unconditionalDraw() override {
            const Rect16 r = GetRect();
            // Son coin arrondi : il se redessine seul après un glissement de la fiche
            display::draw_rounded_rect(r, style::veil, style::surface, band_radius, MIC_TOP_LEFT);
            draw_handle(top);

            const VirtualToolIndex tool = VirtualToolIndex::from_raw(index);
            const FilamentType type = FilamentType::for_tool(tool);
            const Slot slot = Slot::read(index);
            constexpr uint8_t d = 40;
            const point_i16_t at { 16, top + 13 };
            draw_spool(at, slot.look, d, style::surface, slot.loaded ? std::optional(style::accent) : std::nullopt);

            char name[24];
            atelier::spool_name(name, type);
            StringViewUtf8Parameters<32> params;
            const int16_t x = 66;
            const uint16_t w = r.Left() + r.Width() - x;
            draw_text(Rect16(x, top + 14, w, height(Font::normal)), _("Slot %d · %s").formatted(params, index + 1, name), Font::normal, style::text_strong, style::surface);

            // État, l'alerte de la pointe (F10, en jaune ; détail dans Atelier › Infos du MMU), la température de la
            // matière, puis le reste (F13 ou étiquette OpenPrintTag)
            char status[72];
            const char *state = slot.loaded ? N_("In the nozzle") : (type ? N_("In the MMU") : N_("Empty"));
            _(state).copyToRAM(status, sizeof(status));
            const bool tip_drift = index < tip::slots && tip::drifting(tip::stats(index));
            if (tip_drift) {
                strlcat(status, " · ", sizeof(status));
                const size_t len = strlen(status);
                _("tip drift").copyToRAM(status + len, sizeof(status) - len);
            }
            if (type) {
                const FilamentTypeParameters p = type.parameters();
                const size_t len = strlen(status);
                snprintf(status + len, sizeof(status) - len, " · %d/%d\xC2\xB0", p.nozzle_temperature, p.heatbed_temperature);
            }
            if (const auto usage = spool_usage::report(index); usage && type) {
                const size_t len = strlen(status);
                snprintf(status + len, sizeof(status) - len, " · ~%d g", int(usage->remaining_g));
            }
            draw_text(Rect16(x, top + 36, w, height(Font::small)), string_view_utf8::MakeRAM(status), Font::small, tip_drift ? style::warning : style::text_muted, style::surface);
        }

    private:
        uint8_t index;
    };

    class SpoolSheet : public IDialog {
    public:
        explicit SpoolSheet(uint8_t index);
        ~SpoolSheet();

        Choice choice = Choice::none;

        Rect16 get_rect_for_touch() const override {
            return style::screen_rect; // un toucher sur l'en-tête aussi ferme la fiche
        }

    protected:
        void unconditionalDraw() override;
        void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

    private:
#if HAS_TOUCH()
        /// Fiche tirée vers le bas par sa poignée, toucher du fond givré : suivis à chaque tour de l'interface
        void follow_finger();
        /// Fiche peinte à offset sous sa place : voile au-dessus, bord arrondi et poignée ; ce que le bord a effacé
        /// dans la fiche se redessine quand elle est revenue (restore)
        void paint(int16_t offset);
        void restore();

        enum class Contact : uint8_t {
            none,
            veil, ///< posé sur le fond givré (ou l'en-tête) : ferme la fiche au lever
            grab, ///< posé sur la poignée ou le haut de la fiche
            other,
        };
        Touchscreen_Base::LenientClickGuard lenient; ///< un toucher qui bouge de 3 px reste un toucher (fond givré, boutons)
        Contact contact = Contact::none;
        bool pulling = false; ///< la fiche suit le doigt : aucun toucher ni balayage n'agit dedans
        bool springing = false; ///< lâchée avant le tiers : elle remonte
        int16_t offset = 0; ///< décalage voulu
        int16_t drawn = 0; ///< décalage peint
        int16_t wiped = top; ///< bas de ce que le bord a effacé dans la fiche
        uint32_t painted_ms = 0;
#endif
        bool frosted = false; ///< le fond au-dessus de la fiche a été givré (1er dessin)
        StringViewUtf8Parameters<8> usage_params; ///< « Reste 350 g », avant le bouton qui garde le texte formaté
        std::array<char, 12> pa_text {}; ///< « PA 0.035 » ou « PA ? » (F4), avant le bouton qui le montre
        Head head;
        Button main;
        Button calibrate; ///< F4 : « Calibrer le PA » en un clic (chargement compris)
        Button change;
        Button material;
        Button usage;
        Button preload;
        Button eject;
        Button cut;
        Button pa;
        Button close; ///< dernier à la molette (ordre de la maquette), mais toujours atteignable
        Choice main_choice;
    };

    SpoolSheet::SpoolSheet(uint8_t index)
        : IDialog(style::below_header) // l'écran dessous ne se redessine plus (il est givré) ; l'en-tête reste vivant
        , head(this, index)
        , main(this, main_body, tag_main, Button::Look::primary, {}, nullptr, style::surface, style::radius_big)
        , calibrate(this, calibrate_body, tag_calibrate, Button::Look::secondary, _("Calibrate PA"), nullptr, style::surface, style::radius_big)
        , change(this, secondary_body(0), tag_change, Button::Look::secondary, _("Change spool"), &img::spool_16x16)
        , material(this, secondary_body(1), tag_material, Button::Look::secondary, _("Material and colour"), &img::user_filament_16x16)
        , usage(this, secondary_body(2), tag_usage, Button::Look::secondary, _("Spool usage"), &img::info_16x16)
        , preload(this, chip_body(0), tag_preload, Button::Look::outline, _("Preload"), nullptr, style::surface, chip_body(0).Height() / 2)
        , eject(this, chip_body(1), tag_eject, Button::Look::outline, _("Eject"), nullptr, style::surface, chip_body(1).Height() / 2)
        , cut(this, chip_body(2), tag_cut, Button::Look::outline, _("Cut"), nullptr, style::surface, chip_body(2).Height() / 2)
        , pa(this, chip_body(3), tag_pa, Button::Look::outline, {}, nullptr, style::surface, chip_body(3).Height() / 2)
        , close(this, close_body, tag_close, Button::Look::secondary, _("Close"), nullptr, style::surface, 13) {
        SetBackColor(style::surface); // ce que la fiche repeint d'elle-même : seulement sa bande

        const VirtualToolIndex tool = VirtualToolIndex::from_raw(index);
        const FilamentType type = FilamentType::for_tool(tool);
        const bool loaded = VirtualToolIndex::currently_selected_opt() == tool;
        const bool mmu = atelier::mmu_active();
        const bool idle = atelier::can_run();
        // Sans MMU actif, seul le premier emplacement existe : chargement et déchargement de Prusa sans MMU
        const bool reachable = mmu || (index == 0 && tool.is_enabled());
        // Le MMU ne peut ni précharger, ni éjecter, ni couper si un filament va jusqu'à la tête (comme le menu Prusa)
        const bool mmu_free = mmu && idle && !atelier::filament_in_nozzle();

        if (!type && atelier::mmu_enabled()) {
            main_choice = Choice::insert;
            main.set(Button::Look::primary, _("Insert a spool"), &img::spool_16x16);
            main.set_enabled(mmu_free);
        } else if (loaded && type) {
            // Sans MMU, l'outil 0 est toujours choisi : sans matière, c'est un chargement (Prusa demande la matière)
            main_choice = Choice::unload;
            main.set(Button::Look::primary, _("Unload"), &img::arrow_left_16x16);
            main.set_enabled(idle && reachable);
        } else {
            main_choice = Choice::load;
            main.set(Button::Look::primary, _("Load to nozzle"), &img::nozzle_16x16);
            main.set_enabled(idle && reachable);
        }
        main.set_font(Font::normal);

        // La mesure charge elle-même la bobine (MMU actif) ; pas pendant une autre mesure
        calibrate.set_enabled(idle && (loaded || mmu) && !pa::is_running());
        change.set_enabled(mmu_free);
        preload.set_enabled(mmu_free);
        eject.set_enabled(mmu_free);
        cut.set_enabled(mmu_free);
        // F4 : K mesuré de l'emplacement, « PA ? » sans mesure (en jaune si le PA de l'imprimante prime) ; ouvre l'écran
        // du PA, toujours (on y regarde la valeur et la courbe même pendant une impression)
        if (const auto k = pa::k_for_slot(index)) {
            snprintf(pa_text.data(), pa_text.size(), "PA %.3f", double(*k));
        } else {
            strlcpy(pa_text.data(), "PA ?", pa_text.size());
        }
        pa.set(pa::needs_measure(index) ? Button::Look::alert : Button::Look::outline, string_view_utf8::MakeRAM(pa_text.data()));

        // « Reste 350 g » (SPEC_G.md §3.3) quand le poids est suivi : ouvre l'utilisation, où il se corrige après une pesée
        if (const auto report = spool_usage::report(index)) {
            usage.set(Button::Look::secondary, _("Left %d g").formatted(usage_params, int(report->remaining_g)), &img::spool_16x16);
            if (report->low()) {
                usage.set_look(Button::Look::alert);
            }
        }

        if (!type) {
            // Emplacement vide : seul le bouton principal (SPEC_G.md §3.3)
            for (Button *b : { &calibrate, &change, &material, &usage, &preload, &eject, &cut, &pa }) {
                b->set_enabled(false);
                b->set_visible(false);
            }
        }

        // Il faut toujours un élément ciblé (sinon le cadre ignore les touchers) : « Fermer » à défaut
        if (main.IsEnabled()) {
            main.SetFocus();
        } else {
            close.SetFocus();
        }
    }

    SpoolSheet::~SpoolSheet() {
        frost::close();
    }

    void SpoolSheet::unconditionalDraw() {
        // Au-dessus, l'écran d'origine sous l'en-tête, flouté et assombri d'une traite avant la fiche (retour du 22/11 :
        // « le flou arrive après, alors qu'il devait être en même temps ») ; puis seulement la bande, ses coins de la
        // couleur que prend le fond uni
        if (!std::exchange(frosted, true)) {
            frost::run(style::below_header, band);
        }
        display::draw_rounded_rect(band, style::veil, style::surface, band_radius, MIC_TOP_LEFT | MIC_TOP_RIGHT);
#if HAS_TOUCH()
        drawn = 0; // repeinte à sa place : si elle est tirée, le voile se repeint au-dessus d'elle
#endif
    }

#if HAS_TOUCH()
    void SpoolSheet::follow_finger() {
        if (const auto at = touchscreen.held_point()) {
            const point_ui16_t from = touchscreen.held_start();
            if (contact == Contact::none) {
                contact = from.y < top ? Contact::veil : (grab_rect.Contain(from) && !close.GetRect().Contain(from) ? Contact::grab : Contact::other);
            }
            if (contact == Contact::grab) {
                const int16_t dy = int16_t(at->y - from.y);
                pulling = pulling || gesture::pull::grabs(int16_t(at->x - from.x), dy);
                if (pulling) {
                    offset = gesture::pull::offset(dy, max_pull);
                    springing = false;
                }
                gui_reset_loop_timer(); // le doigt est suivi à chaque tour de l'interface, pas toutes les 100 ms
            }
        } else if (contact != Contact::none) {
            // Doigt levé : sur le fond givré, ou la fiche tirée jusqu'au tiers, c'est « Fermer » ; sinon elle remonte
            const bool closing = contact == Contact::veil || (pulling && gesture::pull::closes(offset, band.Height()));
            springing = pulling && !closing;
            contact = Contact::none;
            pulling = false;
            if (closing) {
                close_dialog();
                return;
            }
        }

        if (drawn == offset && !springing) {
            return;
        }
        if (const uint32_t now = ticks_ms(); ticks_diff(now, painted_ms) >= int32_t(gesture::pull::frame_ms)) {
            painted_ms = now;
            if (springing) {
                offset = gesture::pull::spring(drawn);
            }
            paint(offset);
            if (springing && drawn == 0) {
                springing = false;
                restore();
                return;
            }
        }
        gui_reset_loop_timer();
    }

    void SpoolSheet::paint(int16_t to) {
        // Rien que le bord : la fiche entière se redessinerait à chaque image (~60 ms). Le voile uni (la couleur du fond
        // givré) couvre ce que la fiche découvre ; en remontant, le fond de la fiche recouvre le voile.
        const int16_t was = top + drawn, now = top + to;
        if (now > was) {
            display::fill_rect(Rect16::fromLTRB(0, was, style::screen_w, now), style::veil);
        }
        display::draw_rounded_rect(Rect16(0, now, style::screen_w, band_radius), style::veil, style::surface, band_radius, MIC_TOP_LEFT | MIC_TOP_RIGHT);
        if (now < was) {
            display::fill_rect(Rect16::fromLTRB(0, now + band_radius, style::screen_w, was + band_radius), style::surface);
        }
        draw_handle(now);
        drawn = to;
        wiped = std::max<int16_t>(wiped, now + band_radius);
    }

    void SpoolSheet::restore() {
        // Revenue à sa place, une passe : seulement les éléments que le bord a effacés, sur le fond déjà repeint
        const Rect16 rect = Rect16::fromLTRB(0, top, style::screen_w, wiped);
        for (window_t *w = getFirstNormal(); w; w = w->GetNext()) {
            w->Invalidate(rect);
        }
        wiped = top;
    }
#endif

    void SpoolSheet::windowEvent(window_t *sender, GUI_event_t event, void *param) {
#if HAS_TOUCH()
        if (pulling && GUI_event_is_touch_event(event)) {
            return; // le geste qui lâche la fiche ne touche rien dedans : le lever décide (follow_finger)
        }
#endif
        switch (event) {
        case GUI_event_t::LOOP:
#if HAS_TOUCH()
            follow_finger();
#endif
            break;

        case GUI_event_t::CHILD_CLICK:
            switch (static_cast<Tag>(reinterpret_cast<uintptr_t>(param))) {
            case tag_main:
                choice = main_choice;
                break;
            case tag_calibrate:
                choice = Choice::calibrate;
                break;
            case tag_change:
                choice = Choice::change;
                break;
            case tag_material:
                choice = Choice::material;
                break;
            case tag_usage:
                choice = Choice::usage;
                break;
            case tag_preload:
                choice = Choice::preload;
                break;
            case tag_eject:
                choice = Choice::eject;
                break;
            case tag_cut:
                choice = Choice::cut;
                break;
            case tag_pa:
                choice = Choice::pa;
                break;
            case tag_close:
                break;
            }
            close_dialog();
            return;

        case GUI_event_t::HELD_RELEASED:
        case GUI_event_t::TOUCH_SWIPE_LEFT: // le balayage est le retour, comme « Fermer »
        case GUI_event_t::TOUCH_SWIPE_RIGHT:
            close_dialog();
            return;

        case GUI_event_t::TOUCH_CLICK:
            if (event_conversion_union { .pvoid = param }.point.y < top) {
                close_dialog(); // le voile au-dessus de la fiche
                return;
            }
            break;

        default:
            break;
        }
        IDialog::windowEvent(sender, event, param);
    }

} // namespace

void open_sheet(uint8_t index) {
    Choice choice;
    {
        // L'appui long de l'accueil (« Déplacer Z ») fermerait l'accueil au lieu de la fiche : il ferme la fiche
        const auto old_long_press = gui::knob::GetLongPressScreenAction();
        gui::knob::RegisterLongPressScreenAction([] { Screens::Access()->close_dialog(); });
        SpoolSheet sheet(index);
        Screens::Access()->gui_loop_until_dialog_closed();
        gui::knob::RegisterLongPressScreenAction(old_long_press);
        choice = sheet.choice;
    } // la fiche disparaît avant l'action : l'écran de progression se pose sur l'écran d'origine

    switch (choice) {
    case Choice::none:
        break;
    case Choice::load:
        atelier::run(Op::load, index);
        break;
    case Choice::unload:
        atelier::run(Op::unload, index);
        break;
    case Choice::preload:
        atelier::run(Op::preload, index);
        break;
    case Choice::eject:
        atelier::run(Op::eject, index);
        break;
    case Choice::cut:
        atelier::run(Op::cut, index);
        break;
    case Choice::insert:
        open_change_spool(index, SpoolChange::insert);
        break;
    case Choice::change:
        open_change_spool(index, SpoolChange::change);
        break;
    case Choice::material:
        open_change_spool(index, SpoolChange::material);
        break;
    case Choice::usage:
        open_spool_usage(index);
        break;
    case Choice::pa:
        open_pa(index);
        break;
    case Choice::calibrate:
        open_pa(index, true); // l'écran du PA propose la mesure, puis montre ses étapes et « Annuler »
        break;
    }
}

} // namespace od3d
