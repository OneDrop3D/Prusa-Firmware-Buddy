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
#include <optional>
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
    // « Fermer » en haut à gauche, comme le retour de nos écrans (retour de l'utilisateur, 23/11 : « pas de retour en
    // haut à droite ») ; le haut de la fiche (bobine, nom, état) prend la droite
    constexpr Rect16 close_body = { 16, top + 16, 67, 28 };
    constexpr Rect16 head_rect = Rect16::fromLTRB(close_body.Right() + 1 + style::ring + 2, top, style::screen_w, top + 55);
    constexpr uint8_t band_radius = style::radius; ///< arrondi normé (18 avant)
    // Le grand bouton (charger / décharger) partage sa ligne avec « Calibrer le PA » (F4, en un clic)
    constexpr Rect16 main_body = { 16, top + 60, 288, 42 };
    constexpr Rect16 calibrate_body = { 312, top + 60, 152, 42 };
    static_assert(style::with_ring(close_body).Right() < head_rect.Left(), "le haut de la fiche ne repeint pas « Fermer »");
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
    /// Trois puces (Précharger, Éjecter, Couper) de la largeur des boutons du dessus : le PA a rejoint « Calibrer le PA »,
    /// et les puces de 106 × 32 étaient petites au doigt (retour du 24/11)
    constexpr Rect16 chip_body(int i) {
        return Rect16(16 + 152 * i, top + 156, 144, 36);
    }
    static_assert(style::inside(band, style::with_ring(chip_body(2))) && style::inside(band, style::with_ring(secondary_body(2))));
    static_assert(head_rect.Bottom() < style::with_ring(main_body).Top() && style::with_ring(main_body).Bottom() < style::with_ring(secondary_body(0)).Top());
    static_assert(style::with_ring(main_body).Right() < style::with_ring(calibrate_body).Left() && style::inside(band, style::with_ring(calibrate_body)));
    static_assert(style::with_ring(secondary_body(0)).Bottom() < style::with_ring(chip_body(0)).Top());

    /// Poignée (« tige ») de la fiche dont le haut est en y
    constexpr Rect16 handle_rect(int16_t y) {
        return Rect16(222, y + 6, 36, 4);
    }
    void draw_handle(int16_t y) {
        display::draw_rounded_rect(handle_rect(y), style::surface, style::handle, 2, MIC_ALL_CORNERS);
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
            const VirtualToolIndex tool = VirtualToolIndex::from_raw(index);
            const FilamentType type = FilamentType::for_tool(tool);
            const Slot slot = Slot::read(index);
            constexpr uint8_t d = 40;
            const point_i16_t at { int16_t(r.Left() + 6), top + 13 };
            const int16_t x = at.x + d + 10;
            const uint16_t w = r.Left() + r.Width() - 16 - x; // marge de droite, comme les boutons
            const Rect16 name_rect(x, top + 14, w, height(Font::normal));
            const Rect16 status_rect(x, top + 36, w, height(Font::small));

            // D'une passe : le corps autour de ce qui suit, puis chaque morceau (chacun repeint sa place). Son coin
            // arrondi (en haut à droite, celui de la fiche) est lissé sur le voile.
            const Rect16 corner(r.Left() + r.Width() - band_radius, r.Top(), band_radius, band_radius);
            const Rect16 spool_rect = slot.loaded ? Rect16(at.x - 2, at.y - 2, d + 4, d + 4) : Rect16(at.x, at.y, d, d);
            const Rect16 parts[] = { corner, handle_rect(top), spool_rect, name_rect, status_rect };
            style::for_each_gap(r, parts, [](Rect16 gap) { display::fill_rect(gap, style::surface); });
            display::draw_rounded_rect(corner, style::veil, style::surface, band_radius, MIC_TOP_RIGHT);
            draw_handle(top);
            draw_spool(at, slot.look, d, style::surface, slot.loaded ? std::optional(style::accent) : std::nullopt);

            char name[24];
            atelier::spool_name(name, type);
            StringViewUtf8Parameters<32> params;
            draw_text(name_rect, _("Slot %d · %s").formatted(params, index + 1, name), Font::normal, style::text_strong, style::surface);

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
            draw_text(status_rect, string_view_utf8::MakeRAM(status), Font::small, tip_drift ? style::warning : style::text_muted, style::surface);
        }

    private:
        uint8_t index;
    };

    class SpoolSheet : public IDialog {
    public:
        explicit SpoolSheet(uint8_t index);

        Choice choice = Choice::none;

        Rect16 get_rect_for_touch() const override {
            return style::screen_rect; // un toucher sur l'en-tête aussi ferme la fiche
        }

    protected:
        /// À repeindre en entier : la fiche s'ouvre depuis la bobine touchée sur le voile (frost_od3d.hpp), chaque
        /// élément peint une fois ; sinon les éléments changés
        void draw() override;
        void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

    private:
#if HAS_TOUCH()
        /// Fiche tirée vers le bas par sa poignée, toucher du voile : suivis à chaque tour de l'interface
        void follow_finger();
        /// Fiche peinte à offset sous sa place : voile au-dessus, bord arrondi et poignée ; ce que le bord a effacé
        /// dans la fiche se redessine quand elle est revenue (restore)
        void paint(int16_t offset);
        void restore();

        enum class Contact : uint8_t {
            none,
            veil, ///< posé sur le voile (ou l'en-tête) : ferme la fiche au lever
            grab, ///< posé sur la poignée ou le haut de la fiche
            other,
        };
        Touchscreen_Base::LenientClickGuard lenient; ///< un toucher qui bouge de 3 px reste un toucher (voile, boutons)
        Contact contact = Contact::none;
        bool pulling = false; ///< la fiche suit le doigt : aucun toucher ni balayage n'agit dedans
        bool springing = false; ///< lâchée avant le tiers : elle remonte
        int16_t offset = 0; ///< décalage voulu
        int16_t drawn = 0; ///< décalage peint
        int16_t wiped = top; ///< bas de ce que le bord a effacé dans la fiche
        uint32_t painted_ms = 0;
#endif
        Rect16 from = frost::origin(); ///< la bobine touchée, d'où s'ouvre la fiche (sa place une fois ouverte)
        StringViewUtf8Parameters<8> usage_params; ///< « Reste 350 g », avant le bouton qui garde le texte formaté
        std::array<char, 12> pa_text {}; ///< « PA 0.035 » (F4), avant le bouton qui le montre
        Head head;
        Button main;
        /// F4 : un seul bouton du PA (retour du 24/11 : « Calibrer le PA » et la puce « PA ? » ouvraient le même écran).
        /// « PA 0.035 » quand l'emplacement est mesuré (l'écran du PA, sa courbe, « Mesurer ») ; sinon « Calibrer le PA »,
        /// qui propose la mesure en un clic (chargement compris)
        Button calibrate;
        Button change;
        Button material;
        Button usage;
        Button preload;
        Button eject;
        Button cut;
        Button close; ///< dernier à la molette (ordre de la maquette), mais toujours atteignable
        Choice main_choice;
        Choice pa_choice = Choice::calibrate;
    };

    SpoolSheet::SpoolSheet(uint8_t index)
        : IDialog(style::below_header) // l'écran dessous ne se redessine plus (il est voilé) ; l'en-tête reste vivant
        , head(this, index)
        , main(this, main_body, tag_main, Button::Look::primary, {}, nullptr, style::surface, style::radius_big)
        , calibrate(this, calibrate_body, tag_calibrate, Button::Look::secondary, _("Calibrate PA"), nullptr, style::surface, style::radius_big)
        , change(this, secondary_body(0), tag_change, Button::Look::secondary, _("Change spool"), &img::spool_16x16)
        , material(this, secondary_body(1), tag_material, Button::Look::secondary, _("Material and colour"), &img::user_filament_16x16)
        , usage(this, secondary_body(2), tag_usage, Button::Look::secondary, _("Spool usage"), &img::info_16x16)
        , preload(this, chip_body(0), tag_preload, Button::Look::outline, _("Preload"), nullptr, style::surface, chip_body(0).Height() / 2)
        , eject(this, chip_body(1), tag_eject, Button::Look::outline, _("Eject"), nullptr, style::surface, chip_body(1).Height() / 2)
        , cut(this, chip_body(2), tag_cut, Button::Look::outline, _("Cut"), nullptr, style::surface, chip_body(2).Height() / 2)
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

        change.set_enabled(mmu_free);
        preload.set_enabled(mmu_free);
        eject.set_enabled(mmu_free);
        cut.set_enabled(mmu_free);
        // F4 : K mesuré de l'emplacement : « PA 0.035 », qui ouvre l'écran du PA, toujours (on y regarde la valeur et la
        // courbe même pendant une impression). Sans mesure : « Calibrer le PA » (en jaune si le PA de l'imprimante prime),
        // la mesure en un clic, qui charge elle-même la bobine (MMU actif) ; pas pendant une autre mesure
        const auto k = pa::k_for_slot(index);
        const Button::Look pa_look = pa::needs_measure(index) ? Button::Look::alert : Button::Look::secondary;
        if (k) {
            snprintf(pa_text.data(), pa_text.size(), "PA %.3f", double(*k));
            pa_choice = Choice::pa;
            calibrate.set(pa_look, string_view_utf8::MakeRAM(pa_text.data()));
        } else {
            const bool can_measure = idle && (loaded || mmu) && !pa::is_running();
            pa_choice = can_measure ? Choice::calibrate : Choice::pa;
            // M-02 : un K mesuré avec une autre buse ou une autre matière ne sert pas
            calibrate.set(pa_look, pa::stale(index) ? _("Remeasure PA") : _("Calibrate PA"));
        }

        // Le reste est déjà écrit en haut de la fiche (« ~119 g ») : le bouton garde « Utilisation » (retour du 24/11,
        // « Reste 119 g » le répétait), en jaune quand la bobine passe sous son seuil d'alerte
        if (const auto report = spool_usage::report(index); report && report->low()) {
            usage.set_look(Button::Look::alert);
        }

        if (!type) {
            // Emplacement vide : seul le bouton principal (SPEC_G.md §3.3)
            for (Button *b : { &calibrate, &change, &material, &usage, &preload, &eject, &cut }) {
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

    void SpoolSheet::draw() {
        if (!IsVisible() || !IsInvalid()) {
            IDialog::draw();
            return;
        }
        static_assert(band_radius == frost::radius);
        frost::draw(getFirstNormal(), band, MIC_TOP_LEFT | MIC_TOP_RIGHT, std::exchange(from, band));
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
            // Doigt levé : sur le voile, ou la fiche tirée jusqu'au tiers, c'est « Fermer » ; sinon elle remonte
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
        // Rien que le bord : la fiche entière se redessinerait à chaque image (~60 ms). Le voile couvre ce que la fiche
        // découvre ; en remontant, le fond de la fiche recouvre le voile.
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
                choice = pa_choice;
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
    } // la fiche disparaît avant l'action

    std::optional<Op> op;
    switch (choice) {
    case Choice::none:
        break;
    case Choice::load:
        op = Op::load;
        break;
    case Choice::unload:
        op = Op::unload;
        break;
    case Choice::preload:
        op = Op::preload;
        break;
    case Choice::eject:
        op = Op::eject;
        break;
    case Choice::cut:
        op = Op::cut;
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
    // Après un écran ouvert : rien n'est repeint dessous, il se dessine à sa place. Avant l'opération : un message de
    // Prusa (imprimante occupée) se pose sur l'écran repeint, et le chargement s'ouvre depuis la bobine.
    frost::close();
    if (op) {
        atelier::run(*op, index);
    }
}

} // namespace od3d
