/// @file
#include "spool_sheet_od3d.hpp"

#include "atelier_od3d.hpp"
#include "screen_spool_od3d.hpp"

#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <feature/od3d/spool_usage_od3d.hpp>
#include <i18n.h>
#include <IDialog.hpp>
#include <img_resources.hpp>
#include <knob_event.hpp>
#include <ScreenHandler.hpp>

namespace od3d {

namespace {

    using atelier::Op;

    // Géométrie de la fiche (SPEC_G.md §3.3), resserrée : elle commence sous la bande des bobines de l'accueil (jusqu'à
    // y 111 avec sa marge) au lieu de la recouvrir de 2 px, ce qui coupait leurs coins du bas
    constexpr int16_t top = 114;
    constexpr Rect16 band = { 0, top, style::screen_w, style::screen_h - top };
    constexpr Rect16 head_rect = { 0, top + 1, 393, 54 };
    constexpr uint8_t band_radius = style::radius; ///< arrondi normé (18 avant)
    constexpr Rect16 main_body = { 16, top + 60, 448, 42 };
    constexpr Rect16 close_body = { 397, top + 16, 67, 28 };

    enum Tag : uint8_t {
        tag_main,
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
    };

    constexpr Rect16 secondary_body(int i) {
        return Rect16(16 + 152 * i, top + 110, 144, 38);
    }
    constexpr Rect16 chip_body(int i) {
        return Rect16(16 + 114 * i, top + 156, 106, 32);
    }
    static_assert(style::inside(band, style::with_ring(chip_body(3))) && style::inside(band, style::with_ring(secondary_body(2))));
    static_assert(head_rect.Bottom() < style::with_ring(main_body).Top() && style::with_ring(main_body).Bottom() < style::with_ring(secondary_body(0)).Top());
    static_assert(style::with_ring(secondary_body(0)).Bottom() < style::with_ring(chip_body(0)).Top());

    /// Haut de la fiche : poignée, bobine, « Emplacement n · matière », état ; rien d'actif
    class Head : public window_t {
    public:
        Head(window_t *parent, uint8_t index)
            : window_t(parent, head_rect)
            , index(index) {}

    protected:
        void unconditionalDraw() override {
            const Rect16 r = GetRect();
            display::fill_rect(r, style::surface);
            display::draw_rounded_rect(Rect16(222, top + 6, 36, 4), style::surface, Color::from_raw(0x5B5B5B), 2, MIC_ALL_CORNERS);

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

            // État, la température de la matière, puis le reste (F13 ou étiquette OpenPrintTag)
            char status[56];
            const char *state = slot.loaded ? N_("In the nozzle") : (type ? N_("In the MMU") : N_("Empty"));
            _(state).copyToRAM(status, sizeof(status));
            if (type) {
                const FilamentTypeParameters p = type.parameters();
                const size_t len = strlen(status);
                snprintf(status + len, sizeof(status) - len, " · %d/%d\xC2\xB0", p.nozzle_temperature, p.heatbed_temperature);
            }
            if (const auto usage = spool_usage::report(index); usage && type) {
                const size_t len = strlen(status);
                snprintf(status + len, sizeof(status) - len, " · ~%d g", int(usage->remaining_g));
            }
            draw_text(Rect16(x, top + 36, w, height(Font::small)), string_view_utf8::MakeRAM(status), Font::small, style::text_muted, style::surface);
        }

    private:
        uint8_t index;
    };

    class SpoolSheet : public IDialog {
    public:
        explicit SpoolSheet(uint8_t index);

        Choice choice = Choice::none;

    protected:
        void unconditionalDraw() override;
        void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

    private:
        StringViewUtf8Parameters<8> usage_params; ///< « Reste 350 g », avant le bouton qui garde le texte formaté
        Head head;
        Button main;
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
        : IDialog(style::screen_rect) // tout l'écran : un toucher au-dessus de la fiche la ferme
        , head(this, index)
        , main(this, main_body, tag_main, Button::Look::primary, {}, nullptr, style::surface, style::radius_big)
        , change(this, secondary_body(0), tag_change, Button::Look::secondary, _("Change spool"), &img::spool_16x16)
        , material(this, secondary_body(1), tag_material, Button::Look::secondary, _("Material and colour"), &img::user_filament_16x16)
        , usage(this, secondary_body(2), tag_usage, Button::Look::secondary, _("Spool usage"), &img::info_16x16)
        , preload(this, chip_body(0), tag_preload, Button::Look::outline, _("Preload"), nullptr, style::surface, chip_body(0).Height() / 2)
        , eject(this, chip_body(1), tag_eject, Button::Look::outline, _("Eject"), nullptr, style::surface, chip_body(1).Height() / 2)
        , cut(this, chip_body(2), tag_cut, Button::Look::outline, _("Cut"), nullptr, style::surface, chip_body(2).Height() / 2)
        , pa(this, chip_body(3), tag_pa, Button::Look::alert, _("PA: soon"), nullptr, style::surface, chip_body(3).Height() / 2)
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

        if (!type) {
            main_choice = Choice::insert;
            main.set(Button::Look::primary, _("Insert a spool"), &img::spool_16x16);
            main.set_enabled(mmu_free);
        } else if (loaded) {
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
        pa.set_enabled(false); // F4 (mesure du PA) n'existe pas encore

        // « Reste 350 g » (SPEC_G.md §3.3) quand le poids est suivi : ouvre l'utilisation, où il se corrige après une pesée
        if (const auto report = spool_usage::report(index)) {
            usage.set(Button::Look::secondary, _("Left %d g").formatted(usage_params, int(report->remaining_g)), &img::spool_16x16);
            if (report->low()) {
                usage.set_look(Button::Look::alert);
            }
        }

        if (!type) {
            // Emplacement vide : seul le bouton principal (SPEC_G.md §3.3)
            for (Button *b : { &change, &material, &usage, &preload, &eject, &cut, &pa }) {
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

    void SpoolSheet::unconditionalDraw() {
        // Seulement la bande : au-dessus, l'écran d'origine reste tel quel
        display::draw_rounded_rect(band, style::screen, style::surface, band_radius, MIC_TOP_LEFT | MIC_TOP_RIGHT);
    }

    void SpoolSheet::windowEvent(window_t *sender, GUI_event_t event, void *param) {
        switch (event) {
        case GUI_event_t::CHILD_CLICK:
            switch (static_cast<Tag>(reinterpret_cast<uintptr_t>(param))) {
            case tag_main:
                choice = main_choice;
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
            case tag_close:
                break;
            }
            close_dialog();
            return;

        case GUI_event_t::HELD_RELEASED:
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
    }
}

} // namespace od3d
