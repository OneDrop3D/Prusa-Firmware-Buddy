/// @file
#include "screen_home_od3d.hpp"
#include "atelier_od3d.hpp"
#include "layout_od3d.hpp"
#include "screen_files_od3d.hpp"
#include "screen_mmu_od3d.hpp"
#include "screen_printing_od3d.hpp"
#include "sleep_gui_od3d.hpp"
#include "spool_draw.hpp"
#include "spool_sheet_od3d.hpp"

#include <common/marlin_vars.hpp>
#include <common/thumbnail_sizes.hpp>
#include <cstring>
#include <DialogHandler.hpp>
#include <display.hpp>
#include <feature/prusa/MMU2/mmu2_state.h>
#include <gcode/gcode_reader_any.hpp>
#include <gui.hpp>
#include <gui_media_events.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <lfn.h>
#include <marlin_client.hpp>
#include <option/has_human_interactions.h>
#include <option/has_mmu2.h>
#include <option/has_power_panic.h>
#include <option/has_selftest.h>
#include <raii/auto_restore.hpp>
#include <screen_home.hpp>
#include <screen_menu_control.hpp>
#include <tasks.hpp>
#include <transfers/changed_path.hpp>
#include <usb_host.h>

#if HAS_POWER_PANIC()
    #include <power_panic.hpp>
#endif

// FatFS et dirent.h (lazyfilelist) déclarent chacun un DIR : celui de FatFS est renommé, seul le montage est lu ici
#define DIR FATFS_DIR
#include <buddy/fatfs.h>
#undef DIR

namespace od3d {

namespace {

    // Géométrie de l'accueil (SPEC_G.md §3.1), sous l'en-tête de 40 px. Retour du 22/11 : aperçu du dernier fichier plus
    // haut et plus large (UI-8) ; la bande des bobines passe à 56 px et les cartes du bas à 48 px pour lui faire place.
    // La bande prend une rangée jusqu'à 5 bobines, deux au-delà (INDX 8 têtes : 2 × 4).
    constexpr Rect16 spool_area = { style::margin, style::top, 390, 56 };
    constexpr uint8_t spool_gap = 5;
    constexpr Rect16 spool_body(uint8_t i, uint8_t tools) {
        return layout::spool_cell(spool_area, tools, spool_gap, i < tools ? i : 0); // cachées : à la place de la 1re
    }
    constexpr Rect16 mmu_body = { 403, style::top, 69, 56 };
    constexpr Rect16 file_rect = { style::margin, 108, 464, 110 };
    // 313 × 173 de l'aperçu Prusa aux 3/5 : 187 × 103 (156 × 86 avant)
    constexpr uint8_t thumb_num = 3, thumb_den = 5;
    constexpr Rect16 thumb_rect = { 12, file_rect.Top() + 4, thumbnail_sizes::preview_thumbnail_width * thumb_num / thumb_den,
        thumbnail_sizes::preview_thumbnail_height * thumb_num / thumb_den };
    constexpr Rect16 print_body = { 342, 178, 122, 34 };
    constexpr Rect16 files_body = { style::margin, 224, 128, 48 };
    constexpr Rect16 temps_body = { 144, 224, 192, 48 };
    constexpr Rect16 control_body = { 344, 224, 128, 48 };
    constexpr int16_t text_x = thumb_rect.Right() + 1 + 12; ///< colonne du texte de la carte du fichier
    constexpr uint8_t name_chars = (file_rect.Right() + 1 - style::margin - text_x) / 11; ///< nom en gras (11 px)

    static_assert(thumb_rect.Width() == 187 && thumb_rect.Height() == 103 && thumb_rect.Width() <= ScaledQoiReader::max_width);
    static_assert(style::with_ring(spool_body(0, 5)).Bottom() < file_rect.Top() && file_rect.Bottom() < style::with_ring(files_body).Top());
    static_assert(style::inside(style::content, style::with_ring(spool_body(0, 5))) && style::inside(style::content, style::with_ring(mmu_body)));
    static_assert(spool_body(4, 5) == Rect16(324, style::top, 74, 56), "5 bobines du MMU : les colonnes de la maquette");
    static_assert(spool_area.Right() + 1 + 5 == mmu_body.Left() && mmu_body.Right() + 1 == style::screen_w - style::margin);
    static_assert(style::inside(file_rect, style::with_ring(print_body)) && style::inside(file_rect, thumb_rect));
    static_assert(style::with_ring(print_body).Top() > file_rect.Top() + 24 + 19 && name_chars >= 20, "« Imprimer » sous le nom, qui garde 20 caractères");
    static_assert(style::inside(style::content, style::with_ring(files_body)) && style::inside(style::content, style::with_ring(control_body)));
    static_assert(files_body.Right() + 1 + 8 == temps_body.Left() && temps_body.Right() + 1 + 8 == control_body.Left());
    static_assert(layout::small_h == height(Font::small) && layout::normal_h == height(Font::normal));
    constexpr ScaledQoiReader::Params thumb_params { .factor = thumb_den, .back = style::thumb_back, .radius = style::radius_small, .outside = style::surface, .num = thumb_num };

    /// Montage de la clé : FatFS numérote ses montages (0 : pas encore montée). Un autre numéro : une autre clé, ou la même
    /// rebranchée.
    uint16_t usb_mount() {
        return USBHFatFS.fs_type ? USBHFatFS.id : 0;
    }

    /// Change à chaque fichier envoyé ou effacé par PrusaLink ou Connect
    uint32_t usb_changes() {
        return transfers::ChangedPath::instance.change_chain_hash("/usb");
    }

    /// La carte du fichier autour de sa vignette, d'une passe : bandes à gauche, en haut et en bas (coins lissés comme
    /// display::draw_rounded_rect), puis la partie droite. La vignette ne passe plus sur un aplat (case vide, puis image).
    void draw_card_around_thumb() {
        const Rect16 card = file_rect;
        const int r = style::radius_big;
        const auto pixel = [&](int x, int y) { // relatif à la carte
            const int cx = x < r ? x : (x >= card.Width() - r ? card.Width() - 1 - x : -1);
            const int cy = y < r ? y : (y >= card.Height() - r ? card.Height() - 1 - y : -1);
            return cx >= 0 && cy >= 0 ? spool::mix_quarters(style::surface, style::screen, spool::corner_beyond_quarters(cx, cy, r)) : style::surface;
        };
        const int16_t right = thumb_rect.Right() + 1;
        for (const Rect16 band : { Rect16::fromLTRB(card.Left(), card.Top(), right, thumb_rect.Top()),
                 Rect16::fromLTRB(card.Left(), thumb_rect.Top(), thumb_rect.Left(), thumb_rect.Bottom() + 1),
                 Rect16::fromLTRB(card.Left(), thumb_rect.Bottom() + 1, right, card.Bottom() + 1) }) {
            draw_pixels(band, [&](int x, int y) { return pixel(x + band.Left() - card.Left(), y + band.Top() - card.Top()); });
        }
        display::draw_rounded_rect(Rect16::fromLTRB(right, card.Top(), card.Right() + 1, card.Bottom() + 1), style::screen, style::surface, r, MIC_TOP_RIGHT | MIC_BOT_RIGHT);
    }
    static_assert(file_rect.Right() - thumb_rect.Right() >= style::radius_big);

    /// Cartes de bobine (1 à 8 outils) : dans la zone des éléments, sans se chevaucher ni toucher le bouton du MMU et la
    /// carte du fichier, et leur contenu hors des coins arrondis (retour du 22/11 : carte coupée en bas)
    constexpr bool spool_cards_fit(uint8_t tools) {
        for (uint8_t i = 0; i < tools; ++i) {
            const Rect16 card = style::with_ring(spool_body(i, tools));
            if (!style::inside(style::content, card) || layout::overlap(card, style::with_ring(mmu_body)) || layout::overlap(card, file_rect)
                || !layout::spool_parts_fit(spool_body(i, tools), style::radius, tools == 1)) {
                return false;
            }
            for (uint8_t j = 0; j < i; ++j) {
                if (layout::overlap(card, style::with_ring(spool_body(j, tools)))) {
                    return false;
                }
            }
        }
        return true;
    }
    static_assert(spool_cards_fit(1) && spool_cards_fit(2) && spool_cards_fit(3) && spool_cards_fit(4) && spool_cards_fit(5)
        && spool_cards_fit(6) && spool_cards_fit(7) && spool_cards_fit(8));

    /// Coupe s à n caractères affichés, « ... » compris (UTF-8 ; « … » n'est pas dans la police)
    void ellipsize(char *s, size_t n) {
        size_t chars = 0;
        char *cut = nullptr;
        for (char *p = s; *p; ++p) {
            if ((*p & 0xC0) == 0x80) {
                continue; // suite d'un caractère sur plusieurs octets
            }
            if (chars == n - 3) {
                cut = p;
            }
            if (++chars > n) {
                strcpy(cut, "...");
                return;
            }
        }
    }

} // namespace

// ---- Bobines ----

SpoolCard::SpoolCard(window_t *parent, uint8_t index, uint8_t tools)
    : Card(parent, style::with_ring(spool_body(index, tools)), nullptr)
    , slot(Slot::read(index))
    , index(index)
    , single(tools == 1) {
    if (index >= tools) {
        set_visible(false); // outil absent (MMU désactivé, têtes de l'INDX non montées)
        set_enabled(false);
    }
}

void SpoolCard::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if ((event == GUI_event_t::CLICK || event == GUI_event_t::TOUCH_CLICK) && IsEnabled()) {
        open_sheet(index); // la fiche de la bobine (onglet Atelier)
        return;
    }
    Card::windowEvent(sender, event, param);
}

void SpoolCard::update(bool mmu_ready) {
    if (!HasVisibleFlag()) {
        return; // outil absent
    }
    const Slot s = Slot::read(index);
    const bool enabled = s.enabled && mmu_ready;
    if (!(s == slot) || enabled != IsEnabled()) {
        slot = s;
        set_enabled(enabled);
        invalidate_content();
    }
}

void SpoolCard::draw_content(Rect16 body) {
    // Bientôt vide (F13) : en jaune, même chargée (l'anneau orange dit déjà qu'elle est dans la buse)
    const Color color = slot.low && IsEnabled() ? style::warning : (slot.loaded ? style::accent : (IsEnabled() ? style::text_soft : style::text_faint));
    // Chargée dans la buse : anneau orange de 2 px autour du disque ; sinon le carré de l'anneau au fond de la carte
    const std::optional<Color> ring(slot.loaded ? style::accent : fill());
    char label[12];
    snprintf(label, sizeof(label), "%d %s", index + 1, slot.material.data());

    // Disposition selon la place (layout::spool_parts) : rien ne touche les coins arrondis de la carte
    const layout::SpoolParts p = layout::spool_parts(body, radius, single);
    draw_spool(p.spool, slot.look, p.d, fill(), ring);
    if (p.state.IsEmpty()) {
        // Étroite (MMU3) : « n MATIÈRE » centré sous la bobine ; basse (INDX 8 têtes) : à droite de la bobine
        draw_text(p.label, string_view_utf8::MakeRAM(label), Font::small, color, fill(), body.Height() < 40 ? Align_t::LeftCenter() : Align_t::Center());
        return;
    }
    // Large : seule bobine, la matière en gras ; trois têtes (~60 px), « n MATIÈRE » en petit
    draw_text(p.label, string_view_utf8::MakeRAM(single ? slot.material.data() : label), single ? Font::normal : Font::small, color, fill());
    char state_text[16];
    Color state_color;
    const string_view_utf8 state = atelier::slot_state(slot, index, state_text, state_color);
    draw_text(p.state, state, Font::small, IsEnabled() ? state_color : style::text_faint, fill());
}

// ---- Dernier fichier ----

FileCard::State FileCard::state = FileCard::State::no_usb;
std::array<char, filename_defs::path_buffer_size> FileCard::path {};
std::array<char, 64> FileCard::name {};

FileCard::FileCard(window_t *parent)
    : window_t(parent, file_rect) {}

FileCard::~FileCard() {
    big_thumb::stop(scaler);
    big_thumb::release(); // l'accueil relira sa vignette au retour plutôt que de garder le tas
}

bool FileCard::set(bool usb, const char *latest) {
    const State s = !usb ? State::no_usb : (latest ? State::file : State::no_file);
    if (s == state && (!latest || strcmp(latest, path.data()) == 0)) {
        return false;
    }
    state = s;
    path[0] = name[0] = '\0';
    if (latest) {
        strlcpy(path.data(), latest, path.size());
        std::array<char, filename_defs::filename_buffer_size> lfn;
        get_LFN(lfn.data(), lfn.size(), path.data());
        if (char *dot = strrchr(lfn.data(), '.'); dot && dot != lfn.data()) {
            *dot = '\0'; // sans « .bgcode »
        }
        ellipsize(lfn.data(), name_chars);
        strlcpy(name.data(), lfn.data(), name.size());
    }
    Invalidate();
    return true;
}

void FileCard::print() const {
    if (print_active()) {
        open_screen<ScreenPrinting>(); // « Voir » : l'impression en cours
    } else if (state == State::file) {
        open_preview(path.data()); // « Avant l'impression » ; « Imprimer » y passe par print_start de Prusa
    }
}

void FileCard::unconditionalDraw() {
    const Rect16 card = GetRect();
    big_thumb::stop(scaler); // une lecture commencée reprend du début
    loading = false;
    // Vignette gardée (retour à l'accueil) : la carte autour d'elle, puis l'image, sans lire la clé
    thumb_drawn = state == State::file && big_thumb::cached(path.data(), thumb_params);
    if (thumb_drawn) {
        draw_card_around_thumb();
        big_thumb::draw_cached(path.data(), thumb_params, point_ui16(thumb_rect.Left(), thumb_rect.Top()), scaler);
    } else {
        // La carte, la case vide de la vignette (elle arrive par bandes aux LOOP suivants) et les textes
        display::draw_rounded_rect(card, style::screen, style::surface, style::radius_big, MIC_ALL_CORNERS);
        display::draw_rounded_rect(thumb_rect, style::surface, style::thumb_back, style::radius_small, MIC_ALL_CORNERS);
        if (state != State::file) {
            draw_icon({ int16_t(thumb_rect.Left() + (thumb_rect.Width() - 16) / 2), int16_t(thumb_rect.Top() + (thumb_rect.Height() - 16) / 2) }, img::folder_full_16x16, style::thumb_back);
        }
        thumb_drawn = state != State::file;
    }

    const int16_t x = thumb_rect.Left() + thumb_rect.Width() + 12;
    const uint16_t w = card.Left() + card.Width() - style::margin - x;
    draw_text(Rect16(x, card.Top() + 6, w, height(Font::small)), _("Latest file"), Font::small, style::text_muted, style::surface);
    const string_view_utf8 line = state == State::file
        ? string_view_utf8::MakeRAM(name.data())
        : (state == State::no_file ? _("No printable file") : _("Insert a USB drive"));
    draw_text(Rect16(x, card.Top() + 24, w, height(Font::normal)), line, Font::normal, state == State::file ? style::text_strong : style::text_muted, style::surface);
}

void FileCard::load_step() {
    if (thumb_drawn || !IsVisible()) {
        return;
    }
    if (!IsInvalid()) {
        // Par bandes de 8 lignes, comme la grille : l'accueil répond pendant la lecture (avant : figé d'un bloc). Pendant
        // une impression, la clé est à elle : l'icône.
        const bool was_loading = loading;
        if (print_uses_usb()) {
            big_thumb::stop(scaler);
            loading = false;
        } else {
            loading = loading ? big_thumb::step(scaler, point_ui16(thumb_rect.Left(), thumb_rect.Top())) : big_thumb::start(path.data(), thumb_params, scaler);
        }
        thumb_drawn = !loading;
        if (thumb_drawn && (!was_loading || print_uses_usb())) {
            draw_icon({ int16_t(thumb_rect.Left() + (thumb_rect.Width() - 16) / 2), int16_t(thumb_rect.Top() + (thumb_rect.Height() - 16) / 2) }, img::folder_full_16x16, style::thumb_back);
        }
    }
    if (!thumb_drawn) {
        gui_reset_loop_timer(); // la carte à dessiner d'abord, ou la bande suivante : sans attendre 100 ms
    }
}

PrintButton::PrintButton(window_t *parent, FileCard &file)
    : Card(parent, style::with_ring(print_body), nullptr, style::accent, style::surface)
    , file(file) {}

Rect16 PrintButton::get_rect_for_touch() const {
    return file.GetRect();
}

void PrintButton::draw_content(Rect16 body) {
    // « Print » est déjà traduit « Impression » chez Prusa, d'où une autre clé
    const string_view_utf8 label = _("Print now");
    const uint16_t text_w = label.computeNumUtf8Chars() * width(Font::small);
    const int16_t x = body.Left() + (body.Width() - 16 - 6 - text_w) / 2;
    const int16_t y = body.Top() + (body.Height() - 16) / 2;
    draw_icon({ x, y }, img::print_16x16, fill(), ropfn { .swap_bw = has_swapped_bw::yes });
    draw_text(Rect16(x + 16 + 6, y, text_w, height(Font::small)), label, Font::small, style::on_accent, fill());
}

void PrintButton::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if ((event == GUI_event_t::CLICK || event == GUI_event_t::TOUCH_CLICK) && IsEnabled()) {
        file.print();
        return;
    }
    Card::windowEvent(sender, event, param);
}

// ---- Écran ----

bool ScreenHome::usb_was_already_inserted = false;
bool ScreenHome::need_check_wifi_credentials = true;
uint16_t ScreenHome::scanned_mount = 0;
uint32_t ScreenHome::scanned_changes = 0;

ScreenHome::ScreenHome()
    : ScreenHome(std::make_index_sequence<VirtualToolIndex::count>()) {}

template <size_t... I>
ScreenHome::ScreenHome(std::index_sequence<I...>)
    : Screen(false)
    , header(this, _("Home"), false)
    , spools { SpoolCard(this, I, atelier::visible_tools())... }
    // L'étagère de l'onglet Atelier ; « MMU > » seulement avec un MMU activé (sans lui, l'icône seule)
#if HAS_MMU2()
    , mmu(this, style::with_ring(mmu_body), open_screen<ScreenMMU>, img::settings_16x16, atelier::mmu_enabled() ? string_view_utf8::MakeCPUFLASH("MMU >") : string_view_utf8 {})
#else
    , mmu(this, style::with_ring(mmu_body), open_filament_menu, img::settings_16x16, string_view_utf8 {}) // INDX : menu du filament de Prusa, par tête
#endif
    , file(this)
    , print(this, file)
    , files(this, files_body, open_files, img::folder_full_16x16, style::tile, _("Files"), string_view_utf8::MakeCPUFLASH("USB"))
    , temps(this, temps_body)
    , control(this, control_body, open_screen<ScreenMenuControl>, img::calibrate_white_16x16, style::tile_motion, _("Control"), string_view_utf8::MakeCPUFLASH("Axes"))
    , tabs(this, TabBar::Tab::home)
    , usb_inserted(marlin_vars().media_inserted) {
    // Comme l'accueil Prusa : appui long = « Déplacer Z », pas de fermeture par délai ni par une impression série
    EnableLongHoldScreenAction();
    ClrMenuTimeoutClose();
    ClrOnSerialClose();
    screen_home_data_t::SetEverBeenOpened(); // le tactile attend l'ouverture de l'accueil (touchscreen_gt911.cpp)

    if (usb_inserted) {
        usb_was_already_inserted = true;
    }
    // Le dernier fichier connu tout de suite ; la clé n'est relue (au LOOP, l'écran affiché) que si elle a changé
    print.set_visible(FileCard::has_file());
    print.set_enabled(FileCard::has_file());
    if (!usb_inserted || !FileCard::known()) {
        refresh_file();
    }
    update_spools();

    // Sans élément ciblé, le cadre ignore les touchers : il en faut toujours un
    spools[0].SetFocus();
    if (!get_focused_child_window()) {
        temps.SetFocus();
    }
}

ScreenHome::~ScreenHome() {
    GuiMediaEventsHandler::ConsumeOneClickPrinting();
}

bool ScreenHome::file_stale() const {
    return usb_inserted && (!FileCard::known() || usb_mount() != scanned_mount || usb_changes() != scanned_changes);
}

void ScreenHome::refresh_file() {
    if (usb_inserted && print_uses_usb()) {
        return; // la clé est à l'impression : relue après (file_stale)
    }
    if (usb_inserted && usb_mount() != scanned_mount) {
        forget_file_infos(); // peut-être une autre clé : métadonnées et vignettes gardées oubliées
    }
    std::array<char, filename_defs::path_buffer_size> latest;
    const bool found = usb_inserted && screen_home_data_t::find_latest_gcode(latest.data(), latest.size());
    scanned_mount = usb_mount(); // après la lecture : elle monte la clé
    scanned_changes = usb_changes();
    if (file.set(usb_inserted, found ? latest.data() : nullptr)) {
        print.Invalidate(); // la carte s'est redessinée dessous
    }
    print.set_visible(found);
    print.set_enabled(found);
    files.set_subtitle(usb_inserted ? string_view_utf8::MakeCPUFLASH("USB") : _("No USB"));
    if (!found && print.IsFocused()) {
        temps.SetFocus();
    }
}

void ScreenHome::update_spools() {
    const auto state = MMU2::xState(marlin_vars().mmu2_state.get());
    const bool mmu_ready = state != MMU2::xState::Connecting && state != MMU2::xState::Bootloader;
    for (SpoolCard &spool : spools) {
        spool.update(mmu_ready);
    }
    mmu.set_enabled(mmu_ready);
    // Un élément désactivé ne doit pas garder le focus (comme le bouton Filament de l'accueil Prusa)
    if (window_t *focused = get_focused_child_window(); focused && !focused->IsEnabled()) {
        temps.SetFocus();
    }
}

#if HAS_NFC()
void ScreenHome::update_nfc_state() {
    // Comme l'accueil Prusa : le préchauffage s'ouvre par-dessus l'accueil sans le remplacer
    if (GetLastDialog()) {
        nfc_enable.reset();
    } else if (!nfc_enable) {
        nfc_enable.emplace();
    }
}
#endif

void ScreenHome::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    // Reprise de screen_home_data_t::windowEvent
    if (event == GUI_event_t::MEDIA) { // gardé aussi pendant la récursion
        media_event = MediaState_t(int(param));
    }

    if (event_in_progress) {
        return;
    }
    AutoRestore avoid_recursion(event_in_progress, true);

    if (media_event != MediaState_t::unknown) {
        if (media_event == MediaState_t::inserted && !usb_inserted) {
            usb_inserted = true;
            refresh_file();
            if (!usb_was_already_inserted) {
                usb_was_already_inserted = true;
                print.SetFocus(); // première clé depuis le démarrage : « Imprimer » ciblé
            }
        } else if (media_event == MediaState_t::removed && usb_inserted) {
            usb_inserted = false;
            refresh_file();
        }
        media_event = MediaState_t::unknown;
    }

    if (event == GUI_event_t::LOOP) {
        if (file_stale()) {
            refresh_file(); // autre clé, ou fichier envoyé ou effacé par le réseau ; sinon le dossier n'est pas relu
        }
        if (!GetFirstDialog() && !sleep::screen_off()) {
            file.load_step();
        }
        update_spools();

#if HAS_POWER_PANIC()
        if (TaskDeps::check(TaskDeps::Dependency::autostart_done) && !power_panic::is_power_panic_resuming())
#endif
        { // à chaque clé insérée, on cherche les identifiants Wi-Fi (.ini)
            if (usb_inserted) {
                if (need_check_wifi_credentials) {
                    need_check_wifi_credentials = false;
                    screen_home_data_t::handle_wifi_credentials();
                }
            } else {
                need_check_wifi_credentials = true;
            }
        }

#if HAS_SELFTEST()
        // Impression en un clic : le dernier fichier de la clé qu'on vient d'insérer
        if (!DialogHandler::Access().IsOpen()) {
            if (HAS_HUMAN_INTERACTIONS() &&
    #if HAS_POWER_PANIC()
                TaskDeps::check(TaskDeps::Dependency::autostart_done) && !power_panic::is_power_panic_resuming() &&
    #endif
                GuiMediaEventsHandler::ConsumeOneClickPrinting() && !usbh_power_cycle::block_one_click_print()) {
                std::array<char, filename_defs::path_buffer_size> filepath;
                if (screen_home_data_t::find_latest_gcode(filepath.data(), filepath.size())) {
                    open_preview(filepath.data()); // comme l'aperçu Prusa : rien ne part sans « Imprimer »
                }
            }
        }
#endif
    }

    Screen::windowEvent(sender, event, param);

#if HAS_NFC()
    update_nfc_state();
#endif
}

} // namespace od3d
