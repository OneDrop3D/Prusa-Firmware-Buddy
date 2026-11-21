/// @file
#include "screen_files_od3d.hpp"

#include <algorithm>
#include <bit>
#include <common/marlin_vars.hpp>
#include <common/thumbnail_sizes.hpp>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <display_helper.h>
#include <gcode/gcode_reader_any.hpp>
#include <gui/event/gui_event.hpp>
#include <gui/event/knob_event.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <lfn.h>
#include <marlin_client.hpp>
#include <media_state.hpp>
#include <strings.h>

namespace od3d {

namespace {

    using print_info::FileInfo;

    constexpr uint16_t thumb_w = thumbnail_sizes::preview_thumbnail_width;
    constexpr uint16_t thumb_h = thumbnail_sizes::preview_thumbnail_height;

    // Grille (SPEC_G.md §3.9) : 4 cartes de 228 × 114, haut de 72 px pour la vignette (313 × 173 réduite au tiers)
    constexpr Rect16 tile_body(int i) {
        return Rect16(style::margin + (i % 2) * 236, 32 + (i / 2) * 120, 228, 114);
    }
    constexpr uint16_t tile_thumb_h = 72;
    constexpr uint8_t grid_factor = 3;
    constexpr Rect16 grid_rect = Rect16::fromLTRB(style::margin, 32, style::screen_w - style::margin, 266);
    static_assert(thumb_w / grid_factor <= tile_body(0).Width() && thumb_h / grid_factor <= tile_thumb_h);
    static_assert(style::inside(style::content, style::with_ring(tile_body(3))));

    // Avant l'impression (§3.10)
    constexpr Rect16 preview_box = { style::margin, 32, 212, 146 };
    constexpr uint8_t preview_factor = 2;
    constexpr int16_t info_x = 230;
    constexpr Rect16 print_body = { style::margin, 220, 228, 44 };
    constexpr Rect16 map_body = { 244, 220, 228, 44 };
    static_assert(thumb_w / preview_factor <= preview_box.Width() && thumb_h / preview_factor <= preview_box.Height());

    constexpr const char *dot = " \xC2\xB7 "; ///< « · » : dans la police grâce au titre « Fichiers · clé USB » du .po

    /// Coupe s à n caractères affichés, « ... » compris (UTF-8)
    void ellipsize(char *s, size_t n) {
        size_t chars = 0;
        char *cut = nullptr;
        for (char *p = s; *p; ++p) {
            if ((*p & 0xC0) == 0x80) {
                continue;
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

    /// Nom affiché d'un fichier : sans extension, coupé à n caractères
    void display_name(const char *name, bool strip_extension, std::span<char> out, size_t n) {
        strlcpy(out.data(), name, out.size());
        if (char *ext = strrchr(out.data(), '.'); strip_extension && ext && ext != out.data()) {
            *ext = '\0';
        }
        ellipsize(out.data(), std::min(n, out.size() - 1));
    }

    /// « 2 h 14 · 38 g », plus « · 12.6 m » si with_meters
    void summary(const FileInfo &info, bool with_meters, std::span<char> out) {
        size_t len = 0;
        out[0] = '\0';
        if (info.seconds) {
            print_info::format_hm(info.seconds, out.data(), out.size());
            len = strlen(out.data());
        }
        const auto append = [&](const char *fmt, float value) {
            len += snprintf(out.data() + len, out.size() - len, fmt, len ? dot : "", double(value));
            len = std::min(len, out.size() - 1);
        };
        if (info.used) {
            append("%s%.0f g", info.total_grams());
        }
        if (with_meters && info.millimeters > 0) {
            append("%s%.1f m", info.millimeters / 1000);
        }
    }

    bool can_print() {
        // Comme le serveur : un nouveau départ est accepté aussi après une impression finie ou arrêtée
        return marlin_vars().media_inserted && !marlin_client::is_printing();
    }

    void start_print(marlin_client::PreviewSkipIfAble skip) {
        if (can_print()) {
            // Départ normal de Prusa : chargement, vérifications, questions sur le filament, association des outils
            marlin_client::print_start(ScreenPreview::path.data(), skip);
        }
    }

    void print_now() {
        // Notre écran remplace l'aperçu Prusa ; l'association des outils ne s'affiche que si Prusa trouve un écart
        start_print(marlin_client::PreviewSkipIfAble::tool_mapping);
    }

    void map_colours() {
        start_print(marlin_client::PreviewSkipIfAble::preview); // l'association des outils de Prusa, toujours
    }

} // namespace

void open_preview(const char *path) {
    strlcpy(ScreenPreview::path.data(), path, ScreenPreview::path.size());
    open_screen<ScreenPreview>();
}

FileInfo read_file_info(const char *path) {
    FileInfo info;
    AnyGcodeFormatReader reader(path);
    if (!reader.is_open() || !reader->stream_metadata_start()) {
        return info;
    }
    GcodeBuffer buffer;
    while (reader->stream_get_line(buffer, IGcodeReader::Continuations::Discard) == IGcodeReader::Result_t::RESULT_OK) {
        const auto [name, value] = buffer.line.parse_metadata();
        if (name.begin != nullptr && value.begin != nullptr) {
            print_info::parse_metadata(info, std::string_view(&*name.begin, name.len()), std::string_view(&*value.begin, value.len()));
        }
    }
    return info;
}

bool draw_thumbnail(const char *path, Rect16 box, uint8_t factor, std::optional<ScaledQoiReader> &scaler, uint16_t grey_rows) {
    AnyGcodeFormatReader gcode(path);
    if (!gcode.is_open()) {
        return false;
    }
    AbstractByteReader *qoi = gcode->stream_thumbnail_start(thumb_w, thumb_h, IGcodeReader::ImgType::QOI);
    if (!qoi) {
        return false;
    }
    scaler.emplace(*qoi, ScaledQoiReader::Params { .factor = factor, .back = style::thumb_back, .radius = 0, .outside = style::thumb_back, .grey_rows = grey_rows });
    display::draw_img(point_ui16(box.Left() + (box.Width() - thumb_w / factor) / 2, box.Top() + (box.Height() - thumb_h / factor) / 2), *scaler);
    scaler.reset();
    return true;
}

void draw_ring(Rect16 outer, Color ring, Color back, uint8_t radius) {
    const int w = outer.Width(), h = outer.Height();
    const auto span = [&](int y, int from, int to, Color c) {
        if (to > from) {
            display::fill_rect(Rect16(outer.Left() + from, outer.Top() + y, to - from, 1), c);
        }
    };
    for (int y = 0; y < h; ++y) {
        const int out = print_info::corner_inset(y, h, radius + style::ring);
        span(y, 0, out, back);
        span(y, w - out, w, back);
        if (y < style::ring || y >= h - style::ring) {
            span(y, out, w - out, ring);
        } else {
            // Bande de l'anneau : la marge et les coins du corps, concentriques à ceux du bord
            const int in = style::ring + print_info::corner_inset(y - style::ring, h - 2 * style::ring, radius);
            span(y, out, in, ring);
            span(y, w - in, w - out, ring);
        }
    }
}

// ---- Grille des fichiers ----

std::array<char, filename_defs::path_buffer_size> ScreenFiles::dir { '/', 'u', 's', 'b' };
int ScreenFiles::first = 0;

FileTile::FileTile(ScreenFiles &screen, uint8_t index)
    : window_t(&screen, style::with_ring(tile_body(index)))
    , screen(screen) {
    set_visible(false);
}

void FileTile::set(Kind k, uint8_t index_in_view) {
    kind = k;
    view_index = index_in_view;
    info = {};
    if (kind == Kind::file) {
        std::array<char, filename_defs::path_buffer_size> path;
        if (screen.entry_path(*this, path)) {
            info = read_file_info(path.data());
        }
    }
    set_visible(kind != Kind::none);
    set_enabled(kind != Kind::none);
    Invalidate();
}

void FileTile::unconditionalDraw() {
    const Rect16 outer = GetRect();
    const Color ring = IsFocused() ? style::focus_ring : style::screen;
    draw_ring(outer, ring, style::screen, style::radius_big);

    const Rect16 body(outer.Left() + style::ring, outer.Top() + style::ring, outer.Width() - 2 * style::ring, outer.Height() - 2 * style::ring);
    const Rect16 text_area(body.Left(), body.Top() + tile_thumb_h, body.Width(), body.Height() - tile_thumb_h);
    display::draw_rounded_rect(text_area, ring, style::surface, style::radius_big, MIC_BOT_LEFT | MIC_BOT_RIGHT);

    const int16_t x = body.Left() + 10;
    const uint16_t w = body.Width() - 20;
    std::array<char, 32> name;
    display_name(screen.display_name(*this), kind == Kind::file, name, w / width(Font::small));
    draw_text(Rect16(x, body.Top() + 78, w, height(Font::small)), string_view_utf8::MakeRAM(name.data()), Font::small, style::text_strong, style::surface);

    const Rect16 line(x, body.Top() + 96, w, height(Font::small));
    switch (kind) {
    case Kind::file: {
        std::array<char, 32> text;
        summary(info, false, text);
        draw_text(line, string_view_utf8::MakeRAM(text.data()), Font::small, style::text_muted, style::surface);
        // Une bobine pleine par emplacement utilisé, alignées à droite
        constexpr uint8_t d = 12, step = 16;
        int16_t spool_x = body.Left() + body.Width() - 10 - (std::popcount(info.used) * step - (step - d));
        for (uint8_t t = 0; t < print_info::max_tools; ++t) {
            if (info.is_used(t)) {
                spool::Look look = Slot::read(t).look;
                look.fill_percent = 100;
                draw_spool({ spool_x, int16_t(line.Top() + 2) }, look, d);
                spool_x += step;
            }
        }
        break;
    }
    case Kind::folder:
        draw_text(line, _("Folder"), Font::small, style::text_muted, style::surface);
        break;
    case Kind::up:
        draw_text(line, _("Back"), Font::small, style::text_muted, style::surface);
        break;
    case Kind::none:
        break;
    }
}

void FileTile::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::CLICK:
    case GUI_event_t::TOUCH_CLICK:
        screen.open(*this); // consommé, comme Card
        return;
    case GUI_event_t::KNOB: {
        // Au bord de la page, la molette tourne la page au lieu de quitter la grille
        auto &ctx = *static_cast<GuiEventContext *>(param);
        const int diff = ctx.event.value<gui_event::KnobEvent>().diff;
        const bool last = this == &screen.tile(3), first_tile = this == &screen.tile(0);
        if ((diff > 0 && last && screen.turn_page(1)) || (diff < 0 && first_tile && screen.turn_page(-1))) {
            ctx.accept();
            return;
        }
        break;
    }
    default:
        break;
    }
    window_t::windowEvent(sender, event, param);
}

FileThumb::FileThumb(ScreenFiles &screen, const FileTile &tile, uint8_t index)
    : window_t(&screen, Rect16(tile_body(index).Left(), tile_body(index).Top(), tile_body(index).Width(), tile_thumb_h))
    , screen(screen)
    , tile(tile) {
    set_visible(false);
}

void FileThumb::unconditionalDraw() {
    const Rect16 box = GetRect();
    // Les coins hors de l'arrondi sont dans la bande de l'anneau de la carte
    const Color corner = tile.IsFocused() ? style::focus_ring : style::screen;
    display::draw_rounded_rect(box, corner, style::thumb_back, style::radius_big, MIC_TOP_LEFT | MIC_TOP_RIGHT);

    const img::Resource *icon = nullptr;
    switch (tile.kind) {
    case FileTile::Kind::file: {
        std::array<char, filename_defs::path_buffer_size> path;
        if (screen.entry_path(tile, path) && draw_thumbnail(path.data(), box, grid_factor, screen.scaler)) {
            return;
        }
        icon = &img::print_16x16;
        break;
    }
    case FileTile::Kind::folder:
        icon = &img::folder_full_16x16;
        break;
    case FileTile::Kind::up:
        icon = &img::folder_up_16x16;
        break;
    case FileTile::Kind::none:
        return;
    }
    draw_icon({ int16_t(box.Left() + (box.Width() - 16) / 2), int16_t(box.Top() + (box.Height() - 16) / 2) }, *icon, style::thumb_back);
}

ScreenFiles::ScreenFiles()
    : Screen(true)
    , header(this, _("Files \xC2\xB7 USB drive"), true)
    , thumbs { { *this, tiles[0], 0 }, { *this, tiles[1], 1 }, { *this, tiles[2], 2 }, { *this, tiles[3], 3 } } // avant les cartes : leur anneau passe sur les coins de la vignette
    , tiles { { *this, 0 }, { *this, 1 }, { *this, 2 }, { *this, 3 } }
    , tabs(this, TabBar::Tab::print) {}

void ScreenFiles::InitState(screen_init_variant var) {
    if (!var.GetPosition()) {
        // Nouvelle ouverture : racine de la clé, première page. Au retour d'« Avant l'impression », la même place.
        strlcpy(dir.data(), "/usb", dir.size());
        first = 0;
    }
    load();
    if (var.GetPosition()) {
        Screen::InitState(var);
    } else if (tiles[0].IsVisible()) {
        tiles[0].SetFocus();
    }
}

bool ScreenFiles::in_subfolder() const {
    return strcmp(dir.data(), "/usb") != 0;
}

int ScreenFiles::item_count() const {
    return view.TotalFilesCount() - 1 + (in_subfolder() ? 1 : 0); // la liste compte toujours « .. »
}

void ScreenFiles::load() {
    loaded = true;
    view.ChangeDirectory(dir.data(), FileSort::SortPolicy::BY_CRMOD_DATETIME);
    const int count = item_count();
    first = std::clamp(first, 0, std::max(0, count - 1)) / tiles_count * tiles_count;

    // Entrées numérotées -1 (« .. ») à last ; à la racine, « .. » ne s'affiche pas. Près de la fin,
    // LazyDirView recule sa fenêtre pour qu'elle reste pleine : on décale alors l'index dans la fenêtre.
    const int last = view.TotalFilesCount() - 2;
    const int wanted = first - (in_subfolder() ? 1 : 0);
    const int start = view.set_window_offset(wanted);
    for (int i = 0; i < tiles_count; ++i) {
        const int entry = wanted + i;
        FileTile::Kind kind = FileTile::Kind::none;
        uint8_t index = 0;
        if (entry <= last) {
            index = entry - start;
            kind = entry < 0 ? FileTile::Kind::up
                             : (view.display_name_at(index).second == FileSort::EntryType::DIR ? FileTile::Kind::folder : FileTile::Kind::file);
        }
        tiles[i].set(kind, index);
        thumbs[i].set_visible(kind != FileTile::Kind::none);
        thumbs[i].Invalidate();
    }

    // Une carte cachée ne garde pas le focus
    if (window_t *focused = get_focused_child_window(); !focused || !focused->IsVisible()) {
        if (window_t *w = tiles[0].IsVisible() ? &tiles[0] : GetFirstEnabledSubWin()) {
            w->SetFocus();
        }
    }
}

bool ScreenFiles::entry_path(const FileTile &t, std::span<char> out) const {
    const int n = snprintf(out.data(), out.size(), "%s/%s", dir.data(), view.canonical_name_at(t.view_index).first);
    return n > 0 && size_t(n) < out.size();
}

const char *ScreenFiles::display_name(const FileTile &t) const {
    return view.display_name_at(t.view_index).first;
}

void ScreenFiles::show(int new_first, FileTile &focus) {
    first = new_first;
    load();
    Invalidate(); // barre de défilement, message
    if (focus.IsVisible()) {
        focus.SetFocus();
    }
}

void ScreenFiles::open(const FileTile &t) {
    std::array<char, filename_defs::path_buffer_size> path;
    switch (t.kind) {
    case FileTile::Kind::file:
        if (entry_path(t, path)) {
            open_preview(path.data());
        }
        break;
    case FileTile::Kind::folder:
        if (entry_path(t, path)) {
            dir = path;
            show(0, tiles[0]);
        }
        break;
    case FileTile::Kind::up:
        if (char *slash = strrchr(dir.data(), '/'); slash && in_subfolder()) {
            *slash = '\0';
            show(0, tiles[0]);
        }
        break;
    case FileTile::Kind::none:
        break;
    }
}

bool ScreenFiles::turn_page(int direction) {
    const int new_first = first + direction * tiles_count;
    if (new_first < 0 || new_first >= item_count()) {
        return false;
    }
    show(new_first, tiles[direction > 0 ? 0 : tiles_count - 1]);
    return true;
}

void ScreenFiles::unconditionalDraw() {
    Screen::unconditionalDraw();
    const int count = item_count();
    if (count == 0) {
        const string_view_utf8 text = marlin_vars().media_inserted ? _("No printable file") : _("Insert a USB drive");
        draw_text(Rect16(grid_rect.Left(), 130, grid_rect.Width(), height(Font::small)), text, Font::small, style::text_muted, style::screen, Align_t::Center());
    } else if (count > tiles_count) {
        // Barre de défilement (§1.7) dans la marge droite
        const int h = std::max(grid_rect.Height() * tiles_count / count, 12);
        const int y = grid_rect.Top() + (grid_rect.Height() - h) * first / std::max(count - tiles_count, 1);
        display::fill_rect(Rect16(475, std::min<int>(y, grid_rect.Bottom() + 1 - h), 3, h), style::text_faint);
    }
}

void ScreenFiles::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        if (!loaded) {
            load(); // si InitState n'a pas été appelé
        }
        break;
    case GUI_event_t::MEDIA:
        if (const auto state = MediaState_t(int(param)); state == MediaState_t::inserted || state == MediaState_t::removed) {
            strlcpy(dir.data(), "/usb", dir.size());
            show(0, tiles[0]);
        }
        break;
    case GUI_event_t::TOUCH_SWIPE_UP:
        turn_page(1);
        return;
    case GUI_event_t::TOUCH_SWIPE_DOWN:
        turn_page(-1);
        return;
    default:
        break;
    }
    Screen::windowEvent(sender, event, param);
}

// ---- Avant l'impression ----

std::array<char, filename_defs::path_buffer_size> ScreenPreview::path {};

ScreenPreview::Button::Button(window_t *parent, Rect16 body, Action action, bool primary)
    : Card(parent, style::with_ring(body), action, primary ? style::accent : style::secondary, style::screen, style::radius_big)
    , primary(primary) {}

void ScreenPreview::Button::set_active(bool set) {
    if (set != active) {
        active = set;
        body_color = active && primary ? style::accent : style::secondary;
        Invalidate();
    }
}

void ScreenPreview::Button::draw_content(Rect16 body) {
    const string_view_utf8 label = primary ? _("Print now") : _("Map colours");
    const Color fg = !active ? style::text_faint : (primary ? style::on_accent : style::text);
    const uint16_t text_w = label.computeNumUtf8Chars() * width(Font::small);
    const int16_t y = body.Top() + (body.Height() - height(Font::small)) / 2;
    if (!primary) {
        draw_text(Rect16(body.Left(), y, body.Width(), height(Font::small)), label, Font::small, fg, body_color, Align_t::Center());
        return;
    }
    const int16_t x = body.Left() + (body.Width() - 16 - 6 - text_w) / 2;
    const ropfn rop = active ? ropfn { .swap_bw = has_swapped_bw::yes } : ropfn { .shadow = is_shadowed::yes };
    draw_icon({ x, y }, img::print_16x16, body_color, rop);
    draw_text(Rect16(x + 16 + 6, y, text_w, height(Font::small)), label, Font::small, fg, body_color);
}

ScreenPreview::ScreenPreview()
    : Screen(true)
    , header(this, _("Before printing"), true)
    , print(this, print_body, print_now, true)
    , map(this, map_body, map_colours, false)
    , tabs(this, TabBar::Tab::print)
    , info(read_file_info(path.data())) {
    std::array<char, filename_defs::path_buffer_size> sfn = path; // get_LFN veut un chemin modifiable
    std::array<char, filename_defs::filename_buffer_size> lfn;
    get_LFN(lfn.data(), lfn.size(), sfn.data());
    display_name(lfn.data(), true, name, (style::screen_w - style::margin - info_x) / width(Font::normal));
    update_buttons();
    print.SetFocus();
}

void ScreenPreview::update_buttons() {
    const bool active = can_print();
    print.set_active(active);
    map.set_active(active);
}

void ScreenPreview::unconditionalDraw() {
    Screen::unconditionalDraw();

    display::draw_rounded_rect(preview_box, style::screen, style::thumb_back, style::radius_big, MIC_ALL_CORNERS);
    if (!draw_thumbnail(path.data(), preview_box, preview_factor, scaler)) {
        draw_icon({ int16_t(preview_box.Left() + (preview_box.Width() - 16) / 2), int16_t(preview_box.Top() + (preview_box.Height() - 16) / 2) }, img::print_16x16, style::thumb_back);
    }

    const uint16_t w = style::screen_w - style::margin - info_x;
    draw_text(Rect16(info_x, 32, w, height(Font::normal)), string_view_utf8::MakeRAM(name.data()), Font::normal, style::text_strong, style::screen);
    std::array<char, 40> text;
    summary(info, true, text);
    draw_text(Rect16(info_x, 55, w, height(Font::small)), string_view_utf8::MakeRAM(text.data()), Font::small, style::text_muted, style::screen);

    // Une ligne par emplacement utilisé : bobine, « n · matière chargée », grammes du fichier
    int row = 0, mismatch = -1;
    Slot mismatch_slot;
    for (uint8_t t = 0; t < print_info::max_tools; ++t) {
        if (!info.is_used(t)) {
            continue;
        }
        const int16_t y = 74 + 19 * row++;
        const Slot slot = Slot::read(t);
        draw_spool({ info_x, y }, slot.look, 16);
        snprintf(text.data(), text.size(), "%d%s%s", t + 1, dot, slot.material.data());
        draw_text(Rect16(info_x + 22, y, 160, height(Font::small)), string_view_utf8::MakeRAM(text.data()), Font::small, style::text_strong, style::screen);
        snprintf(text.data(), text.size(), "%.0f g", double(info.grams[t]));
        draw_text(Rect16(info_x + 182, y, w - 182, height(Font::small)), string_view_utf8::MakeRAM(text.data()), Font::small, style::text_muted, style::screen, Align_t::RightCenter());
        if (mismatch < 0 && info.types[t][0] && slot.look.filament && strcasecmp(info.types[t].data(), slot.material.data()) != 0) {
            mismatch = t;
            mismatch_slot = slot;
        }
    }

    // Alerte (§3.10) : la matière chargée n'est pas celle du fichier ; Prusa la vérifie aussi au départ
    if (mismatch >= 0) {
        StringViewUtf8Parameters<24> params;
        const string_view_utf8 warning = _("Slot %d: %s in the file, %s loaded").formatted(params, mismatch + 1, info.types[mismatch].data(), mismatch_slot.material.data());
        render_text_align(Rect16(style::margin, 184, style::screen_w - 2 * style::margin, 2 * height(Font::small)), warning, Font::small, style::screen, style::warning, {}, text_flags(Align_t::LeftTop(), is_multiline::yes));
    }
}

void ScreenPreview::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP || event == GUI_event_t::MEDIA) {
        update_buttons();
    }
    Screen::windowEvent(sender, event, param);
}

} // namespace od3d
