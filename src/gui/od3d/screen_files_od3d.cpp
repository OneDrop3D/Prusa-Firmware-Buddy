/// @file
#include "screen_files_od3d.hpp"
#include "layout_od3d.hpp"
#include "sleep_gui_od3d.hpp"
#include "spool_draw.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <common/marlin_vars.hpp>
#include <common/thumbnail_sizes.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <display.hpp>
#include <display_helper.h>
#include <FreeRTOS.h>
#include <feature/od3d/mesh_memory_od3d.hpp>
#include <feature/od3d/pa_od3d.hpp>
#include <feature/od3d/spool_usage_od3d.hpp>
#include <gcode/gcode_reader_any.hpp>
#include <gui/event/gui_event.hpp>
#include <gui/event/knob_event.hpp>
#include <gui.hpp>
#include <heap.h>
#include <i18n.h>
#include <img_resources.hpp>
#include <lfn.h>
#include <marlin_client.hpp>
#include <media_state.hpp>
#include <metric.h>
#include <new>
#include <option/has_touch.h>
#include <sound.hpp>
#include <strings.h>
#include <timing.h>

#if HAS_TOUCH()
    #include <hw/touchscreen/touchscreen.hpp>
#endif

// Durées des lectures de la clé, en ms (M331 od3d_files pour les activer) : dir = liste du dossier, info = métadonnées
// d'un fichier (ouverture comprise), thumb = vignette (de l'ouverture du fichier à sa dernière bande, pauses comprises),
// band = la plus longue bande de cette vignette (temps où l'interface ne répond pas), ready = de l'affichage de la page
// à sa dernière vignette, scroll = rangée ou page tournée (dossier relu si besoin et cartes), step = d'un cran de molette
// à l'écran repeint (attente du dessin compris, 40 ms au plus), cache = octets du cache des vignettes (0 : pas assez de
// mémoire libre), window = entrées du dossier gardées en mémoire (4 : pas assez de mémoire libre, relu à chaque rangée)
METRIC_DEF(metric_od3d_files, "od3d_files", METRIC_VALUE_CUSTOM, 0, METRIC_DISABLED);

namespace od3d {

namespace {

    using print_info::FileInfo;

    constexpr uint16_t thumb_w = thumbnail_sizes::preview_thumbnail_width;
    constexpr uint16_t thumb_h = thumbnail_sizes::preview_thumbnail_height;

    // Grille (SPEC_G.md §3.9) : 4 cartes de 228 × 110 (114 dans la maquette, sous l'en-tête de 40 px), haut de 72 px
    // pour la vignette (313 × 173 réduite au tiers)
    constexpr int16_t row_pitch = 116; ///< d'une rangée de cartes à la suivante : aussi le pas du doigt
    constexpr Rect16 tile_body(int i) {
        return Rect16(style::margin + (i % ScreenFiles::cols) * 236, style::top + (i / ScreenFiles::cols) * row_pitch, 228, 110);
    }
    constexpr uint16_t tile_thumb_h = 72;
    constexpr uint8_t grid_factor = 3;
    constexpr Rect16 grid_rect = Rect16::fromLTRB(style::margin, style::top, style::screen_w - style::margin, tile_body(3).Bottom() + 1);
    constexpr Rect16 scroll_rect = { 474, grid_rect.Top(), 4, grid_rect.Height() }; ///< à droite des anneaux des cartes
    static_assert(thumb_w / grid_factor <= tile_body(0).Width() && thumb_h / grid_factor <= tile_thumb_h);
    static_assert(style::inside(style::content, style::with_ring(tile_body(3))));
    static_assert(scroll_rect.Left() > style::with_ring(tile_body(3)).Right() && scroll_rect.Right() < style::screen_w);

    /// Vignette de la grille : 8 lignes par bande (24 de la source, ~10 ms de décodage et une lecture de 4 Ko au plus) ;
    /// l'interface répond entre deux
    constexpr uint16_t band_rows = 8;
    constexpr ScaledQoiReader::Params grid_params { .factor = grid_factor, .back = style::thumb_back, .radius = 0, .outside = style::thumb_back };

    /// Lectures après la dernière molette ou le dernier toucher : l'écran répond d'abord
    constexpr int32_t input_pause_ms = 250;

    // Doigt : la grille suit dès 10 px (comme le défilement de Prusa) ; une rangée d'inertie toutes les 70 ms
    constexpr int16_t drag_min = 10;
    constexpr uint32_t fling_step_ms = 70;

    // Avant l'impression (§3.10), sous l'en-tête de 40 px : case de l'aperçu 136 px de haut (146 dans la maquette)
    constexpr Rect16 preview_box = { style::margin, style::top, 212, 136 };
    constexpr uint8_t preview_factor = 2;
    constexpr int16_t info_x = 230;
    constexpr int16_t warning_y = 188;
    constexpr Rect16 print_body = { style::margin, 228, 228, 44 };
    constexpr Rect16 map_body = { 244, 228, 228, 44 };
    // « Maillage » proposé (F44) : « Imprimer » (icône et 8 à 9 caractères) rétréci de moitié, « Maillage » à sa droite
    constexpr Rect16 print_narrow_body = { style::margin, 228, 110, 44 };
    constexpr Rect16 mesh_body = { style::margin + 118, 228, 110, 44 };
    static_assert(preview_box.Bottom() < warning_y && warning_y + 2 * 16 <= style::with_ring(print_body).Top());
    static_assert(style::inside(style::content, style::with_ring(map_body)));
    static_assert(style::with_ring(print_narrow_body).Right() < style::with_ring(mesh_body).Left() && style::with_ring(mesh_body).Right() < style::with_ring(map_body).Left());
    static_assert(thumb_w / preview_factor <= preview_box.Width() && thumb_h / preview_factor <= preview_box.Height());
    constexpr ScaledQoiReader::Params preview_params { .factor = preview_factor, .back = style::thumb_back, .radius = 0, .outside = style::thumb_back };
    constexpr Rect16 preview_img = { int16_t(preview_box.Left() + (preview_box.Width() - thumb_w / preview_factor) / 2),
        int16_t(preview_box.Top() + (preview_box.Height() - thumb_h / preview_factor) / 2), thumb_w / preview_factor, thumb_h / preview_factor };
    static_assert(preview_img.Left() - preview_box.Left() >= style::radius_big, "les coins de la case tiennent à côté de l'image");

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

    /// « Maillage » de l'« Avant l'impression » ouvert : true repris, false palpé en entier ; nullopt s'il n'est pas proposé
    std::optional<bool> mesh_reuse;

    void start_print(marlin_client::PreviewSkipIfAble skip) {
        if (can_print()) {
            if (mesh_reuse) {
                mesh_memory::choose_for_next_print(*mesh_reuse); // pour la demande qui suit seulement
            }
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

namespace {

    /// Métadonnées des derniers fichiers lus, repérés par une empreinte de leur chemin (FNV-1a) ; ~1,3 Ko
    struct InfoCache {
        static constexpr size_t size = 16;
        std::array<uint32_t, size> keys {}; ///< 0 : place libre
        std::array<FileInfo, size> infos {};
        uint8_t next = 0;

        static uint32_t key(const char *path) {
            uint32_t h = 2166136261u;
            for (const char *p = path; *p; ++p) {
                h = (h ^ uint8_t(*p)) * 16777619u;
            }
            return h ? h : 1;
        }

        const FileInfo *find(const char *path) const {
            const uint32_t k = key(path);
            for (size_t i = 0; i < size; ++i) {
                if (keys[i] == k) {
                    return &infos[i];
                }
            }
            return nullptr;
        }

        void put(const char *path, const FileInfo &info) {
            keys[next] = key(path);
            infos[next] = info;
            next = (next + 1) % size;
        }
    };

    InfoCache info_cache;

} // namespace

namespace {

    /// Métadonnées du G-code ouvert, gardées en cache sous path ; vides (et pas gardées) si elles ne se lisent pas
    FileInfo read_metadata(AnyGcodeFormatReader &gcode, const char *path) {
        FileInfo info;
        if (!gcode->stream_metadata_start()) {
            return info; // pas gardé : la clé peut revenir
        }
        GcodeBuffer buffer;
        while (gcode->stream_get_line(buffer, IGcodeReader::Continuations::Discard) == IGcodeReader::Result_t::RESULT_OK) {
            const auto [name, value] = buffer.line.parse_metadata();
            if (name.begin != nullptr && value.begin != nullptr) {
                print_info::parse_metadata(info, std::string_view(&*name.begin, name.len()), std::string_view(&*value.begin, value.len()));
            }
        }
        info_cache.put(path, info);
        return info;
    }

    /// G-code dont une vignette de la grille se dessine par bandes : ouvert d'un LOOP au suivant. Hors de l'écran, dont la
    /// place est comptée (4 192 octets) ; il n'y a qu'un écran Fichiers à la fois.
    std::optional<AnyGcodeFormatReader> band_gcode;

    /// Titre « Fichiers 5-8/23 » (position dans la liste), gardé tant que l'en-tête le montre
    std::array<char, 40> title_text;

    // Cache des vignettes de la grille : une rangée qui revient (molette, doigt, retour d'« Avant l'impression ») se
    // redessine sans relire la clé. Avant (8.0.2.26), chaque rangée relisait ses 4 vignettes, même les 2 encore à l'écran.
    // RAM : 104 × 57 en RGB565 = 11,9 Ko par vignette (47 Ko la page : impossible) ; en QOI ~2,5 Ko (mesuré sur une
    // vignette PrusaSlicer). Emprunté au tas tant que la grille ou son « Avant l'impression » est ouvert, rendu ensuite
    // (pas pendant une impression), et seulement s'il reste au moins heap_margin pour le reste (réseau, ouvertures de
    // fichiers par 4 Ko) : 16 Ko (~6 vignettes, la page et une rangée), sinon 8 Ko, sinon rien (lecture comme avant).
    QoiCache *thumb_cache = nullptr;
    constexpr size_t heap_margin = 16 * 1024;

    void open_thumb_cache() {
        for (const size_t bytes : { 16 * 1024, 8 * 1024 }) {
            if (thumb_cache || xPortGetFreeHeapSize() < sizeof(QoiCache) + bytes + heap_margin) {
                continue;
            }
            if (void *block = malloc_fallible(sizeof(QoiCache) + bytes)) {
                thumb_cache = new (block) QoiCache(std::span(static_cast<std::byte *>(block) + sizeof(QoiCache), bytes));
                metric_record_custom(&metric_od3d_files, " cache=%ui", unsigned(bytes));
            }
        }
        if (!thumb_cache) {
            metric_record_custom(&metric_od3d_files, " cache=0i");
        }
    }

    /// Rendu au tas quand ni la grille ni son « Avant l'impression » ne sont plus dans la pile des écrans
    void close_thumb_cache() {
        if (thumb_cache && !Screens::Access()->IsScreenOnStack<ScreenFiles>()) {
            thumb_cache->~QoiCache();
            free(thumb_cache);
            thumb_cache = nullptr;
        }
    }

    // Fenêtre du dossier : 24 entrées en mémoire au lieu des 4 de la page. Avant (8.0.2.33), chaque rangée (deux crans de
    // molette) relisait tout le dossier sur la clé : LazyDirView n'avance sa fenêtre qu'en le relisant. Un dossier de 23
    // fichiers au plus n'est plus relu après son ouverture ; un plus grand, une fois toutes les 10 rangées. ~5 Ko du tas
    // tant que la grille est ouverte, avec la même marge que le cache des vignettes ; sinon la page seule, comme avant.
    using DirWindow = LazyDirView<24>;
    DirWindow *dir_window = nullptr;

    /// Taille de la vignette de la grille (la 313 × 173 réduite au tiers), centrée dans sa case
    constexpr uint16_t grid_img_w = thumb_w / grid_factor, grid_img_h = thumb_h / grid_factor;

} // namespace

namespace {

    // Grandes vignettes : accueil 187 × 103, Avant l'impression et Impression 156 × 86, en QOI (4,7 et 3,5 Ko mesurés sur
    // une vignette PrusaSlicer). Avant, chaque retour à l'accueil relisait la sienne sur la clé d'un bloc (0,1 à 0,2 s
    // figé, estimé), et l'écran Impression ~86 fois par impression. Pris au tas à la première image et gardé ensuite
    // (pas de fragmentation), avec la même marge que la grille : 12 Ko (les deux images), sinon 8, sinon rien.
    QoiCache *big_cache = nullptr;

    QoiCache *open_big_cache() {
        for (const size_t bytes : { 12 * 1024, 8 * 1024 }) {
            if (big_cache || xPortGetFreeHeapSize() < sizeof(QoiCache) + bytes + heap_margin) {
                continue;
            }
            if (void *block = malloc_fallible(sizeof(QoiCache) + bytes)) {
                big_cache = new (block) QoiCache(std::span(static_cast<std::byte *>(block) + sizeof(QoiCache), bytes));
            }
        }
        return big_cache;
    }

    /// Une image par fichier et par taille (le gris n'est pas gardé)
    uint32_t big_key(const char *path, const ScaledQoiReader::Params &params) {
        const uint32_t key = InfoCache::key(path) ^ (uint32_t(params.factor) << 24 | uint32_t(params.num) << 16);
        return key ? key : 1;
    }

    std::optional<QoiCache::Reader> find_big(const char *path, const ScaledQoiReader::Params &params) {
        return big_cache ? big_cache->find(big_key(path, params)) : std::nullopt;
    }

    /// Lit la bande en cours sans l'envoyer à l'écran
    void skip_band(ScaledQoiReader &scaler) {
        std::array<std::byte, 64> skip;
        while (!scaler.read(skip).empty()) {
        }
    }

} // namespace

std::optional<FileInfo> cached_file_info(const char *path) {
    if (const FileInfo *hit = info_cache.find(path)) {
        return *hit;
    }
    return std::nullopt;
}

void forget_file_infos() {
    info_cache.keys = {};
    if (thumb_cache) {
        thumb_cache->clear();
    }
    if (big_cache && !print_uses_usb()) {
        big_cache->clear(); // pendant une impression, l'image de l'écran Impression reste (la clé ne peut pas la relire)
    }
}

bool open_gcode(AnyGcodeFormatReader &gcode, const char *path) {
    if (!gcode.open(path)) {
        return false;
    }
    // PlainGcodeReader et PrusaPackGcodeReader dérivent de GcodeReaderCommon (ClosedReader seul n'en dérive pas, et il
    // n'est pas ouvert). 4 Ko : de l'en-tête à la vignette 313 × 173 d'un .bgcode en une lecture, puis 8 secteurs par
    // commande USB.
    static_cast<GcodeReaderCommon *>(gcode.get())->set_read_buffer(4096);
    return true;
}

FileInfo read_file_info(const char *path) {
    if (const FileInfo *hit = info_cache.find(path)) {
        return *hit;
    }
    if (print_uses_usb()) {
        return {}; // la clé est à l'impression
    }
    const uint32_t start_ms = ticks_ms();
    AnyGcodeFormatReader gcode;
    if (!open_gcode(gcode, path)) {
        return {};
    }
    const FileInfo info = read_metadata(gcode, path);
    metric_record_custom(&metric_od3d_files, " info=%ui", unsigned(ticks_diff(ticks_ms(), start_ms)));
    return info;
}

bool print_uses_usb() {
    return marlin_client::is_printing();
}

namespace big_thumb {

    bool cached(const char *path, const ScaledQoiReader::Params &params) {
        return find_big(path, params).has_value();
    }

    bool draw_cached(const char *path, const ScaledQoiReader::Params &params, point_ui16_t at, std::optional<ScaledQoiReader> &scaler, uint16_t first, uint16_t rows) {
        auto image = find_big(path, params);
        if (!image) {
            return false;
        }
        if (!params.grey_rows && !first && !rows) {
            display::draw_img(at, *image); // telle qu'elle a été dessinée la première fois
            return true;
        }
        // Relue à l'échelle 1, grisée en haut ; les lignes avant first sont décodées en RAM, pas envoyées
        scaler.emplace(*image, ScaledQoiReader::Params { .factor = 1, .back = params.back, .radius = 0, .outside = params.back, .grey_rows = params.grey_rows });
        if (scaler->open()) {
            if (first && scaler->band(first)) {
                skip_band(*scaler);
            }
            if (scaler->band(rows ? rows : scaler->height())) {
                display::draw_img(point_ui16(at.x, at.y + first), *scaler);
            }
        }
        scaler.reset();
        return true;
    }

    bool start(const char *path, const ScaledQoiReader::Params &params, std::optional<ScaledQoiReader> &scaler) {
        stop(scaler);
        band_gcode.emplace();
        if (open_gcode(*band_gcode, path)) {
            if (AbstractByteReader *qoi = (*band_gcode)->stream_thumbnail_start(thumb_w, thumb_h, IGcodeReader::ImgType::QOI)) {
                scaler.emplace(*qoi, params);
                if (scaler->open()) {
                    if (QoiCache *cache = open_big_cache()) {
                        cache->start(big_key(path, params), scaler->width(), scaler->height());
                        scaler->record = cache; // gardée en dessinant
                    }
                    return true;
                }
            }
        }
        stop(scaler);
        return false;
    }

    bool step(std::optional<ScaledQoiReader> &scaler, point_ui16_t at, bool draw) {
        const uint16_t row = scaler->row();
        if (scaler->band(band_rows)) {
            if (draw) {
                display::draw_img(point_ui16(at.x, at.y + row), *scaler);
            } else {
                skip_band(*scaler);
            }
        }
        if (!scaler->done() && scaler->row() != row) {
            return true;
        }
        if (scaler->record) {
            scaler->record->finish(); // gardée si elle est complète
        }
        stop(scaler);
        return false;
    }

    void stop(std::optional<ScaledQoiReader> &scaler) {
        if (scaler && scaler->record) {
            scaler->record->cancel(); // rien si elle vient d'être gardée
        }
        scaler.reset(); // avant le G-code : il lit sa vignette
        band_gcode.reset();
    }

    bool can_keep() {
        return open_big_cache();
    }

    void draw_box(Rect16 box, Rect16 img, uint8_t corners) {
        // Bandes gauche et droite avec les coins arrondis (au moins radius_big de large), haut et bas : chaque pixel une
        // fois, pas d'aplat sous l'image (une image qui revient ne clignote pas)
        display::draw_rounded_rect(Rect16::fromLTRB(box.Left(), box.Top(), img.Left(), box.Bottom() + 1), style::screen, style::thumb_back, style::radius_big, corners & (MIC_TOP_LEFT | MIC_BOT_LEFT));
        display::draw_rounded_rect(Rect16::fromLTRB(img.Right() + 1, box.Top(), box.Right() + 1, box.Bottom() + 1), style::screen, style::thumb_back, style::radius_big, corners & (MIC_TOP_RIGHT | MIC_BOT_RIGHT));
        display::fill_rect(Rect16::fromLTRB(img.Left(), box.Top(), img.Right() + 1, img.Top()), style::thumb_back);
        display::fill_rect(Rect16::fromLTRB(img.Left(), img.Bottom() + 1, img.Right() + 1, box.Bottom() + 1), style::thumb_back);
    }

} // namespace big_thumb

// ---- Grille des fichiers ----

std::array<char, filename_defs::path_buffer_size> ScreenFiles::dir { '/', 'u', 's', 'b' };
int ScreenFiles::first = 0;
ScreenFiles *ScreenFiles::instance = nullptr;

FileTile::FileTile(ScreenFiles &screen, uint8_t index)
    : window_t(&screen, style::with_ring(tile_body(index)))
    , index(index)
    , screen(screen) {}

void FileTile::set(Kind k, uint8_t index_in_view) {
    kind = k;
    view_index = index_in_view;
    info = {};
    info_pending = false;
    if (kind == Kind::file) {
        std::array<char, filename_defs::path_buffer_size> path;
        if (screen.entry_path(*this, path)) {
            if (const auto cached = cached_file_info(path.data())) {
                info = *cached;
            } else {
                info_pending = true; // lu avec la vignette (ScreenFiles::start_load)
            }
        }
    }
    set_enabled(kind != Kind::none);
    Invalidate();
}

void FileTile::set_info(const FileInfo &set) {
    info = set;
    info_pending = false;
    Invalidate(); // la partie basse seulement : la vignette est une autre fenêtre
}

void FileTile::unconditionalDraw() {
    // La marge et la partie basse de la carte ; le haut (vignette) est FileThumb, qui ne se redessine pas au focus.
    // Focus : la partie basse plus claire, sans anneau. Pas d'entrée (fin de liste) : le fond de l'écran.
    const Rect16 outer = GetRect();
    if (kind == Kind::none) {
        display::fill_rect(Rect16(outer.Left(), outer.Top() + style::ring + tile_thumb_h, outer.Width(), outer.Height() - style::ring - tile_thumb_h), style::screen);
        draw_margin(outer, style::screen);
        return;
    }
    draw_margin(outer, style::screen);

    const Rect16 body(outer.Left() + style::ring, outer.Top() + style::ring, outer.Width() - 2 * style::ring, outer.Height() - 2 * style::ring);
    const Rect16 text_area(body.Left(), body.Top() + tile_thumb_h, body.Width(), body.Height() - tile_thumb_h);
    const Color bg = IsFocused() ? style::focused(style::surface) : style::surface;

    // D'une passe, comme les lignes des menus (le nom et la ligne disparaissaient à chaque cran) : le fond seulement
    // autour du nom, de la ligne, des bobines et des deux coins arrondis du bas, qui peignent chacun leur place
    constexpr int16_t pad = 10; ///< le texte commence à 10 px : les colonnes 10 et 11 du coin (rayon 12) sont pleines
    constexpr uint8_t d = 12, step = 16, r = style::radius_big;
    const int16_t x = body.Left() + pad;
    const uint16_t w = body.Width() - 2 * pad;
    const uint8_t spools = kind == Kind::file ? std::popcount(info.used) : 0;
    const int16_t spool_x = body.Left() + body.Width() - pad - (spools * step - (step - d)); ///< bobines alignées à droite
    const Rect16 name_rect(x, body.Top() + 76, w, height(Font::small));
    const Rect16 line(x, body.Top() + 92, (spools ? spool_x - 4 : x + w) - x, height(Font::small));
    const Rect16 left_corner(text_area.Left(), text_area.Top() + text_area.Height() - r, pad, r);
    const Rect16 right_corner(text_area.Left() + text_area.Width() - pad, left_corner.Top(), pad, r);
    std::array<Rect16, 4 + print_info::max_tools> parts { name_rect, line, left_corner, right_corner };
    size_t count = 4;
    for (int16_t sx = spool_x; count < size_t(4 + spools); sx += step) {
        parts[count++] = Rect16(sx, line.Top() + 2, d, d);
    }
    style::for_each_gap(text_area, std::span<const Rect16>(parts.data(), count), [&](Rect16 gap) {
        display::fill_rect(gap, bg);
    });
    draw_pixels(left_corner, [&](int px, int py) { return spool::mix_quarters(bg, style::screen, spool::corner_beyond_quarters(px, r - 1 - py, r)); });
    draw_pixels(right_corner, [&](int px, int py) { return spool::mix_quarters(bg, style::screen, spool::corner_beyond_quarters(pad - 1 - px, r - 1 - py, r)); });

    std::array<char, 32> name;
    display_name(screen.display_name(*this), kind == Kind::file, name, w / width(Font::small));
    draw_text(name_rect, string_view_utf8::MakeRAM(name.data()), Font::small, style::text_strong, bg);

    switch (kind) {
    case Kind::file: {
        std::array<char, 32> text;
        summary(info, false, text);
        draw_text(line, string_view_utf8::MakeRAM(text.data()), Font::small, style::text_muted, bg);
        // Une bobine pleine par emplacement utilisé
        size_t i = 4;
        for (uint8_t t = 0; t < print_info::max_tools; ++t) {
            if (info.is_used(t)) {
                spool::Look look = Slot::read(t).look;
                look.fill_percent = 100;
                draw_spool({ parts[i].Left(), parts[i].Top() }, look, d, bg);
                ++i;
            }
        }
        break;
    }
    case Kind::folder:
        draw_text(line, _("Folder"), Font::small, style::text_muted, bg);
        break;
    case Kind::up:
        draw_text(line, _("Back"), Font::small, style::text_muted, bg);
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
        // Un fichier par cran ; la grille avance d'une rangée quand il sort de l'écran
        auto &ctx = *static_cast<GuiEventContext *>(param);
        if (screen.knob(*this, ctx.event.value<gui_event::KnobEvent>().diff)) {
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
    , tile(tile) {}

void FileThumb::unconditionalDraw() {
    const Rect16 box = GetRect();
    image_drawn = tile.kind != FileTile::Kind::file;
    screen.thumb_cleared(tile);
    if (tile.kind == FileTile::Kind::none) {
        display::fill_rect(box, style::screen);
        return;
    }
    if (tile.kind == FileTile::Kind::file && screen.draw_cached(tile, box)) {
        image_drawn = true;
        return;
    }
    // La case et son icône ; pour un fichier, l'image vient ensuite par bandes (ScreenFiles::load_next), ou l'icône reste
    display::draw_rounded_rect(box, style::screen, style::thumb_back, style::radius_big, MIC_TOP_LEFT | MIC_TOP_RIGHT);
    const img::Resource &icon = tile.kind == FileTile::Kind::file ? img::print_16x16
        : tile.kind == FileTile::Kind::folder                    ? img::folder_full_16x16
                                                                  : img::folder_up_16x16;
    draw_icon({ int16_t(box.Left() + (box.Width() - 16) / 2), int16_t(box.Top() + (box.Height() - 16) / 2) }, icon, style::thumb_back);
}

GridScroll::GridScroll(ScreenFiles &screen, Rect16 rect)
    : window_t(&screen, rect)
    , screen(screen) {}

void GridScroll::unconditionalDraw() {
    // Comme l'indicateur des listes (menu_od3d.cpp) : piste au-dessus, curseur, piste en dessous, chaque pixel peint une
    // fois ; le curseur avance d'une rangée à la fois
    const Rect16 r = GetRect();
    const int count = screen.item_count();
    if (count <= ScreenFiles::tiles_count) {
        display::fill_rect(r, style::screen);
        return;
    }
    constexpr uint8_t radius = 2;
    const int total_rows = (count + ScreenFiles::cols - 1) / ScreenFiles::cols;
    const int16_t h = std::max<int16_t>(r.Height() * ScreenFiles::rows / total_rows, 24);
    const int16_t y = (r.Height() - h) * (screen.first_item() / ScreenFiles::cols) / (total_rows - ScreenFiles::rows);
    const int16_t below = r.Height() - y - h;
    if (y > 0) {
        display::draw_rounded_rect(Rect16(r.Left(), r.Top(), r.Width(), y), style::screen, style::line, radius, MIC_TOP_LEFT | MIC_TOP_RIGHT);
    }
    const uint8_t ends = (y == 0 ? MIC_ALT_CL_TOP_LEFT | MIC_ALT_CL_TOP_RIGHT : 0) | (below == 0 ? MIC_ALT_CL_BOT_LEFT | MIC_ALT_CL_BOT_RIGHT : 0);
    display::draw_rounded_rect(Rect16(r.Left(), r.Top() + y, r.Width(), h), style::line, style::text_muted, radius, uint8_t(MIC_ALL_CORNERS | ends), style::screen);
    if (below > 0) {
        display::draw_rounded_rect(Rect16(r.Left(), r.Top() + y + h, r.Width(), below), style::screen, style::line, radius, MIC_BOT_LEFT | MIC_BOT_RIGHT);
    }
}

ScreenFiles::ScreenFiles()
    : Screen(true)
    , header(this, _("Files \xC2\xB7 USB drive"), true, back)
    , thumbs { { *this, tiles[0], 0 }, { *this, tiles[1], 1 }, { *this, tiles[2], 2 }, { *this, tiles[3], 3 } } // avant les cartes : leur marge passe sur les coins de la vignette
    , tiles { { *this, 0 }, { *this, 1 }, { *this, 2 }, { *this, 3 } }
    , scroll(*this, scroll_rect)
    , tabs(this, TabBar::Tab::print) {
    instance = this;
    open_thumb_cache();
    if (xPortGetFreeHeapSize() >= sizeof(DirWindow) + heap_margin) {
        if (void *block = malloc_fallible(sizeof(DirWindow))) {
            view = dir_window = new (block) DirWindow();
        }
    }
    metric_record_custom(&metric_od3d_files, " window=%ii", view->WindowSize());
}

ScreenFiles::~ScreenFiles() {
    stop_load();
    close_thumb_cache();
    if (dir_window) {
        dir_window->~DirWindow();
        free(dir_window);
        dir_window = nullptr;
    }
    if (instance == this) {
        instance = nullptr;
    }
}

void ScreenFiles::InitState(screen_init_variant var) {
    if (!var.GetPosition()) {
        // Nouvelle ouverture : racine de la clé, première page. Au retour d'« Avant l'impression », la même place.
        strlcpy(dir.data(), "/usb", dir.size());
        first = 0;
    }
    load();
    if (var.GetPosition()) {
        Screen::InitState(var);
    } else if (tiles[0].IsEnabled()) {
        tiles[0].SetFocus();
    }
}

bool ScreenFiles::in_subfolder() const {
    return strcmp(dir.data(), "/usb") != 0;
}

int ScreenFiles::item_count() const {
    return view->TotalFilesCount() - 1 + (in_subfolder() ? 1 : 0); // la liste compte toujours « .. »
}

void ScreenFiles::load() {
    loaded = true;
    const uint32_t start_ms = ticks_ms();
    view->ChangeDirectory(dir.data(), FileSort::SortPolicy::BY_CRMOD_DATETIME);
    first = grid::clamp_first(item_count(), cols, rows, first);
    fill();
    metric_record_custom(&metric_od3d_files, " dir=%ui,n=%ii", unsigned(ticks_diff(ticks_ms(), start_ms)), item_count());
    // Dans un sous-dossier, le retour (et le balayage) remonte d'un dossier : Prusa ne doit pas fermer l'écran à sa place
    // (balayage sans preneur, délai des menus) ; à la racine, il ferme l'écran
    if (in_subfolder()) {
        ClrMenuTimeoutClose();
    } else {
        SetMenuTimeoutClose();
    }
}

void ScreenFiles::fill() {
    stop_load();
    ready_logged = false;
    shown_ms = ticks_ms();

    // Entrées numérotées -1 (« .. ») à last ; à la racine, « .. » ne s'affiche pas. La page se lit dans la fenêtre en
    // mémoire ; le dossier n'est relu que si elle en sort (grid::window_start). Près de la fin, LazyDirView garde sa
    // fenêtre pleine : on décale alors l'index dans la fenêtre.
    const int count = item_count();
    const int last = view->TotalFilesCount() - 2;
    const int wanted = first - (in_subfolder() ? 1 : 0);
    const int start = view->set_window_offset(grid::window_start(view->TotalFilesCount(), view->WindowSize(), view->window_offset(), wanted, tiles_count));
    const bool was_shown = tiles[0].HasVisibleFlag();
    for (FileTile &t : tiles) {
        const int entry = wanted + t.index;
        FileTile::Kind kind = FileTile::Kind::none;
        uint8_t index = 0;
        if (entry <= last) {
            index = entry - start;
            kind = entry < 0 ? FileTile::Kind::up
                             : (view->display_name_at(index).second == FileSort::EntryType::DIR ? FileTile::Kind::folder : FileTile::Kind::file);
        }
        t.set(kind, index);
        t.set_visible(count > 0);
        thumbs[t.index].set_visible(count > 0);
        thumbs[t.index].Invalidate();
    }
    if (was_shown != (count > 0)) {
        Invalidate(); // message « Pas de fichier » (ou sa place à repeindre)
    }
    scroll.Invalidate();
    update_title();

    // Une carte vide ne garde pas le focus : la dernière carte pleine le prend (les vides sont en fin de grille)
    if (window_t *focused = get_focused_child_window(); !focused || !focused->IsEnabled()) {
        window_t *last_full = nullptr;
        for (FileTile &t : tiles) {
            if (t.IsEnabled()) {
                last_full = &t;
            }
        }
        if (window_t *w = last_full ? last_full : GetFirstEnabledSubWin()) {
            w->SetFocus();
        }
    }
}

void ScreenFiles::scroll_to(int new_first) {
    new_first = grid::clamp_first(item_count(), cols, rows, new_first);
    if (new_first != first) {
        const uint32_t start_ms = ticks_ms();
        first = new_first;
        fill(); // la carte ciblée garde sa place
        metric_record_custom(&metric_od3d_files, " scroll=%ui", unsigned(ticks_diff(ticks_ms(), start_ms)));
    }
}

void ScreenFiles::change_dir() {
    first = 0;
    load();
    if (tiles[0].IsEnabled()) {
        tiles[0].SetFocus();
    }
}

void ScreenFiles::update_title() {
    const int count = item_count();
    if (count <= tiles_count) {
        header.set_title(_("Files \xC2\xB7 USB drive"));
        return;
    }
    // « Fichiers 5-8/23 » : où l'on est dans la liste
    _("Files").copyToRAM(title_text);
    title_text.back() = '\0';
    const size_t len = strlen(title_text.data());
    snprintf(title_text.data() + len, title_text.size() - len, " %d-%d/%d", first + 1, std::min(first + int(tiles_count), count), count);
    header.set_title(string_view_utf8::MakeRAM(title_text.data()));
}

bool ScreenFiles::entry_path(const FileTile &t, std::span<char> out) const {
    const int n = snprintf(out.data(), out.size(), "%s/%s", dir.data(), view->canonical_name_at(t.view_index).first);
    return n > 0 && size_t(n) < out.size();
}

const char *ScreenFiles::display_name(const FileTile &t) const {
    return view->display_name_at(t.view_index).first;
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
            change_dir();
        }
        break;
    case FileTile::Kind::up:
        back();
        break;
    case FileTile::Kind::none:
        break;
    }
}

void ScreenFiles::back() {
    if (!instance || !instance->in_subfolder()) {
        Screens::Access()->close_screen();
        return;
    }
    if (char *slash = strrchr(dir.data(), '/')) {
        *slash = '\0';
    }
    instance->change_dir();
}

bool ScreenFiles::knob(const FileTile &tile, int step) {
    const auto view_after = grid::step(item_count(), cols, rows, { first, first + tile.index }, step);
    if (!view_after) {
        return false;
    }
    last_input_ms = ticks_ms();
    step_ms = step_ms ? step_ms : last_input_ms;
    fling = 0;
    scroll_to(view_after->first);
    tiles[view_after->focus - first].SetFocus();
    sound::play(SoundType::encoder_move);
    return true;
}

void ScreenFiles::thumb_cleared(const FileTile &tile) {
    if (loading == tile.index) {
        stop_load();
    }
}

bool ScreenFiles::draw_cached(const FileTile &tile, Rect16 box) const {
    std::array<char, filename_defs::path_buffer_size> path;
    if (!thumb_cache || !entry_path(tile, path)) {
        return false;
    }
    auto image = thumb_cache->find(InfoCache::key(path.data()));
    if (!image) {
        return false;
    }
    // La case autour de l'image, puis l'image : une rangée qui revient ne clignote pas
    const Rect16 img(box.Left() + (box.Width() - grid_img_w) / 2, box.Top() + (box.Height() - grid_img_h) / 2, grid_img_w, grid_img_h);
    static_assert(style::radius_big <= (tile_body(0).Width() - grid_img_w) / 2);
    big_thumb::draw_box(box, img, MIC_TOP_LEFT | MIC_TOP_RIGHT);
    display::draw_img(point_ui16(img.Left(), img.Top()), *image);
    return true;
}

void ScreenFiles::draw() {
    Screen::draw();
    if (step_ms) {
        metric_record_custom(&metric_od3d_files, " step=%ui", unsigned(ticks_diff(ticks_ms(), step_ms)));
        step_ms = 0;
    }
}

void ScreenFiles::unconditionalDraw() {
    Screen::unconditionalDraw();
    if (item_count() == 0) {
        const string_view_utf8 text = marlin_vars().media_inserted ? _("No printable file") : _("Insert a USB drive");
        draw_text(Rect16(grid_rect.Left(), 130, grid_rect.Width(), height(Font::small)), text, Font::small, style::text_muted, style::screen, Align_t::Center());
    }
}

void ScreenFiles::follow_finger() {
#if HAS_TOUCH()
    const auto at = touchscreen.held_point();
    if (!at || !grid_rect.Contain(touchscreen.held_start())) {
        if (dragging) {
            end_drag(); // levé sans geste reconnu (de travers) : l'inertie quand même
        }
        touching = false;
        return;
    }
    const uint32_t now = ticks_ms();
    if (!touching) {
        // Nouveau contact sur la grille
        touching = true;
        dragging = false;
        drag_first = first;
        last_y = at->y;
        last_touch_ms = now;
        velocity = 0;
        fling = 0;
    }
    last_input_ms = now; // pas de lecture de la clé sous le doigt
    const int dy = at->y - touchscreen.held_start().y;
    if (!dragging && (dy < 0 ? -dy : dy) < drag_min) {
        gui_reset_loop_timer();
        return;
    }
    dragging = true;
    // Vitesse sur des tranches de 40 ms au moins (l'écran tactile ne donne pas une position à chaque tour)
    if (const int32_t dt = ticks_diff(now, last_touch_ms); dt >= 40) {
        velocity = int32_t(at->y - last_y) * 1000 / dt;
        last_y = at->y;
        last_touch_ms = now;
    }
    scroll_to(grid::drag(item_count(), cols, rows, drag_first, dy, row_pitch));
    gui_reset_loop_timer(); // le doigt est suivi à chaque tour de l'interface, pas toutes les 100 ms
#endif
}

void ScreenFiles::end_drag() {
    dragging = false;
    touching = false;
    fling = int8_t(grid::fling_rows(velocity));
    fling_ms = ticks_ms();
    last_input_ms = fling_ms;
}

void ScreenFiles::stop_load() {
    if (thumb_cache) {
        thumb_cache->cancel(); // vignette pas finie : pas gardée
    }
    scaler.reset(); // avant le G-code : il lit sa vignette
    band_gcode.reset();
    loading = -1;
}

void ScreenFiles::start_load(FileTile &tile) {
    FileThumb &thumb = thumbs[tile.index];
    std::array<char, filename_defs::path_buffer_size> path;
    load_start_ms = ticks_ms();
    longest_band_ms = 0;
    band_gcode.emplace();
    if (!entry_path(tile, path) || !open_gcode(*band_gcode, path.data())) {
        tile.set_info({});
        thumb.set_drawn(); // l'icône reste
        stop_load();
        return;
    }
    if (tile.needs_info()) {
        // Les métadonnées d'un .bgcode sont avant ses vignettes, dans le même tampon de 4 Ko
        tile.set_info(read_metadata(*band_gcode, path.data()));
        metric_record_custom(&metric_od3d_files, " info=%ui", unsigned(ticks_diff(ticks_ms(), load_start_ms)));
    }
    if (thumb.missing()) {
        if (AbstractByteReader *qoi = (*band_gcode)->stream_thumbnail_start(thumb_w, thumb_h, IGcodeReader::ImgType::QOI)) {
            scaler.emplace(*qoi, grid_params);
            if (scaler->open()) {
                loading = tile.index; // les bandes aux LOOP suivants
                if (thumb_cache) {
                    thumb_cache->start(InfoCache::key(path.data()), scaler->width(), scaler->height());
                    scaler->record = thumb_cache; // gardée en dessinant
                }
                return;
            }
        }
        thumb.set_drawn(); // pas de vignette : l'icône reste
    }
    stop_load();
}

void ScreenFiles::load_next() {
    const uint32_t now = ticks_ms();
    // La molette ou le doigt viennent de servir : la clé attend, l'écran répond d'abord. Écran en veille (F39) : rien
    // n'est dessiné. Pendant une impression, la clé est à elle : les vignettes déjà gardées seulement.
    if (touching || dragging || fling || ticks_diff(now, last_input_ms) < input_pause_ms || sleep::screen_off() || print_uses_usb()) {
        return;
    }
    if (loading < 0) {
        // Carte ciblée d'abord, puis les autres dans l'ordre ; une carte pas encore dessinée attend son dessin
        FileTile *next = nullptr;
        const window_t *focused = get_focused_child_window();
        for (FileTile &t : tiles) {
            if ((thumbs[t.index].missing() || t.needs_info()) && !thumbs[t.index].IsInvalid() && !t.IsInvalid()) {
                if (&t == focused) {
                    next = &t;
                    break;
                }
                next = next ? next : &t;
            }
        }
        if (!next) {
            if (!ready_logged && std::ranges::none_of(thumbs, &FileThumb::missing)) {
                ready_logged = true;
                metric_record_custom(&metric_od3d_files, " ready=%ui", unsigned(ticks_diff(now, shown_ms)));
            }
            return;
        }
        start_load(*next); // ouverture et métadonnées : une tranche
    } else if (FileThumb &thumb = thumbs[loading]; !thumb.IsInvalid()) {
        // Une bande de la vignette, dessinée à sa place dans la case
        const Rect16 box = thumb.GetRect();
        const uint16_t row = scaler->row();
        if (scaler->band(band_rows)) {
            display::draw_img(point_ui16(box.Left() + (box.Width() - scaler->width()) / 2, box.Top() + (box.Height() - scaler->height()) / 2 + row), *scaler);
        }
        longest_band_ms = std::max<uint32_t>(longest_band_ms, ticks_diff(ticks_ms(), now));
        if (scaler->done() || scaler->row() == row) { // fini, ou la source s'arrête avant
            if (thumb_cache) {
                thumb_cache->finish(); // gardée si elle est complète
            }
            thumb.set_drawn();
            metric_record_custom(&metric_od3d_files, " thumb=%ui,band=%ui", unsigned(ticks_diff(ticks_ms(), load_start_ms)), unsigned(longest_band_ms));
            stop_load();
        }
    }
    gui_reset_loop_timer(); // la tranche suivante après le dessin et les entrées, sans attendre 100 ms
}

void ScreenFiles::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        if (!loaded) {
            load(); // si InitState n'a pas été appelé
        }
        if (!GetFirstDialog()) {
            follow_finger();
            if (fling != 0 && ticks_diff(ticks_ms(), fling_ms) >= int32_t(fling_step_ms)) {
                // Inertie : une rangée de plus, jusqu'au bout de la liste
                const int step = fling > 0 ? 1 : -1;
                const int before = first;
                scroll_to(first + step * cols);
                fling = first == before ? 0 : int8_t(fling - step);
                fling_ms = last_input_ms = ticks_ms();
            }
            if (fling != 0) {
                gui_reset_loop_timer();
            }
            load_next(); // pas sous un dialogue : chaque lecture de la clé le ralentirait
        }
        break;
    case GUI_event_t::KNOB:
    case GUI_event_t::CLICK:
        last_input_ms = ticks_ms();
        fling = 0;
        break;
    case GUI_event_t::TOUCH_CLICK:
        last_input_ms = ticks_ms();
        if (dragging) {
            end_drag(); // le doigt est revenu près de son départ : la grille a bougé, ce n'est pas un toucher
            return;
        }
        fling = 0;
        break;
    case GUI_event_t::MEDIA:
        if (const auto state = MediaState_t(int(param)); state == MediaState_t::inserted || state == MediaState_t::removed) {
            forget_file_infos();
            strlcpy(dir.data(), "/usb", dir.size());
            change_dir();
        }
        break;
    case GUI_event_t::TOUCH_SWIPE_UP:
    case GUI_event_t::TOUCH_SWIPE_DOWN:
        last_input_ms = ticks_ms();
        if (dragging) {
            end_drag(); // la grille a suivi le doigt : le lancer continue de quelques rangées
        } else {
            // Balayage trop rapide pour être suivi : une page
            fling = 0;
            scroll_to(first + (event == GUI_event_t::TOUCH_SWIPE_UP ? 1 : -1) * tiles_count);
        }
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
        draw_text(Rect16(body.Left(), y, body.Width(), height(Font::small)), label, Font::small, fg, fill(), Align_t::Center());
        return;
    }
    const int16_t x = body.Left() + (body.Width() - 16 - 6 - text_w) / 2;
    const ropfn rop = active ? ropfn { .swap_bw = has_swapped_bw::yes } : ropfn { .shadow = is_shadowed::yes };
    draw_icon({ x, y }, img::print_16x16, fill(), rop);
    draw_text(Rect16(x + 16 + 6, y, text_w, height(Font::small)), label, Font::small, fg, fill());
}

PreviewThumb::PreviewThumb(window_t *parent, Rect16 rect, const char *path)
    : window_t(parent, rect)
    , path(path) {}

PreviewThumb::~PreviewThumb() {
    big_thumb::stop(scaler);
}

void PreviewThumb::unconditionalDraw() {
    const Rect16 box = GetRect();
    big_thumb::stop(scaler); // une lecture commencée reprend du début
    loading = false;
    // Gardée (retour à l'aperçu, ou vue sur l'accueil) : la case autour de l'image, d'une passe
    image_drawn = big_thumb::cached(path, preview_params);
    if (image_drawn) {
        big_thumb::draw_box(box, preview_img, MIC_ALL_CORNERS);
        big_thumb::draw_cached(path, preview_params, point_ui16(preview_img.Left(), preview_img.Top()), scaler);
        return;
    }
    display::draw_rounded_rect(box, style::screen, style::thumb_back, style::radius_big, MIC_ALL_CORNERS);
    draw_icon({ int16_t(box.Left() + (box.Width() - 16) / 2), int16_t(box.Top() + (box.Height() - 16) / 2) }, img::print_16x16, style::thumb_back);
}

void PreviewThumb::load_step() {
    if (image_drawn || print_uses_usb()) {
        return; // pendant une impression, la case et son icône restent
    }
    if (!IsInvalid()) {
        // Par bandes de 8 lignes, comme la grille : l'écran répond pendant la lecture (avant : figé d'un bloc)
        loading = loading ? big_thumb::step(scaler, point_ui16(preview_img.Left(), preview_img.Top())) : big_thumb::start(path, preview_params, scaler);
        image_drawn = !loading; // fini, ou pas de vignette (l'icône reste)
    }
    if (!image_drawn) {
        gui_reset_loop_timer(); // la case à dessiner d'abord, ou la bande suivante : sans attendre 100 ms
    }
}

ScreenPreview::MeshChoice::MeshChoice(window_t *parent, Rect16 body, bool shown)
    : Card(parent, style::with_ring(body), [] { mesh_reuse = !mesh_reuse.value_or(true); }, style::secondary, style::screen, style::radius_big) {
    set_visible(shown);
    set_enabled(shown);
}

void ScreenPreview::MeshChoice::draw_content(Rect16 body) {
    // « Maillage », puis l'interrupteur et « repris » ou « nouveau » à places fixes : un changement repeint tout le texte
    const Color bg = fill();
    const bool reuse = mesh_reuse.value_or(false);
    draw_text(Rect16(body.Left(), body.Top() + 4, body.Width(), height(Font::small)), _("Bed mesh"), Font::small, style::text_muted, bg, Align_t::Center());
    const int16_t x = body.Left() + 4, y = body.Top() + 22;
    draw_icon({ x, y }, reuse ? img::switch_on_36x18 : img::switch_off_36x18, bg);
    draw_text(Rect16(x + 39, y + 1, body.Left() + body.Width() - 4 - (x + 39), height(Font::small)), reuse ? _("Reused") : _("Probed"), Font::small, style::text_strong, bg);
}

void ScreenPreview::MeshChoice::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    Card::windowEvent(sender, event, param);
    if (event == GUI_event_t::CLICK || event == GUI_event_t::TOUCH_CLICK) {
        invalidate_content();
    }
}

ScreenPreview::ScreenPreview()
    : Screen(true)
    , mesh_offered(mesh_memory::offered())
    , header(this, _("Before printing"), true)
    , thumb(this, preview_box, path.data())
    , print(this, mesh_offered ? print_narrow_body : print_body, print_now, true)
    , mesh(this, mesh_body, mesh_offered)
    , map(this, map_body, map_colours, false)
    , tabs(this, TabBar::Tab::print)
    , info(read_file_info(path.data())) {
    // Par défaut le réglage (désactivé : nouveau) ; pour cette impression seulement
    mesh_reuse = mesh_offered ? std::optional(mesh_memory::max_age_hours() != 0) : std::nullopt;
    std::array<char, filename_defs::path_buffer_size> sfn = path; // get_LFN veut un chemin modifiable
    std::array<char, filename_defs::filename_buffer_size> lfn;
    get_LFN(lfn.data(), lfn.size(), sfn.data());
    display_name(lfn.data(), true, name, (style::screen_w - style::margin - info_x) / width(Font::normal));
    update_buttons();
    print.SetFocus();
}

ScreenPreview::~ScreenPreview() {
    close_thumb_cache(); // ouvert depuis la grille : le cache reste pour son retour, sinon il est rendu
}

void ScreenPreview::update_buttons() {
    const bool active = can_print();
    print.set_active(active);
    map.set_active(active);
}

void ScreenPreview::unconditionalDraw() {
    Screen::unconditionalDraw();

    const uint16_t w = style::screen_w - style::margin - info_x;
    draw_text(Rect16(info_x, style::top, w, height(Font::normal)), string_view_utf8::MakeRAM(name.data()), Font::normal, style::text_strong, style::screen);
    std::array<char, 40> text;
    summary(info, true, text);
    draw_text(Rect16(info_x, style::top + 23, w, height(Font::small)), string_view_utf8::MakeRAM(text.data()), Font::small, style::text_muted, style::screen);

    // Une ligne par emplacement utilisé : bobine, « n · matière chargée », PA mesuré (F4), grammes du fichier (en
    // jaune s'ils dépassent le reste de la bobine, F13)
    int row = 0, mismatch = -1, shortage = -1, pa_missing = -1;
    float shortage_left = 0;
    Slot mismatch_slot;
    for (uint8_t t = 0; t < print_info::max_tools; ++t) {
        if (!info.is_used(t)) {
            continue;
        }
        const int16_t y = style::top + 42 + 19 * row++;
        const Slot slot = Slot::read(t);
        draw_spool({ info_x, y }, slot.look, 16, style::screen);
        snprintf(text.data(), text.size(), "%d%s%s", t + 1, dot, slot.material.data());
        draw_text(Rect16(info_x + 22, y, 100, height(Font::small)), string_view_utf8::MakeRAM(text.data()), Font::small, style::text_strong, style::screen);
        // PA : la valeur mesurée ; « PA ? » en jaune si le PA de l'imprimante prime et qu'il manque (sinon le
        // trancheur décide : rien)
        Color pa_color = style::text_muted;
        if (const auto k = pa::k_for_slot(t)) {
            snprintf(text.data(), text.size(), "%.3f", double(*k));
        } else if (pa::needs_measure(t) && slot.look.filament) {
            strlcpy(text.data(), "PA ?", text.size());
            pa_color = style::warning;
            if (pa_missing < 0) {
                pa_missing = t;
            }
        } else {
            text[0] = '\0';
        }
        draw_text(Rect16(info_x + 124, y, 56, height(Font::small)), string_view_utf8::MakeRAM(text.data()), Font::small, pa_color, style::screen);
        const auto usage = spool_usage::report(t);
        const bool too_little = usage && slot.look.filament && spool_usage::not_enough(info.grams[t], usage->remaining_g);
        if (too_little && shortage < 0) {
            shortage = t;
            shortage_left = usage->remaining_g;
        }
        snprintf(text.data(), text.size(), "%.0f g", double(info.grams[t]));
        draw_text(Rect16(info_x + 182, y, w - 182, height(Font::small)), string_view_utf8::MakeRAM(text.data()), Font::small, too_little ? style::warning : style::text_muted, style::screen, Align_t::RightCenter());
        if (mismatch < 0 && info.types[t][0] && slot.look.filament && strcasecmp(info.types[t].data(), slot.material.data()) != 0) {
            mismatch = t;
            mismatch_slot = slot;
        }
    }

    // Alertes (§3.10) : la bobine n'a pas assez de filament pour le fichier (F13 ; la fin d'une bobine n'arrête pas
    // l'impression, le capteur la met en pause) ; la matière chargée n'est pas celle du fichier (Prusa la vérifie aussi
    // au départ). Une alerte prend les deux lignes ; deux alertes, une ligne chacune.
    StringViewUtf8Parameters<24> shortage_params, mismatch_params, pa_params;
    string_view_utf8 warnings[3];
    int count = 0;
    if (shortage >= 0) {
        warnings[count++] = _("Slot %d: %d g needed, about %d g left").formatted(shortage_params, shortage + 1, int(std::lround(info.grams[shortage])), int(shortage_left));
    }
    if (mismatch >= 0) {
        warnings[count++] = _("Slot %d: %s in the file, %s loaded").formatted(mismatch_params, mismatch + 1, info.types[mismatch].data(), mismatch_slot.material.data());
    }
    if (pa_missing >= 0) {
        // F4 : « PA à mesurer » ; le M572 du trancheur reste utilisé pour cet emplacement
        warnings[count++] = _("Slot %d: PA to measure (slicer value used)").formatted(pa_params, pa_missing + 1);
    }
    count = std::min(count, 2); // deux lignes au plus : les alertes de filament d'abord
    const uint16_t warning_w = style::screen_w - 2 * style::margin;
    if (count == 1) {
        render_text_align(Rect16(style::margin, warning_y, warning_w, 2 * height(Font::small)), warnings[0], Font::small, style::screen, style::warning, {}, text_flags(Align_t::LeftTop(), is_multiline::yes));
    } else {
        for (int i = 0; i < count; ++i) {
            draw_text(Rect16(style::margin, warning_y + i * height(Font::small), warning_w, height(Font::small)), warnings[i], Font::small, style::warning, style::screen);
        }
    }
}

void ScreenPreview::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    if (event == GUI_event_t::LOOP || event == GUI_event_t::MEDIA) {
        update_buttons();
    }
    if (event == GUI_event_t::LOOP && !GetFirstDialog() && !sleep::screen_off()) {
        thumb.load_step(); // après le premier dessin de l'écran
    }
    Screen::windowEvent(sender, event, param);
}

} // namespace od3d
