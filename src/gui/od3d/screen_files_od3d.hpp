/// @file
/// OneDrop3D (F1, interface G) : « Fichiers » (grille 2 × 2 des G-codes de la clé avec leur aperçu, SPEC_G.md §3.9)
/// et « Avant l'impression » (aperçu, bobines du fichier, « Imprimer », §3.10). « Imprimer » passe par le départ
/// d'impression de Prusa (marlin_client::print_start) : ses vérifications et l'association des outils restent.
#pragma once

#include "print_info_od3d.hpp"
#include "thumbnail_od3d.hpp"
#include "widgets_od3d.hpp"

#include <array>
#include <buddy/filename_defs.hpp>
#include <lazyfilelist.hpp>
#include <optional>
#include <span>

class AnyGcodeFormatReader;

namespace od3d {

/// Ouvre « Avant l'impression » pour un G-code (chemin canonique, comme celui de la liste des fichiers)
void open_preview(const char *path);

/// Ouvre un G-code de la clé, lu par blocs de 4 Ko : newlib lirait la clé secteur par secteur (une commande USB par
/// 512 octets, ~60 pour une vignette)
bool open_gcode(AnyGcodeFormatReader &gcode, const char *path);

/// Lit les métadonnées d'un G-code (temps, filament, matières) ; vide s'il ne s'ouvre pas. Les 16 dernières lues
/// restent en mémoire : la grille, ses pages et « Avant l'impression » ne relisent pas la clé.
print_info::FileInfo read_file_info(const char *path);

/// Les métadonnées déjà lues de ce fichier, sans toucher à la clé
std::optional<print_info::FileInfo> cached_file_info(const char *path);

/// Oublie les métadonnées lues (clé retirée ou changée)
void forget_file_infos();

/// Vignette 313 × 173 du G-code réduite selon params, centrée dans box. false s'il n'y a pas de vignette (box n'est
/// pas touchée).
bool draw_thumbnail(const char *path, Rect16 box, const ScaledQoiReader::Params &params, std::optional<ScaledQoiReader> &scaler);

/// La même, réduite d'un facteur entier sur le fond des vignettes ; grey_rows lignes du haut en gris
bool draw_thumbnail(const char *path, Rect16 box, uint8_t factor, std::optional<ScaledQoiReader> &scaler, uint16_t grey_rows = 0);

class ScreenFiles;

/// Carte d'un fichier ou d'un dossier de la grille : nom, « durée · grammes », bobines utilisées. La vignette est une
/// fenêtre à part (FileThumb) : le focus ne la redessine pas.
class FileTile : public window_t {
public:
    enum class Kind : uint8_t {
        none, ///< pas d'entrée : la place est repeinte au fond de l'écran
        file,
        folder,
        up, ///< « .. » : dossier parent
    };

    FileTile(ScreenFiles &screen, uint8_t index);

    /// Affiche l'entrée index_in_view de la liste. Les métadonnées viennent du cache, sinon avec la vignette (la carte
    /// s'affiche sans attendre la clé).
    void set(Kind kind, uint8_t index_in_view);

    /// Métadonnées lues avec la vignette : la ligne « durée · grammes » se redessine
    void set_info(const print_info::FileInfo &set);

    /// Les métadonnées restent à lire sur la clé
    bool needs_info() const { return info_pending; }

    Kind kind = Kind::none;
    uint8_t view_index = 0; ///< place dans la fenêtre de la liste (LazyDirView)
    const uint8_t index; ///< place dans la grille

protected:
    void unconditionalDraw() override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    ScreenFiles &screen;
    print_info::FileInfo info;
    bool info_pending = false;
};

/// Haut d'une carte : la case de la vignette et son icône ; l'image y est dessinée par bandes par ScreenFiles, quand
/// la molette et le doigt sont au repos
class FileThumb : public window_t {
public:
    /// index : celui de la carte, construite après la vignette (son anneau passe sur les coins de la case)
    FileThumb(ScreenFiles &screen, const FileTile &tile, uint8_t index);

    /// Vignette encore à lire
    bool missing() const { return tile.kind == FileTile::Kind::file && !image_drawn; }
    void set_drawn() { image_drawn = true; }

protected:
    void unconditionalDraw() override;

private:
    ScreenFiles &screen;
    const FileTile &tile;
    bool image_drawn = false;
};

/// Indicateur de position de la grille (piste de 4 px et curseur clair, comme les listes), dans la marge droite
class GridScroll : public window_t {
public:
    GridScroll(ScreenFiles &screen, Rect16 rect);

protected:
    void unconditionalDraw() override;

private:
    ScreenFiles &screen;
};

/// Grille 2 × 2 des fichiers. La molette avance d'un fichier et la grille d'une rangée ; le doigt tire la grille rangée
/// par rangée, un lancer continue de quelques rangées, un balayage rapide tourne la page. Les noms s'affichent tout de
/// suite ; vignettes et « durée · grammes » se lisent ensuite, carte ciblée d'abord, une bande par LOOP.
class ScreenFiles final : public Screen {
public:
    ScreenFiles();
    ~ScreenFiles();

    void InitState(screen_init_variant var) override;

    /// Chemin de l'entrée d'une carte (dossier courant + nom canonique) ; false si trop long
    bool entry_path(const FileTile &tile, std::span<char> out) const;
    const char *display_name(const FileTile &tile) const;

    /// Action d'une carte : ouvre le fichier ou le dossier
    void open(const FileTile &tile);

    /// Un cran de molette sur une carte ; false au bout de la liste (la molette passe à l'en-tête ou aux onglets)
    bool knob(const FileTile &tile, int step);

    /// La case d'une vignette vient d'être repeinte : sa lecture en cours est abandonnée (elle reprendra)
    void thumb_cleared(const FileTile &tile);

    int item_count() const; ///< entrées affichables, « .. » compris dans un sous-dossier
    int first_item() const { return first; }

    static constexpr uint8_t cols = 2, rows = 2, tiles_count = cols * rows;

    /// Retour (bouton, balayage) : le dossier parent comme « .. », l'écran ne se ferme qu'à la racine de la clé
    static void back();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
    void unconditionalDraw() override;

private:
    void load(); ///< relit le dossier, puis fill()
    void fill(); ///< cartes à partir de first (la liste ne se relit pas en entier)
    void scroll_to(int new_first); ///< la carte ciblée garde sa place dans la grille
    void change_dir(); ///< nouveau dossier : première page, focus sur la première carte
    bool in_subfolder() const;
    void update_title();

    void follow_finger(); ///< LOOP : la grille suit le doigt posé sur elle
    void end_drag(); ///< doigt levé après un glissement : inertie

    /// Une bande de vignette (ou l'ouverture d'un fichier) par LOOP, si la molette et le doigt sont au repos ; le LOOP
    /// suivant est avancé tant qu'il reste à lire
    void load_next();
    void start_load(FileTile &tile);
    void stop_load();

    Header header;
    FileThumb thumbs[tiles_count];
    FileTile tiles[tiles_count];
    GridScroll scroll;
    TabBar tabs;
    LazyDirView<tiles_count> view;
    std::optional<ScaledQoiReader> scaler; ///< une vignette à la fois ; dans l'écran plutôt que sur la pile (6 Ko)
    bool loaded = false;
    bool ready_logged = false; ///< metric « ready » envoyée pour cette page
    int8_t loading = -1; ///< carte dont la vignette se dessine par bandes, -1 sans
    uint32_t load_start_ms = 0;
    uint32_t longest_band_ms = 0; ///< metric « band » : le plus long moment sans réponse de l'écran
    uint32_t last_input_ms = 0; ///< dernière molette, clic ou toucher
    uint32_t shown_ms = 0; ///< page affichée (metric « ready »)

    // Doigt posé sur la grille
    bool touching = false;
    bool dragging = false; ///< la grille a suivi le doigt : son lever n'est ni un toucher ni un balayage
    int drag_first = 0; ///< first au moment où le doigt s'est posé
    int16_t last_y = 0;
    uint32_t last_touch_ms = 0;
    int32_t velocity = 0; ///< px/s, moyenne glissante
    int8_t fling = 0; ///< rangées d'inertie restantes (signe : sens du défilement)
    uint32_t fling_ms = 0;

    static ScreenFiles *instance; ///< pour back()

    // Gardés quand l'écran est recréé (retour d'« Avant l'impression ») ; remis à zéro à chaque ouverture
    static std::array<char, filename_defs::path_buffer_size> dir;
    static int first; ///< rang de la carte en haut à gauche (début de rangée)
};

/// Aperçu d'« Avant l'impression » : la case et son icône tout de suite, la vignette au LOOP suivant (lecture sur la clé)
class PreviewThumb : public window_t {
public:
    PreviewThumb(window_t *parent, Rect16 rect, const char *path);

    /// Demande le dessin de la vignette une fois la case affichée ; false s'il n'y a rien à faire
    bool request_image();

    /// La case n'est pas encore dessinée : la vignette sera demandée au LOOP suivant
    bool waiting() const { return !image_drawn && !image_requested; }

protected:
    void unconditionalDraw() override;

private:
    const char *path;
    std::optional<ScaledQoiReader> scaler;
    bool image_drawn = false;
    bool image_requested = false;
};

/// « Avant l'impression » : vignette, nom, « durée · grammes · mètres », une ligne par emplacement utilisé,
/// alerte si la matière chargée diffère de celle du fichier, « Imprimer » et « Associer les couleurs »
class ScreenPreview final : public Screen {
public:
    ScreenPreview();

    static std::array<char, filename_defs::path_buffer_size> path; ///< fichier montré (posé par open_preview)

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
    void unconditionalDraw() override;

private:
    /// Bouton du bas : « Imprimer » (principal, orange) ou « Associer les couleurs » (secondaire)
    class Button : public Card {
    public:
        Button(window_t *parent, Rect16 body, Action action, bool primary);

        /// Inactif (pas de clé, impression en cours) : grisé ; l'action vérifie aussi
        void set_active(bool set);

    protected:
        void draw_content(Rect16 body) override;

    private:
        bool primary;
        bool active = true;
    };

    void update_buttons();

    Header header;
    PreviewThumb thumb;
    Button print;
    Button map;
    TabBar tabs;
    print_info::FileInfo info;
    std::array<char, 32> name {};
};

} // namespace od3d
