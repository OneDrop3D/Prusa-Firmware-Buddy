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

namespace od3d {

/// Ouvre « Avant l'impression » pour un G-code (chemin canonique, comme celui de la liste des fichiers)
void open_preview(const char *path);

/// Lit les métadonnées d'un G-code (temps, filament, matières) ; vide s'il ne s'ouvre pas. Les 16 dernières lues
/// restent en mémoire : la grille, ses pages et « Avant l'impression » ne relisent pas la clé (lente, ~0,1 s par fichier).
print_info::FileInfo read_file_info(const char *path);

/// Les métadonnées déjà lues de ce fichier, sans toucher à la clé
std::optional<print_info::FileInfo> cached_file_info(const char *path);

/// Oublie les métadonnées lues (clé retirée ou changée)
void forget_file_infos();

/// Vignette 313 × 173 du G-code réduite d'un facteur entier, centrée dans box sur le fond des vignettes ;
/// grey_rows lignes du haut en gris. false s'il n'y a pas de vignette (box n'est pas touchée).
bool draw_thumbnail(const char *path, Rect16 box, uint8_t factor, std::optional<ScaledQoiReader> &scaler, uint16_t grey_rows = 0);

class ScreenFiles;

/// Carte d'un fichier ou d'un dossier de la grille : anneau, nom, « durée · grammes », bobines utilisées.
/// La vignette est une fenêtre à part (FileThumb) : le focus ne la fait pas relire sur la clé.
class FileTile : public window_t {
public:
    enum class Kind : uint8_t {
        none,
        file,
        folder,
        up, ///< « .. » : dossier parent
    };

    FileTile(ScreenFiles &screen, uint8_t index);

    /// Affiche l'entrée index_in_view de la liste (Kind::none : carte cachée). Les métadonnées viennent du cache,
    /// sinon plus tard (load_info) : la grille s'affiche sans attendre la clé.
    void set(Kind kind, uint8_t index_in_view);

    /// Lit les métadonnées qui manquent, une fois la carte dessinée ; false s'il n'y avait rien à lire
    bool load_info();

    /// Les métadonnées restent à lire sur la clé
    bool needs_info() const { return info_pending; }

    Kind kind = Kind::none;
    uint8_t view_index = 0; ///< place dans la fenêtre de la liste (LazyDirView)

protected:
    void unconditionalDraw() override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    ScreenFiles &screen;
    print_info::FileInfo info;
    bool info_pending = false;
};

/// Haut d'une carte : la vignette du fichier, ou l'icône du dossier. La vignette (lecture sur la clé, ~0,1 s) vient
/// après le dessin de la grille, une par LOOP : l'écran s'affiche sans l'attendre et la molette répond entre deux.
class FileThumb : public window_t {
public:
    FileThumb(ScreenFiles &screen, const FileTile &tile, uint8_t index);

    /// Demande le dessin de la vignette si elle manque ; false s'il n'y a rien à faire
    bool request_image();

    /// Vignette demandée, lue au prochain dessin
    bool requested() const { return image_requested; }

    /// Vignette encore à lire (carte visible)
    bool missing() const { return IsVisible() && !image_drawn; }

protected:
    void unconditionalDraw() override;

private:
    ScreenFiles &screen;
    const FileTile &tile;
    bool image_drawn = false;
    bool image_requested = false;
};

/// Barre de défilement de la grille (SPEC_G.md §1.7), dans la marge droite : seule à se repeindre avec les cartes
/// quand la page tourne (l'écran entier, en-tête et onglets compris, se repeignait)
class GridScroll : public window_t {
public:
    GridScroll(ScreenFiles &screen, Rect16 rect);

protected:
    void unconditionalDraw() override;

private:
    ScreenFiles &screen;
};

class ScreenFiles final : public Screen {
public:
    ScreenFiles();

    void InitState(screen_init_variant var) override;

    /// Chemin de l'entrée d'une carte (dossier courant + nom canonique) ; false si trop long
    bool entry_path(const FileTile &tile, std::span<char> out) const;
    const char *display_name(const FileTile &tile) const;

    /// Action d'une carte : ouvre le fichier ou le dossier
    void open(const FileTile &tile);

    /// Page suivante (+1) ou précédente (-1), focus sur la première ou la dernière carte ; false s'il n'y en a pas
    bool turn_page(int direction);

    FileTile &tile(uint8_t i) { return tiles[i]; }

    int item_count() const; ///< entrées affichables, « .. » compris dans un sous-dossier
    int first_item() const { return first; }
    static constexpr uint8_t tiles_count = 4;

    std::optional<ScaledQoiReader> scaler; ///< une vignette à la fois ; dans l'écran plutôt que sur la pile (6 Ko)

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
    void unconditionalDraw() override;

private:
    void load();
    void show(int new_first, FileTile &focus); ///< recharge à partir de new_first : cartes et barre seulement
    bool in_subfolder() const;

    /// Une lecture de la clé (métadonnées, puis vignettes) si la molette et le toucher sont au repos ; le LOOP
    /// suivant est avancé tant qu'il reste à lire : les cartes se complètent sans attendre 100 ms entre deux
    void load_next();

    Header header;
    FileThumb thumbs[tiles_count];
    FileTile tiles[tiles_count];
    GridScroll scroll;
    TabBar tabs;
    LazyDirView<tiles_count> view;
    bool loaded = false;
    bool ready_logged = false; ///< metric « ready » envoyée pour cette page
    uint32_t last_input_ms = 0; ///< dernière molette, clic ou toucher
    uint32_t shown_ms = 0; ///< page affichée (metric « ready »)

    // Gardés quand l'écran est recréé (retour d'« Avant l'impression ») ; remis à zéro à chaque ouverture
    static std::array<char, filename_defs::path_buffer_size> dir;
    static int first; ///< rang de la première carte dans la liste
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
