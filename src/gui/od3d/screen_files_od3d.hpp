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

/// Lit les métadonnées d'un G-code (temps, filament, matières) ; vide s'il ne s'ouvre pas
print_info::FileInfo read_file_info(const char *path);

/// Vignette 313 × 173 du G-code réduite d'un facteur entier, centrée dans box sur le fond des vignettes ;
/// grey_rows lignes du haut en gris. false s'il n'y a pas de vignette (box n'est pas touchée).
bool draw_thumbnail(const char *path, Rect16 box, uint8_t factor, std::optional<ScaledQoiReader> &scaler, uint16_t grey_rows = 0);

/// Anneau de focus seul : la marge de 2 px autour du corps et les coins du corps, comme draw_body() sans le corps
void draw_ring(Rect16 outer, Color ring, Color back, uint8_t radius);

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

    /// Affiche l'entrée index_in_view de la liste (Kind::none : carte cachée)
    void set(Kind kind, uint8_t index_in_view);

    Kind kind = Kind::none;
    uint8_t view_index = 0; ///< place dans la fenêtre de la liste (LazyDirView)

protected:
    void unconditionalDraw() override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    ScreenFiles &screen;
    print_info::FileInfo info;
};

/// Haut d'une carte : la vignette du fichier, ou l'icône du dossier
class FileThumb : public window_t {
public:
    FileThumb(ScreenFiles &screen, const FileTile &tile, uint8_t index);

protected:
    void unconditionalDraw() override;

private:
    ScreenFiles &screen;
    const FileTile &tile;
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

    std::optional<ScaledQoiReader> scaler; ///< une vignette à la fois ; dans l'écran plutôt que sur la pile (6 Ko)

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
    void unconditionalDraw() override;

private:
    static constexpr uint8_t tiles_count = 4;

    void load();
    void show(int new_first, FileTile &focus); ///< recharge à partir de new_first et redessine tout
    int item_count() const; ///< entrées affichables, « .. » compris dans un sous-dossier
    bool in_subfolder() const;

    Header header;
    FileThumb thumbs[tiles_count];
    FileTile tiles[tiles_count];
    TabBar tabs;
    LazyDirView<tiles_count> view;
    bool loaded = false;

    // Gardés quand l'écran est recréé (retour d'« Avant l'impression ») ; remis à zéro à chaque ouverture
    static std::array<char, filename_defs::path_buffer_size> dir;
    static int first; ///< rang de la première carte dans la liste
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
    Button print;
    Button map;
    TabBar tabs;
    print_info::FileInfo info;
    std::array<char, 32> name {};
    std::optional<ScaledQoiReader> scaler;
};

} // namespace od3d
