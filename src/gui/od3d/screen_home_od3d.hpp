/// @file
/// OneDrop3D (F1, interface G) : l'accueil, écran de démarrage et bas de la pile (SPEC_G.md §3.1).
/// Bande des 5 bobines du MMU, dernier fichier avec sa vignette et « Imprimer », cartes Fichiers,
/// Températures et Contrôle. Il reprend ce que faisait l'accueil Prusa (screen_home_data_t) : clé USB,
/// impression en un clic, .ini du Wi-Fi, NFC, état du MMU.
#pragma once

#include <option/has_mmu2.h>

#include "thumbnail_od3d.hpp"
#include "widgets_od3d.hpp"

#include <array>
#include <buddy/filename_defs.hpp>
#include <media_state.hpp>
#include <optional>
#include <tool_index.hpp>
#include <utility>
#include <option/has_nfc.h>

#if HAS_NFC()
    #include <nfc.hpp>
#endif

namespace od3d {

/// Bobine de la bande de l'accueil, en orange si elle est chargée dans la buse. Selon la place (layout_od3d.hpp) :
/// étroite (MMU3) bobine en haut et « n MATIÈRE » dessous ; large (sans MMU, INDX 3 têtes) bobine Ø40 à gauche, matière
/// et état à droite ; basse (INDX 8 têtes, deux rangées) petite bobine et « n MATIÈRE » sur une ligne.
class SpoolCard : public Card {
public:
    /// tools : bobines montrées (lu à l'exécution) ; au-delà, la carte est cachée
    SpoolCard(window_t *parent, uint8_t index, uint8_t tools);

    /// Relit l'emplacement et le redessine s'il a changé ; mmu_ready est faux pendant que le MMU démarre
    void update(bool mmu_ready);

protected:
    void draw_content(Rect16 body) override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    Slot slot;
    uint8_t index;
    bool single; ///< seule bobine (sans MMU) : la matière sans numéro
};

/// Carte du dernier fichier : vignette, « Dernier fichier », nom. « Imprimer » est un élément à part,
/// pour que le focus ne redessine pas la vignette. Gardée en mémoire, la vignette est dessinée avec la carte, d'une
/// passe ; sinon elle est lue sur la clé par bandes aux LOOP qui suivent le dessin de la carte (jamais pendant une
/// impression : l'icône reste).
class FileCard : public window_t {
public:
    explicit FileCard(window_t *parent);
    ~FileCard();

    /// @param latest chemin du dernier fichier, nullptr s'il n'y en a pas
    /// @return la carte a changé (elle se redessine)
    bool set(bool usb, const char *latest);

    /// Un fichier était connu (clé insérée) à la dernière ouverture de l'accueil : l'écran le reprend sans relire la clé
    static bool known() { return state != State::no_usb; }
    static bool has_file() { return state == State::file; }

    void print() const;

    /// LOOP : une bande de la vignette, une fois la carte affichée
    void load_step();

protected:
    void unconditionalDraw() override;

private:
    enum class State : uint8_t {
        no_usb,
        no_file,
        file,
    };

    // Gardés d'une ouverture de l'accueil à l'autre (l'écran est recréé à chaque retour)
    static State state;
    static std::array<char, filename_defs::path_buffer_size> path;
    static std::array<char, 64> name; ///< nom affiché, sans extension, coupé à la largeur de la carte

    bool thumb_drawn = false; ///< la vignette (ou son icône) est à l'écran
    bool loading = false; ///< vignette en cours de lecture sur la clé
    std::optional<ScaledQoiReader> scaler; ///< dans l'écran plutôt que sur la pile de l'interface (6 Ko)
};

/// « Imprimer » de la carte du fichier ; le toucher marche sur toute la carte
class PrintButton : public Card {
public:
    PrintButton(window_t *parent, FileCard &file);

    Rect16 get_rect_for_touch() const override;

protected:
    void draw_content(Rect16 body) override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    FileCard &file;
};

class ScreenHome final : public Screen {
public:
    ScreenHome();
    ~ScreenHome();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    template <size_t... I>
    explicit ScreenHome(std::index_sequence<I...>);

    void refresh_file();
    bool file_stale() const; ///< la clé a changé depuis sa dernière lecture (autre clé, fichier envoyé ou effacé)
    void update_spools();
#if HAS_NFC()
    void update_nfc_state();
#endif

    Header header;
    SpoolCard spools[VirtualToolIndex::count]; ///< une par outil possible ; celles des outils absents sont cachées
#if HAS_MMU2()
    IconButton mmu; ///< l'étagère ; INDX : la bande entière aux têtes (IG-08), l'étagère par l'onglet Atelier
#endif
    FileCard file;
    PrintButton print;
    IconCard files;
    TempsCard temps;
    IconCard control;
    TabBar tabs;

    bool usb_inserted;
    bool event_in_progress = false;
    MediaState_t media_event = MediaState_t::unknown;
    static bool usb_was_already_inserted; ///< une clé a déjà été vue depuis le démarrage
    static bool need_check_wifi_credentials;
    // La clé à sa dernière lecture : le dossier n'est relu que s'ils changent
    static uint16_t scanned_mount;
    static uint32_t scanned_changes;

#if HAS_NFC()
    std::optional<nfc::SharedEnabler> nfc_enable { std::in_place };
#endif
};

} // namespace od3d
