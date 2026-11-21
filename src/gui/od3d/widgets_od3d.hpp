/// @file
/// OneDrop3D (F1, interface G) : briques communes de nos écrans (SPEC_G.md §1) — base d'écran, en-tête,
/// barre d'onglets, cartes, bobines, températures. Chaque élément actif se touche et se choisit à la molette
/// (ordre de la molette = ordre de construction ; clic = toucher ; appui long = retour sur les sous-écrans).
#pragma once

#include "spool_draw.hpp"
#include "style_od3d.hpp"

#include <array>
#include <optional>
#include <raster_opfn.hpp>
#include <ScreenFactory.hpp>
#include <ScreenHandler.hpp>
#include <screen.hpp>
#include <window_icon.hpp>
#include <window_text.hpp>

namespace od3d {

/// Emplacement du MMU tel qu'affiché
struct Slot {
    spool::Look look; ///< bobine sans couleur : emplacement vide ou inutilisable
    std::array<char, 8> material {}; ///< nom de la matière, « - » sans bobine
    bool loaded = false; ///< chargé dans la buse
    bool enabled = false; ///< utilisable (sans MMU activé, seul le premier l'est)

    static Slot read(uint8_t index);
    bool operator==(const Slot &) const = default;
};

// ---- Dessin ----
// Chaque pixel est peint une fois, sans effacement préalable : c'est ce qui évite les clignotements, et l'écran (SPI à
// 3 octets par pixel) est lent. Un élément repeint tout son rect ; l'écran ne peint que ce qui reste entre eux.

/// La marge de 2 px autour du corps (ancienne place de l'anneau de focus), au fond back, sans toucher au corps
void draw_margin(Rect16 outer, Color back);

/// Corps arrondi dans outer moins la marge de 2 px, marge comprise, au fond back
void draw_body(Rect16 outer, Color body, Color back, uint8_t radius);

/// Rectangle arrondi avec un contour de border_w px, d'une seule passe : les coins du contour suivent l'arrondi (deux
/// rectangles arrondis emboîtés débordaient aux coins) ; hors de l'arrondi, la couleur back
void draw_rounded_frame(Rect16 rect, Color back, Color border, Color fill, uint8_t radius, uint8_t border_w);

/// draw_rounded_frame() d'un élément (puces, choix, danger) : outer comprend sa marge de 2 px, peinte au fond back
void draw_frame(Rect16 outer, Color back, Color border, Color fill, uint8_t radius, uint8_t border_w);

/// Une ligne de texte dans rect, fond repeint ; ce qui dépasse est coupé
void draw_text(Rect16 rect, const string_view_utf8 &text, Font font, Color fg, Color bg, Align_t align = Align_t::LeftCenter());

/// Icône sur un fond uni ; swap_bw = noire au lieu de blanche (sur l'orange), shadow = atténuée
void draw_icon(point_i16_t at, const img::Resource &icon, Color bg, ropfn rop = {});

/// Bobine de diamètre d, coin haut gauche en at, en une image : le carré hors du disque prend la couleur back ;
/// ring (emplacement chargé) : anneau de 2 px autour du disque, l'image fait alors d + 4 px depuis at - 2
void draw_spool(point_i16_t at, const spool::Look &look, uint8_t d, Color back, std::optional<Color> ring = std::nullopt);

/// Fond de frame entre ses fenêtres normales visibles (first à last), qui repeignent chacune tout leur rect
void fill_background(const window_t &frame, window_t *first, window_t *last);

// ---- Navigation ----

/// Ouvre un écran par-dessus l'écran courant
template <class S>
void open_screen() {
    Screens::Access()->Open(ScreenFactory::Screen<S>);
}

/// Ouvre une section (onglet) : la pile redevient [accueil, écran]
template <class S>
void open_section() {
    Screens::Access()->CloseAll(); // avant Open(), qui serait sinon oublié
    open_screen<S>();
}

/// Menu du filament : celui du MMU s'il est actif, celui de Prusa sans MMU s'il est arrêté, rien pendant qu'il démarre
void open_filament_menu();

/// Fichiers de la clé USB (rien sans clé : le navigateur de Prusa se refermerait aussitôt)
void open_files();

/// Aperçu des températures (screen_temperatures_od3d.hpp)
void open_temperatures();

// ---- Écran ----

/// Base de nos écrans : fond de la maquette, focus retrouvé au retour d'un sous-écran, appui long = retour
class Screen : public screen_t {
public:
    explicit Screen(bool has_back);

    screen_init_variant GetCurrentState() const override;
    void InitState(screen_init_variant var) override;

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
    void unconditionalDraw() override; ///< le fond seulement entre les éléments

private:
    bool has_back;
};

// ---- Éléments ----

/// Élément actif : corps arrondi, plus clair quand il a le focus (style::focused), action au clic ou au toucher.
/// Son rect comprend une marge de 2 px : le créer avec style::with_ring(corps).
class Card : public window_t {
public:
    using Action = void (*)();

    Card(window_t *parent, Rect16 rect, Action action, Color body_color = style::surface, Color back = style::screen, uint8_t radius = style::radius);

protected:
    /// Le rect sans la marge
    Rect16 body() const;

    /// Couleur du corps telle que dessinée (plus claire avec le focus) : le fond du contenu
    Color fill() const;

    /// Redessine le contenu seul, sans repeindre le corps (une valeur a changé : pas de clignotement)
    void invalidate_content();

    /// Contenu, dessiné sur le corps après lui (ou seul après invalidate_content) : il repeint tout ce qu'il occupe
    virtual void draw_content([[maybe_unused]] Rect16 body) {}

    void unconditionalDraw() override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

    Action action;
    Color body_color;
    Color back;
    uint8_t radius;
};

/// Bouton « icône au centre », avec un libellé dessous s'il en a un (retour de l'en-tête, « MMU > »)
class IconButton : public Card {
public:
    IconButton(window_t *parent, Rect16 rect, Action action, const img::Resource &icon, const string_view_utf8 &label = {}, Color body_color = style::surface, uint8_t radius = style::radius);

protected:
    void draw_content(Rect16 body) override;

private:
    const img::Resource &icon;
    string_view_utf8 label;
};

/// Carte « tuile d'icône + titre + sous-titre » (Fichiers, Contrôle, raccourcis)
class IconCard : public Card {
public:
    IconCard(window_t *parent, Rect16 body, Action action, const img::Resource &icon, Color tile, const string_view_utf8 &title, const string_view_utf8 &subtitle);

    void set_subtitle(const string_view_utf8 &set);

protected:
    void draw_content(Rect16 body) override;

private:
    const img::Resource &icon;
    Color tile;
    string_view_utf8 title;
    string_view_utf8 subtitle;
};

/// Températures actuelles en °C entiers (chambre sans capteur : -1)
struct Temps {
    int16_t nozzle = 0;
    int16_t bed = 0;
    int16_t chamber = -1;

    static Temps read();
    bool operator==(const Temps &) const = default;
};

/// « buse° plateau° chambre° » avec leurs icônes, sur une ligne de 16 px de haut, centrée ou à gauche
void draw_temps(Rect16 rect, const Temps &temps, Color fg, Color bg, bool centered);

/// Bouton des températures de l'en-tête ; il se redessine seulement quand une valeur change
class TempsButton : public Card {
public:
    TempsButton(window_t *parent, Rect16 body);

protected:
    void draw_content(Rect16 body) override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    Temps temps;
};

/// Carte « Températures » de l'accueil : titre et valeurs actuelles
class TempsCard : public Card {
public:
    TempsCard(window_t *parent, Rect16 body);

protected:
    void draw_content(Rect16 body) override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    Temps temps;
};

/// Icône du réseau (Wi-Fi selon le signal, ou LAN), grisée hors connexion, cachée sans lien ; comme l'en-tête Prusa
class NetworkIcon : public window_icon_t {
public:
    NetworkIcon(window_t *parent, point_i16_t at);

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();
    uint32_t last_signal_ms = 0;
    int8_t signal = 0;
};

/// Heure de l'en-tête (format 12/24 h des réglages)
class Clock : public window_text_t {
public:
    Clock(window_t *parent, Rect16 rect);

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
};

/// Retour de l'en-tête : disque de 32 px, zone tactile style::back_touch (46 × 44)
class BackButton : public IconButton {
public:
    /// action : retour à l'écran précédent sans action donnée (l'écran de progression s'en sert pour se réduire)
    explicit BackButton(window_t *parent, Action action = nullptr);

    Rect16 get_rect_for_touch() const override;
};

/// Pastille de l'opération du filament mise en arrière-plan (island_od3d.hpp) : anneau qui tourne, emplacement et
/// avancement ; entre le titre et les températures, qu'elle raccourcit. La toucher rouvre l'écran de progression.
class IslandPill : public Card {
public:
    IslandPill(window_t *parent, Rect16 body, window_text_t &title, Rect16 title_full, Rect16 title_short);

protected:
    void draw_content(Rect16 body) override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void sync(); ///< visible si une opération est réduite ; le titre se raccourcit pour lui laisser la place
    window_text_t &title;
    Rect16 title_full;
    Rect16 title_short;
    uint16_t drawn_revision = 0;
    uint32_t last_spin_ms = 0;
};

/// En-tête de tous nos écrans : retour (sous-écrans), titre, pastille de l'opération en arrière-plan, bouton des
/// températures, réseau, heure, trait
class Header {
public:
    /// back_action : action du retour à la place de la fermeture de l'écran
    Header(window_t *parent, const string_view_utf8 &title, bool has_back, Card::Action back_action = nullptr);

    /// Montre ou cache le retour
    void show_back(bool show);

    /// Change le titre, même s'il est dans le même tampon (menus Prusa)
    void set_title(const string_view_utf8 &text);

    /// Caractères du titre à la police small (heure sur 5 ou 8 caractères)
    static uint8_t title_chars();

private:
    BackButton back; ///< caché et inactif sur l'accueil
    window_text_t title;
    IslandPill island;
    TempsButton temps;
    NetworkIcon network;
    Clock clock;
    BasicWindow line;
};

/// Barre d'onglets du bas : Accueil, Imprimer, Atelier, Réglages
class TabBar {
public:
    enum class Tab : uint8_t {
        home,
        print,
        workshop,
        settings,
    };

    TabBar(window_t *parent, Tab active);

private:
    class Button : public Card {
    public:
        Button(window_t *parent, Tab tab, Tab active);

    protected:
        void unconditionalDraw() override;

    private:
        const img::Resource &icon;
        string_view_utf8 label;
        bool active;
    };

    BasicWindow line;
    Button buttons[4];
};

} // namespace od3d
