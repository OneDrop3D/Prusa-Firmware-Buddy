/// @file
/// OneDrop3D (F1, interface G) : briques communes de nos écrans (SPEC_G.md §1) — base d'écran, en-tête,
/// barre d'onglets, cartes, bobines, températures. Chaque élément actif se touche et se choisit à la molette
/// (ordre de la molette = ordre de construction ; clic = toucher ; appui long = retour sur les sous-écrans).
#pragma once

#include "island_face_od3d.hpp"
#include "spool_draw.hpp"
#include "style_od3d.hpp"

#include <array>
#include <optional>
#include <raster_opfn.hpp>
#include <ScreenFactory.hpp>
#include <ScreenHandler.hpp>
#include <screen.hpp>
#include <text_roll.hpp>
#include <window_icon.hpp>
#include <window_text.hpp>

namespace od3d {

/// Emplacement du MMU tel qu'affiché
struct Slot {
    spool::Look look; ///< bobine sans couleur : emplacement vide ou inutilisable
    std::array<char, 8> material {}; ///< nom de la matière, « - » sans bobine
    bool loaded = false; ///< chargé dans la buse
    bool enabled = false; ///< utilisable (sans MMU activé, seul le premier l'est)
    bool low = false; ///< reste sous le seuil d'alerte de fin (F13)

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

/// draw() d'un cadre G à repeindre en entier : ses fenêtres normales visibles (first à last, qui repeignent chacune tout
/// leur rect) de haut en bas, chacune avec la bande du fond où elle commence (style::paint_top_down), au lieu de tout
/// le fond d'abord. Puis on_background (ce que le cadre peint sur son fond, entre ses fenêtres), puis les dialogues.
/// false si le cadre n'est pas à repeindre en entier : window_frame_t::draw() (seuls les éléments changés).
bool draw_whole(window_frame_t &frame, window_t *first, window_t *last, void (*on_background)(window_frame_t &) = nullptr);

/// Cadre dont gaps_only ne peint que le fond entre ses fenêtres (qui repeignent chacune tout leur rect) : un écran
/// apparaît d'une passe, sans être d'abord effacé ; sinon comme window_frame_t (tout le fond, puis les fenêtres)
class GapFrame : public window_frame_t {
public:
    using window_frame_t::window_frame_t;
    bool gaps_only = false;

protected:
    void draw() override;
};

/// Anneau d'attente (spinner_od3d.hpp) de bord w dans le carré box, au fond bg, l'arc à la position step : tout, ou
/// seulement ce qui a changé depuis la position from (les deux bouts de l'arc)
void draw_spinner(Rect16 box, uint8_t w, uint8_t step, Color bg, std::optional<uint8_t> from = std::nullopt);

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

// ---- Changement d'écran ----

/// Éléments fixes (retour, températures, onglets) d'un écran G au suivant : chaque écran a les siens, mais celui qui
/// est déjà à l'écran tel quel (même position, même état), laissé par l'écran d'avant, n'est pas repeint. L'en-tête et
/// les onglets ne clignotent plus à chaque page, et il y a moins à dessiner.
namespace chrome {
    enum class Slot : uint8_t {
        back,
        temps,
        tab0, ///< puis un par onglet
        count = tab0 + 4,
    };

    /// Un écran (pas un dialogue) qui ne peint que le fond entre ses éléments s'annonce à sa construction : seuls ses
    /// éléments peuvent s'épargner un dessin
    void gap_screen(const window_t *screen);

    /// À chaque dessin d'un élément fixe (parent : son écran) : true si c'est son premier dessin et que l'écran le montre
    /// déjà tel quel (clé identique) : rien à peindre. Note ce qui est à l'écran.
    bool kept(Slot slot, uint64_t key, bool first, const window_t *parent);

    /// L'élément va être repeint, caché ou couvert : l'écran ne le montre plus de façon sûre
    void forget(Slot slot);
    void forget_all();

    /// ScreenHandler, à chaque changement d'écran : avant la destruction de l'ancien (ses éléments fixes restent à l'écran
    /// pour le suivant s'il n'a ni dialogue ni zone à repeindre), puis avant la création du nouveau
    void hand_over(window_frame_t &screen);
    void next_screen();
} // namespace chrome

// ---- Écran ----

/// Base de nos écrans : fond de la maquette, focus retrouvé au retour d'un sous-écran, appui long = retour
class Screen : public screen_t {
public:
    explicit Screen(bool has_back);

    screen_init_variant GetCurrentState() const override;
    void InitState(screen_init_variant var) override;

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
    void draw() override; ///< à repeindre en entier : draw_whole(), le fond avec chaque élément
    void unconditionalDraw() override; ///< ce que l'écran peint sur son fond entre ses éléments (rien ici)

private:
    bool has_back;
};

// ---- Éléments ----

/// Anneau d'attente G (« Patientez », opération sans avancement connu) à la place du sablier de Prusa : 25 images/s, et
/// à chaque image seuls les deux bouts de l'arc sont repeints. Fond : celui du parent.
class Spinner : public window_t {
public:
    static constexpr uint8_t size = 40;

    Spinner(window_t *parent, point_i16_t at);

protected:
    void unconditionalDraw() override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    txtroll_t tick; ///< tant qu'il en existe un, l'interface envoie TEXT_ROLL toutes les 40 ms : le rythme de l'anneau
    std::optional<uint8_t> drawn; ///< position de l'arc à l'écran
};

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
    bool first_draw = true; ///< pour chrome::kept()
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
    void unconditionalDraw() override; ///< chrome::kept()
    void invalidate(Rect16 rect) override; ///< chrome::forget()

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

/// Retour de l'en-tête : disque de 32 px, zone tactile style::back_touch (46 × 44). Un balayage horizontal
/// (gesture_od3d.hpp : 60 px en moins de 600 ms, dans un sens ou dans l'autre) fait la même chose, s'il est visible : sous un dialogue (éditeur de valeur, question, dialogue
/// d'une machine à états), le balayage ne va qu'au dialogue et ne l'atteint pas.
class BackButton : public IconButton {
public:
    /// action : retour à l'écran précédent sans action donnée (l'écran de progression s'en sert pour se réduire)
    explicit BackButton(window_t *parent, Action action = nullptr);

    Rect16 get_rect_for_touch() const override;

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
    void unconditionalDraw() override; ///< chrome::kept()
    void invalidate(Rect16 rect) override; ///< chrome::forget()
};

/// Pastille « Dynamic Island » (island_od3d.hpp), à gauche des températures ; visible, elle cache le titre (retour de
/// l'utilisateur, 22/11 : le titre raccourci ne disait plus rien) :
/// - un de nos processus : anneau qui tourne, « PA 2/5 », « PA chauffe » ; la toucher rouvre son écran ;
/// - chargement réduit : anneau qui tourne, emplacement et avancement ; la toucher rouvre l'écran de progression ;
/// - impression : anneau de progression, « 47% 1h23 », en couleur selon l'état ; la toucher ramène à l'écran Impression ;
/// - préchauffage : icône de la buse et « 185/215° » ; la toucher ouvre l'aperçu des températures.
/// Seul son contenu se redessine, et seulement quand ce qu'elle montre change ; l'anneau qui tourne (10 images/s) se
/// repeint seul, sans le texte.
class IslandPill : public Card {
public:
    IslandPill(window_t *parent, Rect16 body, window_t &title);

protected:
    void draw_content(Rect16 body) override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void sync(bool show); ///< visible s'il y a quelque chose à montrer ; le titre est alors caché
    window_t &title;
    island::Face drawn; ///< ce qui est dessiné
    island::Face text_drawn; ///< ce dont le texte est à l'écran
    uint32_t last_spin_ms = 0;
};

/// Alerte non bloquante (RET-22 : une boîte de message ouverte pendant l'impression retardait les dialogues de Prusa) :
/// deux lignes courtes (« Bobine 2 : 48 g », « presque vide ») à la place du titre de l'en-tête, pendant 8 s à partir
/// du premier titre qui peut l'écrire en entier (la pastille raccourcit le titre). Une nouvelle alerte remplace l'ancienne.
void show_notice(const string_view_utf8 &text);

/// Titre de l'en-tête, qui cède sa place à l'alerte de show_notice()
class HeaderTitle : public window_text_t {
public:
    using window_text_t::window_text_t;

protected:
    void unconditionalDraw() override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    bool showing_notice = false;
};

/// En-tête de tous nos écrans : retour (sous-écrans), titre, pastille de l'opération en arrière-plan, bouton des
/// températures, réseau, heure, trait
class Header {
public:
    /// back_action : action du retour à la place de la fermeture de l'écran. compact : 31 px et sans retour, pour les
    /// écrans de Prusa (machines à états) dont le contenu commence à 32 px
    Header(window_t *parent, const string_view_utf8 &title, bool has_back, Card::Action back_action = nullptr, bool compact = false);

    /// Montre ou cache le retour
    void show_back(bool show);

    /// Change le titre, même s'il est dans le même tampon (menus Prusa)
    void set_title(const string_view_utf8 &text);

    /// Caractères du titre à la police small (heure sur 5 ou 8 caractères)
    static uint8_t title_chars();

private:
    BackButton back; ///< caché et inactif sur l'accueil
    HeaderTitle title;
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
        void invalidate(Rect16 rect) override; ///< chrome::forget()

    private:
        chrome::Slot slot() const;
        const img::Resource &icon;
        string_view_utf8 label;
        bool active;
    };

    BasicWindow line;
    Button buttons[4];
};

} // namespace od3d
