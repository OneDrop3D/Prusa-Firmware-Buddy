/// @file
/// OneDrop3D (F1, interface G) : briques communes de nos écrans (SPEC_G.md §1) — base d'écran, en-tête,
/// barre d'onglets, cartes, bobines, températures. Chaque élément actif se touche et se choisit à la molette
/// (ordre de la molette = ordre de construction ; clic = toucher ; appui long = retour sur les sous-écrans).
#pragma once

#include "spool_draw.hpp"
#include "style_od3d.hpp"

#include <array>
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

/// Corps arrondi d'un élément dans outer moins la marge de l'anneau ; l'anneau de focus (2 px, blanc) occupe la marge
void draw_body(Rect16 outer, Color body, Color back, uint8_t radius, bool focused);

/// Une ligne de texte dans rect, fond repeint ; ce qui dépasse est coupé
void draw_text(Rect16 rect, const string_view_utf8 &text, Font font, Color fg, Color bg, Align_t align = Align_t::LeftCenter());

/// Icône sur un fond uni ; swap_bw = noire au lieu de blanche (sur l'orange), shadow = atténuée
void draw_icon(point_i16_t at, const img::Resource &icon, Color bg, ropfn rop = {});

/// Bobine de diamètre d, coin haut gauche en at ; les pixels hors du disque ne sont pas touchés
void draw_spool(point_i16_t at, const spool::Look &look, uint8_t d);

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

/// Aperçu des températures (pour l'instant le menu Prusa « Température et ventilateurs »)
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

private:
    bool has_back;
};

// ---- Éléments ----

/// Élément actif : corps arrondi, anneau de focus, action au clic ou au toucher.
/// Son rect comprend la marge de l'anneau : le créer avec style::with_ring(corps).
class Card : public window_t {
public:
    using Action = void (*)();

    Card(window_t *parent, Rect16 rect, Action action, Color body_color = style::surface, Color back = style::screen, uint8_t radius = style::radius);

protected:
    /// Le rect sans la marge de l'anneau
    Rect16 body() const;

    /// Contenu, dessiné sur le corps après lui
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

/// En-tête de tous nos écrans : retour (sous-écrans), titre, bouton des températures, réseau, heure, trait
class Header {
public:
    Header(window_t *parent, const string_view_utf8 &title, bool has_back);

private:
    IconButton back; ///< caché et inactif sur l'accueil
    window_text_t title;
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
