/// @file
/// OneDrop3D (F1, interface G) : ce que partagent les écrans de l'onglet Atelier (SPEC_G.md §3.2 à §3.8) —
/// bouton de la maquette, couleurs proposées, nom d'une bobine, et les actions sur les bobines, qui passent
/// toutes par les G-codes du MMU de Prusa (M701 P, M702 W2, M704, M705, M706, M708).
#pragma once

#include "widgets_od3d.hpp"

#include <array>
#include <filament.hpp>
#include <optional>
#include <span>

namespace od3d {

/// Bouton des écrans de l'atelier. Un clic envoie CHILD_CLICK à son parent avec son numéro (param = tag) :
/// l'écran ou la fiche décide, sans pointeur de fonction par bouton.
class Button : public Card {
public:
    enum class Look : uint8_t {
        primary, ///< orange, texte noir
        secondary, ///< gris #303337
        outline, ///< contour 1 px, texte clair (Annuler, Fermer)
        alert, ///< contour et texte jaunes
        selected, ///< choix courant : fond brun, contour orange 2 px
        card, ///< carte #26282B sur le fond de l'écran
    };

    Button(window_t *parent, Rect16 body, uint8_t tag, Look look, const string_view_utf8 &label, const img::Resource *icon = nullptr, Color back = style::surface, uint8_t radius = style::radius);

    void set(Look look, const string_view_utf8 &label, const img::Resource *icon = nullptr);
    void set_look(Look set);
    void set_font(Font set);

    /// set_enabled() qui redessine le bouton s'il change (set_enabled() ne le fait pas)
    void enable(bool set);

    const uint8_t tag;

protected:
    void draw() override; ///< unconditionalDraw() deux fois : le relevé du contenu, puis le corps autour et le contenu
    void unconditionalDraw() override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

    /// Ce que le contenu peint lui-même (relevé par draw()) : le corps se peint autour
    static std::span<const Rect16> around();

    /// Couleurs du corps et du texte selon l'aspect et l'état (inactif : texte éteint)
    struct Colors {
        Color fill;
        Color border;
        Color fg;
        uint8_t border_w;
    };
    Colors colors() const;

    /// Corps (plus clair avec le focus) et contour aux couleurs de l'aspect, marge comprise, autour du contenu ; rend
    /// l'intérieur du contour
    Rect16 draw_frame(Rect16 body, const Colors &c);

    string_view_utf8 label;
    const img::Resource *icon;
    Look look;
    Font font = Font::small;
};

/// Icône (facultative) et texte centrés dans area, déjà peinte au fond bg ; le texte passe sur deux lignes s'il ne tient pas
void draw_label(Rect16 area, const string_view_utf8 &label, const img::Resource *icon, Font font, Color fg, Color bg, ropfn rop = {});

namespace atelier {

    /// Couleurs proposées par l'assistant (SPEC_G.md §1.1) ; seules celles-ci ont un nom
    struct Paint {
        Color color;
        const char *name; ///< format « blanc %s » / « %s blanc », traduit
    };
    extern const std::array<Paint, 8> paints;

    /// « PLA blanc » si la bobine a une couleur de la liste, sinon « PLA » ; « - » sans bobine
    void spool_name(std::span<char> out, FilamentType type);

    /// Couleur de la liste, -1 si la bobine n'en a pas (préréglage) ou une autre
    int paint_index(FilamentType type);

    /// Nom seul de la couleur (« blanc »), vide sans couleur de la liste
    void paint_name(std::span<char> out, int paint);

    /// Le MMU répond et l'interface peut lui envoyer des commandes
    bool mmu_active();

    /// Le MMU est activé dans les réglages (même s'il démarre encore) : sinon une seule bobine, celle de la buse
    bool mmu_enabled();

    /// Bobines à montrer : outils jusqu'au dernier activé (MMU3 : 5, sans MMU : 1, INDX : ses têtes), au moins 1.
    /// Lu à l'exécution : le même firmware suit le matériel (MMU activé ou non).
    uint8_t visible_tools();

    /// 2e ligne d'une bobine : « Chargé » (orange), « Vide », le reste « ~800 g » (jaune s'il est bas) ou le nom de la
    /// couleur ; buf garde le texte formaté
    string_view_utf8 slot_state(const Slot &slot, uint8_t index, std::span<char> buf, Color &color);

    /// Le filament est engagé jusqu'à la tête : le MMU ne peut ni précharger, ni éjecter, ni couper (règle de Prusa)
    bool filament_in_nozzle();

    /// Aucune impression en cours : les commandes de bobine attendraient la fin de l'impression
    bool can_run();

    enum class Op : uint8_t {
        load, ///< M701 S"…" W2 P<i> : jusqu'à la buse
        unload, ///< M702 W2
        preload, ///< M704 P<i> : jusqu'au FINDA
        preload_all, ///< M704 sur chaque emplacement utilisable
        eject, ///< M705 P<i>
        cut, ///< M706 P<i>
    };

    /// Envoie l'opération (emplacement 0 à 4, comme les P de Prusa) ; faux si la file était pleine (Prusa prévient)
    bool run(Op op, uint8_t slot);

    /// Opération demandée par nos écrans, pour que l'écran de progression la nomme
    struct Request {
        Op op;
        uint8_t slot;
    };

    /// Pour l'écran de progression, à sa création : la demande récente, s'il y en a une
    std::optional<Request> take_request();

    /// Pour l'écran de progression, à sa fermeture : une opération en plusieurs étapes (Prusa ferme et rouvre
    /// l'écran entre deux commandes du MMU) garde son nom quelques secondes de plus
    void release_request();

    /// L'écran de progression se rouvre depuis la pastille de l'en-tête (island) : il retrouve le nom de l'opération
    void restore_request(Request request);

    /// Une demande de cette opération a été montrée par l'écran de progression (donc lancée)
    bool request_served(Op op, uint8_t slot);

    /// Envoie M708 au MMU : registre 0x1b (sélecteur, 5 = parking), 0x1c (galet, 5 = libre) ou 0x24 (poulie, F12)
    bool write_register(uint8_t reg, uint16_t value);

} // namespace atelier

} // namespace od3d
