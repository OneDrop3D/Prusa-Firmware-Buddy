/// @file
/// OneDrop3D (interface G) : « Annuler des objets » sur le plan du plateau (retour du 26/11 : « un visuel du plateau et
/// cliquer sur l'objet à enlever, et quand il imprime [...] d'une certaine couleur, quand il est annulé d'une autre »).
/// Ouvert par « Annuler Objet » du menu Ajuster, à la place de la liste de Prusa.
///
/// Chaque objet est à sa place (object_map_od3d.hpp : l'étendue de ses extrusions) : orange pendant qu'il s'imprime,
/// rouge sombre une fois annulé, gris sinon ; l'objet choisi a un cadre blanc. On le choisit au doigt sur le plan, ou par
/// ‹ › (molette) : un objet pas encore imprimé n'a pas de place, mais il se choisit et s'annule par son nom.
/// « Annuler cet objet » demande une confirmation : c'est définitif.
#pragma once

#include "screen_temperatures_od3d.hpp" // TextButton
#include "widgets_od3d.hpp"

#include <feature/od3d/object_map_math_od3d.hpp>
#include <feature/od3d/object_map_od3d.hpp>

#include <array>

namespace od3d {

/// État d'un objet, dans l'ordre où ils se recouvrent sur le plan
enum class ObjectState : uint8_t {
    waiting, ///< à imprimer
    canceled,
    printing,
};

/// Plan du plateau, peint d'une passe pixel par pixel (rien ne clignote quand l'objet en cours change) ; le numéro des
/// objets assez grands par-dessus
class ObjectMap : public window_t {
public:
    struct Item {
        object_map::Px rect; ///< sur le plan ; w == 0 : pas de place connue
        ObjectState state = ObjectState::waiting;
        bool operator==(const Item &) const = default;
    };
    struct View {
        std::array<Item, object_map::max_objects> items {};
        uint8_t count = 0; ///< objets suivis (les 16 premiers)
        int16_t selected = -1;
        bool operator==(const View &) const = default;
    };

    explicit ObjectMap(window_t *parent);

    /// Ce qu'il faut dessiner ; ne redessine que si ça change
    void set(const View &view);

    /// L'objet sous le point touché (coordonnées de l'écran), -1 sinon
    int hit(point_ui16_t p) const;

protected:
    void unconditionalDraw() override;

private:
    View view;
};

/// Objet choisi : numéro et nom, état, place connue ou non ; légende des couleurs dessous
class ObjectInfo : public window_t {
public:
    struct View {
        int16_t object = -1;
        ObjectState state = ObjectState::waiting;
        bool placed = false;
        std::array<char, 40> label {};
        bool operator==(const View &) const = default;
    };

    explicit ObjectInfo(window_t *parent);

    void set(const View &view);

protected:
    void unconditionalDraw() override;

private:
    View view;
};

class ScreenObjects final : public Screen {
public:
    ScreenObjects();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();

    Header header;
    ObjectMap map;
    ObjectInfo info;
    TextButton previous;
    TextButton next;
    TextButton cancel;
    uint32_t updated_ms = 0;
};

} // namespace od3d
