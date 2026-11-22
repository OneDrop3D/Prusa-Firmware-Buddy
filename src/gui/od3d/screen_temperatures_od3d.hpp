/// @file
/// OneDrop3D (F1, interface G) : aperçu des températures (SPEC_G.md §3.15). Buse, plateau et chambre (actuelle, cible,
/// état, courbe de la dernière minute), ventilateur, ventilateurs de la chambre, filtration ; Préchauffer (M1700 de
/// Prusa), Préréglages (filaments de Prusa, chambre comprise) et Refroidir (cibles à 0, comme Prusa).
/// Les cibles se changent dans l'éditeur de valeur, aux limites de Prusa.
#pragma once

#include "widgets_od3d.hpp"

#include <array>

namespace od3d {

/// Buse, plateau ou chambre ; le clic ouvre l'éditeur de la cible
class HeaterCard : public Card {
public:
    enum class Heater : uint8_t {
        nozzle,
        bed,
        chamber,
    };

    HeaterCard(window_t *parent, Heater heater);

protected:
    void draw_content(Rect16 body) override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    struct Reading {
        int16_t current = -1; ///< négatif : pas de mesure
        int16_t target = 0; ///< 0 : pas de cible (chauffe arrêtée, chambre sans limite)
        bool operator==(const Reading &) const = default;
    };
    static Reading read(Heater heater);

    static constexpr size_t samples = 30; ///< une mesure toutes les 2 s : la dernière minute
    Heater heater;
    Reading reading;
    std::array<int16_t, samples> history;
    uint32_t last_sample_ms;
};

/// Ventilateur, ventilateurs de la chambre ou filtration : libellé, puis valeur ou interrupteur
class FanCard : public Card {
public:
    enum class Fan : uint8_t {
        print,
        chamber,
        filtration,
    };

    FanCard(window_t *parent, Fan fan);

protected:
    void draw_content(Rect16 body) override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    static int16_t read(Fan fan); ///< en % ; -1 : auto ; filtration : 0 ou 1

    Fan fan;
    int16_t value;
};

/// Bouton à texte, icône facultative, grisé quand il est désactivé
class TextButton : public Card {
public:
    TextButton(window_t *parent, Rect16 body, Action action, const string_view_utf8 &label, const img::Resource *icon, Color body_color, Color fg);

    /// Active ou grise, et redessine s'il change ; le bouton orange devient gris (orange inactif, il tromperait)
    void enable(bool set);

protected:
    void draw_content(Rect16 body) override;

private:
    string_view_utf8 label;
    const img::Resource *icon;
    Color fg;
};

class ScreenTemperatures final : public Screen {
public:
    ScreenTemperatures();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update_buttons();

    Header header;
    HeaterCard nozzle;
    HeaterCard bed;
    HeaterCard chamber;
    FanCard fan;
    FanCard chamber_fans;
    FanCard filtration;
    TextButton preheat;
    TextButton presets;
    TextButton cooldown;
    TabBar tabs;
};

} // namespace od3d
