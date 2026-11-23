/// @file
/// OneDrop3D (F1, interface G) : éditeur de valeur (SPEC_G.md §1.8) — feuille en bas de l'écran, valeur en grand,
/// boutons − et +, barre, OK. Les limites et le pas viennent d'un NumericInputConfig de Prusa (températures
/// maximales comprises) ; la valeur n'est appliquée par l'appelant qu'après OK.
#pragma once

#include <array>
#include <IDialog.hpp>
#include <numeric_input_config.hpp>
#include <optional>

class IWindowMenu;
class IWindowMenuItem;

namespace od3d {

/// Molette : un pas par cran, clic = OK, appui long = Annuler. Toucher : −, +, la barre (valeur sous le doigt, qui peut
/// glisser), Annuler, OK ; glisser hors de la barre = annuler. La valeur reste toujours dans les limites : OK est
/// toujours possible (le clavier numérique de Prusa, qu'il remplace pour toutes les valeurs WiSpin, bloquait, BUG-6).
class ValueEditor final : public IDialog {
public:
    using Apply = void (*)(float value);

    /// @param apply si donnée, appliquée à chaque pas (comme la maquette) et remise à la valeur de départ si
    ///              l'éditeur est fermé sans OK ; sinon c'est à l'appelant d'appliquer la valeur rendue
    /// @param unit si donnée, affichée après la valeur à la place de l'unité de config (« g » : Prusa n'a pas d'unité
    ///             de masse)
    /// @param from l'élément d'où s'ouvre la feuille ; vide : l'élément ciblé (une liste entière n'en est pas un : depuis
    ///             le centre, sauf si l'appelant donne le rect de sa ligne)
    /// @returns la valeur choisie, ou nullopt si l'éditeur a été fermé sans OK
    static std::optional<float> exec(const string_view_utf8 &label, float value, const NumericInputConfig &config, Apply apply = nullptr, const char *unit = nullptr, Rect16 from = {});

    /// Rect de la ligne de item dans menu (le from d'exec : la feuille s'ouvre depuis elle) ; vide hors de l'écran
    static Rect16 row(const IWindowMenu &menu, const IWindowMenuItem &item);

private:
    ValueEditor(const string_view_utf8 &label, float value, const NumericInputConfig &config, Apply apply, const char *unit, Rect16 from);

    /// À repeindre en entier : la feuille s'ouvre depuis l'élément touché sur le voile (frost_od3d.hpp), chaque partie
    /// peinte une fois (paint_part) ; sinon la valeur seule
    void draw() override;
    void paint_part(size_t i);
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
    void screenEvent(window_t *sender, GUI_event_t event, void *param) override;

    /// Valeur et barre : seule partie redessinée quand la valeur change, en delta (les chiffres changés, les colonnes de
    /// la barre où le disque a bougé)
    class ValueView : public window_t {
    public:
        ValueView(ValueEditor &editor);

        void draw_value();
        void draw_bar();
        void forget(); ///< la feuille se repeint : valeur et barre entières

    protected:
        void unconditionalDraw() override;

    private:
        ValueEditor &editor;
        std::array<char, 12> drawn {}; ///< chiffres à l'écran (vide : rien de sûr, tout se repeint)
        int16_t drawn_filled = -1; ///< bout du remplissage à l'écran, en px de la barre (-1 : rien de sûr)
    };

    void step(int steps);
    void set(float set, bool quiet = false); ///< quiet : sans le son de pas (glissement sur la barre)
    void follow_finger(); ///< LOOP : la valeur suit le doigt posé sur la barre
    float fraction() const; ///< position sur la barre, 0 à 1

    string_view_utf8 label;
    const NumericInputConfig &config;
    float value;
    Apply apply;
    const char *unit;
    bool accepted = false;
    bool dragged = false; ///< le doigt a glissé sur la barre : le balayage qui suit n'annule pas
    Rect16 from; ///< l'élément touché, d'où s'ouvre la feuille (sa place une fois ouverte)
    ValueView view;
};

} // namespace od3d
