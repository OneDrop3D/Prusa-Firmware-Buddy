/// @file
/// OneDrop3D (F1, interface G) : éditeur de valeur (SPEC_G.md §1.8) — feuille en bas de l'écran, valeur en grand,
/// boutons − et +, barre, OK. Les limites et le pas viennent d'un NumericInputConfig de Prusa (températures
/// maximales comprises) ; la valeur n'est appliquée par l'appelant qu'après OK.
#pragma once

#include <IDialog.hpp>
#include <numeric_input_config.hpp>
#include <optional>

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
    /// @returns la valeur choisie, ou nullopt si l'éditeur a été fermé sans OK
    static std::optional<float> exec(const string_view_utf8 &label, float value, const NumericInputConfig &config, Apply apply = nullptr, const char *unit = nullptr);

private:
    ValueEditor(const string_view_utf8 &label, float value, const NumericInputConfig &config, Apply apply, const char *unit);

    void unconditionalDraw() override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
    void screenEvent(window_t *sender, GUI_event_t event, void *param) override;

    /// Valeur et barre : seule partie redessinée quand la valeur change
    class ValueView : public window_t {
    public:
        ValueView(ValueEditor &editor);

    protected:
        void unconditionalDraw() override;

    private:
        ValueEditor &editor;
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
    bool frosted = false; ///< le fond au-dessus de la feuille a été givré (1er dessin)
    ValueView view;
};

} // namespace od3d
