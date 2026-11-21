/// @file
/// OneDrop3D (F1, interface G) : éditeur de valeur (SPEC_G.md §1.8) — feuille en bas de l'écran, valeur en grand,
/// boutons − et +, barre, OK. Les limites et le pas viennent d'un NumericInputConfig de Prusa (températures
/// maximales comprises) ; la valeur n'est appliquée par l'appelant qu'après OK.
#pragma once

#include <IDialog.hpp>
#include <numeric_input_config.hpp>
#include <optional>

namespace od3d {

/// Molette : un pas par cran, clic = OK. Toucher : −, +, la barre (valeur à l'endroit touché), OK. Glisser = annuler.
class ValueEditor final : public IDialog {
public:
    /// @returns la valeur choisie, ou nullopt si l'éditeur a été fermé sans OK
    static std::optional<float> exec(const string_view_utf8 &label, float value, const NumericInputConfig &config);

private:
    ValueEditor(const string_view_utf8 &label, float value, const NumericInputConfig &config);

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
    void set(float set);
    float fraction() const; ///< position sur la barre, 0 à 1

    string_view_utf8 label;
    const NumericInputConfig &config;
    float value;
    bool accepted = false;
    ValueView view;
};

} // namespace od3d
