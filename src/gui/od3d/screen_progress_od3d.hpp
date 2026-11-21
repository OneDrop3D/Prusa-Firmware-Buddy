/// @file
/// OneDrop3D (F1, interface G) : écran de progression des opérations sur les bobines (SPEC_G.md §1.9, §3.4).
/// Il prend la place de l'apparence de ScreenFilamentChange (Prusa) pour la machine à états Load_unload, sans
/// changer la machine : mêmes phases, mêmes réponses (« Annuler » = Response::Stop, seulement là où Prusa permet
/// d'arrêter), mêmes erreurs du MMU et du capteur de filament, mêmes sons. Un dialogue sur l'écran d'origine,
/// qui reste visible autour (pas de voile sans alpha).
#pragma once

#include "atelier_od3d.hpp"

#include <array>
#include <common/fsm_base_types.hpp>
#include <fsm/filament_change_phases.hpp>
#include <optional>
#include <screen.hpp>

namespace od3d {

class ScreenLoadUnload final : public screen_t {
public:
    ScreenLoadUnload();
    ~ScreenLoadUnload();

    /// Nouvelle phase ou nouvelles données de la machine à états (DialogHandler)
    void Change(fsm::BaseData data);

    void InitState(screen_init_variant var) override;
    screen_init_variant GetCurrentState() const override;

protected:
    void unconditionalDraw() override;
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    enum class Kind : uint8_t {
        progress, ///< étapes et barre ; « Annuler » si la phase s'arrête
        prompt, ///< une question de Prusa (« La couleur est-elle correcte ? »…) et ses réponses
        notice, ///< une erreur (MMU, filament coincé…) : titre, texte, lien d'aide, boutons de Prusa
    };

    /// Le cadre du dialogue : ses enfants (contenu, barre, boutons) se redessinent sur son fond
    class Box : public window_frame_t {
    public:
        explicit Box(window_t *parent);

    protected:
        void unconditionalDraw() override;
    };

    /// Bobine (ou icône d'erreur), titre, sous-titre, puis étapes, question ou texte de l'erreur
    class Body : public window_t {
    public:
        Body(window_t *parent, const ScreenLoadUnload &owner);

    protected:
        void unconditionalDraw() override;

    private:
        void draw_header(Rect16 r) const;
        const ScreenLoadUnload &owner;
    };

    class Bar : public window_t {
    public:
        Bar(window_t *parent, const ScreenLoadUnload &owner);

    protected:
        void unconditionalDraw() override;

    private:
        const ScreenLoadUnload &owner;
    };

    void enter_phase();
    void setup_buttons();
    void respond(Response response);

    std::optional<atelier::Request> request;
    fsm::BaseData data;
    bool has_data = false;
    bool painted = false; ///< le 1er dessin laisse l'écran d'origine autour du dialogue
    PhasesLoadUnload phase = PhasesLoadUnload::initial;
    LoadUnloadMode mode = LoadUnloadMode::Load;
    uint8_t progress = 0;
    Kind kind = Kind::progress;

    // Erreur affichée (textes de Prusa, à traduire au dessin)
    const char *notice_title = nullptr;
    const char *notice_text = nullptr;
    uint16_t notice_code = 0; ///< code d'erreur Prusa, 0 sans lien d'aide
    uint8_t notice_type = 0; ///< ErrType

    /// Phases vues, la dernière en cours (Prusa ne publie pas les étapes à venir)
    std::array<PhasesLoadUnload, 6> history {};
    uint8_t history_count = 0;

    std::array<Response, 3> responses {};

    Box box;
    Body body;
    Bar bar;
    Button buttons[3];
};

} // namespace od3d
