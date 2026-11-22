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

    /// Prusa attend l'utilisateur dans cette phase (question, erreur) : l'écran ne reste pas réduit dans l'en-tête
    static bool needs_user(fsm::BaseData data);

    /// « Continuer en arrière-plan » ou la flèche de l'en-tête : l'opération continue, réduite en pastille dans
    /// l'en-tête (island_od3d.hpp)
    void minimize();

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

    /// Le cadre du dialogue : ses enfants (contenu, barre, boutons) se redessinent sur son fond. Il capte la molette et
    /// le toucher (l'écran n'ouvre rien par-dessus la machine à états), tout l'écran pour le toucher : la flèche de
    /// l'en-tête, hors du cadre, n'était jamais atteinte.
    class Box : public window_frame_t {
    public:
        explicit Box(ScreenLoadUnload &owner);

        Rect16 get_rect_for_touch() const override;

    protected:
        void unconditionalDraw() override;
        void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

    private:
        ScreenLoadUnload &owner;
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

    /// Bobine de l'emplacement, icône de l'erreur, ou anneau tournant quand l'opération ne vient pas de l'atelier
    class SpoolMark : public window_t {
    public:
        SpoolMark(window_t *parent, const ScreenLoadUnload &owner);

    protected:
        void unconditionalDraw() override;
        void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

    private:
        bool spinning() const;
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
    uint8_t background_button = 0xff; ///< bouton « Continuer en arrière-plan » (étape sans question), 0xff : aucun

    Header header; ///< températures en direct pendant la chauffe (le pied de page de Prusa les montrait)
    Box box;
    Body body;
    SpoolMark spool_mark; ///< après le contenu : il se dessine par-dessus sa case
    Bar bar;
    Button buttons[3];
};

} // namespace od3d
