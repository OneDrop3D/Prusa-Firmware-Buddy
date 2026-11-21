/// @file
/// OneDrop3D (F1, interface G) : l'assistant « Changer la bobine » en 3 étapes (SPEC_G.md §3.5, §3.6) et
/// l'utilisation d'une bobine (§3.7). L'assistant passe par les commandes du MMU : M705 (éjection) avant l'étape 1,
/// M704 (préchargement jusqu'au FINDA) à l'étape 2 ; l'étape 3 enregistre la matière et la couleur de l'emplacement.
#pragma once

#include "atelier_od3d.hpp"

#include <array>

namespace od3d {

enum class SpoolChange : uint8_t {
    change, ///< retirer, insérer, choisir
    insert, ///< emplacement vide : insérer, choisir
    material, ///< choisir seulement (« Matière et couleur »)
};

/// Ouvre l'assistant de l'emplacement (0 à 4) et envoie la commande de sa première étape
void open_change_spool(uint8_t slot, SpoolChange mode);

/// Ouvre « Bobine n · utilisation »
void open_spool_usage(uint8_t slot);

/// Bandeau des 3 étapes
class StepBand : public window_t {
public:
    explicit StepBand(window_t *parent);

protected:
    void unconditionalDraw() override;
};

/// Étapes 1 et 2 : la bobine, des flèches vers ou depuis le MMU, la case « MMU n »
class SpoolPicture : public window_t {
public:
    explicit SpoolPicture(window_t *parent);

protected:
    void unconditionalDraw() override;
};

/// Couleur proposée : bobine Ø40, anneau orange de 3 px si elle est choisie
class PaintButton : public Button {
public:
    PaintButton(window_t *parent, uint8_t index);

protected:
    void unconditionalDraw() override;
};

/// Bouton principal de l'assistant ; à l'étape 3, la bobine Ø24 du choix devant « Terminé · … »
class DoneButton : public Button {
public:
    using Button::Button;

protected:
    void unconditionalDraw() override;
};

class ScreenChangeSpool final : public Screen {
public:
    ScreenChangeSpool();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void layout();
    void finish();

    // Avant les fenêtres qui affichent ces textes (ordre de construction)
    StringViewUtf8Parameters<4> header_params;
    StringViewUtf8Parameters<4> title_params;
    StringViewUtf8Parameters<4> text_params;
    StringViewUtf8Parameters<36> done_params;
    std::array<char, 36> done_name {}; ///< « PETG vert · 1 kg »
    std::array<char, 12> other_weight {}; ///< « Autre » ou la valeur libre choisie (« 1250 g »)

    Header header;
    StepBand band;
    SpoolPicture picture;
    window_text_t title;
    window_text_t text;
    Button materials[6];
    PaintButton paints[8];
    Button weights[6]; ///< poids neuf (F13) : 250 g, 500 g, 750 g, 1 kg, 2 kg, autre
    Button cancel;
    DoneButton primary;
    TabBar tabs;
};

/// Une ligne « icône, libellé … valeur » de l'utilisation (SPEC_G.md §3.7) : un clic ouvre l'éditeur de valeur
/// (CHILD_CLICK avec son tag, comme Button)
class ValueRow : public Button {
public:
    ValueRow(window_t *parent, Rect16 body, uint8_t tag, const img::Resource &icon, const string_view_utf8 &label);

    std::array<char, 16> value {};

protected:
    void unconditionalDraw() override;
};

/// Carte de gauche : bobine Ø104, reste, longueur et remplissage, nom, « Bientôt vide »
class UsageCard : public window_t {
public:
    UsageCard(window_t *parent, uint8_t slot);

protected:
    void unconditionalDraw() override;

private:
    uint8_t slot;
};

/// « Utilisation » d'une bobine (F13) : poids net, reste (à corriger après une pesée), alerte de fin, consommations.
/// Une bobine à étiquette OpenPrintTag montre les données de l'étiquette, sans modification.
class ScreenSpoolUsage final : public Screen {
public:
    explicit ScreenSpoolUsage(uint8_t slot);

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void refresh(); ///< valeurs, textes et éléments actifs d'après le suivi
    void edit(uint8_t tag);

    uint8_t slot;
    int shown_g = -1; ///< reste affiché, pour ne redessiner que s'il change (pendant une impression)
    StringViewUtf8Parameters<4> title_params; ///< avant l'en-tête, qui garde le titre formaté
    std::array<char, 112> note_text {};
    Header header;
    UsageCard card;
    ValueRow net;
    ValueRow remaining;
    ValueRow alert;
    window_text_t note;
    Button full;
    TabBar tabs;
};

} // namespace od3d
