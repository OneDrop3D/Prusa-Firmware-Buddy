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
    StringViewUtf8Parameters<24> done_params;
    std::array<char, 24> done_name {};

    Header header;
    StepBand band;
    SpoolPicture picture;
    window_text_t title;
    window_text_t text;
    Button materials[6];
    PaintButton paints[8];
    window_text_t weight_note;
    Button cancel;
    DoneButton primary;
    TabBar tabs;
};

/// Une ligne « libellé … valeur » de l'utilisation, sans action
class InfoRow : public window_t {
public:
    InfoRow(window_t *parent, Rect16 rect, const img::Resource &icon, const string_view_utf8 &label);

    std::array<char, 16> value {};

protected:
    void unconditionalDraw() override;

private:
    const img::Resource &icon;
    string_view_utf8 label;
};

/// Carte de gauche : bobine Ø104, reste, nom
class UsageCard : public window_t {
public:
    UsageCard(window_t *parent, uint8_t slot);

protected:
    void unconditionalDraw() override;

private:
    uint8_t slot;
};

class ScreenSpoolUsage final : public Screen {
public:
    explicit ScreenSpoolUsage(uint8_t slot);

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    uint8_t slot;
    StringViewUtf8Parameters<4> title_params; ///< avant l'en-tête, qui garde le titre formaté
    Header header;
    UsageCard card;
    InfoRow material;
    InfoRow temperatures;
    InfoRow remaining;
    window_text_t note;
    Button edit;
    TabBar tabs;
};

} // namespace od3d
