/// @file
/// OneDrop3D (N4) : écran « Vitesse de la poulie », ouvert depuis Atelier › Réglages du MMU (et par l'assistant de
/// calibration) : vitesse envoyée au MMU, celle de Prusa et le gain mesuré d'un changement d'outil ; paliers de M2002
/// en direct (durée de chaque cycle, passé ou non) ; « Lancer » (M2002 P<n>) avec la bobine choisie, « Valeur Prusa »
/// (M2002 R). Quitter l'écran laisse la mesure continuer : la pastille la montre.
///
/// OneDrop3D (F10 réduite) : pointe du filament de chaque emplacement dans Atelier › Infos du MMU (MI_TIP), alerte
/// si elle dérive (feature/od3d/tip_math_od3d.hpp).
#pragma once

#include "atelier_od3d.hpp"

#include <array>
#include <cstdint>
#include <WindowMenuItems.hpp>
#include <feature/od3d/mmu_speed_od3d.hpp>
#include <string_view_utf8.hpp>

namespace od3d {

/// Raison d'un résultat (N_, à traduire) ; nullptr sans raison (pas mesuré, en cours, enregistré) : écran et bilan de
/// l'assistant de calibration
const char *mmu_speed_result_text(mmu_speed::Result result);

/// Vitesse envoyée au MMU en grand, celle de Prusa, et le gain mesuré d'un changement d'outil
class MmuSpeedPanel final : public window_t {
public:
    MmuSpeedPanel(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

/// Un palier par colonne : vitesse, durée du cycle (blanche : passé, rouge : non passé, orange : en cours), puis la
/// vérification à la vitesse gardée
class MmuSpeedLevels final : public window_t {
public:
    MmuSpeedLevels(window_t *parent, Rect16 rect);

protected:
    void unconditionalDraw() override;
};

class ScreenMmuSpeed final : public Screen {
public:
    ScreenMmuSpeed();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    void update();
    void show_info();
    void start();

    // Ordre de construction = ordre de la molette
    Header header;
    MmuSpeedPanel panel;
    MmuSpeedLevels levels;
    window_text_t info;
    Button action;
    Button slot_button;
    Button prusa_button;
    TabBar tabs;

    StringViewUtf8Parameters<4> slot_params;
    std::array<char, 128> info_text {};

    bool pending = false;
    uint16_t revision_at_request = 0;
    uint16_t shown_revision = 0xffff;
    bool shown_busy = false;
    mmu_speed::Saved shown_saved {};
};

/// Atelier › Réglages du MMU : ouvre l'écran, la vitesse actuelle en 2e ligne
class MI_MMU_SPEED final : public IWindowMenuItem {
public:
    MI_MMU_SPEED();
    void click(IWindowMenu &) override;

private:
    std::array<char, 64> text {};
};

/// Pointe du filament d'un emplacement (Atelier › Infos du MMU) : moyenne ± écart et nombre de changements, ou la
/// dérive (icône d'alerte)
class MI_TIP_SLOT : public IWindowMenuItem {
protected:
    explicit MI_TIP_SLOT(uint8_t slot);

private:
    std::array<char, 96> text {};
};

template <uint8_t slot>
class MI_TIP final : public MI_TIP_SLOT {
public:
    MI_TIP()
        : MI_TIP_SLOT(slot) {}
};

} // namespace od3d
