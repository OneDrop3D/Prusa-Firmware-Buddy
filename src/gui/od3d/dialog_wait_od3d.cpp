/// @file
#include "dialog_wait_od3d.hpp"

#include "island_od3d.hpp"

#include <guitypes.hpp>
#include <i18n.h>

namespace od3d {

namespace {
    constexpr uint16_t button_w = 300;
    constexpr uint16_t button_h = 44; ///< cible tactile
    static_assert(GuiDefaults::DialogFrameRect.Top() + 130 + Spinner::size + style::margin <= GuiDefaults::DialogFrameRect.Bottom() - 2 * style::margin - button_h, "sous l'anneau d'attente (FrameWait)");
} // namespace

DialogWait::DialogWait(fsm::BaseData data)
    : window_dlg_wait_t(data)
    , background(this, Rect16((style::screen_w - button_w) / 2, GetRect().Bottom() - 2 * style::margin - button_h, button_w, button_h), 0, Button::Look::secondary, _("Continue in background"), nullptr, GetBackColor()) {
    background.SetFocus(); // la molette et le toucher passent par le bouton (window_frame_t)
}

Rect16 DialogWait::get_rect_for_touch() const {
    return style::screen_rect;
}

void DialogWait::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::CHILD_CLICK:
        island::minimize_wait(); // DialogHandler ferme le dialogue au tour suivant, hors de ses événements
        return;
    case GUI_event_t::TOUCH_CLICK:
        if (const point_ui16_t at = event_conversion_union { .pvoid = param }.point; !GetRect().Contain(at)) {
            if (style::back_touch.Contain(at)) {
                island::minimize_wait(); // la flèche de l'en-tête
            }
            return;
        }
        break;
    default:
        break;
    }
    window_dlg_wait_t::windowEvent(sender, event, param);
}

} // namespace od3d
