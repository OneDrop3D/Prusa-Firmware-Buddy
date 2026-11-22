#pragma once

#include <filament.hpp>
#include <string_view_utf8.hpp>
#include <screen_menu.hpp>
#include <screen_fsm.hpp>
#include <filament_list.hpp>
#include <dynamic_index_mapping.hpp>
#include <window_menu_virtual.hpp>
#include <window_menu_callback_item.hpp>
#include <option/has_anfc.h>
#include <tool_index.hpp>

#include <MItem_tools.hpp>
#include <fsm_preheat_type.hpp>
#include <fsm/preheat_phases.hpp>

class ScreenPreheat : public ScreenFSM {

public:
    ScreenPreheat();
    ~ScreenPreheat();

    static bool handle_filament_selection(FilamentType filament_type, PreheatData::ToolIndex tool, PreheatMode mode);

protected:
    inline PhasesPreheat get_phase() const {
        return GetEnumFromPhaseIndex<PhasesPreheat>(fsm_base_data.GetPhase());
    }

protected:
    void screenEvent(window_t *, GUI_event_t event, void *) override;
    void unconditionalDraw() override; ///< OneDrop3D (interface G) : le fond seulement entre l'en-tête et la liste

    bool can_return = false; ///< OneDrop3D (interface G) : le retour de l'en-tête (et l'appui long) répond « Abort »

    void create_frame();
    void destroy_frame();
    void update_frame();
};
