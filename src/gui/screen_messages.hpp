#pragma once

#include "window_header.hpp"
#include <gui/od3d/menu_od3d.hpp>
#include "status_footer.hpp"
#include "window_term.hpp"
#include "screen.hpp"

struct screen_messages_data_t : public screen_t {
    od3d::MenuHeader header; ///< OneDrop3D (interface G) : en-tête G compact (31 px, sans retour) au lieu de celui de Prusa
    StatusFooter footer;
    window_term_t term;
    term_buff_t<20, 13> term_buff;
    uint32_t last_message_id = 0;

public:
    screen_messages_data_t();

protected:
    virtual void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
};
