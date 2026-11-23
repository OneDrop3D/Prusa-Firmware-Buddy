/**
 * @file screen_help_fw_update.hpp
 */
#pragma once
#include "gui.hpp"
#include "screen.hpp"
#include "window_text.hpp"
#include "window_header.hpp"
#include <gui/od3d/menu_od3d.hpp>
#include "status_footer.hpp"
#include "radio_button.hpp"
#include <guiconfig/guiconfig.h>
#include <gui/qr.hpp>

class ScreenHelpFWUpdate : public screen_t {
    od3d::MenuHeader header; ///< OneDrop3D (interface G) : en-tête G compact (31 px, sans retour) au lieu de celui de Prusa
    window_text_t description;
#if HAS_MINI_DISPLAY()
    window_text_t description2;
#endif
    window_text_t help;
    QRDynamicStringWindow<32> qr;
    RadioButton radio;

    std::array<char, 128> help_text;

public:
    ScreenHelpFWUpdate();

protected:
    virtual void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
};
