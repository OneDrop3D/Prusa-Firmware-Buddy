/// @file
#pragma once

#include "gui.hpp"
#include "screen.hpp"
#include "window_text.hpp"
#include "window_header.hpp"
#include <gui/od3d/menu_od3d.hpp>
#include "radio_button.hpp"
#include <gui/qr.hpp>

class ScreenChangeFilter : public screen_t {
    od3d::MenuHeader header; ///< OneDrop3D (interface G) : en-tête G compact (31 px, sans retour) au lieu de celui de Prusa
    window_text_t description;
    window_text_t help;
    QRStaticStringWindow qr;
    RadioButton radio;

public:
    ScreenChangeFilter();
};
