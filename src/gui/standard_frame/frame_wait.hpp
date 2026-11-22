/// \file
#pragma once

#include <window_frame.hpp>
#include <window_text.hpp>
#include <window_icon.hpp>
#include <gui/od3d/widgets_od3d.hpp>

/**
 * Standard layout frame.
 * Contains:
 * - Centered title
 * - Centered text (alignment can be changed)
 */
class FrameWait {

public:
    FrameWait(window_frame_t *parent, const string_view_utf8 &text);

public:
    inline void set_text(const string_view_utf8 &text) {
        text_custom.SetText(text);
    }

protected:
    window_text_t text_wait;
    window_text_t text_custom;
    od3d::Spinner animation; ///< OneDrop3D (interface G) : anneau d'attente à la place du sablier
};
