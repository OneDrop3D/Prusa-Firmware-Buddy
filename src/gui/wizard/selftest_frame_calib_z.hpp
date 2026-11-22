/**
 * @file selftest_frame_calib_z.hpp
 * @author Radek Vana
 * @brief part of screen containing Z calibration selftest
 * @date 2021-12-10
 */

#pragma once

#include "selftest_frame.hpp"
#include "window_icon.hpp"
#include <gui/od3d/widgets_od3d.hpp>
#include "window_wizard_progress.hpp"
#include "status_footer.hpp"

class SelftestFrameCalibZ : public SelftestFrameNamed {
    window_wizard_line_t line;

    od3d::Spinner animation; ///< OneDrop3D (interface G) : anneau d'attente à la place du sablier

    window_text_t text_info;

protected:
    virtual void change() override;

public:
    SelftestFrameCalibZ(window_t *parent, PhasesSelftest ph, fsm::PhaseData data);
};
