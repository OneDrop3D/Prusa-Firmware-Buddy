/// @file
#include "accelerometer_guide_od3d.hpp"

#include <i18n.h>
#include <window_msgbox.hpp>

namespace od3d {

bool ask_accelerometer_retry() {
    return MsgBoxWarning(_("The accelerometer does not answer. Attach it to the nozzle and plug it in, then Retry."), Responses_RetryCancel)
        == Response::Retry;
}

} // namespace od3d
