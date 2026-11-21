/// @file
#include "lookahead_math_od3d.hpp"

#include <cmath>

namespace od3d::lookahead {

float block_duration(float millimeters, float nominal_speed) {
    if (!(std::isfinite(millimeters) && std::isfinite(nominal_speed) && millimeters > 0 && nominal_speed > 0)) {
        return 0;
    }
    const float d = millimeters / nominal_speed;
    return std::isfinite(d) ? d : 0;
}

bool FlowScan::add(const BlockInfo &b) {
    if (time_s_ >= window_s_) {
        return false;
    }
    if (!b.is_move || !(b.duration_s > 0) || !std::isfinite(b.duration_s)) {
        return true;
    }
    float e_speed = std::isfinite(b.e_mm) ? b.e_mm / b.duration_s : 0;
    if (!(e_speed >= 0 && e_speed <= max_e_speed_)) {
        e_speed = 0;
    }
    const float used = std::min(b.duration_s, window_s_ - time_s_);
    e_sum_ += e_speed * used;
    time_s_ += used;
    return time_s_ < window_s_;
}

float FlowScan::mean_e_speed() const {
    return time_s_ > 0 ? e_sum_ / time_s_ : 0;
}

} // namespace od3d::lookahead
