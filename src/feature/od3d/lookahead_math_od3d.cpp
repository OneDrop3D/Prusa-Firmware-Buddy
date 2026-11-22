/// @file
#include "lookahead_math_od3d.hpp"

#include <cmath>
#include <numeric>

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

bool FanScan::add(const BlockInfo &b) {
    if (!b.is_move) {
        return true; // bloc de synchronisation : ventilateur à 0, ignoré comme chez Prusa
    }
    if (!has_first_) {
        has_first_ = true;
        fan_ = b.fan;
    } else if (elapsed_s_ < lead_s_) {
        fan_ = std::max(fan_, b.fan);
    } else {
        return false;
    }
    if (b.duration_s > 0 && std::isfinite(b.duration_s)) {
        elapsed_s_ += b.duration_s;
    }
    return elapsed_s_ < lead_s_;
}

FanRise fan_rise(std::span<const uint8_t> edges, float command_s, float min_rpm, size_t settle_bins) {
    // Passage cherché sur des sommes glissantes de 4 créneaux (200 ms) : près de 90 %, la courbe monte lentement et un
    // front de plus ou de moins y pèse ~1 % au lieu de ~5 % sur un créneau seul
    constexpr size_t window = 4;
    FanRise r;
    const size_t n = edges.size();
    const size_t first = command_s > 0 ? size_t(command_s / fan_bin_s) : 0; // créneau de la commande
    if (settle_bins < window || n < first + settle_bins + 1) {
        return r;
    }
    const float final_edges = float(std::accumulate(edges.end() - settle_bins, edges.end(), 0u)) / float(settle_bins);
    r.rpm = fan_rpm(final_edges);
    if (!(r.rpm >= min_rpm) || !(final_edges > 0)) {
        return r;
    }
    const float threshold = 0.9f * final_edges * window;
    const auto sum = [&](size_t i) { return float(std::accumulate(edges.begin() + i, edges.begin() + i + window, 0u)); };
    for (size_t i = first; i + window <= n; ++i) {
        const float s = sum(i);
        if (s < threshold || (i + window < n && sum(i + 1) < threshold)) {
            continue;
        }
        float t = (float(i) + window / 2.f) * fan_bin_s; // milieu de la fenêtre
        if (i > first) { // la fenêtre d'avant est sous le seuil (sinon elle aurait été retenue)
            t -= fan_bin_s * (s - threshold) / (s - sum(i - 1));
        }
        r.rise_s = std::max(0.f, t - command_s);
        r.found = true;
        break;
    }
    return r;
}

uint8_t fan_lead_for(float rise_s) {
    constexpr long max_ds = long(max_fan_lead_s * 10 + 0.5f);
    return std::isfinite(rise_s) ? uint8_t(std::clamp(std::lround(rise_s * 10.f), 1L, max_ds)) : 0;
}

} // namespace od3d::lookahead
