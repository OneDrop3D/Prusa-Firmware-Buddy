/// @file
#include "mesh_memory_math_od3d.hpp"

#include <algorithm>
#include <cmath>

namespace od3d::mesh_memory {

int16_t to_um(float mm) {
    if (!std::isfinite(mm)) {
        return 0;
    }
    return int16_t(std::clamp(std::lround(mm * 1000.f), -32000L, 32000L));
}

int16_t to_c10(float c) {
    if (!std::isfinite(c)) {
        return no_temperature;
    }
    return int16_t(std::clamp(std::lround(c * 10.f), -32000L, 32000L));
}

namespace {

    bool near_c10(int16_t a, int16_t b) {
        return std::abs(int(a) - int(b)) <= int(temperature_tolerance * 10.f);
    }

    size_t count(uint64_t mask) {
        size_t n = 0;
        for (; mask; mask &= mask - 1) {
            ++n;
        }
        return n;
    }

} // namespace

Refusal compare(const Record &stored, const Record &now, uint32_t max_age_s) {
    if (max_age_s == 0) {
        return Refusal::disabled;
    }
    if (stored.empty()) {
        return Refusal::empty;
    }
    if (stored.time == 0 || now.time == 0) {
        return Refusal::clock;
    }
    if (now.time < stored.time || now.time - stored.time > max_age_s) {
        return Refusal::too_old;
    }
    if (stored.bed_c10 == no_temperature || now.bed_c10 == no_temperature || !near_c10(stored.bed_c10, now.bed_c10)) {
        return Refusal::bed;
    }
    if ((stored.chamber_c10 == no_temperature) != (now.chamber_c10 == no_temperature)
        || (stored.chamber_c10 != no_temperature && !near_c10(stored.chamber_c10, now.chamber_c10))) {
        return Refusal::chamber;
    }
    if (std::abs(int(stored.probe_c) - int(now.probe_c)) > int(temperature_tolerance)) {
        return Refusal::probe_temperature;
    }
    if (stored.nozzle_um != now.nozzle_um || stored.flags != now.flags || stored.hotend != now.hotend || stored.sheet != now.sheet) {
        return Refusal::nozzle;
    }
    if (stored.probe_offset_um != now.probe_offset_um) {
        return Refusal::z_offset;
    }
    return Refusal::none;
}

Refusal covers(const Record &stored, uint64_t needed) {
    if ((stored.valid & needed) != needed) {
        return Refusal::not_covered;
    }
    return count(needed) > check_points ? Refusal::none : Refusal::too_few_points;
}

uint64_t pick_checks(uint64_t needed) {
    // Coins : ix + iy le plus petit et le plus grand, ix − iy le plus grand et le plus petit
    std::array<int, 4> best { 1000, -1000, -1000, 1000 };
    std::array<size_t, 4> at {};
    for (size_t i = 0; i < majors; ++i) {
        if (!(needed & major_bit(i))) {
            continue;
        }
        const int ix = int(i % majors_x), iy = int(i / majors_x);
        const int sum = ix + iy, diff = ix - iy;
        if (sum < best[0]) {
            best[0] = sum;
            at[0] = i;
        }
        if (sum > best[1]) {
            best[1] = sum;
            at[1] = i;
        }
        if (diff > best[2]) {
            best[2] = diff;
            at[2] = i;
        }
        if (diff < best[3]) {
            best[3] = diff;
            at[3] = i;
        }
    }
    if (!needed) {
        return 0;
    }
    uint64_t checks = 0;
    for (size_t c = 0; c < at.size() && count(checks) < check_points; ++c) {
        checks |= major_bit(at[c]);
    }
    // Zone d'une seule rangée : les coins se confondent ; on complète par le point le plus loin des points choisis
    while (count(checks) < check_points && (needed & ~checks)) {
        size_t far = 0;
        int far_d = -1;
        for (size_t i = 0; i < majors; ++i) {
            if (!(needed & ~checks & major_bit(i))) {
                continue;
            }
            int d = 1000;
            for (size_t j = 0; j < majors; ++j) {
                if (checks & major_bit(j)) {
                    d = std::min(d, std::abs(int(i % majors_x) - int(j % majors_x)) + std::abs(int(i / majors_x) - int(j / majors_x)));
                }
            }
            if (d > far_d) {
                far_d = d;
                far = i;
            }
        }
        checks |= major_bit(far);
    }
    return checks;
}

Check evaluate(const Record &stored, uint64_t checks, std::span<const float, majors> fresh_mm) {
    Check c;
    float sum = 0;
    size_t n = 0;
    for (size_t i = 0; i < majors; ++i) {
        if (!(checks & major_bit(i))) {
            continue;
        }
        if (!std::isfinite(fresh_mm[i]) || !(stored.valid & major_bit(i))) {
            c.refusal = Refusal::probing;
            return c;
        }
        sum += fresh_mm[i] - from_um(stored.z_um[i]);
        ++n;
    }
    if (n == 0) {
        c.refusal = Refusal::probing;
        return c;
    }
    c.offset_mm = sum / float(n);
    for (size_t i = 0; i < majors; ++i) {
        if (checks & major_bit(i)) {
            c.spread_mm = std::max(c.spread_mm, std::fabs(fresh_mm[i] - from_um(stored.z_um[i]) - c.offset_mm));
        }
    }
    if (n < check_points || c.spread_mm > check_spread_mm) {
        c.refusal = Refusal::check_spread;
    } else if (std::fabs(c.offset_mm) > check_offset_mm) {
        c.refusal = Refusal::check_offset;
    }
    return c;
}

} // namespace od3d::mesh_memory
