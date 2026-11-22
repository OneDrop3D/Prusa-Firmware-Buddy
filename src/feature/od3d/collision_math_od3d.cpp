/// @file
#include "collision_math_od3d.hpp"

#include <algorithm>

namespace od3d::collision {

Params params_for(Sensitivity s) {
    switch (s) {
    case Sensitivity::low:
        return { .threshold_g = 600.f, .hits = 4 };
    case Sensitivity::high:
        return { .threshold_g = 250.f, .hits = 2 };
    case Sensitivity::normal:
    default:
        return { .threshold_g = 400.f, .hits = 3 };
    }
}

void Detector::accel(float az_g) {
    if (!std::isfinite(az_g)) {
        return;
    }
    if (!std::isfinite(accel_base)) {
        accel_base = az_g;
    }
    accel_base += accel_alpha * (az_g - accel_base); // gravité et inclinaison : seule la secousse reste
    accel_peak = std::fmax(accel_peak, std::fabs(az_g - accel_base));
}

void Detector::restart(uint32_t time_us, float up_g) {
    have_base = true;
    in_pulse = false;
    rest_valid = false; // le repos d'avant (autre filament, autre outil) ne vaut plus
    armed_us = time_us;
    base_g = up_g;
    last_up = NAN;
    noise = 0;
}

bool Detector::sample(uint32_t time_us, float up_g, float x, float y, float z, float e, bool armed, float log_g) {
    const float jolt = accel_peak;
    accel_peak = NAN;

    const uint32_t dt_us = std::min<uint32_t>(time_us - last_us, 100'000); // trou dans les échantillons : borné
    last_us = time_us;
    if (std::isfinite(e)) {
        if (std::isfinite(last_e) && dt_us > 0) {
            const float v = (e - last_e) / (float(dt_us) * 1e-6f);
            if (v > fast_e_mm_s || v < -back_e_mm_s) {
                fast_e_seen = true;
                last_fast_e_us = time_us;
            }
        }
        if (!(std::fabs(e - e_still) <= still_e_mm)) {
            e_still = e; // l'extrudeur bouge : le repos recommence d'ici
            e_still_us = time_us;
        }
        last_e = e;
    }
    // positionLookback peut ne pas connaître l'instant : dernière position connue
    if (std::isfinite(x)) {
        last_x = x;
    }
    if (std::isfinite(y)) {
        last_y = y;
    }
    if (std::isfinite(z)) {
        last_z = z;
    }

    if (!armed || !std::isfinite(up_g) || !(last_z >= min_z_mm)) {
        have_base = false; // au réarmement, la moyenne repart du premier échantillon
        in_pulse = false;
        return false;
    }
    if (!have_base) {
        restart(time_us, up_g);
        return false;
    }

    const float dt = float(dt_us);
    const bool fast_e_recent = fast_e_seen && time_us - last_fast_e_us < fast_e_guard_us;
    const bool at_rest = std::isfinite(e_still) && time_us - e_still_us >= rest_after_us && !fast_e_recent;
    const float reference = rest_valid ? std::max(base_g, rest_g) : base_g;
    const float d = up_g - reference;

    if (!in_pulse) {
        if (d >= log_g && rest_valid && time_us - armed_us >= settle_us && !fast_e_recent) {
            in_pulse = true;
            pulse_start_us = time_us;
            current = Pulse { .peak_g = d, .noise_g = noise, .x = last_x, .y = last_y, .z = last_z, .accel_g = jolt };
            return false;
        }
        // Moyenne, repos et bruit ne suivent que hors impulsion (un choc ne les déplace pas). Le bruit est l'écart d'un
        // échantillon au suivant : les changements de débit, lents devant 3 ms, ne le gonflent pas
        base_g += std::min(1.f, dt / float(baseline_tau_us)) * (up_g - base_g);
        if (std::isfinite(last_up)) {
            noise += std::min(1.f, dt / float(noise_tau_us)) * (std::fabs(up_g - last_up) - noise);
        }
        last_up = up_g;
        if (at_rest) {
            rest_g = rest_valid ? rest_g + std::min(1.f, dt / float(rest_tau_us)) * (up_g - rest_g) : up_g;
            rest_valid = true;
        }
        return false;
    }

    if (fast_e_recent) {
        in_pulse = false; // rétraction pendant l'impulsion : pas un choc
        return false;
    }
    current.accel_g = std::fmax(current.accel_g, jolt);
    if (d > current.peak_g) {
        current.peak_g = d;
        current.x = last_x;
        current.y = last_y;
        current.z = last_z;
    }
    const uint32_t duration_us = time_us - pulse_start_us;
    if (d < 0.5f * log_g) {
        in_pulse = false;
        if (duration_us > max_pulse_us) {
            return false;
        }
        current.duration_ms = uint16_t(duration_us / 1000);
        done = current;
        return true;
    }
    if (duration_us > max_pulse_us) {
        // Une marche : la moyenne (et le repos, si l'extrudeur est arrêté) repartent d'ici
        base_g = up_g;
        if (at_rest) {
            rest_g = up_g;
        }
        in_pulse = false;
    }
    return false;
}

bool counts(const Pulse &p, float threshold_g) {
    return p.peak_g >= threshold_g && p.peak_g >= noise_factor * p.noise_g;
}

bool confirmed(const Pulse &p) {
    return p.accel_g >= accel_confirm_g; // NAN (pas d'accéléromètre) : faux
}

HitMap::Result HitMap::add(uint32_t time_ms, float x, float y, bool is_confirmed, uint8_t hits_needed) {
    // Oublie les chocs trop anciens
    uint8_t kept = 0;
    for (uint8_t i = 0; i < count; ++i) {
        if (time_ms - hits[i].time_ms <= window_ms) {
            hits[kept++] = hits[i];
        }
    }
    count = kept;

    const auto near = [&](const Hit &h) {
        return std::hypot(h.x - x, h.y - y) <= radius_mm;
    };
    const uint8_t weight = is_confirmed ? 2 : 1;

    Hit *rebound = nullptr;
    for (uint8_t i = 0; i < count; ++i) {
        if (near(hits[i]) && time_ms - hits[i].time_ms < rebound_ms) {
            rebound = &hits[i];
        }
    }
    if (rebound) {
        rebound->weight = std::max(rebound->weight, weight); // même choc (rebond) : pas un de plus
        rebound->time_ms = time_ms;
    } else {
        if (count == hits.size()) {
            std::rotate(hits.begin(), hits.begin() + 1, hits.end()); // le plus ancien sort
            --count;
        }
        hits[count++] = Hit { .time_ms = time_ms, .x = x, .y = y, .weight = weight };
    }

    Result r;
    for (uint8_t i = 0; i < count; ++i) {
        if (near(hits[i])) {
            ++r.near;
            r.weight += hits[i].weight;
        }
    }
    r.alert = r.near >= 2 && r.weight >= hits_needed;
    if (r.alert) {
        clear();
    }
    return r;
}

} // namespace od3d::collision
