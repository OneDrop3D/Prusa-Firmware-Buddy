/// @file
#include "force_probe_od3d.hpp"

#include <atomic>
#include <cmath>

namespace od3d::force_probe {

namespace {

    force::TracePoint buffer[capacity];

    std::atomic<bool> armed = false;
    std::atomic<uint16_t> count = 0;
    std::atomic<bool> overflow = false;

    // État de l'interruption (écrit par start() seulement quand armed est faux)
    bool have_base = false;
    float base_force = 0;
    float base_x = 0;
    float last_x = 0;
    float sum_force = 0;
    float sum_x = 0;
    uint8_t summed = 0;
    uint32_t first_us = 0;
    std::atomic<uint32_t> last_us = 0;

} // namespace

void on_loadcell_sample(uint32_t time_us, float force_g, float x_mm) {
    if (!armed.load(std::memory_order_acquire)) {
        return;
    }
    if (!std::isfinite(force_g)) {
        return;
    }
    if (std::isfinite(x_mm)) {
        last_x = x_mm;
    } else {
        x_mm = last_x; // positionLookback ne connaît pas encore cet instant : dernière position connue
    }
    if (!have_base) {
        have_base = true;
        base_force = force_g;
        base_x = x_mm;
        first_us = time_us;
    }
    sum_force += force_g;
    sum_x += x_mm;
    if (++summed < decimation) {
        return;
    }
    const uint16_t n = count.load(std::memory_order_relaxed);
    if (n < capacity) {
        buffer[n] = force::make_point(sum_force / float(summed) - base_force, sum_x / float(summed) - base_x);
        last_us.store(time_us, std::memory_order_relaxed);
        count.store(n + 1, std::memory_order_release);
    } else {
        overflow.store(true, std::memory_order_relaxed);
    }
    sum_force = 0;
    sum_x = 0;
    summed = 0;
}

void start() {
    armed.store(false, std::memory_order_release);
    // L'interruption ne touche plus à son état (armed faux) : on peut le remettre à zéro
    have_base = false;
    sum_force = 0;
    sum_x = 0;
    summed = 0;
    last_x = NAN;
    count.store(0, std::memory_order_relaxed);
    overflow.store(false, std::memory_order_relaxed);
    armed.store(true, std::memory_order_release);
}

void stop() {
    armed.store(false, std::memory_order_release);
}

bool recording() {
    return armed.load(std::memory_order_relaxed);
}

force::Trace trace() {
    return force::Trace(buffer, count.load(std::memory_order_acquire));
}

float dt() {
    const uint16_t n = count.load(std::memory_order_acquire);
    if (n < 2) {
        return 0;
    }
    // Du premier échantillon au dernier du dernier point : n × decimation − 1 intervalles de la cellule
    const uint32_t span_us = last_us.load(std::memory_order_relaxed) - first_us;
    return float(span_us) * 1e-6f * float(decimation) / float(uint32_t(n) * decimation - 1);
}

bool overflowed() {
    return overflow.load(std::memory_order_relaxed);
}

bool write_csv(FILE *file, float label) {
    if (!file) {
        return false;
    }
    const force::Trace t = trace();
    const float step = dt();
    for (size_t i = 0; i < t.size(); ++i) {
        if (fprintf(file, "D,%.4f,%.4f,%.2f,%.2f\n", double(label), double(float(i) * step), double(force::x_mm(t[i])), double(force::force_g(t[i]))) < 0) {
            return false;
        }
    }
    return true;
}

} // namespace od3d::force_probe
