/// @file
#include "force_probe_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdlib>
#include <heap.h>

namespace od3d::force_probe {

namespace {

    /// 6 Ko pris au tas par reserve() ou start(), rendus par release() à la fin de la mesure (session::Session) : statiques avant,
    /// ils manquaient au réseau (écran bleu « Out of heap memory » du 25/11). L'interruption n'écrit que si armed est
    /// vrai, et release() le remet à faux avant de libérer : elle ne peut pas l'interrompre au milieu d'une écriture.
    std::atomic<force::TracePoint *> buffer = nullptr;

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
    bool use_y = false; ///< position le long de Y plutôt que X
    uint8_t summed = 0;
    uint32_t first_us = 0;
    std::atomic<uint32_t> last_us = 0;

    // Fenêtre de mesure : écrite par l'interruption seulement quand window_on est vrai
    std::atomic<bool> window_on = false;
    uint32_t window_count = 0;
    float window_shift = 0; ///< premier point de la fenêtre : sommes des écarts à lui (précision des flottants)
    float window_sum = 0;
    float window_sum_sq = 0;

} // namespace

void on_loadcell_sample(uint32_t time_us, float force_g, float x_mm, float y_mm) {
    if (!armed.load(std::memory_order_acquire)) {
        return;
    }
    if (use_y) {
        x_mm = y_mm; // la trace garde la position le long des lignes
    }
    if (!std::isfinite(force_g)) {
        return;
    }
    force_g = -force_g; // la cellule donne une charge négative quand la buse appuie : force vers le haut positive
    if (std::isfinite(x_mm)) {
        last_x = x_mm;
    } else if (!have_base) {
        return; // pas encore de position connue : un zéro de X indéfini fausserait toute la trace
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
    const float point_force = sum_force / float(summed) - base_force;
    if (window_on.load(std::memory_order_acquire)) {
        if (window_count == 0) {
            window_shift = point_force;
        }
        const float d = point_force - window_shift;
        window_sum += d;
        window_sum_sq += d * d;
        ++window_count;
    }
    const uint16_t n = count.load(std::memory_order_relaxed);
    force::TracePoint *const points = buffer.load(std::memory_order_relaxed);
    if (points && n < capacity) {
        points[n] = force::make_point(point_force, sum_x / float(summed) - base_x);
        last_us.store(time_us, std::memory_order_relaxed);
        count.store(n + 1, std::memory_order_release);
    } else {
        overflow.store(true, std::memory_order_relaxed);
    }
    sum_force = 0;
    sum_x = 0;
    summed = 0;
}

bool reserve() {
    if (!buffer.load(std::memory_order_relaxed)) {
        buffer.store(static_cast<force::TracePoint *>(malloc_fallible(capacity * sizeof(force::TracePoint))), std::memory_order_relaxed);
    }
    return buffer.load(std::memory_order_relaxed) != nullptr;
}

bool start(bool along_y) {
    armed.store(false, std::memory_order_release);
    window_on.store(false, std::memory_order_release);
    // L'interruption ne touche plus à son état (armed faux) : on peut le remettre à zéro
    const bool traced = reserve();
    use_y = along_y;
    have_base = false;
    sum_force = 0;
    sum_x = 0;
    summed = 0;
    last_x = NAN;
    count.store(0, std::memory_order_relaxed);
    overflow.store(false, std::memory_order_relaxed);
    armed.store(true, std::memory_order_release); // après buffer : l'interruption le voit avec lui
    return traced;
}

void stop() {
    armed.store(false, std::memory_order_release);
    window_on.store(false, std::memory_order_release);
}

void release() {
    stop();
    count.store(0, std::memory_order_relaxed);
    free(buffer.exchange(nullptr));
}

void window_begin() {
    window_on.store(false, std::memory_order_release);
    // L'interruption ne touche plus aux sommes (window_on faux) : on peut les remettre à zéro
    window_count = 0;
    window_sum = 0;
    window_sum_sq = 0;
    window_on.store(true, std::memory_order_release);
}

void window_end() {
    window_on.store(false, std::memory_order_release);
}

WindowStats window() {
    if (window_on.load(std::memory_order_acquire) || window_count == 0) {
        return {};
    }
    const float n = float(window_count);
    const float mean = window_sum / n;
    const float var = window_sum_sq / n - mean * mean;
    return { .count = window_count, .mean_g = window_shift + mean, .sd_g = std::sqrt(std::max(var, 0.f)) };
}

bool recording() {
    return armed.load(std::memory_order_relaxed);
}

force::Trace trace() {
    force::TracePoint *const points = buffer.load(std::memory_order_relaxed);
    return force::Trace(points, points ? count.load(std::memory_order_acquire) : 0);
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

int csv_printf(FILE *file, const char *format, ...) {
    static char line[256]; // hors de la pile de Marlin (seul fil qui écrit les CSV)
    va_list args;
    va_start(args, format);
    const int n = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (n < 0) {
        return -1;
    }
    const size_t length = std::min(size_t(n), sizeof(line) - 1);
    return fwrite(line, 1, length, file) == length ? int(length) : -1;
}

bool write_csv(FILE *file, float label) {
    if (!file) {
        return false;
    }
    const force::Trace t = trace();
    const float step = dt();
    for (size_t i = 0; i < t.size(); ++i) {
        if (csv_printf(file, "D,%.4f,%.4f,%.2f,%.2f\n", double(label), double(float(i) * step), double(force::x_mm(t[i])), double(force::force_g(t[i]))) < 0) {
            return false;
        }
    }
    return true;
}

} // namespace od3d::force_probe
