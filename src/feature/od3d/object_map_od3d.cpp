/// @file
#include "object_map_od3d.hpp"

#include <array>
#include <atomic>

namespace od3d::object_map {

namespace {

    std::array<std::atomic<uint32_t>, max_objects> boxes {}; ///< une place par objet, lue d'un bloc par l'interface
    std::atomic<uint32_t> changes { 0 };
    int16_t current = -1; ///< M486 S en cours ; fil de Marlin seulement

} // namespace

void reset() {
    for (auto &b : boxes) {
        b.store(unknown, std::memory_order_relaxed);
    }
    current = -1;
    changes.fetch_add(1, std::memory_order_relaxed);
}

void set_current(int16_t object) {
    current = object;
}

void on_extrusion(float xa, float ya, float xb, float yb) {
    if (current < 0 || static_cast<size_t>(current) >= max_objects) {
        return; // hors objet (M486 S-1 : ligne de purge, tour d'essuyage) ou au-delà des 16 premiers
    }
    std::atomic<uint32_t> &b = boxes[current];
    const uint32_t was = b.load(std::memory_order_relaxed);
    if (const uint32_t now = grow(was, xa, ya, xb, yb); now != was) {
        b.store(now, std::memory_order_relaxed);
        changes.fetch_add(1, std::memory_order_relaxed);
    }
}

uint32_t box(size_t object) {
    return object < max_objects ? boxes[object].load(std::memory_order_relaxed) : unknown;
}

uint32_t revision() {
    return changes.load(std::memory_order_relaxed);
}

} // namespace od3d::object_map
