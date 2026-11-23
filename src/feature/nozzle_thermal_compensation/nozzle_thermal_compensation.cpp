#include "nozzle_thermal_compensation.hpp"

#include <atomic>
#include <cmath>
#include <config_store/store_instance.hpp>
#include <metric.h>
#include <module/temperature.h>
#include <tool_index.hpp>

namespace buddy::nozzle_thermal_compensation {

namespace {

    METRIC_DEF(metric_offset, "noz_therm_z", METRIC_VALUE_FLOAT, 0, METRIC_ENABLED);

    /// OneDrop3D (F2) : -1 tant que le réglage n'est pas lu. Le garder ici évite de prendre le verrou du
    /// config_store à chaque transformation de position.
    std::atomic<int8_t> enabled_state { -1 };

    /// OneDrop3D (M2003) : NAN tant que la pente n'est pas lue (même raison que enabled_state)
    std::atomic<float> slope_cache { NAN };

} // namespace

uint8_t current_nozzle_key() {
    const PhysicalToolIndex tool = PhysicalToolIndex::from_raw(0);
    return nozzle_key(config_store().get_nozzle_is_high_flow(tool), config_store().get_nozzle_is_hardened(tool));
}

uint16_t measured_nm_per_c(uint8_t key) {
    return config_store().od3d_nozzle_expansion.get(key % nozzle_keys);
}

void set_measured_nm_per_c(uint8_t key, uint16_t nm_per_c) {
    config_store().od3d_nozzle_expansion.set(key % nozzle_keys, nm_per_c);
    reload();
}

float slope_mm_per_c() {
    float slope = slope_cache.load(std::memory_order_relaxed);
    if (std::isnan(slope)) {
        slope = slope_from_nm(measured_nm_per_c(current_nozzle_key()));
        slope_cache.store(slope, std::memory_order_relaxed);
    }
    return slope;
}

void reload() {
    slope_cache.store(NAN, std::memory_order_relaxed);
}

bool is_enabled() {
    int8_t state = enabled_state.load(std::memory_order_relaxed);
    if (state < 0) {
        state = config_store().od3d_nozzle_thermal_compensation.get();
        enabled_state.store(state, std::memory_order_relaxed);
    }
    return state;
}

void set_enabled(bool enabled) {
    config_store().od3d_nozzle_thermal_compensation.set(enabled);
    enabled_state.store(enabled, std::memory_order_relaxed);
}

float current_elongation_vs_reference_mm() {
    const auto tool = PhysicalToolIndex::currently_selected_opt();
    if (!tool || !is_enabled()) {
        return 0;
    }

    const float offset_mm = elongation_vs_reference_mm(Temperature::degTargetHotend(*tool), slope_mm_per_c());

    // Recorded on change rather than on a tick, so the metric shows only the events that moved it
    static float recorded_mm = NAN;
    if (offset_mm != recorded_mm) {
        recorded_mm = offset_mm;
        metric_record_float(&metric_offset, offset_mm);
    }

    return offset_mm;
}

} // namespace buddy::nozzle_thermal_compensation
