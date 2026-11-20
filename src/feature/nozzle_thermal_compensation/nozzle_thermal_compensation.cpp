#include "nozzle_thermal_compensation.hpp"

#include <atomic>
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

} // namespace

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

    const float offset_mm = elongation_vs_reference_mm(Temperature::degTargetHotend(*tool));

    // Recorded on change rather than on a tick, so the metric shows only the events that moved it
    static float recorded_mm = NAN;
    if (offset_mm != recorded_mm) {
        recorded_mm = offset_mm;
        metric_record_float(&metric_offset, offset_mm);
    }

    return offset_mm;
}

} // namespace buddy::nozzle_thermal_compensation
