/// @file
#include "accelerometer_od3d.hpp"

#include <cstdint>

#include <Marlin/src/Marlin.h>
#include <Marlin/src/module/planner.h>
#include <Marlin/src/module/prusa/accelerometer.h>
#include <Marlin/src/module/motion.h>
#include <Marlin/src/module/temperature.h>
#include <mapi/parking.hpp>
#include <timing.h>
#include <tool_index.hpp>

#include <option/has_remote_accelerometer.h>
#include <option/has_toolchanger.h>
#if HAS_REMOTE_ACCELEROMETER() && HAS_TOOLCHANGER()
    #include <Marlin/src/module/prusa/toolchanger.h>
    #include <variant>
#endif

namespace od3d::accel {

bool take_head() {
#if HAS_REMOTE_ACCELEROMETER() && HAS_TOOLCHANGER()
    // Comme mapi::ensure_tool_with_accelerometer_picked (la première tête au dock calibré, sans levée : Z est déjà
    // monté), mais un échec arrête la mesure au lieu d'un écran rouge
    if (!PhysicalToolIndex::currently_selected_opt()) {
        return prusa_toolchanger.pick_any_tool(tool_return_t::no_return, {}, tool_change_lift_t::no_lift, false);
    }
#endif
    return true;
}

bool move_to(float x, float y, [[maybe_unused]] float speed) {
#if HAS_TOOLCHANGER()
    return mapi::park({ .x = x, .y = y });
#else
    do_blocking_move_to_xy(x, y, speed);
    return !planner.draining();
#endif
}

bool head_available() {
#if HAS_REMOTE_ACCELEROMETER() && HAS_TOOLCHANGER()
    // pick_any_tool ne prend que les têtes au dock calibré (is_tool_enabled) : sans aucune, Prusa ferait un écran rouge
    return PhysicalToolIndex::currently_selected_opt().has_value() || prusa_toolchanger.get_num_enabled_tools() > 0;
#else
    return true;
#endif
}

bool nozzle_hot(float max_c) {
    const auto hot = [max_c](PhysicalToolIndex tool) {
        return Temperature::degHotend(tool) > max_c || Temperature::degTargetHotend(tool) > max_c;
    };
    if (const auto tool = PhysicalToolIndex::currently_selected_opt()) {
        return hot(*tool);
    }
    for (const PhysicalToolIndex tool : PhysicalToolIndex::all()) {
        if (hot(tool)) {
            return true;
        }
    }
    return false;
}

bool ready() {
#if HAS_REMOTE_ACCELEROMETER() && HAS_TOOLCHANGER()
    // INDX sans tête prise : l'accéléromètre de la tête ne peut pas répondre. Il est dans chaque tête : la mesure en
    // prend une après l'origine (take_head) et le lit alors ; son échec y est signalé comme une mesure ratée
    if (std::holds_alternative<NoTool>(PhysicalToolIndex::currently_selected())) {
        return true;
    }
#endif
    planner.synchronize(); // tête à l'arrêt : seule la pesanteur est mesurée

    // Comme l'assistant input shaper de Prusa (M1959, info_proceed) : la construction recherche la puce si elle
    // n'était pas déjà vue
    PrusaAccelerometer accelerometer;
    if (accelerometer.get_error() != accelerometer::Error::none) {
        return false;
    }

    // Puis quelques échantillons (1 344 par seconde) : une puce débranchée depuis la dernière session passe le
    // contrôle de Prusa, pas celui de la pesanteur
    constexpr uint32_t samples = 64;
    constexpr uint32_t timeout_ms = 300;
    float sum[3] = {};
    uint32_t count = 0;
    accelerometer.clear();
    const uint32_t start = ticks_ms();
    while (count < samples && ticks_ms() - start < timeout_ms) {
        accelerometer::RawAcceleration raw;
        switch (accelerometer.get_sample(raw)) {
        case PrusaAccelerometer::GetSampleResult::ok: {
            const accelerometer::Acceleration a = raw.to_acceleration();
            for (int i = 0; i < 3; ++i) {
                sum[i] += a.val[i];
            }
            ++count;
            break;
        }
        case PrusaAccelerometer::GetSampleResult::error:
            if (accelerometer.get_error() == accelerometer::Error::communication) {
                return false;
            }
            accelerometer.clear(); // file pleine : on continue
            break;
        case PrusaAccelerometer::GetSampleResult::buffer_empty:
            idle(true);
            break;
        }
    }
    return count == samples && gravity_plausible(sum[0] / count, sum[1] / count, sum[2] / count);
}

} // namespace od3d::accel
