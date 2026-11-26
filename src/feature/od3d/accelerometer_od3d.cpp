/// @file
#include "accelerometer_od3d.hpp"

#include <cstdint>

#include <Marlin/src/Marlin.h>
#include <Marlin/src/module/planner.h>
#include <Marlin/src/module/prusa/accelerometer.h>
#include <mapi/motion.hpp>
#include <timing.h>

#include <option/has_remote_accelerometer.h>
#include <option/has_toolchanger.h>
#if HAS_REMOTE_ACCELEROMETER() && HAS_TOOLCHANGER()
    #include <tool_index.hpp>
    #include <variant>
#endif

namespace od3d::accel {

void take_head() {
    mapi::ensure_tool_with_accelerometer_picked();
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
