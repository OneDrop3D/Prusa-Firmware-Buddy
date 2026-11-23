/// @file
#include "filament_memory_od3d.hpp"
#include "filament_memory_math_od3d.hpp"

#include <ctime>

#include <config_store/store_instance.hpp>
#include <feature/filament_sensor/filament_sensors_handler.hpp>
#include <feature/prusa/MMU2/mmu2_mk4.h>
#include <filament.hpp>
#include <marlin_server.hpp>
#include <timing.h>
#include <tool_index.hpp>

namespace od3d::filament_memory {

namespace {

    static_assert(slots == VirtualToolIndex::count);

    constexpr uint32_t step_interval_ms = 250;

    // Fil de Marlin seulement (step)
    uint32_t last_step_ms = 0;
    std::optional<uint32_t> active_since_ms; ///< le MMU répond depuis (rien : il ne répond pas)
    bool decided = false; ///< décision du démarrage prise pour cette connexion du MMU

    bool extruder_sees_filament() {
        return FSensors_instance().has_filament_surely(LogicalFilamentSensor::extruder);
    }

    /// Matière de l'emplacement (vide sans emplacement)
    FilamentTypeParameters::Name material_of(uint8_t slot) {
        return slot < slots ? FilamentType::for_tool(VirtualToolIndex::from_raw(slot)).parameters().name : FilamentTypeParameters::Name {};
    }

    uint32_t now() {
        const time_t t = time(nullptr);
        return t > 1'600'000'000 ? uint32_t(t) : 0; // heure pas encore reçue : inconnue
    }

} // namespace

std::optional<uint8_t> loaded_slot() {
    const auto tool = VirtualToolIndex::currently_selected_opt();
    const FilamentSensorState head = FSensors_instance().sensor_state(LogicalFilamentSensor::extruder);
    return nozzle_slot({
        .mmu_configured = config_store().mmu2_enabled.get(),
        .mmu_active = MMU2::mmu2.Enabled(),
        .prusa_slot = tool ? tool->to_raw() : no_slot,
        .finda = MMU2::mmu2.FindaDetectsFilament(),
        .extruder = head == FilamentSensorState::HasFilament ? Sensor::has : (head == FilamentSensorState::NoFilament ? Sensor::none : Sensor::unknown),
    });
}

void step() {
    const uint32_t now_ms = ticks_ms();
    if (ticks_diff(now_ms, last_step_ms) < int32_t(step_interval_ms)) {
        return;
    }
    last_step_ms = now_ms;

    MMU2::MMU2 &mmu = MMU2::mmu2;
    if (mmu.State() != MMU2::xState::Active) {
        // Démarrage, redémarrage, erreur de communication ou MMU coupé : rien n'est écrit, la décision attend le MMU
        active_since_ms.reset();
        decided = false;
        return;
    }
    if (!active_since_ms) {
        active_since_ms = now_ms;
    }
    const bool command = mmu.commandInProgressManager.isCommandInProgress();
    const uint8_t prusa_slot = mmu.get_current_tool();
    const Record stored = config_store().od3d_loaded_filament.get();

    if (!decided) {
        if (prusa_slot >= slots) {
            // MMU2::Start a tout oublié : filament retrouvé seulement si les capteurs et le MMU sont d'accord
            const BootView view {
                .stored = stored,
                .material_matches = make_record(stored, stored.slot, material_of(stored.slot).data(), stored.time) == stored,
                .finda = mmu.FindaDetectsFilament(),
                .extruder = extruder_sees_filament(),
                .selector = mmu.SelectorSlot(),
                .mmu_busy = command || mmu.MMUCurrentErrorCode() != ErrorCode::OK,
                .active_ms = uint32_t(ticks_diff(now_ms, *active_since_ms)),
            };
            switch (boot_decision(view)) {
            case Boot::wait:
                return;
            case Boot::restore:
                mmu.RestoreLoadedTool(stored.slot);
                break;
            case Boot::forget:
                break;
            }
        }
        decided = true; // sinon : Prusa connaît déjà l'emplacement (seule la communication a repris)
        return;
    }

    if (const auto keep = slot_to_keep(command, marlin_server::is_printing(), prusa_slot)) {
        const Record next = make_record(stored, *keep, material_of(*keep).data(), now());
        if (!(next == stored)) {
            config_store().od3d_loaded_filament.set(next);
        }
    }
}

} // namespace od3d::filament_memory
