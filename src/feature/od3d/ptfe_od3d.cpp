/// @file
#include "ptfe_od3d.hpp"
#include "ptfe_math_od3d.hpp"

#include <atomic>

#include <feature/filament_sensor/filament_sensors_handler.hpp>
#include <feature/prusa/MMU2/mmu2_mk4.h>
#include <feature/prusa/MMU2/registers.h>
#include <marlin_server.hpp>

namespace od3d::ptfe {

namespace {

    std::atomic<Result> last_result = Result::none;
    std::atomic<uint16_t> last_measured = 0;
    std::atomic<uint16_t> last_saved = 0;

    // Fil de Marlin seulement (measure() et step())
    bool armed = false;
    Tracker tracker;

    FilamentSensorState extruder_sensor() {
        return FSensors_instance().sensor_state(LogicalFilamentSensor::extruder);
    }

    Result finish(Result result, uint16_t measured = 0, uint16_t saved = 0) {
        last_measured = measured;
        last_saved = saved;
        last_result = result;
        return result;
    }

} // namespace

Status status() {
    return { last_result.load(), last_measured.load(), last_saved.load() };
}

void forget() {
    finish(Result::none);
}

Result measure(uint8_t slot) {
    if (marlin_server::is_printing()) {
        return finish(Result::refused_printing);
    }
    if (slot >= slots || MMU2::mmu2.State() != MMU2::xState::Active) {
        return finish(Result::refused_mmu);
    }
    switch (extruder_sensor()) {
    case FilamentSensorState::NoFilament:
        break;
    case FilamentSensorState::HasFilament:
        return finish(Result::refused_extruder);
    default:
        return finish(Result::refused_sensor);
    }
    if (MMU2::mmu2.FindaDetectsFilament()) {
        return finish(Result::refused_selector);
    }

    const uint16_t before = MMU2::mmu2.BowdenLength();
    // Tout le trajet au-delà de 341 mm se fait lentement : jamais d'erreur « capteur trop tôt », et la poulie est lue
    // pendant l'avance lente
    if (!MMU2::mmu2.WriteRegister(uint8_t(MMU2::Register::Bowden_Length), min_mm)) {
        return finish(Result::refused_mmu);
    }
    finish(Result::running);
    tracker = {};
    armed = true;
    MMU2::mmu2.loading_test(slot); // Prusa : charge jusqu'aux engrenages puis décharge vers le MMU, erreurs comprises
    armed = false;

    const Tracker::Outcome o = tracker.outcome(min_mm);
    // Échec : l'ancienne longueur revient (le registre est persistant dans le MMU)
    MMU2::mmu2.WriteRegister(uint8_t(MMU2::Register::Bowden_Length), o.result == Result::saved ? o.saved_mm : before);
    return finish(o.result, o.measured_mm, o.saved_mm);
}

void step() {
    if (armed) {
        tracker.sample(MMU2::mmu2.Progress() == ::ProgressCode::FeedingToFSensor, MMU2::mmu2.PulleyPosition(), extruder_sensor() == FilamentSensorState::HasFilament);
    }
}

} // namespace od3d::ptfe
