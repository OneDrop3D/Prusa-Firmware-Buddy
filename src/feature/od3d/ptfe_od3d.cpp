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
    std::atomic<uint16_t> last_mmu = 0;
    std::atomic<uint32_t> last_write_packed = 0; ///< demandée << 16 | relue

    // Fil de Marlin seulement (measure() et step())
    bool armed = false;
    Tracker tracker;

    FilamentSensorState extruder_sensor() {
        return FSensors_instance().sensor_state(LogicalFilamentSensor::extruder);
    }

    Result finish(Result result, uint16_t measured = 0, uint16_t saved = 0, uint16_t mmu = 0) {
        last_measured = measured;
        last_saved = saved;
        last_mmu = mmu;
        last_result = result;
        return result;
    }

} // namespace

Status status() {
    return { last_result.load(), last_measured.load(), last_saved.load(), last_mmu.load() };
}

Write last_write() {
    const uint32_t w = last_write_packed.load();
    return { uint16_t(w >> 16), uint16_t(w) };
}

uint16_t write_length(uint16_t mm) {
    constexpr uint8_t reg = uint8_t(MMU2::Register::Bowden_Length);
    uint16_t mmu = MMU2::mmu2.BowdenLength();
    if (valid_length(mm) && MMU2::mmu2.WriteRegister(reg, mm)) {
        mmu = MMU2::mmu2.ReadRegister(reg) ? MMU2::mmu2.GetLastReadRegisterValue() : 0;
    }
    last_write_packed = (uint32_t(mm) << 16) | mmu;
    return mmu;
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
    // pendant l'avance lente. L'avance rapide est celle relue : si le MMU n'a pas pris 341, la distance reste juste.
    const uint16_t fast = write_length(min_mm);
    if (!valid_length(fast)) {
        return finish(Result::refused_mmu);
    }
    finish(Result::running);
    tracker = {};
    armed = true;
    MMU2::mmu2.loading_test(slot); // Prusa : charge jusqu'aux engrenages puis décharge vers le MMU, erreurs comprises
    armed = false;

    const Tracker::Outcome o = tracker.outcome(fast);
    if (o.result != Result::saved) {
        // Échec : l'ancienne longueur revient (le registre est persistant dans le MMU)
        if (valid_length(before)) {
            write_length(before);
        }
        return finish(o.result, o.measured_mm);
    }
    const uint16_t mmu = write_length(o.saved_mm);
    return finish(mmu == o.saved_mm ? Result::saved : Result::not_applied, o.measured_mm, o.saved_mm, mmu);
}

void step() {
    if (armed) {
        tracker.sample(MMU2::mmu2.Progress() == ::ProgressCode::FeedingToFSensor, MMU2::mmu2.PulleyPosition(), extruder_sensor() == FilamentSensorState::HasFilament);
    }
}

} // namespace od3d::ptfe
