/// @file
#include "mmu_speed_od3d.hpp"
#include "live_od3d.hpp"

#include <algorithm>
#include <atomic>

#include <config_store/store_instance.hpp>
#include <feature/filament_sensor/filament_sensors_handler.hpp>
#include <feature/prusa/MMU2/mmu2_mk4.h>
#include <feature/prusa/MMU2/registers.h>
#include <i18n.h>
#include <Marlin/src/module/planner.h>
#include <marlin_server.hpp>
#include <timing.h>

namespace od3d::mmu_speed {

static_assert(prusa_speed == MMU2::ProtocolLogic::pulleyFeedRatePrusa);

namespace {

    Measurement measurement;
    std::atomic<uint16_t> revisions = 0;
    std::atomic<bool> running = false;
    std::atomic<bool> stop_requested = false;

    // Fil de Marlin seulement (measure() et step())
    std::array<Cycle, levels.size()> done; ///< hors de la pile de Marlin
    bool armed = false;
    bool slow_now = false;
    uint32_t slow_since_ms = 0;
    uint32_t slow_total_ms = 0;

    void publish() {
        ++revisions;
    }

    Result finish(Result result) {
        measurement.result = result;
        running = false;
        publish();
        return result;
    }

    uint16_t ds(uint32_t ms) {
        return uint16_t(std::min<uint32_t>((ms + 50) / 100, UINT16_MAX));
    }

    FilamentSensorState extruder_sensor() {
        return FSensors_instance().sensor_state(LogicalFilamentSensor::extruder);
    }

    /// Vitesse au MMU maintenant (s'il répond) et à ses prochains démarrages
    bool set_speed(uint8_t mm_s) {
        MMU2::MMU2 &mmu = MMU2::mmu2;
        mmu.PlanPulleyFeedRate(mm_s);
        return mmu.State() == MMU2::xState::Active
            && mmu.WriteRegister(uint8_t(MMU2::Register::Pulley_Load_Feedrate), mm_s)
            && mmu.WriteRegister(uint8_t(MMU2::Register::Pulley_Unload_Feedrate), mm_s);
    }

    /// Test de chargement de Prusa à la vitesse déjà envoyée ; une erreur, un réessai automatique ou un chargement raté
    /// (avec réessai caché) comptent
    Cycle cycle(uint8_t slot) {
        MMU2::MMU2 &mmu = MMU2::mmu2;
        const uint16_t errors = mmu.ErrorReports();
        const auto load_fails = config_store().mmu2_load_fails.get();
        slow_total_ms = 0;
        slow_now = false;
        armed = true;
        const uint32_t start = ticks_ms();
        mmu.loading_test(slot);
        const uint32_t end = ticks_ms();
        armed = false;
        if (slow_now) {
            slow_total_ms += uint32_t(ticks_diff(end, slow_since_ms));
        }
        return Cycle {
            .error = mmu.ErrorReports() != errors || config_store().mmu2_load_fails.get() != load_fails || planner.draining(),
            .total_ms = uint32_t(ticks_diff(end, start)),
            .slow_ms = slow_total_ms,
        };
    }

    /// Cycle i fait : sa durée et s'il est passé
    void show_cycle(uint8_t i, const Cycle &c, bool ok) {
        measurement.ds[i] = ds(c.total_ms);
        measurement.passed[i] = ok;
        publish();
    }

    bool must_stop() {
        return stop_requested || planner.draining();
    }

} // namespace

const Measurement &last() {
    return measurement;
}

uint16_t revision() {
    return revisions;
}

bool is_running() {
    return running;
}

void forget() {
    if (!running) {
        measurement = {};
        publish();
    }
}

void request_stop() {
    if (running) {
        stop_requested = true;
    }
}

Saved saved() {
    return config_store().od3d_mmu_speeds.get();
}

void plan_saved() {
    MMU2::mmu2.PlanPulleyFeedRate(speed_of(saved()));
}

Result measure(uint8_t slot) {
    measurement = Measurement { .result = Result::running, .slot = slot };
    stop_requested = false;
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
    running = true;
    publish();

    const uint8_t before = speed_of(saved());
    // Paliers jusqu'au premier qui ne passe pas
    uint8_t n = 0;
    while (n < levels.size() && !must_stop()) {
        measurement.step = n;
        publish();
        live::set(N_("MMU load and unload %d/%d at %d mm/s"), int(n + 1), int(levels.size()), int(levels[n]));
        if (!set_speed(levels[n])) {
            done[n++] = Cycle { .error = true };
            break;
        }
        done[n] = cycle(slot);
        const bool ok = choose(std::span(done.data(), n + 1)).passed > n;
        show_cycle(n, done[n], ok);
        ++n;
        if (!ok) {
            break;
        }
    }
    const Choice choice = choose(std::span(done.data(), n));
    if (must_stop() || !choice.speed) {
        set_speed(before); // arrêt, ou erreur à la vitesse de Prusa : rien ne change
        return finish(must_stop() ? Result::stopped : Result::failed);
    }

    // Vérification à la vitesse gardée (celle de Prusa est déjà mesurée)
    const uint16_t prusa_ds = measurement.ds[0];
    uint16_t cycle_ds = prusa_ds;
    if (choice.speed != prusa_speed) {
        measurement.step = levels.size();
        measurement.speed = choice.speed; // vitesse vérifiée, montrée pendant le cycle
        publish();
        const Cycle check = set_speed(choice.speed) ? cycle(slot) : Cycle { .error = true };
        const bool ok = passed(check, &done[0], nullptr);
        show_cycle(levels.size(), check, ok);
        if (!ok) {
            set_speed(prusa_speed);
            config_store().od3d_mmu_speeds.set(Saved { .assist = saved().assist }); // F11 gardée
            return finish(Result::failed_check);
        }
        cycle_ds = ds(check.total_ms);
    } else {
        set_speed(prusa_speed);
    }
    config_store().od3d_mmu_speeds.set(Saved { .speed = choice.speed, .prusa_ds = prusa_ds, .assist = saved().assist, .cycle_ds = cycle_ds });
    measurement.speed = choice.speed;
    return finish(Result::saved);
}

void reset() {
    set_speed(prusa_speed);
    config_store().od3d_mmu_speeds.set(Saved { .assist = saved().assist }); // F11 gardée
    forget();
}

void step() {
    if (!armed) {
        return;
    }
    const bool slow = MMU2::mmu2.Progress() == ::ProgressCode::FeedingToFSensor;
    const uint32_t now = ticks_ms();
    if (slow && !slow_now) {
        slow_since_ms = now;
    } else if (!slow && slow_now) {
        slow_total_ms += uint32_t(ticks_diff(now, slow_since_ms));
    }
    slow_now = slow;
}

} // namespace od3d::mmu_speed
