/// @file
#include "mmu_assist_od3d.hpp"
#include "lookahead_od3d.hpp"

#include <atomic>

#include <config_store/store_instance.hpp>
#include <feature/prusa/MMU2/protocol_logic.h>
#include <common/marlin_vars.hpp>
#include <timing.h>

namespace od3d::mmu_assist {

namespace {

    // Fil de Marlin seulement (ProtocolLogic)
    Link link;
    Speed speed;
    std::atomic<Support> support_seen = Support::unknown;
    std::atomic<uint16_t> applied_seen = 0;

} // namespace

bool enabled() {
    return config_store().od3d_mmu_speeds.get().assist;
}

void set_enabled(bool on) {
    auto s = config_store().od3d_mmu_speeds.get();
    s.assist = on;
    config_store().od3d_mmu_speeds.set(s);
}

Support support() {
    return support_seen.load();
}

uint16_t applied() {
    return applied_seen.load();
}

// ---- Appelés par ProtocolLogic (fil de Marlin), MMU au repos ----

Exchange next_exchange(uint16_t &value) {
    const uint32_t now = ticks_ms();
    if (!link.due(now)) {
        return Exchange::none; // la plupart des appels : rien à lire ni à calculer
    }
    const bool run = link.support() == Support::yes && enabled() && marlin_vars().print_state == marlin_server::State::Printing;
    if (!link.next(now, speed.next(now, run, run ? lookahead::planner_e_speed_ahead(max_e_speed) : 0.0f))) {
        return Exchange::none;
    }
    if (link.detecting()) {
        return Exchange::read;
    }
    value = link.value();
    return Exchange::write;
}

void read_back(uint16_t raw) {
    link.read_back(raw);
    support_seen = link.support();
    applied_seen = link.applied();
}

void restart() {
    link = Link {};
    speed = Speed {};
    support_seen = Support::unknown;
    applied_seen = 0;
}

} // namespace od3d::mmu_assist
