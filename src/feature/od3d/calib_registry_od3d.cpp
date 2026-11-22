/// @file
#include "calib_registry_od3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <numbers>

namespace od3d::calib_registry {

namespace {

    int spool_count(uint8_t spools) {
        int n = 0;
        for (; spools; spools &= uint8_t(spools - 1)) {
            ++n;
        }
        return n;
    }

    [[gnu::format(printf, 2, 3)]] bool write(std::span<char> out, const char *format, ...) {
        va_list args;
        va_start(args, format);
        const int n = vsnprintf(out.data(), out.size(), format, args);
        va_end(args);
        return n > 0 && size_t(n) < out.size();
    }

} // namespace

bool applicable(const Entry &e, const Context &ctx) {
    return !((e.needs & accelerometer) && ctx.accelerometer == Accelerometer::no)
        && !((e.needs & mmu) && !ctx.mmu)
        && !((e.needs & (filament | one_spool)) && !ctx.spools)
        && !((e.needs & flow_limit) && !ctx.flow_limit);
}

uint32_t select(const Context &ctx, std::span<const Record> records, bool only_outdated) {
    uint32_t mask = 0;
    for (const Entry &e : entries) {
        const size_t i = size_t(e.test);
        const Record r = i < records.size() ? records[i] : Record {};
        if (applicable(e, ctx) && (!only_outdated || outdated(e, r))) {
            mask |= test_bit(e.test);
        }
    }
    return mask;
}

uint32_t automatic_mask() {
    uint32_t mask = 0;
    for (const Entry &e : entries) {
        mask |= e.automatic ? test_bit(e.test) : 0;
    }
    return mask;
}

uint32_t needs_mask(uint8_t needs) {
    uint32_t mask = 0;
    for (const Entry &e : entries) {
        mask |= (e.needs & needs) ? test_bit(e.test) : 0;
    }
    return mask;
}

bool should_propose(const Context &ctx, std::span<const Record> records, uint32_t dismissed) {
    return (select(ctx, records, true) & automatic_mask()) && dismissed != signature();
}

uint32_t signature() {
    uint32_t h = 2166136261u; // FNV-1a sur (test, version)
    for (const Entry &e : entries) {
        for (const uint8_t byte : { uint8_t(e.test), e.version }) {
            h = (h ^ byte) * 16777619u;
        }
    }
    return h;
}

uint16_t minutes(uint32_t mask, const Context &ctx) {
    unsigned total = 0;
    for (const Entry &e : entries) {
        if (e.automatic && (mask & test_bit(e.test))) {
            total += unsigned(e.minutes) * unsigned((e.needs & filament) ? spool_count(ctx.spools) : 1);
        }
    }
    return uint16_t(total);
}

size_t build_steps(uint32_t mask, const Context &ctx, std::span<Step> out) {
    size_t n = 0;
    const auto add = [&](Test test, uint8_t slot) {
        if (n < out.size()) {
            out[n++] = Step { test, slot };
        }
    };
    const uint8_t first_spool = uint8_t(std::countr_zero(unsigned(ctx.spools))); // 8 et plus : aucune
    for (const Entry &e : entries) {
        if (e.automatic && (mask & test_bit(e.test)) && !(e.needs & filament) && (!(e.needs & one_spool) || first_spool < 8)) {
            add(e.test, (e.needs & one_spool) ? first_spool : 0);
        }
    }
    for (uint8_t slot = 0; slot < 8; ++slot) {
        if (!(ctx.spools & (1u << slot))) {
            continue;
        }
        for (const Entry &e : entries) {
            if (e.automatic && (mask & test_bit(e.test)) && (e.needs & filament)) {
                add(e.test, slot);
            }
        }
    }
    return n;
}

std::optional<int16_t> drop_percent(float remaining, float reference) {
    if (!(reference > 0.f) || !std::isfinite(remaining) || remaining < 0.f) {
        return std::nullopt;
    }
    return int16_t(std::lround(std::clamp(100.f * (1.f - remaining / reference), -999.f, 100.f)));
}

float ringing_cycles(float zeta) {
    return zeta > 0.f ? std::log(20.f) / (2.f * std::numbers::pi_v<float> * zeta) : NAN;
}

bool pa_gcode(uint8_t slot, std::span<char> out) {
    return write(out, "M1994 P%u", unsigned(slot));
}

bool step_gcode(const Step &step, std::span<char> out) {
    switch (step.test) {
    case Test::input_shaper:
        return write(out, "M1959 A"); // A : accéléromètre déjà fixé et vérifié, sans question (patch OneDrop3D)
    case Test::phase_stepping:
        return write(out, "M1977 U A"); // U : sans question (Prusa) ; A : accéléromètre déjà fixé (OneDrop3D)
    case Test::belts:
        return write(out, "M1992");
    case Test::speed_map:
        return write(out, "M1993");
    case Test::damping:
        return write(out, "M1997");
    case Test::mpc:
        return write(out, "M306 T");
    case Test::pa:
        return pa_gcode(step.slot, out);
    case Test::max_flow:
        return write(out, "M1995 P%u", unsigned(step.slot));
    case Test::min_temp:
        return write(out, "M1996 P%u", unsigned(step.slot));
    case Test::mmu_ptfe:
        return write(out, "M1999 B1 P%u", unsigned(step.slot)); // emplacement brut du MMU
    case Test::fan_response:
        return write(out, "M2001");
    case Test::skew:
    case Test::count:
        break;
    }
    return false;
}

} // namespace od3d::calib_registry
