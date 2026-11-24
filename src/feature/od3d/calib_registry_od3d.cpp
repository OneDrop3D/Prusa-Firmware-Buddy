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
        && !((e.needs & flow_limit) && !ctx.flow_limit)
        && !(e.test == Test::mpc && !ctx.mpc); // M-16 : pas de bit de Need libre
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

uint32_t redo_mask(std::span<const Record> records) {
    uint32_t mask = 0;
    for (const Entry &e : entries) {
        const size_t i = size_t(e.test);
        const uint8_t done = i < records.size() ? records[i].version : 0;
        mask |= (e.needs & filament) && done > 0 && done < e.version ? test_bit(e.test) : 0u;
    }
    return mask;
}

uint32_t series_tests(bool heat) {
    return automatic_mask() & (heat ? needs_mask(heated) : ~needs_mask(heated));
}

uint32_t proposed(const Context &ctx, std::span<const Record> records) {
    const uint32_t redo = redo_mask(records);
    uint32_t mask = 0;
    for (const Entry &e : entries) {
        const size_t i = size_t(e.test);
        const Record r = i < records.size() ? records[i] : Record {};
        if (!e.automatic || !applicable(e, ctx)) {
            continue;
        }
        const bool wanted = (e.needs & filament) ? spools_to_measure(e.test, ctx, redo & test_bit(e.test)) != 0 : outdated(e, r);
        mask |= wanted ? test_bit(e.test) : 0u;
    }
    return mask;
}

std::array<uint8_t, 8> spool_order(std::span<const int16_t> bed, std::span<const int16_t> nozzle, std::optional<uint8_t> loaded) {
    std::array<uint8_t, 8> order { 0, 1, 2, 3, 4, 5, 6, 7 };
    const auto key = [&](uint8_t slot) -> int32_t {
        if (slot >= bed.size() || slot >= nozzle.size()) {
            return INT32_MAX; // emplacement inexistant : à la fin
        }
        return (int32_t(bed[slot]) * 1024 + int32_t(nozzle[slot])) * 2 + (loaded == slot ? 0 : 1);
    };
    std::stable_sort(order.begin(), order.end(), [&](uint8_t a, uint8_t b) { return key(a) < key(b); });
    return order;
}

bool numbers_shown(const Entry &e, Result result, std::span<const int16_t> after) {
    const bool any = std::any_of(after.begin(), after.end(), [](int16_t v) { return v != no_value; });
    return any && (result == Result::ok || (e.needs & filament));
}

uint16_t signature(std::span<const Entry> list) {
    uint32_t h = 2166136261u; // FNV-1a sur (test, version)
    for (const Entry &e : list) {
        for (const uint8_t byte : { uint8_t(e.test), e.version }) {
            h = (h ^ byte) * 16777619u;
        }
    }
    const uint16_t folded = uint16_t(h ^ (h >> 16));
    return folded ? folded : 1;
}

uint16_t minutes(uint32_t mask, const Context &ctx, uint32_t redo) {
    unsigned total = 0;
    for (const Entry &e : entries) {
        if (e.automatic && (mask & test_bit(e.test))) {
            total += unsigned(e.minutes) * unsigned((e.needs & filament) ? spool_count(spools_to_measure(e.test, ctx, redo & test_bit(e.test))) : 1);
        }
    }
    // Plateau de la dilatation chauffé et reposé pendant les tests du MMU à froid (bed_step()) : ~2 min de moins
    if ((mask & test_bit(Test::nozzle_expansion)) && (mask & needs_mask(mmu))) {
        total -= 2;
    }
    return uint16_t(total);
}

size_t build_steps(uint32_t mask, const Context &ctx, std::span<Step> out, uint32_t redo) {
    size_t n = 0;
    const auto add = [&](Test test, uint8_t slot) {
        if (n < out.size()) {
            out[n++] = Step { test, slot };
        }
    };
    const uint8_t first_spool = uint8_t(std::countr_zero(unsigned(ctx.spools))); // 8 et plus : aucune
    const auto add_machine = [&](const Entry &e) {
        if (e.automatic && (mask & test_bit(e.test)) && !(e.needs & filament) && (!(e.needs & one_spool) || first_spool < 8)) {
            add(e.test, (e.needs & one_spool) ? first_spool : 0);
        }
    };
    for (const Entry &e : entries) {
        // Tests du MMU à froid ensemble juste avant la dilatation : le plateau chauffe et se repose pour elle pendant eux
        if (e.test == Test::nozzle_expansion) {
            for (const Entry &m : entries) {
                if (bed_free(m.test)) {
                    add_machine(m);
                }
            }
        }
        if (!bed_free(e.test)) {
            add_machine(e);
        }
    }
    for (const uint8_t slot : ctx.order) {
        if (slot >= 8 || !(ctx.spools & (1u << slot))) {
            continue;
        }
        const auto wanted = [&](const Entry &e) {
            return e.automatic && (mask & test_bit(e.test)) && (e.needs & filament) && (spools_to_measure(e.test, ctx, redo & test_bit(e.test)) & (1u << slot));
        };
        for (const Entry &e : entries) {
            if (wanted(e) && e.test != Test::min_temp) {
                add(e.test, slot);
            }
        }
        if (wanted(entry(Test::min_temp))) {
            add(Test::min_temp, slot); // finit sous la température de la matière : en dernier
        }
    }
    return n;
}

std::optional<size_t> bed_step(std::span<const Step> steps, size_t i, bool loading) {
    size_t j = i;
    while (j < steps.size() && bed_free(steps[j].test)) {
        ++j;
    }
    if (j == i && !loading) {
        return std::nullopt; // l'étape i tourne avec le plateau tel qu'il est (MPC : l'air de la chambre ne doit pas changer)
    }
    return j < steps.size() && (entry(steps[j].test).needs & empty_bed) ? std::optional<size_t>(j) : std::nullopt;
}

void defer(std::span<Step> steps, size_t i) {
    if (i >= steps.size() || steps[i].deferred) {
        return;
    }
    steps[i].deferred = true;
    std::rotate(steps.begin() + i, steps.begin() + i + 1, steps.end());
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
    return write(out, "M1994 P%u V1", unsigned(slot)); // V1 : motif compact (~1 min au lieu de ~4), même avec « Grand motif PA »
}

bool load_gcode(uint8_t slot, const char *filament, int16_t nozzle, std::optional<Spot> spot, std::span<char> out) {
    if (spot) {
        return write(out, "M1989 L%u\nG28 O I\nM104 S%d\nG27 X%.1f Y%.1f Z%.1f P1\nM701 S\"%s\" W2 P%u", unsigned(slot), int(nozzle), double(spot->x), double(spot->y),
            double(spot->z), filament, unsigned(slot));
    }
    return write(out, "G28 O I\nM104 S%d\nG27 W2\nM701 S\"%s\" W2 P%u", int(nozzle), filament, unsigned(slot));
}

bool step_gcode(const Step &step, std::span<char> out) {
    switch (step.test) {
    case Test::input_shaper:
        return write(out, "M1959 A"); // A : accéléromètre déjà fixé et vérifié, sans question (patch OneDrop3D)
    case Test::phase_stepping:
        return write(out, "M1977 U A"); // U : sans question (Prusa) ; A : accéléromètre déjà fixé (OneDrop3D)
    case Test::belts:
        return write(out, "M1992 C"); // C : comparaison des modes, tant que la mesure n'est pas validée (V-03)
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
        return write(out, "M2001 V"); // V : puis les vibrations des ventilateurs, si l'accéléromètre répond
    case Test::accel_limit:
        return write(out, "M2004");
    case Test::accel_mount:
        return write(out, "M2005");
    case Test::shaper_check:
        return write(out, "M2006");
    case Test::nozzle_expansion:
        return write(out, "M2003");
    case Test::e_jump:
        return write(out, "M2000 S%u", unsigned(step.slot));
    case Test::mmu_speed:
        return write(out, "M2002 P%u", unsigned(step.slot));
    case Test::skew:
    case Test::count:
        break;
    }
    return false;
}

} // namespace od3d::calib_registry
