/// @file
#include "perf_od3d.hpp"

#include <array>
#include <cstdio>
#include <device/hal.h>
#include <feature/od3d/force_probe_od3d.hpp>
#include <FreeRTOS.h>
#include <freertos/critical_section.hpp>
#include <heap.h>
#include <timing.h>
#include <version/version.hpp>

namespace od3d::perf {

Counters counters;
Usb usb;

uint32_t cycles() {
    return DWT->CYCCNT; // actif dans toutes les compilations (main.cpp)
}

namespace {

    constexpr size_t ring_rows = 16; // 16 × 28 o en RAM (64 avant : 1,3 Ko rendus au tas, 25/11)

    // Tas libre le plus bas vu à la fin d'un dessin (écran bleu « Out of heap memory » du 25/11). La part que newlib n'a
    // jamais prise (heap_bytes_remaining) ne fait que baisser : elle est déjà son plus bas, toutes tâches comprises
    uint32_t heap_free_min = UINT32_MAX;

    std::array<Row, ring_rows> rows;
    uint32_t written = 0; ///< lignes écrites depuis le dernier vidage (la dernière est rows[(written - 1) % ring_rows])
    bool dumping = false;

    Counters at_begin;
    uint32_t begin_cycles = 0;
    uint32_t begin_ms = 0;
    Cause input_cause = Cause::none;
    uint32_t input_us = 0;
    uint32_t input_count = 0;

    uint16_t sat16(uint32_t v) {
        return uint16_t(std::min<uint32_t>(v, UINT16_MAX));
    }

    uint32_t cycles_to_us(uint32_t c) {
        return c / (SystemCoreClock / 1000000);
    }

} // namespace

uint32_t inputs() {
    return input_count;
}

void input(Cause cause) {
    ++input_count;
    if (input_cause == Cause::none) { // la première entrée pas encore dessinée : la latence part d'elle
        input_cause = cause;
        input_us = ticks_us();
    }
}

void drop_input() {
    input_cause = Cause::none;
}

void draw_begin() {
    at_begin = counters;
    begin_ms = ticks_ms();
    begin_cycles = cycles();
}

void draw_end(uint32_t screen) {
    const uint32_t draw_cycles = cycles() - begin_cycles;
    heap_free_min = std::min<uint32_t>(heap_free_min, xPortGetFreeHeapSize());
    const Row row {
        .t_ms = begin_ms,
        .screen = screen,
        .draw_us = cycles_to_us(draw_cycles),
        .bytes = counters.bytes - at_begin.bytes,
        .dma_wait_us = cycles_to_us(counters.dma_cycles - at_begin.dma_cycles),
        .windows = sat16(counters.windows - at_begin.windows),
        .span_ms = sat16(cycles_to_us(draw_cycles) / 1000),
        .input_10us = input_cause == Cause::none ? uint16_t(0) : sat16((ticks_us() - input_us) / 10),
        .draws = 1,
        .cause = input_cause,
    };
    input_cause = Cause::none;
    if (row.bytes == 0) {
        return; // invalidé mais rien d'envoyé
    }
    const uint32_t end_ms = begin_ms + row.span_ms;
    freertos::CriticalSection lock; // M2010 lit l'anneau depuis le fil de Marlin
    if (dumping) {
        return;
    }
    if (written == 0 || !merge(rows[(written - 1) % ring_rows], row, end_ms)) {
        rows[written++ % ring_rows] = row;
    }
}

void note(Step step, uint32_t start_ms, uint32_t us, const Usb &delta) {
    const Row piece {
        .t_ms = start_ms,
        .screen = uint32_t(step) | delta.split << 8,
        .draw_us = us,
        .bytes = delta.sectors * 512,
        .dma_wait_us = delta.us,
        .windows = sat16(delta.commands),
        .span_ms = sat16(ticks_ms() - start_ms),
        .input_10us = sat16(us / 10),
        .draws = 1,
        .cause = Cause::usb,
    };
    freertos::CriticalSection lock; // M2010 lit l'anneau depuis le fil de Marlin
    if (dumping) {
        return;
    }
    // Les dessins de la carte (ligne « durée · grammes ») s'intercalent entre ses tranches : sa ligne est cherchée parmi
    // les dernières
    for (uint32_t back = 1; back <= std::min<uint32_t>(written, 4); ++back) {
        if (merge_step(rows[(written - back) % ring_rows], piece)) {
            return;
        }
    }
    rows[written++ % ring_rows] = piece;
}

void note_heap(uint32_t free_before, uint32_t free_after, uint8_t caches, uint8_t thumbs_kb) {
    const Row row {
        .t_ms = ticks_ms(),
        .screen = uint32_t(Step::heap),
        .draw_us = 0,
        .bytes = free_before,
        .dma_wait_us = free_after,
        .windows = caches,
        .span_ms = 0,
        .input_10us = 0,
        .draws = thumbs_kb,
        .cause = Cause::heap,
    };
    freertos::CriticalSection lock;
    if (!dumping) {
        rows[written++ % ring_rows] = row;
    }
}

bool dump(const char *path) {
    FILE *probe = fopen(path, "r");
    const bool fresh = !probe;
    if (probe) {
        fclose(probe);
    }
    FILE *f = fopen(path, "a");
    if (!f) {
        return false;
    }
    uint32_t count;
    {
        freertos::CriticalSection lock;
        dumping = true; // l'interface n'écrit plus dans l'anneau
        count = written;
    }
    const uint32_t kept = std::min<uint32_t>(count, ring_rows);
    bool ok = true;
    if (fresh) {
        ok &= force_probe::csv_printf(f, "t_ms,screen,cause,input_ms,draws,span_ms,draw_us,bytes,windows,dma_wait_us\n") > 0;
    }
    ok &= force_probe::csv_printf(f, "# OneDrop3D UI %s, at %lu ms, %lu rows, %lu lost\n", version::project_version_full,
              static_cast<unsigned long>(ticks_ms()), static_cast<unsigned long>(kept), static_cast<unsigned long>(count - kept))
        > 0;
    ok &= force_probe::csv_printf(f, "# heap free now %lu B, lowest at a draw %lu B, never used %lu B of %lu B\n",
              static_cast<unsigned long>(xPortGetFreeHeapSize()), static_cast<unsigned long>(heap_free_min),
              static_cast<unsigned long>(heap_bytes_remaining()), static_cast<unsigned long>(heap_total_size()))
        > 0;
    // Onglet Imprimer (AUDIT_PERF_IMPRIMER.md) : lignes « u » et « h », colonnes réutilisées
    ok &= force_probe::csv_printf(f, "# u: screen=step (1 folder 2 card 3 page 4 preview 5 preview image)+256*split reads, "
                                     "draw_us=work, input_ms=longest slice, draws=slices, bytes=read, windows=USB reads, "
                                     "dma_wait_us=USB wait; h: bytes=free heap before, dma_wait_us=after, windows=1 folder "
                                     "2 infos 4 thumbs, draws=thumbs KB\n")
        > 0;
    for (uint32_t i = count - kept; i < count; ++i) {
        const Row &r = rows[i % ring_rows];
        char input[12] = "";
        if (r.cause != Cause::none && r.cause != Cause::heap) {
            snprintf(input, sizeof(input), "%u.%02u", unsigned(r.input_10us / 100), unsigned(r.input_10us % 100));
        }
        static constexpr char cause_letter[] = { '-', 'k', 't', 'u', 'h' };
        ok &= force_probe::csv_printf(f, "%lu,0x%08lx,%c,%s,%u,%u,%lu,%lu,%u,%lu\n", static_cast<unsigned long>(r.t_ms),
                  static_cast<unsigned long>(r.screen), cause_letter[uint8_t(r.cause)], input, unsigned(r.draws), unsigned(r.span_ms),
                  static_cast<unsigned long>(r.draw_us), static_cast<unsigned long>(r.bytes), unsigned(r.windows),
                  static_cast<unsigned long>(r.dma_wait_us))
            > 0;
    }
    ok &= fclose(f) == 0;
    freertos::CriticalSection lock;
    written = 0;
    dumping = false;
    return ok;
}

} // namespace od3d::perf
