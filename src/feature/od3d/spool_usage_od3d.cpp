/// @file
#include "spool_usage_od3d.hpp"

#include <atomic>
#include <cmath>
#include <mutex>

#include <config_store/store_instance.hpp>
#include <feature/filament_tracker/filament_tracker.hpp>
#include <filament.hpp>
#include <freertos/mutex.hpp>
#include <marlin_server.hpp>
#include <timing.h>
#include <tool_index.hpp>

#include <option/has_anfc.h>
#if HAS_ANFC()
    #include <feature/openprinttag/filament_usage_tracker/filament_usage_tracker.hpp>
#endif

namespace od3d::spool_usage {

namespace {

    constexpr uint8_t slots = VirtualToolIndex::count;
    static_assert(slots == 5);

    constexpr uint32_t step_interval_ms = 1000;

    /// Ce qui n'est pas en mémoire : longueurs du FilamentTracker (depuis le démarrage, en mm) déjà comptées
    struct State {
        freertos::Mutex mutex;
        uint32_t counted_mm[slots] {}; ///< déjà retiré du reste enregistré
        uint32_t print_start_mm[slots] {}; ///< au départ de l'impression en cours
        uint32_t seen_mm[slots] {}; ///< au tour précédent de step()
        uint32_t moved_ms[slots] {}; ///< dernière fois que l'extrusion de l'emplacement a avancé
        uint32_t last_step_ms = 0;
        bool printing = false; ///< une impression a commencé et n'est pas encore finie
    };

    State state_instance; ///< à portée de fichier (comme les mutex de Prusa) : construit avant les threads

    State &state() {
        return state_instance;
    }

    std::atomic<uint8_t> low_alerts { 0 }; ///< bit i : l'emplacement i vient de passer sous son seuil

    uint32_t extruded_mm(uint8_t slot) {
        return buddy::filament_tracker().get_extruded_distance(VirtualToolIndex::from_raw(slot));
    }

    float slot_density(uint8_t slot) {
        const FilamentType type = FilamentType::for_tool(VirtualToolIndex::from_raw(slot));
        return type ? density(type.parameters().name.data()) : default_density;
    }

    /// Longueur extrudée pas encore comptée [mm] ; à appeler sous le mutex
    uint32_t pending_mm(const State &s, uint8_t slot) {
        const uint32_t now = extruded_mm(slot);
        return now > s.counted_mm[slot] ? now - s.counted_mm[slot] : 0;
    }

    /// Écrit ce qui est en attente (et, en fin d'impression, la dernière impression) ; sous le mutex
    void commit(State &s, uint8_t slot, bool end_of_print) {
        const uint32_t now = extruded_mm(slot);
        const uint32_t pending = now > s.counted_mm[slot] ? now - s.counted_mm[slot] : 0;
        s.counted_mm[slot] = now;

        const Record before = config_store().od3d_spool_usage.get(slot);
        if (!is_tracked(before)) {
            return; // pas de poids net : rien à suivre, l'extrusion est simplement oubliée
        }
        const float g_cm3 = slot_density(slot);
        Record after = consume(before, grams(float(pending), g_cm3));
        if (end_of_print && now > s.print_start_mm[slot]) {
            after.last_print_g = grams(float(now - s.print_start_mm[slot]), g_cm3);
        }
        if (after == before) {
            return;
        }
        config_store().od3d_spool_usage.set(slot, after);
        if (crossed_alert(before.remaining_g, after.remaining_g, after.alert_g)) {
            low_alerts.fetch_or(uint8_t(1 << slot));
        }
    }

    /// Modification par l'interface : ce qui est en attente est d'abord écrit, puis f(record) remplace l'enregistrement
    template <class F>
    void modify(uint8_t slot, F &&f) {
        if (slot >= slots) {
            return;
        }
        State &s = state();
        std::lock_guard lock(s.mutex);
        commit(s, slot, false);
        const Record before = config_store().od3d_spool_usage.get(slot);
        config_store().od3d_spool_usage.set(slot, f(before));
    }

} // namespace

std::optional<Report> report(uint8_t slot) {
    if (slot >= slots) {
        return std::nullopt;
    }
    const VirtualToolIndex tool = VirtualToolIndex::from_raw(slot);

#if HAS_ANFC()
    // Étiquette OpenPrintTag suivie par Prusa : ses données priment
    if (const auto tag = buddy::openprinttag::filament_usage_tracker().filament_report(tool); tag && tag->full_weight_g > 0) {
        return Report {
            .remaining_g = tag->remaining_weight_g,
            .net_g = tag->full_weight_g,
            .alert_g = 0,
            .used_g = NAN,
            .last_print_g = NAN,
            .g_per_mm = tag->g_per_mm,
            .from_tag = true,
        };
    }
#endif

    State &s = state();
    Record r;
    uint32_t pending;
    {
        std::lock_guard lock(s.mutex);
        r = config_store().od3d_spool_usage.get(slot);
        pending = pending_mm(s, slot);
    }
    if (!is_tracked(r)) {
        return std::nullopt;
    }
    const float g_per_mm = grams_per_mm(slot_density(slot));
    const Record live = consume(r, float(pending) * g_per_mm);
    return Report {
        .remaining_g = live.remaining_g,
        .net_g = float(live.net_g),
        .alert_g = float(live.alert_g),
        .used_g = live.used_g,
        .last_print_g = live.last_print_g,
        .g_per_mm = g_per_mm,
        .from_tag = false,
    };
}

Record record(uint8_t slot) {
    return slot < slots ? config_store().od3d_spool_usage.get(slot) : Record {};
}

void step() {
    State &s = state();
    const uint32_t now_ms = ticks_ms();
    if (ticks_diff(now_ms, s.last_step_ms) < int32_t(step_interval_ms)) {
        return;
    }
    s.last_step_ms = now_ms;

    const std::optional<VirtualToolIndex> active = VirtualToolIndex::currently_selected_opt();
    const bool printing = marlin_server::is_printing();

    std::lock_guard lock(s.mutex);
    for (uint8_t slot = 0; slot < slots; ++slot) {
        const uint32_t now = extruded_mm(slot);
        if (now != s.seen_mm[slot]) {
            s.seen_mm[slot] = now;
            s.moved_ms[slot] = now_ms;
        }
        const uint32_t pending = pending_mm(s, slot);
        if (pending == 0) {
            continue;
        }
        const float pending_g = grams(float(pending), slot_density(slot));
        const bool is_active = active.has_value() && active->to_raw() == slot;
        if (should_commit(pending_g, printing, is_active, uint32_t(ticks_diff(now_ms, s.moved_ms[slot])))) {
            commit(s, slot, false);
        }
    }
}

void on_print_start() {
    State &s = state();
    std::lock_guard lock(s.mutex);
    for (uint8_t slot = 0; slot < slots; ++slot) {
        commit(s, slot, false); // ce qui précède l'impression (chargement…) n'en fait pas partie
        s.print_start_mm[slot] = extruded_mm(slot);
    }
    s.printing = true;
}

void on_print_end() {
    State &s = state();
    std::lock_guard lock(s.mutex);
    const bool end = s.printing; // finalize_print peut être appelé plusieurs fois : la dernière impression une fois
    s.printing = false;
    for (uint8_t slot = 0; slot < slots; ++slot) {
        commit(s, slot, end);
    }
}

void new_spool(uint8_t slot, uint16_t net_g) {
    modify(slot, [net_g](const Record &r) { return spool_usage::new_spool(r, net_g); });
}

void set_net(uint8_t slot, uint16_t net_g) {
    modify(slot, [net_g](const Record &r) { return spool_usage::set_net(r, net_g); });
}

void set_remaining(uint8_t slot, float remaining_g) {
    modify(slot, [remaining_g](Record r) {
        r.remaining_g = std::fmax(0.0f, remaining_g);
        return r;
    });
}

void set_alert(uint8_t slot, uint16_t alert_g) {
    modify(slot, [alert_g](Record r) {
        r.alert_g = alert_g;
        return r;
    });
}

std::optional<uint8_t> take_low_alert() {
    uint8_t bits = low_alerts.load();
    while (bits) {
        const uint8_t slot = uint8_t(__builtin_ctz(bits));
        if (low_alerts.compare_exchange_weak(bits, uint8_t(bits & ~(1 << slot)))) {
            return slot;
        }
    }
    return std::nullopt;
}

} // namespace od3d::spool_usage
