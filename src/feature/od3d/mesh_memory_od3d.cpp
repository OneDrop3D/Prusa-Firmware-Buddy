/// @file
#include "mesh_memory_od3d.hpp"
#include "mesh_memory_math_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>

#include <config_store/store_instance.hpp>
#include <feature/bedlevel/bedlevel.h>
#include <feature/nozzle_thermal_compensation/nozzle_thermal_compensation.hpp>
#include <metric.h>
#include <module/motion.h>
#include <module/probe.h>
#include <module/temperature.h>
#include <tool_index.hpp>
#include <option/has_chamber_api.h>
#include <option/has_hotend_type_support.h>
#include <option/has_sheet_profiles.h>
#if HAS_CHAMBER_API()
    #include <feature/chamber/chamber.hpp>
#endif

static_assert(GRID_MAJOR_POINTS_X == od3d::mesh_memory::majors_x && GRID_MAJOR_POINTS_Y == od3d::mesh_memory::majors_y);
static_assert(GRID_BORDER + (GRID_MAJOR_POINTS_X - 1) * GRID_MAJOR_STEP == GRID_MAX_POINTS_X - 1 - GRID_BORDER);
static_assert(GRID_BORDER + (GRID_MAJOR_POINTS_Y - 1) * GRID_MAJOR_STEP == GRID_MAX_POINTS_Y - 1 - GRID_BORDER);
static_assert(od3d::mesh_memory::age_choices.back() == od3d::mesh_memory::chosen_max_age_hours);

namespace od3d::mesh_memory {

uint8_t max_age_hours() {
    return config_store().od3d_mesh_reuse_hours.get();
}

void set_max_age_hours(uint8_t hours) {
    config_store().od3d_mesh_reuse_hours.set(hours);
}

void forget() {
    config_store().od3d_mesh.set(Record {});
}

bool has_mesh() {
    return !config_store().od3d_mesh.get().empty();
}

namespace {

    METRIC_DEF(metric_od3d_mesh, "od3d_mesh", METRIC_VALUE_CUSTOM, 0, METRIC_DISABLED);

    // Hors de la pile de G29 (fil de Marlin, un G29 à la fois)
    Record stored; ///< maillage gardé, relu par restore()
    uint64_t restored = 0; ///< points remis par restore(), à corriger par verify()
    bool chain = false; ///< dernier G29 P1 sans C : zone de l'impression, gardé ou repris (son extension C s'y ajoute)
    uint32_t chain_time = 0; ///< date du maillage de cette suite
    PrintChoice choice; ///< « Maillage » d'« Avant l'impression » pour l'impression demandée

    float &z_of(size_t i) {
        return unified_bed_leveling::z_values[GRID_BORDER + (i % majors_x) * GRID_MAJOR_STEP][GRID_BORDER + (i / majors_x) * GRID_MAJOR_STEP];
    }

    xy_pos_t pos_of(size_t i) {
        return { unified_bed_leveling::mesh_index_to_xpos(GRID_BORDER + (i % majors_x) * GRID_MAJOR_STEP),
            unified_bed_leveling::mesh_index_to_ypos(GRID_BORDER + (i / majors_x) * GRID_MAJOR_STEP) };
    }

    /// Points que probe_major_points() palperait dans area (même tri : atteignables par la sonde, dans la zone)
    uint64_t needed(const PrintArea::rect_t &area) {
        uint64_t m = 0;
        for (size_t i = 0; i < majors; ++i) {
            const xy_pos_t p = pos_of(i);
            if (position_is_reachable_by_probe(p.x, p.y) && area.contains(p)) {
                m |= major_bit(i);
            }
        }
        return m;
    }

    /// Points majeurs valides du maillage actuel (comme probe_major_points : ni NAN, ni 0)
    uint64_t valid_now() {
        uint64_t m = 0;
        for (size_t i = 0; i < majors; ++i) {
            if (std::isfinite(z_of(i)) && z_of(i) != 0.f) {
                m |= major_bit(i);
            }
        }
        return m;
    }

    uint32_t now_s() {
        const time_t t = time(nullptr);
        return t > 1'600'000'000 ? uint32_t(t) : 0; // heure pas encore reçue : inconnue
    }

    /// Ce qui rend un maillage valable, relevé maintenant (plateau mesuré, consigne de la buse de palpage...)
    [[gnu::noinline]] void conditions(Record &r) {
        r = Record {};
        r.version = Record::current_version;
        r.time = now_s();
        r.bed_c10 = to_c10(thermalManager.degBed());
#if HAS_CHAMBER_API()
        r.chamber_c10 = to_c10(buddy::chamber().current_temperature().value_or(NAN));
#endif
        const PhysicalToolIndex tool = PhysicalToolIndex::from_raw(0);
        r.probe_c = int16_t(thermalManager.degTargetHotend(tool));
        r.probe_offset_um = to_um(probe_offset.z);
        r.nozzle_um = uint16_t(std::lround(std::clamp(config_store().get_nozzle_diameter(tool), 0.f, 5.f) * 1000.f));
        r.flags = (config_store().get_nozzle_is_hardened(tool) ? 1 : 0) | (config_store().get_nozzle_is_high_flow(tool) ? 2 : 0)
            | (buddy::nozzle_thermal_compensation::is_enabled() ? 4 : 0);
#if HAS_HOTEND_TYPE_SUPPORT()
        r.hotend = uint8_t(config_store().hotend_type.get(tool.to_raw()));
#endif
#if HAS_SHEET_PROFILES()
        r.sheet = config_store().active_sheet.get();
#endif
    }

    void note(const char *event, Refusal refusal, float offset_mm = 0.f, float spread_mm = 0.f) {
        metric_record_custom(&metric_od3d_mesh, ",event=%s refusal=%ui,offset=%.4f,spread=%.4f", event, unsigned(refusal), double(offset_mm), double(spread_mm));
    }

    /// Garde les points majeurs du maillage actuel avec les conditions de maintenant, datés de when
    [[gnu::noinline]] void store(uint32_t when) {
        static Record r; // hors de la pile
        conditions(r);
        r.time = when;
        r.valid = valid_now();
        for (size_t i = 0; i < majors; ++i) {
            r.z_um[i] = (r.valid & major_bit(i)) ? to_um(z_of(i)) : 0;
        }
        stored = r;
        config_store().od3d_mesh.set(r);
        note("stored", Refusal::none);
    }

} // namespace

bool offered() {
    Record now; // fil de l'interface : 128 octets sur sa pile
    conditions(now);
    return compare_before_print(config_store().od3d_mesh.get(), now, max_age_s(max_age_hours(), true)) == Refusal::none;
}

void choose_for_next_print(bool reuse) {
    choice.choose(reuse);
}

void on_print_request() {
    choice.on_print_request();
}

uint64_t restore(const PrintArea::rect_t &probe_area) {
    restored = 0;
    const uint32_t age_s = max_age_s(max_age_hours(), choice.current());
    if (age_s == 0) {
        return 0; // réglage désactivé sans choix, ou « Maillage : nouveau »
    }
    static Record now; // hors de la pile de G29
    stored = config_store().od3d_mesh.get();
    conditions(now);
    const uint64_t need = needed(probe_area);
    Refusal refusal = compare(stored, now, age_s);
    if (refusal == Refusal::none) {
        refusal = covers(stored, need);
    }
    if (refusal != Refusal::none) {
        note("refused", refusal);
        return 0;
    }
    unified_bed_leveling::invalidate();
    for (size_t i = 0; i < majors; ++i) {
        if (stored.valid & major_bit(i)) {
            z_of(i) = from_um(stored.z_um[i]);
        }
    }
    const uint64_t checks = pick_checks(need);
    for (size_t i = 0; i < majors; ++i) {
        if (checks & major_bit(i)) {
            z_of(i) = NAN; // palpé de nouveau par G29
        }
    }
    restored = stored.valid & ~checks;
    return checks;
}

bool verify(uint64_t checks) {
    static std::array<float, majors> fresh; // hors de la pile de G29
    fresh.fill(NAN);
    for (size_t i = 0; i < majors; ++i) {
        if (checks & major_bit(i)) {
            fresh[i] = z_of(i);
        }
    }
    const Check c = evaluate(stored, checks, fresh);
    if (c.refusal != Refusal::none) {
        note("check_failed", c.refusal, c.offset_mm, c.spread_mm);
        unified_bed_leveling::invalidate();
        restored = 0;
        return false;
    }
    // Écart moyen : l'origine Z de cette impression (sa dispersion) ; les points de contrôle gardent leur palpage
    for (size_t i = 0; i < majors; ++i) {
        if (restored & major_bit(i)) {
            z_of(i) = from_um(stored.z_um[i]) + c.offset_mm;
        }
    }
    restored = 0;
    note("reused", Refusal::none, c.offset_mm, c.spread_mm);
    return true;
}

void on_probed(bool extend, bool print_area, bool reused) {
    if (ubl.g29_probing_failed) {
        chain = false;
        return;
    }
    if (!extend) {
        // Seul le maillage d'une impression est gardé ; repris, il garde sa date (il vieillit depuis son palpage).
        // Gardé même réglage désactivé : « Avant l'impression » peut proposer de le reprendre (128 octets par impression)
        chain_time = reused ? stored.time : now_s();
        chain = print_area && chain_time != 0;
        if (chain && !reused) {
            store(chain_time);
        }
        return;
    }
    // Extension près de la purge, juste après : ajoutée au maillage gardé si elle a palpé des points nouveaux
    if (chain && (valid_now() & ~stored.valid)) {
        store(chain_time);
    }
}

} // namespace od3d::mesh_memory
