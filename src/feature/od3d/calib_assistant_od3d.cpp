/// @file
#include "calib_assistant_od3d.hpp"

#include "accel_cap_od3d.hpp"
#include "accelerometer_od3d.hpp"
#include "belt_tension_od3d.hpp"
#include "damping_od3d.hpp"
#include "fan_response_od3d.hpp"
#include "lookahead_od3d.hpp"
#include "e_jump_od3d.hpp"
#include "filament_memory_od3d.hpp"
#include "live_od3d.hpp"
#include "measure_session_od3d.hpp"
#include "mpc_control_od3d.hpp"
#include "mmu_speed_od3d.hpp"
#include "mmu_usb_od3d.hpp"
#include "mpc_od3d.hpp"
#include "nozzle_expansion_od3d.hpp"
#include "pa_od3d.hpp"
#include "ptfe_od3d.hpp"
#include "speed_map_od3d.hpp"
#include "spool_calib_od3d.hpp"
#include "vibration_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <optional>
#include <tuple>

#include <common/marlin_server.hpp>
#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <feature/nozzle_thermal_compensation/nozzle_thermal_compensation.hpp>
#include <feature/prusa/MMU2/mmu2_mk4.h>
#include <filament.hpp>
#include <i18n.h>
#include <Marlin/src/gcode/gcode.h>
#include <Marlin/src/gcode/queue.h>
#include <Marlin/src/module/motion.h>
#include <Marlin/src/module/planner.h>
#include <Marlin/src/module/temperature.h>
#include <test_result.hpp>
#include <tool_index.hpp>
#include <utils/string_builder.hpp>

namespace od3d::calib_assistant {

using namespace calib_registry;

namespace {

    constexpr size_t max_steps = 32;
    std::array<Step, max_steps> steps;
    std::atomic<uint8_t> step_count = 0;
    std::atomic<uint8_t> current = 0;
    std::atomic<bool> running = false;
    std::atomic<bool> cancel_requested = false;
    std::atomic<bool> bed_full = false; ///< série en pause : plus de place sur le plateau, l'utilisateur le nettoie (M1989 C)
    std::atomic<uint8_t> skip_index = 0xff; ///< étape passée par l'utilisateur (0xff : aucune)
    std::atomic<uint16_t> runs = 0;
    std::array<std::atomic<Result>, max_tests> results {};
    uint32_t run_mask = 0;
    std::atomic<uint32_t> postponed_mask = 0; ///< tests reportés, le MMU indisponible jusqu'à la fin de la série
    Context run_ctx; ///< l'imprimante au début de la série

    std::array<Numbers, max_tests> before_numbers;
    std::array<Numbers, max_tests> after_numbers;

    std::atomic<Check> last_check = Check::none;
    std::atomic<uint16_t> checks = 0;

    /// Résultat de Prusa avant son étape (input shaper, phase stepping) : remis si l'étape est annulée
    TestResult prusa_before = TestResult::unknown;

    /// Chargement de la bobine avant le PA, F5, F6, M2000, ou déchargement avant le tube PTFE (mis en file d'injection,
    /// lu après le retour de M1989)
    char load_sequence[128];

    bool cold() {
        return nozzle_cold(Temperature::degHotend(0), Temperature::degTargetHotend(0));
    }

    uint32_t now() {
        const time_t t = time(nullptr);
        return t > 1'600'000'000 ? uint32_t(t) : 0; // heure pas encore reçue : inconnue
    }

    std::optional<TestResult> prusa_result(Test test) {
        switch (test) {
        case Test::input_shaper:
            return config_store().selftest_result_input_shaper_calibration.get();
        case Test::phase_stepping:
            return config_store().selftest_result_phase_stepping.get();
        default:
            return std::nullopt;
        }
    }

    void set_prusa_result(Test test, TestResult r) {
        if (test == Test::input_shaper) {
            config_store().selftest_result_input_shaper_calibration.set(r);
        } else if (test == Test::phase_stepping) {
            config_store().selftest_result_phase_stepping.set(r);
        }
    }

    bool succeeded(const Step &s) {
        switch (s.test) {
        case Test::input_shaper:
        case Test::phase_stepping: {
            const TestResult r = *prusa_result(s.test);
            if (r == TestResult::unknown) {
                set_prusa_result(s.test, prusa_before); // annulée : l'état de Prusa d'avant
            }
            return r == TestResult::passed;
        }
        case Test::belts: // tant que la mesure n'est pas validée, jamais en échec à cause d'un pic non concluant
            return belts::assistant_passed(belts::status() == belts::Status::done, belts::last().peaks[0].found, belts::last().peaks[1].found);
        case Test::speed_map:
            return speed_map::status() == speed_map::Status::done;
        case Test::damping:
            return damping::status() == damping::Status::done;
        case Test::mpc:
            return mpc::tune_status().phase == mpc::Phase::done;
        case Test::pa: {
            const pa::Status st = pa::status();
            return st.phase == pa::Phase::done && st.slot == s.slot;
        }
        case Test::mmu_ptfe:
            return ptfe::status().result == ptfe::Result::saved;
        case Test::fan_response: { // vibrations mesurées, ou refusées sans accéléromètre ni buse froide (accéléromètre passé :
                                   // le temps de montée suffit, le bilan dit « non mesurée »)
            const fan_vibration::Status v = fan_vibration::status();
            return fan_response::status() == fan_response::Status::done
                && (v == fan_vibration::Status::done || v == fan_vibration::Status::no_accelerometer || v == fan_vibration::Status::hot_nozzle);
        }
        case Test::accel_limit:
            return accel_limit::status() == accel_limit::Status::done;
        case Test::accel_mount: // pose correcte : tourné, de travers ou mal fixé, c'est un échec avec sa raison au bilan
            return accel_mount::passed();
        case Test::shaper_check: // un axe filtré au moins, aucun dont le filtre ne retire presque plus rien
            return shaper_check::passed();
        case Test::nozzle_expansion:
            return nozzle_expansion::status() == nozzle_expansion::Status::saved;
        case Test::e_jump: {
            const e_jump::Status st = e_jump::status();
            return st.phase == e_jump::Phase::done && st.slot == s.slot;
        }
        case Test::mmu_speed:
            return mmu_speed::last().result == mmu_speed::Result::saved;
        case Test::max_flow:
        case Test::min_temp: {
            const calib::Status st = calib::status();
            const calib::Kind kind = s.test == Test::max_flow ? calib::Kind::flow : calib::Kind::temperature;
            return st.kind == kind && st.phase == calib::Phase::done && st.slot == s.slot;
        }
        case Test::skew:
        case Test::count:
            break;
        }
        return false;
    }

    int16_t scaled(float v, float k) {
        return std::isfinite(v) ? int16_t(std::lround(std::clamp(v * k, -32767.f, 32767.f))) : no_value;
    }

    /// Chiffres du test (voir numbers_before()) : avant son étape, ou après son succès
    void capture(const Step &s, Numbers &n, bool after) {
        const Values stored = stored_values(size_t(s.test));
        switch (s.test) {
        case Test::input_shaper:
            break; // M1959 A, qui a le spectre
        case Test::phase_stepping:
            if (!after) {
                std::copy(stored.v.begin(), stored.v.end(), n.begin());
            } // après : M1977
            break;
        case Test::fan_response:
            if (!after) {
                std::copy(stored.v.begin(), stored.v.end(), n.begin()); // temps de montée de la fois d'avant
                n[1] = int16_t(lookahead::fan_lead_ds());
            } else {
                const fan_response::Result &r = fan_response::result();
                n[0] = scaled(r.rise.rise_s, 1000.f);
                n[1] = int16_t(r.lead_ds);
                const auto refs = fan_vibration::stored_references(); // écrites par M2001 V : gardées telles quelles
                n[2] = refs[0];
                n[3] = refs[1];
            }
            break;
        case Test::accel_limit: {
            const auto cap = accel_cap::get(); // gardé tel quel : « Appliquer » le change après la série
            for (uint8_t a = 0; a < accel_limit::axis_count; ++a) {
                n[2 + a] = cap[a];
                if (!after) { // M201 de la machine
                    n[a] = int16_t(std::min<uint32_t>(planner.user_settings.max_acceleration_mm_per_s2[a], INT16_MAX));
                } else {
                    const auto p = accel_limit::proposal(a);
                    n[a] = p ? int16_t(*p) : no_value;
                }
            }
            break;
        }
        case Test::accel_mount:
            if (!after) {
                std::copy(stored.v.begin(), stored.v.end(), n.begin());
            } else {
                const accel_mount::Measurement &m = accel_mount::last();
                n[0] = scaled(m.tilt_deg, 10.f); // 0,1°
                n[1] = scaled(m.share[0], 100.f); // %
                n[2] = scaled(m.share[1], 100.f);
                n[3] = scaled(m.noise_rms, 1000.f); // mm/s²
            }
            break;
        case Test::shaper_check:
            if (!after) {
                std::copy(stored.v.begin(), stored.v.end(), n.begin());
            } else {
                const shaper_check::Measurement &m = shaper_check::last();
                n[0] = scaled(m.reduction[0], 100.f); // % des vibrations retirées par le filtre
                n[1] = scaled(m.reduction[1], 100.f);
                n[2] = m.accel ? int16_t(m.accel) : no_value;
            }
            break;
        case Test::belts:
        case Test::damping:
        case Test::speed_map:
            if (!after) {
                std::copy(stored.v.begin(), stored.v.end(), n.begin());
            } else if (s.test == Test::belts) {
                for (uint8_t b = 0; b < belts::belt_count; ++b) {
                    const auto &peak = belts::last().peaks[b];
                    n[b] = peak.found ? scaled(peak.frequency, 10.f) : no_value;
                }
            } else if (s.test == Test::damping) {
                for (uint8_t a = 0; a < damping::axis_count; ++a) {
                    const auto &res = damping::last().results[a];
                    n[a] = res.found ? scaled(res.zeta, 1000.f) : no_value;
                }
            } else {
                std::array<uint8_t, 16> found;
                n[1] = int16_t(speed_map::critical(speed_map::last(), 0, found));
                n[2] = int16_t(speed_map::critical(speed_map::last(), 1, found));
                n[0] = int16_t(n[1] + n[2]);
            }
            break;
        case Test::mpc:
            if (after || mpc::has_valid_calibration()) {
                const mpc::Params &p = mpc::params();
                n[0] = scaled(p.heater_power, 10.f);
                n[1] = scaled(p.block_heat_capacity, 10.f);
                n[2] = scaled(p.ambient_xfer_coeff, 1000.f);
                n[3] = scaled(p.fan_xfer_coeff, 1000.f);
            }
            break;
        case Test::mmu_ptfe:
            if (after) {
                n[0] = int16_t(ptfe::status().saved_mm);
                n[1] = int16_t(ptfe::status().measured_mm);
            } else if (const uint16_t length = marlin_vars().mmu2_bowden_length.get(); length > 0) {
                n[0] = int16_t(length);
            }
            break;
        case Test::nozzle_expansion: {
            namespace comp = buddy::nozzle_thermal_compensation;
            const int16_t prusa_nm = int16_t(comp::slope_to_nm(comp::prusa_slope_mm_per_c));
            if (!after) {
                if (const uint16_t nm = comp::measured_nm_per_c(comp::current_nozzle_key()); nm > 0) {
                    n[0] = int16_t(std::min<uint16_t>(nm, INT16_MAX)); // sinon : modèle de Prusa
                }
                break;
            }
            const nozzle_expansion::Outcome &o = nozzle_expansion::last().outcome;
            n[0] = scaled(o.slope_mm_per_c, 1e6f);
            n[1] = prusa_nm;
            // Écart avec Prusa à 250 °C, la buse palpée à la référence (170 °C) [0,1 µm]
            n[2] = scaled((o.slope_mm_per_c - comp::prusa_slope_mm_per_c) * (250.f - comp::reference_temperature_c), 1e4f);
            n[3] = scaled(o.hysteresis_mm, 1e4f);
            break;
        }
        case Test::mmu_speed:
            if (after) {
                const mmu_speed::Measurement &m = mmu_speed::last();
                const mmu_speed::Saved saved = mmu_speed::saved();
                n[0] = m.speed;
                n[1] = int16_t(saved.prusa_ds);
                n[2] = int16_t(saved.cycle_ds);
                const auto passed = std::find(m.passed.begin(), m.passed.begin() + mmu_speed::levels.size(), false) - m.passed.begin();
                n[3] = passed ? mmu_speed::levels[passed - 1] : no_value;
            } else {
                n[0] = mmu_speed::speed_of(mmu_speed::saved());
            }
            break;
        case Test::pa:
            if (const pa::Record r = pa::record(s.slot); r.has_k()) {
                n[s.slot % n.size()] = int16_t(r.k_e4);
            }
            break;
        case Test::max_flow:
        case Test::min_temp:
            if (const calib::Record r = calib::record(s.slot); s.test == Test::max_flow ? r.has_flow() : r.has_min_temperature()) {
                n[s.slot % n.size()] = s.test == Test::max_flow ? int16_t(r.max_flow_e1) : r.min_temperature;
            }
            break;
        case Test::e_jump: // F de l'emplacement avec son K : la mesure d'avant (ou 120 de Prusa), puis la nouvelle
            n[s.slot % n.size()] = scaled(e_jump::filament_accel_for(s.slot), 1.f);
            break;
        case Test::skew:
        case Test::count:
            break;
        }
    }

    void add_result(Test test, Result r) {
        auto &slot = results[size_t(test)];
        slot = fold(slot.load(), r);
    }

    void finish() {
        const uint32_t date = now();
        for (const Entry &e : entries) {
            if (run_mask & test_bit(e.test)) {
                const size_t i = size_t(e.test);
                store_record(i, after_run(stored_record(i), e, results[i].load(), date));
                if (results[i].load() == Result::ok && !(e.needs & filament)) {
                    Values kept;
                    std::copy_n(after_numbers[i].begin(), kept.v.size(), kept.v.begin());
                    store_values(i, kept); // le « avant » de la fois suivante
                }
            }
        }
        session::series_end();
        // Chauffe gardée d'un test au suivant pendant la série (measure_session_od3d.hpp) : coupée à la fin
        if (!marlin_server::is_printing()) {
            Temperature::disable_hotend();
            thermalManager.setTargetBed(0);
        }
        bed_full = false;
        running = false;
        ++runs;
    }

    /// Met en file l'étape i et son repère ; false si elle ne peut pas partir
    bool enqueue(uint8_t i) {
        const Step &s = steps[i];
        live::clear(); // rien de l'étape d'avant à l'écran
        if (s.test == Test::mmu_ptfe) {
            ptfe::forget(); // le bilan lit le résultat de cette mesure, jamais celui d'une mesure d'avant
        } else if (s.test == Test::mmu_speed) {
            mmu_speed::forget();
        }
        char gcode[24];
        char marker[16];
        if (!step_gcode(s, gcode) || snprintf(marker, sizeof(marker), "M1989 N%u", unsigned(i)) <= 0) {
            return false;
        }
        if (BUFSIZE - GCodeQueue::length < 2) {
            return false; // la file a été remplie d'ailleurs : l'étape échoue plutôt que de perdre le repère
        }
        // PA, F5, F6, M2000 : bobine pas dans la buse : « Charger dans la buse » de Prusa d'abord (origine si besoin), par
        // la file d'injection qui passe avant ; pour un test qui imprime, au-dessus de la ligne de purge réservée sur le
        // plateau (calib_registry::load_gcode). Tube PTFE, vitesse de la poulie : le test de chargement de Prusa veut
        // l'extrudeur et le sélecteur vides : déchargement d'abord ; de même pour la dilatation de la buse (M2003), si le
        // capteur de l'extrudeur voit du filament
        const auto loaded = VirtualToolIndex::currently_selected_opt();
        const bool prints = s.test == Test::pa || s.test == Test::max_flow || s.test == Test::min_temp;
        const bool needs_load = (prints || s.test == Test::e_jump) && filament_memory::loaded_slot() != s.slot; // F43
        const bool needs_unload = ((s.test == Test::mmu_ptfe || s.test == Test::mmu_speed) && loaded) || (s.test == Test::nozzle_expansion && nozzle_expansion::filament_loaded()); // M2003 : buse vide
        if (needs_load || needs_unload) {
            char filament[16];
            StringBuilder sb(filament);
            const FilamentType type = FilamentType::for_tool(VirtualToolIndex::from_raw(s.slot));
            type.build_encoded_gcode_param(sb);
            std::optional<Spot> spot;
            if (needs_load && prints) {
                const auto purge = session::purge_start(); // réservée jusqu'à la purge du test
                if (!purge) {
                    return false; // plateau plein (session::series_full())
                }
                spot = Spot { purge->x, purge->y, session::purge_z };
            }
            const auto loaded_type = loaded ? FilamentType::for_tool(*loaded) : FilamentType::none;
            const int16_t nozzle = load_temperature(loaded_type ? std::optional<int16_t>(loaded_type.parameters().nozzle_temperature) : std::nullopt,
                type.parameters().nozzle_temperature);
            const bool ok = needs_unload ? snprintf(load_sequence, sizeof(load_sequence), "G28 O I\nG27 W2\nM702 W2") > 0
                                         : load_gcode(s.slot, sb.str(), nozzle, spot, load_sequence);
            if (!ok || !marlin_server::inject(GCodeLiteral { .gcode = ConstexprString::from_str_unsafe(load_sequence) })) {
                return false;
            }
            live::set(needs_unload ? N_("Unloading: this test needs an empty nozzle") : N_("Loading slot %d into the nozzle"), s.slot + 1);
        }
        if (!marlin_server::enqueue_gcode_try(gcode) || !marlin_server::enqueue_gcode_try(marker)) {
            return false;
        }
        capture(s, before_numbers[size_t(s.test)], false);
        // Rien ne tourne avant le retour de M1989 : le résultat lu à la fin sera celui de cette étape, pas d'avant
        if (const auto r = prusa_result(s.test)) {
            prusa_before = *r;
            set_prusa_result(s.test, TestResult::unknown);
        }
        return true;
    }

    /// L'étape a besoin du MMU maintenant : elle le pilote, charge par lui, ou doit vider la buse par lui
    bool needs_mmu(const Step &s) {
        return uses_mmu(entry(s.test), run_ctx) || (run_ctx.mmu && s.test == Test::nozzle_expansion && nozzle_expansion::filament_loaded());
    }

    /// Le MMU répond (pas en M709 S2 pour son flash par l'USB, pas en train de se connecter ou en erreur de liaison)
    bool mmu_available() {
        return MMU2::mmu2.Enabled() && !mmu_usb::active();
    }

    /// Lance l'étape i, ou la suivante qui peut partir ; sinon fin de la série
    void start(uint8_t i) {
        while (i < step_count) {
            current = i;
            if (cancel_requested || marlin_server::is_printing()) {
                for (; i < step_count; ++i) {
                    add_result(steps[i].test, Result::skipped);
                }
                break;
            }
            // MMU indisponible (flash par l'USB avant...) : l'étape passe à la fin, les autres tournent ; toujours
            // indisponible à son tour : reportée (passée, reste à faire), jamais un échec
            if (needs_mmu(steps[i]) && !mmu_available()) {
                if (!steps[i].deferred) {
                    defer(std::span<Step>(steps.data(), size_t(step_count)), i);
                    continue; // l'étape i est maintenant la suivante
                }
                add_result(steps[i].test, Result::skipped);
                postponed_mask = postponed_mask | test_bit(steps[i].test);
                ++i;
                continue;
            }
            // Buse froide exigée (accéléromètre sur la buse) : refusée sans rien bouger
            if ((entry(steps[i].test).needs & cold_nozzle) && !cold()) {
                add_result(steps[i].test, Result::failed);
                ++i;
                continue;
            }
            if (enqueue(i)) {
                return;
            }
            if (session::series_full() && !cancel_requested) {
                bed_full = true; // pas de place pour la purge de son chargement : M1989 C la relance, plateau nettoyé
                return;
            }
            add_result(steps[i].test, Result::failed);
            ++i;
        }
        finish();
    }

} // namespace

// ---- Mémoire et réglages ----

Context context() {
    Context ctx;
    ctx.accelerometer = accelerometer();
    ctx.mmu = config_store().mmu2_enabled.get();
    ctx.flow_limit = calib::flow_limit_enabled();
    std::array<int16_t, VirtualToolIndex::count> bed {}, nozzle {};
    for (uint8_t i = 0; i < VirtualToolIndex::count; ++i) {
        const VirtualToolIndex tool = VirtualToolIndex::from_raw(i);
        const FilamentType type = FilamentType::for_tool(tool);
        if (tool.is_enabled() && type) {
            ctx.spools |= uint8_t(1u << i);
        }
        const FilamentTypeParameters p = type.parameters();
        bed[i] = p.heatbed_temperature;
        nozzle[i] = p.nozzle_temperature;
        // Déjà mesuré pour la bobine en place (effacé au changement de bobine) : pas refait sans le demander
        const uint8_t bit = uint8_t(1u << i);
        ctx.measured[size_t(Test::pa)] |= pa::record(i).has_k() ? bit : 0;
        ctx.measured[size_t(Test::max_flow)] |= calib::record(i).has_flow() ? bit : 0;
        ctx.measured[size_t(Test::min_temp)] |= calib::record(i).has_min_temperature() ? bit : 0;
        ctx.measured[size_t(Test::e_jump)] |= e_jump::record(i).has_jump() ? bit : 0;
    }
    ctx.order = spool_order(bed, nozzle, filament_memory::loaded_slot());
    return ctx;
}

Record stored_record(size_t i) {
    if (i < base_tests) {
        return config_store().od3d_calib_done.get(i);
    }
    return i < max_tests ? config_store().od3d_calib_done_ext.get(i - base_tests) : Record {};
}

void store_record(size_t i, const Record &r) {
    if (i < base_tests) {
        config_store().od3d_calib_done.set(i, r);
    } else if (i < max_tests) {
        config_store().od3d_calib_done_ext.set(i - base_tests, r);
    }
}

Values stored_values(size_t i) {
    if (i < base_tests) {
        return config_store().od3d_calib_values.get(i);
    }
    return i < max_tests ? config_store().od3d_calib_values_ext.get(i - base_tests) : Values {};
}

void store_values(size_t i, const Values &v) {
    if (i < base_tests) {
        config_store().od3d_calib_values.set(i, v);
    } else if (i < max_tests) {
        config_store().od3d_calib_values_ext.set(i - base_tests, v);
    }
}

Record record(Test test) {
    return stored_record(size_t(test));
}

std::array<Record, max_tests> records() {
    std::array<Record, max_tests> out;
    for (size_t i = 0; i < max_tests; ++i) {
        out[i] = stored_record(i);
    }
    return out;
}

Accelerometer accelerometer() {
    return config_store().od3d_accelerometer.get();
}

void set_accelerometer(Accelerometer value) {
    config_store().od3d_accelerometer.set(value);
}

void record_done(Test test) {
    const size_t i = size_t(test);
    store_record(i, after_run(stored_record(i), entry(test), Result::ok, now()));
}

// ---- Série en cours ----

Progress progress() {
    const uint8_t i = current;
    return Progress { .running = running, .index = i, .count = step_count, .step = steps[i % max_steps], .runs = runs, .bed_full = bed_full };
}

Result result(Test test) {
    return results[size_t(test)].load();
}

uint32_t postponed() {
    return postponed_mask.load();
}

void recapture(Test test) {
    const size_t i = size_t(test) % max_tests;
    if (running || results[i].load() != Result::ok || (entry(test).needs & filament)) {
        return;
    }
    capture(Step { test }, after_numbers[i], true);
    Values kept;
    std::copy_n(after_numbers[i].begin(), kept.v.size(), kept.v.begin());
    store_values(i, kept); // le « avant » de la fois suivante
}

bool mmu_in_use() {
    return running && needs_mmu(steps[current % max_steps]);
}

namespace {
    /// Arrête la mesure de l'étape en cours si elle tourne
    void stop_current() {
        // Seulement la mesure qui tourne : un arrêt demandé avant son début vaudrait aussi pour sa prochaine mesure à la main
        switch (steps[current % max_steps].test) {
        case Test::belts:
            if (belts::is_running()) {
                belts::request_stop();
            }
            break;
        case Test::speed_map:
            if (speed_map::is_running()) {
                speed_map::request_stop();
            }
            break;
        case Test::damping:
            if (damping::is_running()) {
                damping::request_stop();
            }
            break;
        case Test::nozzle_expansion:
            if (nozzle_expansion::is_running()) {
                nozzle_expansion::request_stop();
            }
            break;
        case Test::mpc:
            if (mpc::tune_status().running) {
                mpc::request_cancel();
            }
            break;
        case Test::pa:
            if (pa::is_running()) {
                pa::request_stop();
            }
            break;
        case Test::max_flow:
        case Test::min_temp:
            if (calib::is_running()) {
                calib::request_stop();
            }
            break;
        case Test::fan_response:
            if (fan_response::is_running()) {
                fan_response::request_stop();
            } else if (fan_vibration::is_running()) {
                fan_vibration::request_stop();
            }
            break;
        case Test::accel_limit:
            if (accel_limit::is_running()) {
                accel_limit::request_stop();
            }
            break;
        case Test::accel_mount:
            if (accel_mount::is_running()) {
                accel_mount::request_stop();
            }
            break;
        case Test::shaper_check:
            if (shaper_check::is_running()) {
                shaper_check::request_stop();
            }
            break;
        case Test::e_jump:
            if (e_jump::is_running()) {
                e_jump::request_stop();
            }
            break;
        case Test::mmu_speed:
            mmu_speed::request_stop(); // entre deux cycles
            break;
        default:
            break; // assistants de Prusa (input shaper, phase stepping) : leur écran a son « Annuler »
        }
    }
} // namespace

void request_cancel() {
    if (!running) {
        return;
    }
    cancel_requested = true;
    stop_current();
}

bool can_skip() {
    return running && steps[current % max_steps].test != Test::mmu_ptfe; // le tube PTFE ne s'arrête pas en route
}

void request_skip(uint8_t index) {
    if (!can_skip() || index != current) {
        return; // l'étape a déjà changé : la suivante n'est pas passée
    }
    skip_index = index;
    stop_current(); // rien s'il n'a pas encore commencé (chargement) : l'interface redemande à chaque tour
}

const Numbers &numbers_before(Test test) {
    return before_numbers[size_t(test) % max_tests];
}

const Numbers &numbers_after(Test test) {
    return after_numbers[size_t(test) % max_tests];
}

void set_number(Test test, bool after, uint8_t index, int16_t value) {
    if (size_t(test) < max_tests && index < Numbers().size()) {
        (after ? after_numbers : before_numbers)[size_t(test)][index] = value;
    }
}

Check check_result() {
    return last_check;
}

uint16_t check_count() {
    return checks;
}

// ---- M1989 ----

void prepare() {
    if (marlin_server::is_printing()) {
        return;
    }
    // Comme le début de M1959 : chauffe coupée et ventilateur à fond dès maintenant, origine, buse au centre
    Temperature::disable_hotend();
    if (Temperature::degHotend(0) >= cold_nozzle_max) {
        Temperature::set_print_fan_speed(255);
    }
    GcodeSuite::G28_no_parser(true, true, true, { .only_if_needed = true, .precise = false });
    if (!axes_need_homing() && !planner.draining()) {
        do_blocking_move_to(X_BED_SIZE / 2, Y_BED_SIZE / 2, Z_SIZE / 2);
    }
}

void check() {
    Check c = Check::ok;
    if (marlin_server::is_printing()) {
        c = Check::printing;
    } else if (!cold()) {
        c = Check::hot;
    } else if (axes_need_homing()) {
        c = Check::not_homed; // l'origine se refait sans l'accéléromètre (Prusa ne palpe jamais avec lui)
    } else if (!accel::ready()) {
        c = Check::no_accelerometer;
    }
    last_check = c;
    ++checks;
}

void run(uint32_t mask, uint32_t redo) {
    if (running) {
        return;
    }
    const Context ctx = context();
    run_ctx = ctx;
    postponed_mask = 0;
    step_count = uint8_t(build_steps(mask & automatic_mask(), ctx, steps, redo));
    run_mask = 0;
    for (uint8_t i = 0; i < step_count; ++i) {
        run_mask |= test_bit(steps[i].test);
    }
    for (const Entry &e : entries) {
        if (run_mask & test_bit(e.test)) {
            results[size_t(e.test)] = Result::none;
            before_numbers[size_t(e.test)] = no_numbers;
            after_numbers[size_t(e.test)] = no_numbers;
        }
    }
    cancel_requested = false;
    skip_index = 0xff;
    bed_full = false;
    session::series_begin(); // plateau vide et propre confirmé avant (Need::empty_bed)
    running = true;
    start(0);
}

void step_end(uint8_t index) {
    if (!running || index != current) {
        return; // repère d'une autre série
    }
    const Step &s = steps[index];
    const bool ok = succeeded(s);
    // Pas de place sur le plateau pour son motif : l'utilisateur nettoie, M1989 C relance cette étape
    if (!ok && session::series_full() && !cancel_requested && skip_index != index) {
        bed_full = true;
        return;
    }
    if (ok) {
        capture(s, after_numbers[size_t(s.test)], true);
    }
    add_result(s.test, ok ? Result::ok : (cancel_requested || skip_index == index ? Result::skipped : Result::failed));
    start(uint8_t(index + 1));
}

void clean_done() {
    if (!running || !bed_full) {
        return;
    }
    bed_full = false;
    session::series_begin(); // plateau nettoyé : place partout, palpé de nouveau (la plaque a été retirée)
    start(current);
}

void level_bed(uint8_t slot) {
    const FilamentType type = FilamentType::for_tool(VirtualToolIndex::from_raw(slot));
    if (!running || !session::in_series() || session::mesh_ready() || marlin_server::is_printing() || !type) {
        return;
    }
    // Comme le G-code de départ de la CORE One : plateau à la température de la matière, buse à sa température de
    // palpage, puis origine, nettoyage et palpage (le plateau entier : rien n'y est encore imprimé)
    const FilamentTypeParameters m = type.parameters();
    const int16_t probe = std::min<int16_t>(m.nozzle_preheat_temperature, m.nozzle_temperature);
    const int16_t bed = std::clamp<int16_t>(m.heatbed_temperature, 0, session::max_bed_temperature);
    session::Session s(cancel_requested);
    thermalManager.setTargetBed(bed); // le plateau chauffe pendant l'origine (comme M140 avant G28 au départ de la CORE One)
    if (s.home(probe) == session::Result::ok && s.heat_bed(bed, probe) == session::Result::ok) {
        std::ignore = s.level(session::printable_zone(), probe); // échec : le test palpera lui-même
    }
}

} // namespace od3d::calib_assistant
