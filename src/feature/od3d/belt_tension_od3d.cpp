/// @file
#include "belt_tension_od3d.hpp"
#include "accelerometer_od3d.hpp"
#include "force_probe_od3d.hpp"
#include "live_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <new>

#include <common/marlin_server.hpp>
#include <heap.h>
#include <i18n.h>
#include <option/has_indx.h> // utilisé par manual_belt_tuning_config.hpp
#include <feature/manual_belt_tuning/manual_belt_tuning_config.hpp>
#include <feature/motordriver_util.h>
#include <feature/phase_stepping/phase_stepping.hpp>
#include <Marlin/src/gcode/calibrate/M958.hpp>
#include <Marlin/src/gcode/gcode.h>
#include <Marlin/src/Marlin.h>
#include <Marlin/src/module/motion.h>
#include <Marlin/src/module/planner.h>
#include <Marlin/src/module/temperature.h>
#include <raii/scope_guard.hpp>

namespace od3d::belts {

namespace {

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> running = false;
    std::atomic<Status> last_status = Status::idle;
    std::atomic<Stage> current_stage = Stage::preparing;
    std::atomic<uint8_t> progress = 0;
    std::atomic<uint16_t> runs = 0;
    std::atomic<uint16_t> revisions = 0;
    std::atomic<bool> live_on = false; ///< M1992 L
    std::atomic<uint8_t> live_ok = 0; ///< réglage en direct : tours de suite dans la plage
    std::atomic<bool> compare_on = false; ///< M1992 C
    std::atomic<uint8_t> modes_done = 0; ///< comparaison : modes menés au bout

    Measurement measurement; ///< ~770 octets en mémoire statique, lue par l'interface
    Comparison compared_peaks; ///< 96 octets, lus par l'interface

    /// Spectre brut (diagnostic du 23/11 : 78 Hz mesurés, 98 Hz à l'appli de Prusa), ajouté à chaque mesure
    constexpr const char *csv_path = "/usb/OD3D_BELTS.CSV";

    /// Gain par axe (X, Y, Z) des points de la courroie en cours, NAN si pas mesuré. Écrit sur la clé à la fin de son
    /// balayage, jamais entre deux points : une pause des moteurs fausserait le point suivant (M958.cpp, measure()).
    /// ~1,1 Ko pris au tas par run() le temps de la mesure (statique avant : RAM rendue au réseau, 25/11)
    using AxisGains = std::array<std::array<float, 3>, sweep_points>;
    AxisGains *axis_gains = nullptr;

    void write_csv_header(bool live) {
        if (FILE *f = fopen(csv_path, "a")) {
            force_probe::csv_printf(f, "# OneDrop3D belts (F14) M1992 I%u%s%s%s at X%.1f Y%.1f: %s%s\n", unsigned(measurement.harmonic),
                live ? " L" : "", measurement.prusa_excitation ? " P" : "", measurement.ringing ? " R" : "",
                double(current_position.x), double(current_position.y),
                measurement.prusa_excitation ? "both motors, front-back (as Prusa M961)" : "one motor per belt (diagonal)",
                measurement.ringing ? ", measured after the motors stop" : ", measured while exciting");
            force_probe::csv_printf(f, "belt,freq_hz,gain_x,gain_y,gain_z,gain2\n");
            fclose(f);
        }
    }

    /// complete : balayage complet fini, ses pics sont tous écrits (V-03)
    void write_csv_rows(Belt belt, bool complete) {
        FILE *f = fopen(csv_path, "a");
        if (!f) {
            return; // pas de clé
        }
        const auto &spectrum = measurement.spectra[uint8_t(belt)];
        for (size_t i = 0; i < sweep_points; ++i) {
            const auto &g = (*axis_gains)[i];
            if (!std::isnan(g[0])) {
                force_probe::csv_printf(f, "%c,%.1f,%.5f,%.5f,%.5f,%.6f\n", belt == Belt::a ? 'A' : 'B', double(sweep_start_hz + float(i) * sweep_step_hz),
                    double(g[0]), double(g[1]), double(g[2]), double(spectrum[i]));
            }
        }
        const Peak &p = measurement.peaks[uint8_t(belt)];
        force_probe::csv_printf(f, "# peak %c: %.2f Hz, found %d, prominence %.2f\n", belt == Belt::a ? 'A' : 'B', double(p.frequency), int(p.found), double(p.relative_prominence));
        if (complete) {
            std::array<Peak, compare_peaks> all;
            const size_t n = find_peaks(spectrum, sweep_start_hz, sweep_step_hz, all);
            force_probe::csv_printf(f, "# all peaks %c (Hz, prominence):", belt == Belt::a ? 'A' : 'B');
            for (size_t i = 0; i < n; ++i) {
                force_probe::csv_printf(f, " %.2f (%.2f)", double(all[i].frequency), double(all[i].relative_prominence));
            }
            force_probe::csv_printf(f, "\n");
        }
        fclose(f);
    }

    /// Lettres du mode d'une mesure, comme les paramètres de M1992 : R, P, I2 ou I1
    const char *mode_code(const CompareMode &m) {
        return m.ringing ? "R" : (m.prusa_excitation ? "P" : (m.harmonic == 2 ? "I2" : "I1"));
    }

    /// Fin d'une comparaison : tous les pics, mode par mode, à comparer avec l'appli de Prusa (V-03)
    void write_csv_comparison() {
        FILE *f = fopen(csv_path, "a");
        if (!f) {
            return;
        }
        force_probe::csv_printf(f, "# OneDrop3D belts comparison M1992 C: peaks in Hz, most prominent first, to compare with the Prusa app\nmode,belt,peak1_hz,peak2_hz,peak3_hz\n");
        for (uint8_t k = 0; k < modes_done; ++k) {
            for (uint8_t b = 0; b < belt_count; ++b) {
                force_probe::csv_printf(f, "%s,%c", mode_code(compare_modes[k]), 'A' + b);
                for (const float hz : compared_peaks[k][b]) {
                    if (std::isfinite(hz)) {
                        force_probe::csv_printf(f, ",%.2f", double(hz));
                    } else {
                        force_probe::csv_printf(f, ",");
                    }
                }
                force_probe::csv_printf(f, "\n");
            }
        }
        fclose(f);
    }


    /// Excitation : comme le réglage manuel des courroies de Prusa (M961), 50 périodes par fréquence comme M1959
    constexpr uint32_t excitation_cycles = 50;

    constexpr float min_z = 10.f; ///< buse au moins à 10 mm du plateau
    constexpr float above_print = 5.f; ///< et 5 mm au-dessus de ce qui a été imprimé
    constexpr float travel_speed_xy = 50.f; ///< mm/s, comme M961
    constexpr float travel_speed_z = 10.f;

    bool must_stop() {
        return stop_requested.load() || planner.draining() || !marlin_server::printer_idle();
    }

    /// Hauteur sûre, position de mesure de Prusa (M961) bornée par les butées logicielles ; false sans origine
    bool prepare() {
        // Jamais d'origine ici (retour du 23/11 : le plateau montait palper, accéléromètre fixé) : l'écran ou
        // l'assistant la prend avant de demander de le fixer (M1989 H) ; perdue depuis : refusé, sans bouger
        if (must_stop() || axes_need_homing()) {
            return false;
        }
        const float z_floor = std::min(std::max(min_z, planner.max_printed_z + above_print), soft_endstop.max.z - 1.f);
        if (current_position.z < z_floor) {
            do_blocking_move_to_z(z_floor, travel_speed_z);
        }
        if (!accel::take_head()) { // INDX : la tête dont l'accéléromètre mesure, prise Z levé ; rien ailleurs
            return false;
        }
        const float x = std::clamp<float>(manual_belt_tuning::calib_position_x, soft_endstop.min.x, soft_endstop.max.x);
        const float y = std::clamp<float>(manual_belt_tuning::calib_position_y, soft_endstop.min.y, soft_endstop.max.y);
        return accel::move_to(x, y, travel_speed_xy) && !must_stop(); // IC-1 : INDX, le chemin du parcage de Prusa
    }

    void set_progress(uint8_t belt_index, float point_progress) {
        const float done = float(belt_index) * sweep_points + point_progress;
        const float mode = std::clamp(done * 100.f / float(belt_count * sweep_points), 0.f, 100.f);
        progress = uint8_t(compare_on ? (float(modes_done) * 100.f + mode) / float(compare_modes.size()) : mode); // comparaison : sur tous les modes
    }

    /// Balaye une courroie (count points du spectre depuis from ; par défaut tout) ; Status::done si tout est mesuré.
    /// Fenêtre étroite du réglage en direct : le pic est celui de la fenêtre (live_peak()).
    Status sweep(Belt belt, vibrate_measure::MeasureParams &args, const vibrate_measure::MicrostepRestorer &restorer, size_t from = 0, size_t count = sweep_points) {
        using namespace vibrate_measure;
        const uint8_t index = uint8_t(belt);
        // Un seul moteur : A = pilote X, B = pilote Y (l'enchaînement de setup_steppers() de Prusa mettrait les deux).
        // M1992 P : l'excitation de l'assistant de Prusa (M961), les deux moteurs, la tête d'avant en arrière
        args.axis_flag = measurement.prusa_excitation
            ? StepEventFlag_t(STEP_EVENT_FLAG_STEP_X | STEP_EVENT_FLAG_STEP_Y | STEP_EVENT_FLAG_Y_DIR)
            : StepEventFlag_t(belt == Belt::a ? STEP_EVENT_FLAG_STEP_X : STEP_EVENT_FLAG_STEP_Y);
        if (!args.setup(restorer)) {
            return Status::failed;
        }
        for (auto &g : *axis_gains) {
            g[0] = NAN;
        }
        bool complete = false;
        ScopeGuard csv = [belt, &complete] { write_csv_rows(belt, complete); }; // aussi après un arrêt : les points déjà mesurés

        const auto progress_hook = [&](const ProgressHookParams &params) -> Result<void> {
            if (must_stop()) {
                return std::unexpected(Error::aborted);
            }
            if (params.phase == ProgressHookParams::Phase::calibrating) {
                live::set(N_("Accelerometer calibration"));
                current_stage = Stage::calibrating;
                progress = uint8_t(std::clamp(params.progress * 100.f, 0.f, 100.f));
            }
            idle(true);
            return {};
        };

        const size_t end = std::min(from + count, sweep_points);
        const bool full = count == sweep_points;
        // Balayage complet : un point sur deux d'abord, puis les points sautés autour du pic (fine)
        const auto measure = [&](size_t i) -> Status {
            if (must_stop()) {
                return Status::stopped;
            }
            const float frequency = sweep_start_hz + float(i) * sweep_step_hz;
            // Le bruit des moteurs est l'excitation elle-même (retour du 23/11 : « un drôle de bruit au début ») : à
            // 70 Hz, 7,5 m/s² font ±39 µm, deux fois l'amplitude de 120 Hz (calculé d'après calc_accel de Prusa)
            static_assert(compare_modes.size() == 4, "« /4 » du texte de la comparaison");
            if (compare_on) {
                live::set(N_("Comparison %d/4: belt %c at %d Hz"), modes_done + 1, belt == Belt::a ? 'A' : 'B', int(std::lround(frequency)));
            } else {
                live::set(N_("Belt %c ringing at %d Hz: the motor hum is normal"), belt == Belt::a ? 'A' : 'B', int(std::lround(frequency)));
            }
            // Accélération de Prusa pour cette fréquence (7 à 10,5 m/s² entre 60 et 130 Hz : quelques dizaines de µm),
            // celle de 130 Hz au-delà (M961 ne va pas plus haut)
            args.excitation_acceleration = std::fabs(manual_belt_tuning::calc_accel(std::min(frequency, float(manual_belt_tuning::freq_max))) * 0.001f);
            const auto result = measure_repeat(args, frequency, progress_hook);
            args.calibrate_accelerometer = false;
            if (!result.has_value()) {
                return result.error() == Error::aborted ? Status::stopped : Status::failed;
            }
            current_stage = belt == Belt::a ? Stage::belt_a : Stage::belt_b;
            auto &spectrum = measurement.spectra[index];
            spectrum[i] = result->gain_square();
            (*axis_gains)[i] = { result->gain[0], result->gain[1], result->gain[2] };
            if (full && i % coarse_stride == 0 && i > 0 && measurement.points[index] <= i) {
                spectrum[i - 1] = (spectrum[i - 2] + spectrum[i]) / 2.f; // 1er passage : le point sauté, en attendant
            }
            measurement.points[index] = std::max(measurement.points[index], uint8_t(i + 1));
            if (full && i % coarse_stride == 0) {
                set_progress(index, float(i + 1));
            }
            ++revisions;
            return Status::done;
        };
        if (full) {
            for (size_t i = 0; i < sweep_points; i += coarse_stride) {
                if (const Status s = measure(i); s != Status::done) {
                    return s;
                }
            }
            if (const Peak coarse = belt_peak(measurement.spectra[index], sweep_start_hz, sweep_step_hz); coarse.found) {
                for (size_t i = 1; i < sweep_points; i += coarse_stride) {
                    if (std::fabs(sweep_start_hz + float(i) * sweep_step_hz - coarse.frequency) <= fine_half_hz) {
                        if (const Status s = measure(i); s != Status::done) {
                            return s;
                        }
                    }
                }
            }
        } else {
            for (size_t i = from; i < end; ++i) {
                if (const Status s = measure(i); s != Status::done) {
                    return s;
                }
            }
        }
        if (!full) { // réglage en direct : le sommet de la fenêtre, interpolé ou au bord
            measurement.peaks[index] = Peak { .found = true, .frequency = live_peak(std::span(measurement.spectra[index]).subspan(from, end - from), sweep_start_hz + float(from) * sweep_step_hz, sweep_step_hz) };
            ++revisions;
            return Status::done;
        }
        // Sur toute la plage : au bord ou trop peu marqué, non concluant (jamais de conseil de serrer sur du bruit)
        measurement.peaks[index] = belt_peak(measurement.spectra[index], sweep_start_hz, sweep_step_hz);
        if (const Peak &p = measurement.peaks[index]; p.found) {
            const int tenths = int(std::lround(p.frequency * 10.f));
            live::set_value(N_("Measured: belt %c at %d.%d Hz"), belt == Belt::a ? 'A' : 'B', tenths / 10, tenths % 10);
        }
        complete = true;
        ++revisions;
        return Status::done;
    }

    /// Comparaison : les pics du spectre de la courroie que le mode k vient de balayer
    void keep_peaks(uint8_t k, Belt belt) {
        std::array<Peak, compare_peaks> found;
        const size_t n = find_peaks(measurement.spectra[uint8_t(belt)], sweep_start_hz, sweep_step_hz, found);
        for (size_t i = 0; i < compare_peaks; ++i) {
            compared_peaks[k][uint8_t(belt)][i] = i < n ? found[i].frequency : NAN;
        }
    }

    /// Nouvelle mesure, dans un mode (les modes de diagnostic sont repris par le réglage en direct qui suit)
    void reset(uint8_t harmonic, bool prusa_excitation, bool ringing) {
        static constexpr Measurement fresh {}; // copiée : pas de temporaire sur la pile de Marlin pendant la mesure
        measurement = fresh;
        measurement.harmonic = harmonic == 2 ? 2 : 1;
        measurement.prusa_excitation = prusa_excitation;
        measurement.ringing = ringing;
        ++revisions;
    }

    /// Réglages de M958 du mode de la mesure ; calibrate : étalonner l'accéléromètre au premier point (~15 s)
    vibrate_measure::MeasureParams params(bool calibrate) {
        vibrate_measure::MeasureParams args {
            .excitation_acceleration = std::fabs(manual_belt_tuning::calc_accel(sweep_start_hz) * 0.001f),
            .excitation_cycles = excitation_cycles,
            .klipper_mode = measurement.harmonic == 1, // la 2e harmonique n'existe qu'en mode « Prusa »
            .calibrate_accelerometer = calibrate,
            .axis_flag = StepEventFlag::STEP_EVENT_FLAG_STEP_X,
            .measured_harmonic = measurement.harmonic,
        };
        if (measurement.ringing) {
            // M1992 R : la courroie, peu amortie, sonne encore quand la tête (amortie) s'est tue
            args.wait_cycles = 10;
            args.measurement_cycles = 20;
        }
        return args;
    }

    Status measure_belts() {
        if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
            return Status::printing;
        }
        if (stop_requested) {
            return Status::stopped;
        }
        if (accel::nozzle_hot(max_nozzle_temperature)) {
            return Status::hot_nozzle;
        }
        if (!accel::head_available()) {
            return Status::no_dock;
        }
        if (!accel::ready()) { // ACC-1 : avant tout mouvement
            return Status::no_accelerometer;
        }

        current_stage = Stage::preparing;
        live::set(N_("Moving to the measuring position"));
        if (!prepare()) {
            return must_stop() ? Status::stopped : Status::not_homed;
        }

        // Le phase stepping doit être coupé avant de lire les micro-pas (M958, M961) ; il revient après eux
        phase_stepping::EnsureDisabled phase_stepping_disabler;
        vibrate_measure::MicrostepRestorer microstep_restorer;
        enable_all_steppers(); // même état que pendant une impression
        stepper_microsteps(X_AXIS, 128);
        stepper_microsteps(Y_AXIS, 128);

        if (compare_on) {
            // Comparaison (V-03) : chaque mode balaye les deux courroies ; l'accéléromètre étalonné une fois, au premier
            // point du premier mode (~15 s)
            for (uint8_t k = 0; k < compare_modes.size(); ++k) {
                const CompareMode &mode = compare_modes[k];
                reset(mode.harmonic, mode.prusa_excitation, mode.ringing);
                write_csv_header(false);
                auto args = params(k == 0);
                for (const Belt belt : { Belt::a, Belt::b }) {
                    if (const Status s = sweep(belt, args, microstep_restorer); s != Status::done) {
                        return s;
                    }
                    keep_peaks(k, belt);
                }
                modes_done = uint8_t(k + 1);
            }
            write_csv_comparison();
            return Status::done;
        }

        write_csv_header(live_on);
        auto args = params(!live_on); // une fois, au premier point de la courroie A (~15 s) ; en direct : déjà fait
        if (!live_on) {
            for (const Belt belt : { Belt::a, Belt::b }) {
                if (const Status s = sweep(belt, args, microstep_restorer); s != Status::done) {
                    return s;
                }
            }
            return Status::done;
        }
        // Réglage en direct (M1992 L) : les deux courroies remesurées en boucle autour de leur dernier pic pendant
        // qu'on tourne les vis, jusqu'à « Terminé » ou deux tours de suite dans la plage
        const Targets t = targets();
        live_ok = 0;
        for (;;) {
            for (const Belt belt : { Belt::a, Belt::b }) {
                const Peak &p = measurement.peaks[uint8_t(belt)];
                const float centre = p.frequency > 0.f ? p.frequency : (belt == Belt::a ? t.higher_optimal : t.lower_optimal);
                const float start = live_start(centre, sweep_start_hz, sweep_end_hz, sweep_step_hz);
                const size_t from = size_t(std::lround((start - sweep_start_hz) / sweep_step_hz));
                if (const Status s = sweep(belt, args, microstep_restorer, from, live_points); s != Status::done) {
                    return s == Status::stopped && stop_requested ? Status::done : s; // « Terminé » : les dernières valeurs
                }
            }
            live_ok = live_streak(live_ok, measurement.peaks[0].frequency, measurement.peaks[1].frequency, t);
            if (live_done(live_ok)) {
                return Status::done;
            }
        }
    }

} // namespace

void run(uint8_t harmonic, bool live, bool prusa_excitation, bool ringing, bool compare) {
    ScopeGuard count_run = [] { ++runs; }; // après last_status : l'interface lit l'état une fois runs changé
    if (running.exchange(true)) {
        return; // déjà en cours (ne devrait pas arriver : Marlin traite un G-code à la fois)
    }
    // Un arrêt demandé avant le début (écran fermé, M1992 encore en file) vaut pour cette mesure ; effacé à la fin
    ScopeGuard not_running = [] {
        stop_requested = false;
        live_on = false;
        compare_on = false;
        running = false;
    };

    void *const block = malloc_fallible(sizeof(AxisGains));
    if (!block) {
        last_status = Status::no_memory; // refusée sans bouger
        ++revisions;
        return;
    }
    axis_gains = new (block) AxisGains();
    ScopeGuard free_gains = [block] {
        axis_gains = nullptr;
        free(block);
    };

    live_on = !compare && live && measurement.harmonic == 1; // en direct : on repart des pics de la dernière mesure
    if (!live_on) {
        reset(harmonic, prusa_excitation, ringing);
        modes_done = 0; // la comparaison d'avant ne vaut plus pour cette mesure
    }
    compare_on = compare;
    progress = 0;
    current_stage = Stage::preparing;
    last_status = Status::running;
    ++revisions;

    const Status s = measure_belts();
    last_status = s;
    ++revisions;
}

const Comparison &comparison() {
    return compared_peaks;
}

uint8_t compared() {
    return modes_done.load();
}

bool comparing() {
    return compare_on.load();
}

void request_stop() {
    stop_requested = true;
}

bool live() {
    return live_on.load();
}

uint8_t live_in_range() {
    return live_ok.load();
}

bool is_running() {
    return running.load();
}

Status status() {
    return last_status.load();
}

Stage stage() {
    return current_stage.load();
}

uint8_t progress_percent() {
    return progress.load();
}

uint16_t run_count() {
    return runs.load();
}

uint16_t revision() {
    return revisions.load();
}

const Measurement &last() {
    return measurement;
}

Targets targets() {
    namespace mbt = manual_belt_tuning;
    return {
        .higher_optimal = mbt::higher_freq_belt_optimal,
        .lower_optimal = mbt::lower_freq_belt_optimal,
        .result_min = float(mbt::freq_result_min),
        .result_max = float(mbt::freq_result_max),
        .diff_limit = float(mbt::freq_diff_limit),
        .hz_per_rev = float(mbt::belt_hz_per_rev),
        .hz_per_rev2 = float(mbt::belt_hz_per_rev2),
    };
}

} // namespace od3d::belts
