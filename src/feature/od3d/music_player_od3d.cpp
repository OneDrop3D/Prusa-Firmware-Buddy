/// @file
#include "music_player_od3d.hpp"
#include "music_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <tuple>

#include <common/marlin_server.hpp>
#include <config_store/store_instance.hpp>
#include <Marlin.h>
#include <gcode/gcode.h>
#include <module/motion.h>
#include <module/planner.h>
#include <raii/scope_guard.hpp>
#include <timing.h>

namespace od3d::music {

namespace {

    std::atomic<bool> stop_requested = false;
    std::atomic<bool> playing = false;
    std::atomic<Status> last_status = Status::idle;
    std::atomic<uint16_t> runs = 0;

    // Limites (PrintForge, musique.rs : 300 mm/s au plus, ±100 mm autour du centre)
    constexpr float max_speed_xy = 300.f; ///< mm/s du moteur ; au-delà, la note descend d'une octave
    constexpr float max_speed_z = 20.f; ///< mm/s (machine : 30, mode furtif : 12)
    constexpr uint32_t accel_xy = 4000; ///< mm/s² (machine : 10 000)
    constexpr uint32_t accel_z = 1000; ///< mm/s² (machine : 1 000)
    constexpr float bound_xy = 100.f; ///< mm de part et d'autre du centre
    constexpr float bound_z = 15.f; ///< mm de part et d'autre du centre de la course en Z
    constexpr float edge_margin = 10.f; ///< mm gardés aux bords de la course
    constexpr float min_z = 10.f; ///< buse au moins à 10 mm du plateau (et 5 mm au-dessus de la pièce imprimée)
    constexpr float above_print = 5.f;
    constexpr float travel_speed_xy = 100.f; ///< mm/s pour aller au centre
    constexpr float travel_speed_z = 10.f;

    /// Mouvements en file au plus, et durée d'un morceau de note : un arrêt prend effet en moins d'une seconde
    constexpr uint8_t queue_depth = 4;
    constexpr float max_segment_s = 0.2f;

    /// Le fichier de la clé, lu par morceaux (pas de tampon de la libc : chaque lecture repartirait d'un fseek)
    class FileSource final : public Source {
    public:
        explicit FileSource(FILE *f)
            : file(f) {}

        size_t read(uint32_t offset, uint8_t *buf, size_t len) override {
            if (fseek(file, long(offset), SEEK_SET) != 0) {
                return 0;
            }
            return fread(buf, 1, len, file);
        }

    private:
        FILE *file;
    };

    Reader reader; ///< ~1 Ko en mémoire statique plutôt que sur la pile de Marlin

    /// Arrêt demandé par l'interface, arrêt de Marlin (exception de G-code), ou impression lancée pendant la musique
    bool must_stop() {
        return stop_requested.load() || planner.draining() || !marlin_server::printer_idle();
    }

    /// Attend une place dans la file des mouvements ; false s'il faut s'arrêter
    bool wait_queue() {
        while (planner.movesplanned() >= queue_depth) {
            if (must_stop()) {
                return false;
            }
            idle(true);
        }
        return !must_stop();
    }

    /// Silence : les mouvements finissent, puis on attend ; false s'il faut s'arrêter
    bool rest(uint32_t ms) {
        while (planner.processing()) {
            if (must_stop()) {
                return false;
            }
            idle(true);
        }
        const uint32_t start = ticks_ms();
        while (ticks_ms() - start < ms) {
            if (must_stop()) {
                return false;
            }
            idle(true);
        }
        return true;
    }

    /// Géométrie d'une lecture : axe(s), centre et course, pas complets par mm du moteur qui chante
    struct Setup {
        Motors motors;
        float bound; ///< mm de part et d'autre du centre, le long de chaque axe qui bouge
        float full_steps_per_mm;
        float max_speed; ///< vitesse du moteur
    };

    /// Pas complets par mm : les micro-pas réglés comptent (la CORE One : 100 micro-pas par mm, 16 par pas, 6,25)
    float full_steps_per_mm(AxisEnum axis) {
        return 4.f * planner.qsteps_per_mm(axis);
    }

    /// Joue une note ; false s'il faut s'arrêter
    bool play_note(const Setup &s, Swing &swing, const Note &note) {
        const float v = note_speed(note.key, s.full_steps_per_mm, s.max_speed);
        if (v <= 0.f) {
            return true;
        }
        const float duration_s = float(note.end_us - note.start_us) * 1e-6f;
        // X seul (diagonale X = Y) : le moteur A parcourt dx + dy, chaque axe la moitié
        const float axis_share = s.motors == Motors::x ? 0.5f : 1.f;
        const float axis_speed = v * axis_share;
        const float feedrate = s.motors == Motors::x ? axis_speed * 1.41421356f : axis_speed;
        int32_t remaining = int32_t(std::lround(axis_speed * duration_s * 1000.f));
        const int32_t max_um = std::max<int32_t>(1, int32_t(axis_speed * max_segment_s * 1000.f));
        while (true) {
            if (!wait_queue()) {
                return false;
            }
            const int32_t d = swing.step(remaining, max_um);
            if (d == 0) {
                break;
            }
            const float mm = float(d) * 1e-3f;
            switch (s.motors) {
            case Motors::xy:
                current_position.x += mm;
                break;
            case Motors::x:
                current_position.x += mm;
                current_position.y += mm;
                break;
            case Motors::z:
                current_position.z += mm;
                break;
            }
            line_to_current_position(feedrate);
        }
        swing.next_note();
        return true;
    }

    /// Prise d'origine, hauteur sûre, tête au centre de la course ; false si impossible (ou arrêt demandé)
    bool prepare(Motors motors, Setup &s) {
        // Origine des axes qui n'en ont pas (comme le G28 de PrintForge) ; après une impression, rien à faire
        if (axes_need_homing()) {
            GcodeSuite::G28_no_parser(true, true, true, { .only_if_needed = true, .precise = false });
        }
        if (must_stop() || axes_need_homing()) {
            return false;
        }

        // Hauteur sûre : au moins 10 mm, et 5 mm au-dessus de ce qui a été imprimé (fin d'impression)
        const float z_floor = std::min(std::max(min_z, planner.max_printed_z + above_print), soft_endstop.max.z - edge_margin);
        if (current_position.z < z_floor) {
            do_blocking_move_to_z(z_floor, travel_speed_z);
        }

        s.motors = motors;
        switch (motors) {
        case Motors::xy:
        case Motors::x: {
            // Centre du plateau (comme PrintForge), course bornée par les butées logicielles de la machine
            const float cx = std::clamp(X_BED_SIZE / 2.f, soft_endstop.min.x, soft_endstop.max.x);
            const float cy = std::clamp(Y_BED_SIZE / 2.f, soft_endstop.min.y, soft_endstop.max.y);
            const float half_x = std::min(cx - soft_endstop.min.x, soft_endstop.max.x - cx) - edge_margin;
            const float half_y = std::min(cy - soft_endstop.min.y, soft_endstop.max.y - cy) - edge_margin;
            s.bound = std::min(bound_xy, motors == Motors::x ? std::min(half_x, half_y) : half_x);
            s.full_steps_per_mm = full_steps_per_mm(X_AXIS);
            s.max_speed = std::min(max_speed_xy, planner.settings.max_feedrate_mm_s[X_AXIS]);
            if (s.bound < 2.f) {
                return false;
            }
            do_blocking_move_to_xy(cx, cy, travel_speed_xy);
            break;
        }
        case Motors::z: {
            // Le plateau descend (Z monte) d'abord : jamais plus près de la buse que la hauteur sûre
            const float low = std::max(current_position.z, z_floor);
            const float high = std::min(low + 2.f * bound_z, soft_endstop.max.z - edge_margin);
            s.bound = (high - low) / 2.f;
            s.full_steps_per_mm = full_steps_per_mm(Z_AXIS);
            s.max_speed = std::min(max_speed_z, planner.settings.max_feedrate_mm_s[Z_AXIS]);
            if (s.bound < 2.f) {
                return false;
            }
            do_blocking_move_to_z(low + s.bound, travel_speed_z);
            break;
        }
        }
        planner.synchronize();
        return s.bound >= 2.f && !must_stop();
    }

} // namespace

void play(uint8_t motors_override) {
    stop_requested = false;
    ScopeGuard count_run = [] { ++runs; }; // après last_status : l'interface lit l'état une fois runs changé
    if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
        last_status = Status::printing;
        return;
    }
    uint8_t m = motors_override < motors_count ? motors_override : config_store().od3d_music_motors.get();
    const Motors motors = m < motors_count ? Motors(m) : Motors::xy;

    const auto name = config_store().od3d_music_file.get();
    if (name[0] == '\0' || memchr(name.data(), '\0', name.size()) == nullptr) {
        last_status = Status::no_file;
        return;
    }
    char path[24];
    snprintf(path, sizeof(path), "/usb/%s", name.data());
    FILE *file = fopen(path, "rb");
    if (!file) {
        last_status = Status::no_file;
        return;
    }
    ScopeGuard close_file = [&] { fclose(file); };
    setvbuf(file, nullptr, _IONBF, 0);
    FileSource source(file);
    if (reader.open(source) != Error::none) {
        last_status = Status::bad_file;
        return;
    }
    Note note;
    if (!reader.next(note)) {
        last_status = Status::no_notes;
        return;
    }

    playing = true;
    last_status = Status::playing;
    ScopeGuard not_playing = [] { playing = false; };

    // Réglages de mouvement : mode furtif coupé (ses limites fausseraient les notes), vitesses et accélérations
    // bornées ; tout est remis à la fin, même sur un arrêt de Marlin
    planner.synchronize();
    const auto saved_settings = planner.user_settings;
    const bool stealth = config_store().stealth_mode.get();
    ScopeGuard restore = [&] {
        if (!planner.draining()) {
            planner.synchronize();
        }
        planner.apply_settings(saved_settings);
        planner.set_stealth_mode(stealth); // réapplique saved_settings avec les limites du mode furtif
    };
    planner.set_stealth_mode(false);
    {
        auto s = planner.user_settings;
        for (const AxisEnum axis : { X_AXIS, Y_AXIS }) {
            s.max_feedrate_mm_s[axis] = max_speed_xy;
            s.max_acceleration_mm_per_s2[axis] = accel_xy;
        }
        s.max_feedrate_mm_s[Z_AXIS] = max_speed_z;
        s.max_acceleration_mm_per_s2[Z_AXIS] = std::min(s.max_acceleration_mm_per_s2[Z_AXIS], accel_z);
        s.travel_acceleration = float(accel_xy);
        s.acceleration = float(accel_xy);
        planner.apply_settings(s); // les limites matérielles de Prusa s'appliquent encore
    }

    Setup setup {};
    if (!prepare(motors, setup)) {
        last_status = must_stop() ? Status::stopped : Status::no_room;
        return;
    }

    Swing swing(int32_t(setup.bound * 1000.f));
    uint32_t previous_end = note.start_us; // pas de silence avant la première note
    bool more = true;
    while (more) {
        const uint32_t silence_ms = note.start_us > previous_end ? (note.start_us - previous_end) / 1000 : 0;
        if (silence_ms >= 1 && !rest(silence_ms)) {
            break;
        }
        if (!play_note(setup, swing, note)) {
            break;
        }
        previous_end = note.end_us;
        more = reader.next(note);
    }
    last_status = must_stop() ? Status::stopped : Status::done;
}

void request_stop() {
    stop_requested = true;
}

bool is_playing() {
    return playing.load();
}

Status status() {
    return last_status.load();
}

uint16_t run_count() {
    return runs.load();
}

void on_print_finished() {
    if (!config_store().od3d_music_at_print_end.get() || config_store().od3d_music_file.get()[0] == '\0') {
        return;
    }
    std::ignore = marlin_server::enqueue_gcode_try("M1991");
}

} // namespace od3d::music
