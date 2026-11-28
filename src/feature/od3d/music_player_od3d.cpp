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
#include <cstdlib>
#include <heap.h>
#include <mapi/parking.hpp>
#include <new>
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
    std::atomic<uint8_t> live_tempo = 100;
    std::atomic<int8_t> live_transpose = 0;

    /// Durée réelle d'un passage du fichier au tempo réglé
    float tempo_scale() {
        return 100.f / float(std::clamp(live_tempo.load(), min_tempo, max_tempo));
    }

    // Limites (PrintForge, musique.rs : 300 mm/s au plus, ±100 mm autour du centre)
    constexpr float max_speed_xy = 300.f; ///< mm/s du moteur ; au-delà, la note descend d'une octave
    constexpr float max_speed_z = 20.f; ///< mm/s (machine : 30, mode furtif : 12)
    /// mm/s² (machine : 10 000) : la note doit prendre sa hauteur vite, sinon chaque attaque glisse (26/11)
    constexpr uint32_t accel_xy = 6000;
    constexpr float min_frequency = 55.f; ///< Hz : en dessous, un moteur ne fait que cliquer ; la note monte d'une octave
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
            // V-04 : un long passage sans note (percussions) se lit d'un trait dans Reader::next() ; sans idle(), le
            // chien de garde (4 s) redémarrait l'imprimante
            idle(true);
            if (fseek(file, long(offset), SEEK_SET) != 0) {
                return 0;
            }
            return fread(buf, 1, len, file);
        }

    private:
        FILE *file;
    };

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

    /// Géométrie d'une lecture : voix, centre, courses et pas complets par mm des moteurs qui chantent
    struct Setup {
        Motors motors;
        xyz_pos_t centre; ///< mm
        float steps_xy; ///< pas complets par mm d'un moteur du haut (A ou B) le long de sa courroie
        float steps_z;
        float max_speed_xy; ///< mm/s d'un moteur
        float max_speed_z;
        VoiceMotion motion[voices]; ///< A, B (le long de X+Y et X−Y), Z ; positions autour du centre
    };

    /// Pas complets par mm, lus dans le planificateur : les micro-pas réglés et le type de courroie comptent (CORE One :
    /// 100 micro-pas par mm, 16 par pas, 6,25 ; courroies 1,5GT de la Gen2 : 101,587, soit 6,35)
    float full_steps_per_mm(AxisEnum axis) {
        return 4.f * planner.qsteps_per_mm(axis);
    }

    /// Joue un accord ; false s'il faut s'arrêter
    bool play_chord(Setup &s, const Chord &chord) {
        // Note et attaque de chaque moteur (A, B, Z) selon les voix choisies
        const int8_t shift = live_transpose.load();
        uint8_t key[voices] = { transposed(chord.keys[0], shift), 0, 0 };
        uint8_t attack = chord.attack & 1u;
        switch (s.motors) {
        case Motors::three:
            key[1] = transposed(chord.keys[1], shift);
            key[2] = transposed(chord.keys[2], shift);
            attack = chord.attack;
            break;
        case Motors::two:
            key[1] = transposed(chord.keys[2], shift); // la basse sur B
            attack |= (chord.attack & 4u) ? 2u : 0u;
            break;
        case Motors::melody:
            break;
        }
        if (key[1] == 0) { // B n'a rien à jouer : il double la mélodie (deux moteurs, plus fort ; un déplacement en X)
            key[1] = key[0];
            attack = uint8_t((attack & ~2u) | ((attack & 1u) ? 2u : 0u));
        }
        float speed[voices];
        for (uint8_t v = 0; v < voices; ++v) {
            const bool z = v == 2;
            const float steps = z ? s.steps_z : s.steps_xy;
            const float max = z ? s.max_speed_z : s.max_speed_xy;
            speed[v] = key[v] ? 1000.f * note_speed(key[v], steps, max, min_frequency / steps) : 0.f;
            if (attack & (1u << v)) {
                s.motion[v].dir = int8_t(-s.motion[v].dir); // une note frappée repart dans l'autre sens : elle s'entend
            }
        }

        float remaining = float(chord.end_us - chord.start_us) * 1e-6f * tempo_scale();
        while (remaining > 1e-6f) {
            if (!wait_queue()) {
                return false;
            }
            int32_t d[voices];
            const float dt = chord_step(s.motion, speed, remaining, max_segment_s, d);
            if (!(dt > 0.f)) {
                break;
            }
            remaining -= dt;
            if (d[0] == 0 && d[1] == 0 && d[2] == 0) {
                continue;
            }
            // CoreXY : A = X + Y, B = X − Y ; une seule ligne, sa vitesse donne à chaque moteur la sienne
            const float a = float(s.motion[0].pos) * 1e-3f, b = float(s.motion[1].pos) * 1e-3f;
            const float x = s.centre.x + (a + b) / 2.f, y = s.centre.y + (a - b) / 2.f, z = s.centre.z + float(s.motion[2].pos) * 1e-3f;
            const float dx = x - current_position.x, dy = y - current_position.y, dz = z - current_position.z;
            current_position.x = x;
            current_position.y = y;
            current_position.z = z;
            line_to_current_position(std::sqrt(dx * dx + dy * dy + dz * dz) / dt);
        }
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
        s.steps_xy = full_steps_per_mm(X_AXIS);
        s.steps_z = full_steps_per_mm(Z_AXIS);
        s.max_speed_xy = std::min(max_speed_xy, planner.settings.max_feedrate_mm_s[X_AXIS]);
        s.max_speed_z = std::min(max_speed_z, planner.settings.max_feedrate_mm_s[Z_AXIS]);

        // Centre du plateau (comme PrintForge), courses bornées par les butées logicielles : |A|, |B| ≤ r donne
        // |X|, |Y| ≤ r (un losange dans le carré)
        s.centre.x = std::clamp(X_BED_SIZE / 2.f, soft_endstop.min.x, soft_endstop.max.x);
        s.centre.y = std::clamp(Y_BED_SIZE / 2.f, soft_endstop.min.y, soft_endstop.max.y);
        const float half_x = std::min(s.centre.x - soft_endstop.min.x, soft_endstop.max.x - s.centre.x) - edge_margin;
        const float half_y = std::min(s.centre.y - soft_endstop.min.y, soft_endstop.max.y - s.centre.y) - edge_margin;
        const float r = std::min({ bound_xy, half_x, half_y });
        if (r < 2.f) {
            return false;
        }
        // Sens de départ −1 : la première note, qui frappe, part vers +X (comme PrintForge)
        s.motion[0] = s.motion[1] = { .bound = int32_t(r * 1000.f), .dir = -1 };
        s.motion[2] = { .dir = -1 };
        s.centre.z = std::max(current_position.z, z_floor);
        if (motors == Motors::three) {
            // La basse sur le plateau : il descend (Z monte) d'abord, jamais plus près de la buse que la hauteur sûre
            const float high = std::min(s.centre.z + 2.f * bound_z, soft_endstop.max.z - edge_margin);
            const float half = (high - s.centre.z) / 2.f;
            if (half >= 2.f) {
                s.centre.z += half;
                s.motion[2].bound = int32_t(half * 1000.f);
            } // sinon pas de place en Z : la basse se tait, la mélodie et la 2e voix jouent
        }
        // IC-1 : le parcage de Prusa (INDX : sortie du bac en Y, autour de la brosse ; Z monté avant le trajet)
        return mapi::park({ .x = s.centre.x, .y = s.centre.y, .z = s.centre.z }) && !must_stop();
    }

} // namespace

void play(uint8_t motors_override) {
    // Arrêt effacé à la fin, pas au départ (ECR-6) : un « Arrêter » pendant que M1991 attend en file vaut pour ce
    // morceau. Après last_status : l'interface lit l'état une fois runs changé
    ScopeGuard count_run = [] {
        stop_requested = false;
        ++runs;
    };
    if (marlin_server::is_printing() || !marlin_server::printer_idle()) {
        last_status = Status::printing;
        return;
    }
    uint8_t m = motors_override < motors_count ? motors_override : config_store().od3d_music_motors.get();
    const Motors motors = m < motors_count ? Motors(m) : Motors::three;

    const auto name = config_store().od3d_music_file.get();
    if (name[0] == '\0' || memchr(name.data(), '\0', name.size()) == nullptr) {
        last_status = Status::no_file;
        return;
    }
    char path[24];
    snprintf(path, sizeof(path), "/usb/%s", name.data());
    set_live_tuning(load_tuning(tuning_path, name.data()));
    FILE *file = fopen(path, "rb");
    if (!file) {
        last_status = Status::no_file;
        return;
    }
    ScopeGuard close_file = [&] { fclose(file); };
    setvbuf(file, nullptr, _IONBF, 0);
    FileSource source(file);
    // ~1,2 Ko pris au tas le temps du morceau (ni la pile de Marlin, ni la RAM statique : elle manquait au réseau, 25/11)
    void *const block = malloc_fallible(sizeof(Reader));
    if (!block) {
        last_status = Status::no_memory;
        return;
    }
    Reader &reader = *new (block) Reader();
    ScopeGuard free_reader = [&] {
        reader.~Reader();
        free(block);
    };
    if (reader.open(source) != Error::none) {
        last_status = Status::bad_file;
        return;
    }
    Chord chord;
    if (!reader.next(chord)) {
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

    Smoother smoother;
    uint32_t previous_end = chord.start_us; // pas de silence avant le premier accord
    bool more = true;
    while (more) {
        const uint32_t silence_ms = chord.start_us > previous_end ? uint32_t(float(chord.start_us - previous_end) * tempo_scale() / 1000.f) : 0;
        if (silence_ms >= 1 && !rest(silence_ms)) {
            break;
        }
        if (!play_chord(setup, smoother.next(chord))) {
            break;
        }
        previous_end = chord.end_us;
        more = reader.next(chord);
    }
    last_status = must_stop() ? Status::stopped : Status::done;
}

void request_stop() {
    stop_requested = true;
}

void set_live_tuning(Tuning tuning) {
    live_tempo = tuning.tempo;
    live_transpose = tuning.transpose;
}

Tuning live_tuning() {
    return { live_tempo.load(), live_transpose.load() };
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
