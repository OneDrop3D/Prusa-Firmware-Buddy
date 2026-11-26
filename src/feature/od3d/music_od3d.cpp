/// @file
#include "music_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <strings.h>

namespace od3d::music {

namespace {

    constexpr uint32_t be32(const uint8_t *p) {
        return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
    }

    constexpr uint16_t be16(const uint8_t *p) {
        return uint16_t((p[0] << 8) | p[1]);
    }

    constexpr uint8_t percussion_channel = 9; // canal 10

    /// Octets de données d'un message de canal selon son statut (0x80 à 0xEF)
    constexpr uint8_t data_bytes(uint8_t status) {
        const uint8_t kind = status & 0xF0;
        return (kind == 0xC0 || kind == 0xD0) ? 1 : 2;
    }

} // namespace

Error Reader::open(Source &src) {
    source = &src;
    tracks_used = 0;
    smpte = false;
    tick_num = 500'000; // 120 noires par minute tant que le fichier ne donne pas de tempo
    tick_den = 1;
    time_scaled = 0;
    last_tick = 0;
    std::memset(sounding, 0, sizeof(sounding));
    std::memset(struck, 0, sizeof(struck));
    std::memset(current, 0, sizeof(current));
    current_attack = 0;
    current_start = 0;
    finished = true;

    uint8_t h[14];
    if (src.read(0, h, sizeof(h)) != sizeof(h) || std::memcmp(h, "MThd", 4) != 0 || be32(h + 4) < 6) {
        return Error::not_midi;
    }
    const uint16_t format = be16(h + 8);
    const uint16_t count = be16(h + 10);
    const uint16_t division = be16(h + 12);
    if (format > 1) {
        return Error::unsupported_format;
    }
    if (division & 0x8000) {
        // Temps SMPTE : images par seconde (négatif) et tics par image ; le tempo n'y change rien
        const int fps = -int(int8_t(division >> 8));
        const uint32_t sub = division & 0xFF;
        if (fps <= 0 || sub == 0) {
            return Error::not_midi;
        }
        smpte = true;
        if (fps == 29) { // 29,97 images par seconde
            tick_num = 100'000'000;
            tick_den = 2997 * sub;
        } else {
            tick_num = 1'000'000;
            tick_den = uint32_t(fps) * sub;
        }
    } else {
        if (division == 0) {
            return Error::not_midi;
        }
        tick_den = division;
    }

    // Table des pistes : les blocs « MTrk », les autres sont sautés
    uint32_t offset = 8 + be32(h + 4);
    for (uint16_t i = 0; i < count && tracks_used < max_tracks;) {
        uint8_t chunk[8];
        if (src.read(offset, chunk, sizeof(chunk)) != sizeof(chunk)) {
            break;
        }
        const uint32_t length = be32(chunk + 4);
        const uint32_t start = offset + 8;
        if (std::memcmp(chunk, "MTrk", 4) == 0) {
            Track &t = tracks[tracks_used++];
            t = {};
            t.pos = start;
            t.end = start + length < start ? UINT32_MAX : start + length;
            t.buf_pos = start;
            ++i;
        }
        if (start + length < start) {
            break; // longueur absurde
        }
        offset = start + length;
    }
    if (tracks_used == 0) {
        return Error::no_tracks;
    }
    finished = false;
    for (uint8_t i = 0; i < tracks_used; ++i) {
        load_delta(tracks[i]);
    }
    return Error::none;
}

int Reader::read_byte(Track &t) {
    if (t.pos >= t.end) {
        return -1;
    }
    if (t.pos < t.buf_pos || t.pos >= t.buf_pos + t.buf_len) {
        const size_t want = std::min<uint32_t>(buffer_size, t.end - t.pos);
        const size_t got = source->read(t.pos, t.buf, want);
        if (got == 0) {
            t.end = t.pos; // fichier tronqué : la piste s'arrête là
            return -1;
        }
        t.buf_pos = t.pos;
        t.buf_len = uint8_t(got);
    }
    return t.buf[t.pos++ - t.buf_pos];
}

bool Reader::read_vlq(Track &t, uint32_t &value) {
    // Quantité de longueur variable : 7 bits par octet, le bit fort annonce l'octet suivant (4 octets au plus)
    value = 0;
    for (int i = 0; i < 4; ++i) {
        const int b = read_byte(t);
        if (b < 0) {
            return false;
        }
        value = (value << 7) | uint32_t(b & 0x7F);
        if (!(b & 0x80)) {
            return true;
        }
    }
    return false;
}

void Reader::skip(Track &t, uint32_t count) {
    t.pos = (t.end - t.pos < count) ? t.end : t.pos + count;
}

void Reader::load_delta(Track &t) {
    uint32_t delta;
    if (t.done || !read_vlq(t, delta)) {
        t.done = true;
        return;
    }
    t.next_tick += delta;
}

void Reader::process_event(Track &t) {
    int b = read_byte(t);
    if (b < 0) {
        t.done = true;
        return;
    }
    uint8_t status;
    int first_data = -1;
    if (b & 0x80) {
        status = uint8_t(b);
    } else if (t.status) {
        status = t.status; // statut courant : b est déjà la première donnée
        first_data = b;
    } else {
        t.done = true; // donnée sans statut : piste illisible
        return;
    }

    if (status == 0xFF) { // méta-événement
        const int type = read_byte(t);
        uint32_t length;
        if (type < 0 || !read_vlq(t, length)) {
            t.done = true;
            return;
        }
        if (type == 0x2F) { // fin de piste
            t.done = true;
            return;
        }
        if (type == 0x51 && length == 3) { // tempo en µs par noire
            uint32_t tempo = 0;
            for (int i = 0; i < 3; ++i) {
                const int v = read_byte(t);
                if (v < 0) {
                    t.done = true;
                    return;
                }
                tempo = (tempo << 8) | uint32_t(v);
            }
            if (!smpte && tempo > 0) {
                tick_num = tempo;
            }
            return;
        }
        skip(t, length);
        return;
    }
    if (status == 0xF0 || status == 0xF7) { // système exclusif
        uint32_t length;
        if (!read_vlq(t, length)) {
            t.done = true;
            return;
        }
        skip(t, length);
        return;
    }
    if (status >= 0xF0) {
        t.done = true; // messages temps réel : n'existent pas dans un fichier
        return;
    }

    t.status = status;
    uint8_t data[2] = { 0, 0 };
    const uint8_t n = data_bytes(status);
    for (uint8_t i = 0; i < n; ++i) {
        const int v = (i == 0 && first_data >= 0) ? first_data : read_byte(t);
        if (v < 0) {
            t.done = true;
            return;
        }
        data[i] = uint8_t(v & 0x7F);
    }
    if ((status & 0x0F) == percussion_channel) {
        return;
    }
    const uint8_t kind = status & 0xF0;
    const uint8_t key = data[0];
    if (kind == 0x90 && data[1] > 0) {
        if (struck[key] < UINT8_MAX) {
            ++struck[key];
        }
    } else if (kind == 0x80 || kind == 0x90) { // relâchement (ou frappe de vélocité 0)
        // Appliqué tout de suite, les frappes du même instant après : une note relâchée et refrappée est rejouée
        if (sounding[key] > 0) {
            --sounding[key];
        }
    }
}

uint32_t Reader::now_us() const {
    const uint64_t t = time_scaled / tick_den;
    return t > UINT32_MAX ? UINT32_MAX : uint32_t(t);
}

void voices_of(const uint8_t *count, uint8_t (&keys)[voices]) {
    int high = 0, second = 0, low = 0, n = 0;
    for (int k = 127; k >= 1; --k) { // shortcut: la note 0 (do-1, 8 Hz, jamais écrite) sert de silence
        if (count[k] > 0) {
            if (n == 0) {
                high = k;
            } else if (n == 1) {
                second = k;
            }
            low = k;
            ++n;
        }
    }
    keys[0] = uint8_t(high);
    keys[1] = uint8_t(n >= 3 ? second : 0);
    keys[2] = uint8_t(n >= 2 ? low : 0);
}

bool Reader::next(Chord &chord) {
    // L'accord en cours, s'il sonne : la mélodie ne se tait que quand plus rien ne sonne
    const auto emit = [&](uint32_t t) {
        if (current[0] == 0 || t <= current_start) {
            return false;
        }
        chord = { current_start, t, { current[0], current[1], current[2] }, current_attack };
        return true;
    };
    while (!finished) {
        // Prochain instant où une piste a un événement
        uint64_t tick = UINT64_MAX;
        for (uint8_t i = 0; i < tracks_used; ++i) {
            if (!tracks[i].done) {
                tick = std::min(tick, tracks[i].next_tick);
            }
        }
        if (tick == UINT64_MAX) {
            finished = true;
            return emit(now_us()); // une note jamais relâchée s'arrête avec le morceau
        }

        // Le temps avance au tempo d'avant cet instant
        time_scaled += (tick - last_tick) * tick_num;
        last_tick = tick;
        if (now_us() == UINT32_MAX) {
            for (uint8_t i = 0; i < tracks_used; ++i) {
                tracks[i].done = true; // plus de 71 minutes : le morceau s'arrête là
            }
        }
        for (uint8_t i = 0; i < tracks_used; ++i) {
            Track &t = tracks[i];
            while (!t.done && t.next_tick == tick) {
                process_event(t);
                load_delta(t);
            }
        }

        // Les voix de cet instant, frappes comprises ; une voix attaque si sa note change ou est refrappée
        for (int k = 0; k < 128; ++k) {
            sounding[k] = uint8_t(std::min(255, sounding[k] + struck[k]));
        }
        uint8_t keys[voices];
        voices_of(sounding, keys);
        uint8_t attack = 0;
        for (uint8_t v = 0; v < voices; ++v) {
            if (keys[v] != 0 && (keys[v] != current[v] || struck[keys[v]] > 0)) {
                attack |= uint8_t(1u << v);
            }
        }
        std::memset(struck, 0, sizeof(struck));
        if (attack == 0 && std::memcmp(keys, current, voices) == 0) {
            continue;
        }
        const uint32_t t = now_us();
        const bool emitted = emit(t);
        std::memcpy(current, keys, voices);
        current_attack = attack;
        current_start = t;
        if (emitted) {
            return true;
        }
    }
    return false;
}

Chord Smoother::next(const Chord &chord) {
    Chord out = chord;
    const bool glued = played.keys[0] != 0 && played.end_us == chord.start_us;
    if (glued && chord.end_us - chord.start_us < min_chord_us) {
        std::memcpy(out.keys, played.keys, voices); // trop court : les mêmes notes continuent
        out.attack = 0;
    } else {
        out.attack = 0;
        for (uint8_t v = 0; v < voices; ++v) {
            const uint8_t bit = uint8_t(1u << v);
            const uint8_t before = glued ? played.keys[v] : 0;
            // Refrappée par le fichier : la même note qu'à l'accord lu juste avant, avec une attaque
            const bool restruck = (chord.attack & bit) && chord.keys[v] == raw.keys[v];
            if (chord.keys[v] != 0 && (chord.keys[v] != before || restruck)) {
                out.attack |= bit;
            }
        }
    }
    raw = chord;
    played = out;
    return out;
}

float note_frequency(uint8_t key) {
    return 440.f * std::exp2((float(key) - 69.f) / 12.f);
}

uint8_t transposed(uint8_t key, int semitones) {
    if (key == 0) {
        return 0;
    }
    int k = key + semitones;
    while (k > 127) {
        k -= 12;
    }
    while (k < 1) {
        k += 12;
    }
    return uint8_t(k);
}

namespace {

    /// Un morceau dans le fichier des réglages : 16 octets, mis à jour sur place
    struct TuningRecord {
        char name[13]; ///< nom court 8.3, terminé par 0
        uint8_t tempo;
        int8_t transpose;
        uint8_t mark;
    };
    static_assert(sizeof(TuningRecord) == 16);
    constexpr uint8_t record_mark = 0xD3;

    bool same_song(const TuningRecord &r, const char *short_name) {
        return r.mark == record_mark && memchr(r.name, '\0', sizeof(r.name)) && strcasecmp(r.name, short_name) == 0;
    }

    /// Sans tampon de la libc : 1 Ko de tas de moins, et rien à vider avant de passer de la lecture à l'écriture
    FILE *open_unbuffered(const char *path, const char *mode) {
        FILE *f = fopen(path, mode);
        if (f) {
            setvbuf(f, nullptr, _IONBF, 0);
        }
        return f;
    }

} // namespace

Tuning load_tuning(const char *path, const char *short_name) {
    Tuning t;
    FILE *f = open_unbuffered(path, "rb");
    if (!f) {
        return t;
    }
    TuningRecord r;
    while (fread(&r, sizeof(r), 1, f) == 1) {
        if (same_song(r, short_name)) {
            if (r.tempo >= min_tempo && r.tempo <= max_tempo && r.transpose >= -max_transpose && r.transpose <= max_transpose) {
                t = { r.tempo, r.transpose };
            }
            break;
        }
    }
    fclose(f);
    return t;
}

bool save_tuning(const char *path, const char *short_name, Tuning tuning) {
    TuningRecord r {};
    if (strlen(short_name) >= sizeof(r.name)) {
        return false;
    }
    strcpy(r.name, short_name);
    r.tempo = tuning.tempo;
    r.transpose = tuning.transpose;
    r.mark = record_mark;

    FILE *f = open_unbuffered(path, "r+b");
    if (!f) {
        f = open_unbuffered(path, "w+b");
    }
    if (!f) {
        return false;
    }
    // La place du morceau ; sinon la fin du fichier (ou un dernier enregistrement tronqué, réécrit)
    long at = 0;
    TuningRecord old;
    while (fread(&old, sizeof(old), 1, f) == 1 && !same_song(old, short_name)) {
        at += long(sizeof(old));
    }
    const bool written = fseek(f, at, SEEK_SET) == 0 && fwrite(&r, sizeof(r), 1, f) == 1;
    return fclose(f) == 0 && written;
}

float note_speed(uint8_t key, float full_steps_per_mm, float max_speed, float min_speed) {
    if (!(full_steps_per_mm > 0.f) || !(max_speed > 0.f) || !std::isfinite(full_steps_per_mm)) {
        return 0.f;
    }
    float v = note_frequency(key) / full_steps_per_mm;
    while (v > max_speed) {
        v /= 2.f;
    }
    while (v < min_speed && v * 2.f <= max_speed) {
        v *= 2.f;
    }
    return v;
}

float chord_step(VoiceMotion (&m)[voices], const float (&speed)[voices], float remaining_s, float max_s, int32_t (&d)[voices]) {
    float dt = std::min(remaining_s, max_s);
    int32_t room[voices] = {};
    int limit = -1; // la voix qui touche sa borne la première
    bool active = false;
    for (uint8_t v = 0; v < voices; ++v) {
        d[v] = 0;
        if (!(speed[v] > 0.f) || m[v].bound <= 0) {
            continue;
        }
        active = true;
        const auto room_of = [&] { return m[v].dir > 0 ? m[v].bound - m[v].pos : m[v].pos + m[v].bound; };
        room[v] = room_of();
        if (room[v] <= 0) { // demi-tour à la borne
            m[v].dir = int8_t(-m[v].dir);
            room[v] = room_of();
        }
        if (const float t = float(room[v]) / speed[v]; t < dt) {
            dt = t;
            limit = v;
        }
    }
    if (!active || !(dt > 0.f)) {
        return 0.f;
    }
    for (uint8_t v = 0; v < voices; ++v) {
        if (!(speed[v] > 0.f) || m[v].bound <= 0) {
            continue;
        }
        int32_t length = v == limit ? room[v] : std::min<int32_t>(room[v], int32_t(std::lround(speed[v] * dt)));
        if (room[v] - length <= 1) {
            length = room[v]; // à 1 µm près, à la borne : deux voix égales (Mélodie : A et B) restent ensemble
        }
        d[v] = m[v].dir * length;
        m[v].pos += d[v];
    }
    return dt;
}

} // namespace od3d::music
