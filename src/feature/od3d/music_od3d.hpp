/// @file
/// OneDrop3D (F18) : musique des moteurs, partie pure (sans Marlin, testée en unitaire).
///
/// Lecture en flux d'un fichier MIDI simple (format 0 ou 1) : chaque piste est lue par petits morceaux à sa
/// position dans le fichier, jamais en entier. Le canal 10 (percussions, sans hauteur) est ignoré. Même logique que
/// la conversion de PrintForge (musique.rs) : une note relâchée puis frappée au même instant est rejouée, une note
/// jamais relâchée s'arrête avec le morceau.
///
/// Un moteur pas à pas sonne à la fréquence de ses pas complets : pour une note de f Hz, il file à
/// v = f / pas_complets_par_mm, en allers-retours autour d'un point central.
///
/// Trois voix (retour du 26/11 : « il utilise que le X [...] on reconnaît pas bien la musique ») : à chaque instant,
/// la mélodie (la plus aiguë), une 2e voix (la 2e plus aiguë, s'il y en a au moins 3) et la basse (la plus grave),
/// comme midi2cnc et StepperTune : une note par moteur, un seul déplacement linéaire pour tout l'accord. Sur une CoreXY,
/// les moteurs A (X+Y) et B (X−Y) chantent chacun leur voix : vx = (vA + vB) / 2, vy = (vA − vB) / 2 ; Z fait la 3e.
#pragma once

#include <cstddef>
#include <cstdint>

namespace od3d::music {

/// Octets du fichier, lus par morceaux à une position donnée (fichier de la clé, ou tableau dans les tests)
class Source {
public:
    /// Copie au plus len octets depuis offset ; rend le nombre lu (0 en fin de fichier ou sur erreur)
    virtual size_t read(uint32_t offset, uint8_t *buf, size_t len) = 0;

protected:
    ~Source() = default;
};

/// Voix d'un accord : la mélodie, la 2e voix, la basse
inline constexpr uint8_t voices = 3;

/// Un accord à jouer : début et fin en µs depuis le début du morceau, la note de chaque voix (numéro MIDI, 69 = la4,
/// 440 Hz ; 0 : la voix se tait). attack, bit v : la voix v change de note ou la refrappe au début de l'accord (une
/// note tenue d'un accord au suivant continue).
struct Chord {
    uint32_t start_us;
    uint32_t end_us;
    uint8_t keys[voices];
    uint8_t attack;
};

/// Les voix d'un instant, d'après les notes qui sonnent (count[k] > 0, 128 notes) : la plus aiguë, la 2e plus aiguë
/// s'il en sonne au moins 3, la plus grave s'il en sonne au moins 2 ; 0 pour une voix qui se tait
void voices_of(const uint8_t *count, uint8_t (&keys)[voices]);

enum class Error : uint8_t {
    none,
    not_midi, ///< en-tête illisible
    unsupported_format, ///< format 2 (pistes indépendantes) ou inconnu
    no_tracks,
};

/// Lecteur en flux. Environ 1 Ko : à mettre en mémoire statique plutôt que sur la pile.
class Reader {
public:
    /// Pistes lues au plus (les suivantes sont ignorées)
    static constexpr uint8_t max_tracks = 16;
    /// Tampon de lecture par piste
    static constexpr uint8_t buffer_size = 32;

    /// Lit l'en-tête et la table des pistes, et remet la lecture au début
    Error open(Source &source);

    /// Accord suivant, dans l'ordre du temps (les silences sont entre deux accords) ; false à la fin du morceau
    bool next(Chord &chord);

    uint8_t track_count() const { return tracks_used; }

private:
    struct Track {
        uint32_t pos; ///< prochain octet à lire (position dans le fichier)
        uint32_t end; ///< fin de la piste
        uint64_t next_tick; ///< instant du prochain événement
        uint32_t buf_pos; ///< position du tampon dans le fichier
        uint8_t buf_len;
        uint8_t status; ///< statut courant (« running status »), 0 sans
        bool done;
        uint8_t buf[buffer_size];
    };

    int read_byte(Track &t);
    bool read_vlq(Track &t, uint32_t &value);
    void skip(Track &t, uint32_t count);
    void load_delta(Track &t);
    void process_event(Track &t);
    uint32_t now_us() const;

    Source *source = nullptr;
    Track tracks[max_tracks];
    uint8_t tracks_used = 0;
    bool smpte = false;
    uint32_t tick_num = 500'000; ///< durée d'un tic = tick_num / tick_den µs
    uint32_t tick_den = 1;
    uint64_t time_scaled = 0; ///< temps écoulé en µs × tick_den
    uint64_t last_tick = 0;
    uint8_t sounding[128]; ///< notes qui sonnent (nombre de frappes non relâchées)
    uint8_t struck[128]; ///< notes frappées à l'instant en cours, ajoutées après les relâchements
    uint8_t current[voices] = {}; ///< accord en cours (0 : voix muette)
    uint8_t current_attack = 0;
    uint32_t current_start = 0;
    bool finished = false;
};

/// Lissage des accords (retour du 26/11, « c'est brouillon ») : un morceau joué par un humain n'a jamais deux notes
/// frappées pile ensemble, et chaque décalage fait un accord de quelques ms (le générique d'Harry Potter : 507 accords
/// sur 1 367 durent moins de 40 ms, la mélodie y saute 208 fois sur une note d'accompagnement). Un accord plus court
/// que min_chord_us collé au précédent garde les notes jouées ; une voix ne repart que si sa note change vraiment ou
/// si le fichier la refrappe.
class Smoother {
public:
    static constexpr uint32_t min_chord_us = 40'000;

    /// L'accord à jouer à la place de chord (mêmes instants)
    Chord next(const Chord &chord);

private:
    Chord played {}; ///< le dernier accord joué
    Chord raw {}; ///< le dernier accord lu
};

/// Fréquence d'une note MIDI en Hz (la4 = 69 = 440 Hz)
float note_frequency(uint8_t key);

/// Réglages d'un morceau (retour du 26/11 : « modifier la vitesse ou autre de la musique depuis l'imprimante et que ça
/// s'enregistre ») : un par morceau, dans un petit fichier de la clé (le config_store est plein)
struct Tuning {
    uint8_t tempo = 100; ///< % de la vitesse écrite dans le fichier
    int8_t transpose = 0; ///< demi-tons
};
inline constexpr uint8_t min_tempo = 25;
inline constexpr uint8_t max_tempo = 200;
inline constexpr int8_t max_transpose = 24;

/// La note décalée de semitones, ramenée d'une octave tant qu'elle sort des notes MIDI (1 à 127) ; 0 reste un silence
uint8_t transposed(uint8_t key, int semitones);

/// Réglages retenus pour le morceau short_name (nom court 8.3) dans le fichier path ; ceux par défaut sinon
Tuning load_tuning(const char *path, const char *short_name);

/// Retient les réglages du morceau dans path (à sa place s'il y est, sinon à la fin) ; false si l'écriture échoue
bool save_tuning(const char *path, const char *short_name, Tuning tuning);

/// Vitesse du moteur pour une note : v = f / pas_complets_par_mm, descendue d'une octave tant qu'elle dépasse
/// max_speed (mm/s), montée d'une octave tant qu'elle est sous min_speed (un moteur ne fait que cliquer sous ~50 Hz).
/// 0 si les paramètres sont absurdes.
float note_speed(uint8_t key, float full_steps_per_mm, float max_speed, float min_speed = 0.f);

/// Une voix en allers-retours autour de son centre, sans sortir de ±bound. Positions entières en µm : aucune dérive,
/// même sur des milliers de notes.
struct VoiceMotion {
    int32_t bound = 0;
    int32_t pos = 0;
    int8_t dir = 1;
};

/// Prochain morceau d'un accord, un seul déplacement linéaire pour toutes les voix : la voix v file à speed[v] µm/s
/// (0 : immobile) dans son sens. Le morceau dure au plus remaining_s et max_s, et s'arrête quand une voix touche sa
/// borne (elle repart dans l'autre sens au morceau suivant). Rend sa durée (0 : rien ne bouge), d[v] en µm signés.
float chord_step(VoiceMotion (&m)[voices], const float (&speed)[voices], float remaining_s, float max_s, int32_t (&d)[voices]);

} // namespace od3d::music
