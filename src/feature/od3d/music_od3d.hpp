/// @file
/// OneDrop3D (F18) : musique des moteurs, partie pure (sans Marlin, testée en unitaire).
///
/// Lecture en flux d'un fichier MIDI simple (format 0 ou 1) : chaque piste est lue par petits morceaux à sa
/// position dans le fichier, jamais en entier. À chaque instant, on garde la note la plus aiguë qui sonne ; le
/// canal 10 (percussions, sans hauteur) est ignoré. Même logique que la conversion de PrintForge (musique.rs) :
/// une note relâchée puis frappée au même instant est rejouée, une note jamais relâchée s'arrête avec le morceau.
///
/// Un moteur pas à pas sonne à la fréquence de ses pas complets : pour une note de f Hz, l'axe file à
/// v = f / pas_complets_par_mm pendant toute la note, en allers-retours autour d'un point central.
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

/// Une note à jouer : début et fin en µs depuis le début du morceau, numéro MIDI (69 = la4, 440 Hz)
struct Note {
    uint32_t start_us;
    uint32_t end_us;
    uint8_t key;
};

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

    /// Note suivante, dans l'ordre du temps ; false à la fin du morceau
    bool next(Note &note);

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
    int16_t current = -1; ///< note jouée, -1 aucune
    uint32_t current_start = 0;
    bool finished = false;
};

/// Fréquence d'une note MIDI en Hz (la4 = 69 = 440 Hz)
float note_frequency(uint8_t key);

/// Vitesse du moteur pour une note : v = f / pas_complets_par_mm, descendue d'une octave tant qu'elle dépasse
/// max_speed (mm/s). 0 si les paramètres sont absurdes.
float note_speed(uint8_t key, float full_steps_per_mm, float max_speed);

/// Allers-retours autour d'un point central, sans sortir de ±bound. Positions entières en µm : aucune dérive, même
/// sur des milliers de notes.
class Swing {
public:
    explicit Swing(int32_t bound_um)
        : bound(bound_um > 0 ? bound_um : 0) {}

    /// Prochain morceau d'une note dont il reste remaining µm à parcourir (décrémenté), au plus max_um ; demi-tour aux
    /// bornes. Rend le déplacement signé, 0 quand la note est jouée.
    int32_t step(int32_t &remaining_um, int32_t max_um);

    /// Note suivante : elle repart dans l'autre sens (deux notes égales s'entendent séparées)
    void next_note() { dir = -dir; }

    int32_t position() const { return pos; }

private:
    int32_t bound;
    int32_t pos = 0;
    int8_t dir = 1;
};

} // namespace od3d::music
