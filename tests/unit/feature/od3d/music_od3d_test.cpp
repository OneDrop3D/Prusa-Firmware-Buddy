// OneDrop3D (F18) : lecture en flux d'un MIDI simple et allers-retours des moteurs. Les fichiers MIDI sont construits
// dans le test, comme dans PrintForge (musique.rs) dont on reprend les cas.
#include <feature/od3d/music_od3d.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cstdlib>
#include <algorithm>
#include <array>
#include <cstring>
#include <initializer_list>
#include <utility>
#include <vector>

using namespace od3d::music;

namespace {

using Bytes = std::vector<uint8_t>;
/// Une piste : suite de (delta en tics, octets de l'événement) ; la fin de piste est ajoutée
using Piste = std::vector<std::pair<uint32_t, Bytes>>;

const Bytes TEMPO_120 { 0xFF, 0x51, 3, 0x07, 0xA1, 0x20 }; // 500 000 µs par noire
const Bytes TEMPO_60 { 0xFF, 0x51, 3, 0x0F, 0x42, 0x40 }; // 1 000 000 µs par noire
const Bytes LA_ON { 0x90, 69, 100 };
const Bytes LA_OFF { 0x80, 69, 0 };

void vlq(Bytes &out, uint32_t v) {
    Bytes tmp { uint8_t(v & 0x7F) };
    while (v >>= 7) {
        tmp.insert(tmp.begin(), uint8_t((v & 0x7F) | 0x80));
    }
    out.insert(out.end(), tmp.begin(), tmp.end());
}

void be(Bytes &out, uint32_t v, int n) {
    for (int i = n - 1; i >= 0; --i) {
        out.push_back(uint8_t(v >> (8 * i)));
    }
}

/// Un fichier MIDI à 480 tics par noire
Bytes midi(uint16_t format, const std::vector<Piste> &pistes, uint16_t division = 480) {
    Bytes m { 'M', 'T', 'h', 'd' };
    be(m, 6, 4);
    be(m, format, 2);
    be(m, uint32_t(pistes.size()), 2);
    be(m, division, 2);
    for (const Piste &p : pistes) {
        Bytes corps;
        for (const auto &[delta, octets] : p) {
            vlq(corps, delta);
            corps.insert(corps.end(), octets.begin(), octets.end());
        }
        corps.insert(corps.end(), { 0, 0xFF, 0x2F, 0 });
        m.insert(m.end(), { 'M', 'T', 'r', 'k' });
        be(m, uint32_t(corps.size()), 4);
        m.insert(m.end(), corps.begin(), corps.end());
    }
    return m;
}

/// Le fichier en mémoire, lu par morceaux ; compte les lectures
class Memoire final : public Source {
public:
    explicit Memoire(Bytes d)
        : data(std::move(d)) {}

    size_t read(uint32_t offset, uint8_t *buf, size_t len) override {
        ++reads;
        max_len = std::max(max_len, len);
        if (offset >= data.size()) {
            return 0;
        }
        const size_t n = std::min(len, data.size() - offset);
        std::memcpy(buf, data.data() + offset, n);
        return n;
    }

    Bytes data;
    size_t reads = 0;
    size_t max_len = 0;
};

struct N {
    uint32_t start_us, end_us;
    uint8_t key;
    bool operator==(const N &) const = default;
};

struct A {
    uint32_t start_us, end_us;
    uint8_t keys[voices];
    uint8_t attack;
    bool operator==(const A &o) const {
        return start_us == o.start_us && end_us == o.end_us && std::equal(keys, keys + voices, o.keys) && attack == o.attack;
    }
};

std::vector<A> accords(Bytes fichier, Error *erreur = nullptr) {
    static Reader reader; // ~1 Ko, comme dans le firmware
    Memoire source(std::move(fichier));
    const Error e = reader.open(source);
    if (erreur) {
        *erreur = e;
    }
    std::vector<A> out;
    Chord c;
    while (e == Error::none && reader.next(c)) {
        out.push_back({ c.start_us, c.end_us, { c.keys[0], c.keys[1], c.keys[2] }, c.attack });
    }
    return out;
}

/// La mélodie (voix 0) : les accords qui la prolongent sans la refrapper sont réunis
std::vector<N> melodie(const std::vector<A> &as) {
    std::vector<N> notes;
    for (const A &a : as) {
        if (!notes.empty() && notes.back().end_us == a.start_us && notes.back().key == a.keys[0] && !(a.attack & 1)) {
            notes.back().end_us = a.end_us;
        } else {
            notes.push_back({ a.start_us, a.end_us, a.keys[0] });
        }
    }
    return notes;
}

std::vector<N> toutes(Bytes fichier, Error *erreur = nullptr) {
    return melodie(accords(std::move(fichier), erreur));
}

} // namespace

TEST_CASE("musique : tempo, silences et note rejouée") {
    const Piste piste {
        { 0, TEMPO_120 }, { 0, LA_ON }, { 480, LA_OFF }, // 0,5 s
        { 0, LA_ON }, { 480, LA_OFF }, // rejouée aussitôt : 0,5 s
        { 0, TEMPO_60 }, { 240, LA_ON }, { 480, LA_OFF }, // 0,5 s de silence au nouveau tempo, puis 1 s
    };
    Error e;
    const auto notes = toutes(midi(0, { piste }), &e);
    CHECK(e == Error::none);
    CHECK(notes == std::vector<N> { { 0, 500'000, 69 }, { 500'000, 1'000'000, 69 }, { 1'500'000, 2'500'000, 69 } });
}

TEST_CASE("musique : mélodie et basse, percussions ignorées (format 1)") {
    const Piste do4 { { 0, { 0x90, 60, 100 } }, { 960, { 0x80, 60, 0 } } }; // pendant 1 s
    const Piste mi4 {
        { 0, { 0x99, 100, 100 } }, // percussion (canal 10) : ignorée malgré sa hauteur
        { 480, { 0x90, 64, 100 } }, { 240, { 0x90, 64, 0 } }, // de 0,5 à 0,75 s ; vélocité 0 = fin de note
    };
    const auto as = accords(midi(1, { do4, mi4 }));
    // Le mi prend la mélodie, le do tenu passe à la basse puis reprend la mélodie quand le mi se tait
    CHECK(as == std::vector<A> { { 0, 500'000, { 60, 0, 0 }, 1 }, { 500'000, 750'000, { 64, 0, 60 }, 1 | 4 }, { 750'000, 1'000'000, { 60, 0, 0 }, 1 } });
    CHECK(melodie(as) == std::vector<N> { { 0, 500'000, 60 }, { 500'000, 750'000, 64 }, { 750'000, 1'000'000, 60 } });
}

TEST_CASE("musique : lissage des accords trop courts") {
    Smoother s;
    const auto jouer = [&](uint32_t debut_ms, uint32_t fin_ms, uint8_t cle, uint8_t attaque) {
        const Chord o = s.next({ debut_ms * 1000, fin_ms * 1000, { cle, 0, 0 }, attaque });
        return std::pair { o.keys[0], o.attack };
    };
    CHECK(jouer(0, 500, 76, 1) == std::pair<uint8_t, uint8_t> { 76, 1 });
    // 23 ms d'une note d'accompagnement entre deux notes de la mélodie : le mi6 continue, sans attaque
    CHECK(jouer(500, 523, 64, 1) == std::pair<uint8_t, uint8_t> { 76, 0 });
    CHECK(jouer(523, 900, 76, 1) == std::pair<uint8_t, uint8_t> { 76, 0 });
    // Vraiment refrappé par le fichier (la même note qu'à l'accord lu juste avant) : il repart
    CHECK(jouer(900, 1200, 76, 1) == std::pair<uint8_t, uint8_t> { 76, 1 });
    // Une vraie nouvelle note
    CHECK(jouer(1200, 1400, 79, 1) == std::pair<uint8_t, uint8_t> { 79, 1 });
    // Après un silence, même très courte, la note est jouée
    CHECK(jouer(1500, 1510, 60, 1) == std::pair<uint8_t, uint8_t> { 60, 1 });
}

TEST_CASE("musique : trois voix d'un accord, la 4e note se tait") {
    uint8_t count[128] = {};
    uint8_t keys[voices];
    count[60] = 1;
    voices_of(count, keys);
    CHECK((keys[0] == 60 && keys[1] == 0 && keys[2] == 0));
    count[64] = 1;
    voices_of(count, keys);
    CHECK((keys[0] == 64 && keys[1] == 0 && keys[2] == 60)); // deux notes : mélodie et basse
    count[67] = 2; // frappée deux fois : une seule voix
    voices_of(count, keys);
    CHECK((keys[0] == 67 && keys[1] == 64 && keys[2] == 60));
    count[48] = 1;
    voices_of(count, keys);
    CHECK((keys[0] == 67 && keys[1] == 64 && keys[2] == 48)); // do3 devient la basse, le do4 se tait

    // Un accord do-mi-sol tenu pendant que la mélodie change : seule la voix qui change attaque
    const Piste accord { { 0, { 0x90, 48, 100 } }, { 0, { 0x90, 55, 100 } }, { 0, { 0x90, 64, 100 } }, { 480, { 0x80, 64, 0 } },
        { 0, { 0x90, 67, 100 } }, { 480, { 0x80, 67, 0 } }, { 0, { 0x80, 55, 0 } }, { 0, { 0x80, 48, 0 } } };
    CHECK(accords(midi(0, { accord })) == std::vector<A> { { 0, 500'000, { 64, 55, 48 }, 1 | 2 | 4 }, { 500'000, 1'000'000, { 67, 55, 48 }, 1 } });
}

TEST_CASE("musique : tempo de la piste 0 appliqué aux autres pistes") {
    const Piste tempo { { 0, TEMPO_60 } };
    const Piste melodie { { 0, LA_ON }, { 480, LA_OFF } };
    CHECK(toutes(midi(1, { tempo, melodie })) == std::vector<N> { { 0, 1'000'000, 69 } });
}

TEST_CASE("musique : statut courant, méta-événements longs et petit tampon") {
    // Un texte de 100 octets (plus long que le tampon d'une piste), puis 20 notes en statut courant (0x90 omis)
    Bytes texte { 0xFF, 0x01, 100 };
    texte.resize(texte.size() + 100, 'a');
    Piste piste { { 0, texte }, { 0, { 0x90, 60, 100 } }, { 120, { 60, 0 } } };
    for (int i = 1; i < 20; ++i) {
        piste.push_back({ 0, { uint8_t(60 + i), 100 } });
        piste.push_back({ 120, { uint8_t(60 + i), 0 } });
    }
    // Une 2e piste entrelacée, plus grave : elle ne s'entend pas mais oblige à relire l'autre piste par morceaux
    Piste grave;
    for (int i = 0; i < 40; ++i) {
        grave.push_back({ 0, { 0x91, 30, 100 } });
        grave.push_back({ 60, { 0x81, 30, 0 } });
    }
    Memoire source(midi(1, { piste, grave }));
    static Reader reader;
    REQUIRE(reader.open(source) == Error::none);
    CHECK(reader.track_count() == 2);
    std::vector<A> as;
    Chord c;
    while (reader.next(c)) {
        as.push_back({ c.start_us, c.end_us, { c.keys[0], c.keys[1], c.keys[2] }, c.attack });
    }
    const auto notes = melodie(as); // la piste grave, à la basse, coupe les accords mais pas la mélodie
    REQUIRE(notes.size() == 20);
    for (int i = 0; i < 20; ++i) {
        // 120 tics à 480 par noire et 120 noires par minute : 125 ms
        CHECK(notes[i] == N { uint32_t(125'000 * i), uint32_t(125'000 * (i + 1)), uint8_t(60 + i) });
    }
    CHECK(source.max_len <= Reader::buffer_size); // jamais tout le fichier d'un coup
    CHECK(source.reads > 10);
}

TEST_CASE("musique : note jamais relâchée et notes superposées de même hauteur") {
    // Deux frappes du la, un seul relâchement : il sonne jusqu'à la fin du morceau
    const Piste piste { { 0, LA_ON }, { 240, LA_ON }, { 240, LA_OFF }, { 480, { 0xB0, 7, 100 } } };
    // la 2e frappe rejoue la note ; elle sonne encore jusqu'au dernier événement (contrôleur à 1 s, puis fin de piste)
    CHECK(toutes(midi(0, { piste })) == std::vector<N> { { 0, 250'000, 69 }, { 250'000, 1'000'000, 69 } });
}

TEST_CASE("musique : temps SMPTE") {
    // 25 images par seconde, 40 tics par image : 1 ms par tic, le tempo ne change rien
    const Piste piste { { 0, TEMPO_60 }, { 0, LA_ON }, { 300, LA_OFF } };
    const uint16_t division = uint16_t((uint16_t(uint8_t(-25)) << 8) | 40);
    CHECK(toutes(midi(0, { piste }, division)) == std::vector<N> { { 0, 300'000, 69 } });
}

TEST_CASE("musique : fichiers refusés ou vides") {
    const Piste piste { { 0, LA_ON }, { 480, LA_OFF } };
    Error e;
    CHECK(toutes(midi(2, { piste }), &e).empty());
    CHECK(e == Error::unsupported_format);

    const char *texte = "pas un fichier MIDI";
    CHECK(toutes(Bytes(texte, texte + strlen(texte)), &e).empty());
    CHECK(e == Error::not_midi);

    CHECK(toutes(midi(0, {}), &e).empty());
    CHECK(e == Error::no_tracks);

    CHECK(toutes(midi(0, { Piste {} }), &e).empty()); // aucune note
    CHECK(e == Error::none);

    // Fichier coupé au milieu de la piste : la lecture s'arrête sans planter
    Bytes coupe = midi(0, { Piste { { 0, LA_ON }, { 480, LA_OFF }, { 0, LA_ON }, { 480, LA_OFF } } });
    coupe.resize(coupe.size() - 6);
    const auto notes = toutes(coupe, &e);
    CHECK(e == Error::none);
    REQUIRE(!notes.empty());
    CHECK(notes.front() == N { 0, 500'000, 69 });
}

TEST_CASE("musique : vitesse des notes") {
    // la4 à 6,25 pas complets par mm (CORE One : 100 micro-pas par mm, 16 micro-pas par pas) : 70,4 mm/s
    CHECK(note_speed(69, 6.25f, 300.f) == Catch::Approx(70.4f));
    // la4 à 100 pas/mm : 4,4 mm/s ; do4 : 2,616 mm/s
    CHECK(note_speed(69, 100.f, 300.f) == Catch::Approx(4.4f));
    CHECK(note_speed(60, 100.f, 300.f) == Catch::Approx(2.6163f).epsilon(1e-4));
    // trop rapide : une octave plus bas, autant de fois qu'il faut
    CHECK(note_speed(69, 1.f, 300.f) == Catch::Approx(220.f));
    CHECK(note_speed(81, 1.f, 300.f) == Catch::Approx(220.f));
    // Z de la CORE One : 400 micro-pas par mm, 25 pas complets ; borné à 20 mm/s
    CHECK(note_speed(69, 25.f, 20.f) == Catch::Approx(17.6f));
    CHECK(note_speed(81, 25.f, 20.f) == Catch::Approx(17.6f));
    CHECK(note_speed(69, 0.f, 300.f) == 0.f);
    CHECK(note_speed(69, 6.25f, 0.f) == 0.f);
    // trop grave : une octave plus haut tant qu'il le faut (la1, 55 Hz, à 6,25 pas/mm : 8,8 mm/s)
    CHECK(note_speed(21, 6.25f, 300.f, 8.8f) == Catch::Approx(8.8f)); // la0 (27,5 Hz) joué comme la1
    CHECK(note_speed(33, 6.25f, 300.f, 8.8f) == Catch::Approx(8.8f));
    CHECK(note_speed(21, 6.25f, 5.f, 8.8f) == Catch::Approx(4.4f)); // sans dépasser le maximum
}

namespace {

/// Joue un accord de duree_s jusqu'au bout ; rend les déplacements de chaque morceau
std::vector<std::array<int32_t, voices>> jouer(VoiceMotion (&m)[voices], const float (&speed)[voices], float duree_s, float max_s = 10.f) {
    std::vector<std::array<int32_t, voices>> out;
    while (duree_s > 1e-6f) {
        int32_t d[voices];
        const float dt = chord_step(m, speed, duree_s, max_s, d);
        if (!(dt > 0.f)) {
            break;
        }
        duree_s -= dt;
        out.push_back({ d[0], d[1], d[2] });
        for (uint8_t v = 0; v < voices; ++v) {
            CHECK(std::abs(m[v].pos) <= m[v].bound);
        }
    }
    return out;
}

} // namespace

TEST_CASE("musique : allers-retours d'une voix dans les bornes") {
    // la4 à 10 pas/mm : 44 mm/s ; deux notes de 3 s font chacune 132 mm, plus que les ±100 mm permis
    VoiceMotion m[voices] = { { .bound = 100'000 }, {}, {} };
    const float speed[voices] = { 44'000.f, 0.f, 0.f };
    std::vector<int32_t> moves;
    for (int note = 0; note < 2; ++note) {
        for (const auto &d : jouer(m, speed, 3.f)) {
            moves.push_back(d[0]);
            CHECK((d[1] == 0 && d[2] == 0)); // les voix muettes ne bougent pas
        }
        m[0].dir = int8_t(-m[0].dir); // la note suivante repart dans l'autre sens
    }
    CHECK(moves == std::vector<int32_t> { 100'000, -32'000, 32'000, -100'000 });

    // Morceaux d'au plus max_s (arrêt rapide) dans le même sens
    VoiceMotion court[voices] = { { .bound = 100'000 }, {}, {} };
    moves.clear();
    for (const auto &d : jouer(court, speed, 0.5f, 0.2f)) {
        moves.push_back(d[0]);
    }
    CHECK(moves == std::vector<int32_t> { 8'800, 8'800, 4'400 });

    // Sans course : rien ne bouge
    VoiceMotion nul[voices] = {};
    int32_t d[voices];
    CHECK(chord_step(nul, speed, 1.f, 1.f, d) == 0.f);
}

TEST_CASE("musique : trois voix dans un seul déplacement, chacune à sa vitesse") {
    // A, B : moteurs du haut (±100 mm), Z : le plateau (±15 mm) ; mi4, do4 et do3 à 6,25 et 25 pas/mm
    VoiceMotion m[voices] = { { .bound = 100'000 }, { .bound = 100'000 }, { .bound = 15'000 } };
    const float speed[voices] = { 1000.f * note_speed(64, 6.25f, 300.f), 1000.f * note_speed(60, 6.25f, 300.f), 1000.f * note_speed(48, 25.f, 20.f) };
    int64_t parcouru[voices] = {};
    float duree = 0.f;
    int32_t d[voices];
    float reste = 4.f;
    while (reste > 1e-6f) {
        const float dt = chord_step(m, speed, reste, 0.2f, d);
        REQUIRE(dt > 0.f);
        reste -= dt;
        duree += dt;
        for (uint8_t v = 0; v < voices; ++v) {
            parcouru[v] += std::abs(d[v]);
            // Le même déplacement pour toutes : chaque voix y file à sa propre vitesse
            CHECK(std::abs(d[v]) == Catch::Approx(speed[v] * dt).margin(2.0));
            CHECK(std::abs(m[v].pos) <= m[v].bound);
        }
        // CoreXY : X = (A + B) / 2 et Y = (A − B) / 2 restent dans le carré de ±100 mm
        CHECK(std::abs(m[0].pos + m[1].pos) / 2 <= 100'000);
        CHECK(std::abs(m[0].pos - m[1].pos) / 2 <= 100'000);
    }
    CHECK(duree == Catch::Approx(4.f));
    for (uint8_t v = 0; v < voices; ++v) {
        CHECK(float(parcouru[v]) == Catch::Approx(speed[v] * 4.f).epsilon(1e-3)); // la hauteur de chaque voix est juste
    }

    // Mélodie : A et B à la même note restent ensemble (un déplacement en X seulement)
    VoiceMotion x[voices] = { { .bound = 100'000 }, { .bound = 100'000 }, {} };
    const float meme[voices] = { 70'400.f, 70'400.f, 0.f };
    for (const auto &dd : jouer(x, meme, 7.3f, 0.2f)) {
        CHECK(dd[0] == dd[1]);
    }
}
