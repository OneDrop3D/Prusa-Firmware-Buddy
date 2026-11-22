/// @file
/// OneDrop3D (F1, interface G, UI-10) : ce que montre la pastille « Dynamic Island » de l'en-tête, sans affichage ni
/// lecture de l'imprimante (testé dans tests/unit/feature/od3d). Une seule pastille à la fois, par priorité :
/// 1. un de nos processus (mesure du PA, courroies...), hors de son écran : sa chauffe, son chargement et son palpage
///    sont les siens (retour de l'utilisateur, 22/11 : la pastille menait aux températures pendant la mesure du PA) ;
/// 2. l'impression en cours, hors de l'écran Impression ;
/// 3. le chargement ou déchargement réduit par la flèche de son écran ;
/// 4. le « Patientez » de Prusa (origine, palpage) réduit ;
/// 5. la buse qui chauffe hors impression et hors machine à états (préchauffage), hors de l'aperçu des températures.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace od3d::island {

enum class Kind : uint8_t {
    none, ///< pas de pastille
    process, ///< un de nos processus : son nom et son étape
    filament, ///< chargement ou déchargement réduit
    print, ///< impression en cours
    wait, ///< « Patientez » de Prusa réduit
    preheat, ///< buse qui chauffe
};

/// L'anneau tourne (10 images/s) : l'opération n'a pas d'avancement lisible d'un coup d'œil
constexpr bool spins(Kind kind) {
    return kind == Kind::process || kind == Kind::filament || kind == Kind::wait;
}

/// Nos processus longs, chacun avec son écran
enum class Process : uint8_t {
    pa, ///< M1994
    flow, ///< M1995
    min_temp, ///< M1996
    belts, ///< M1992
    speed_map, ///< M1993
    damping, ///< M1997
    mpc, ///< M306 T
    music, ///< M1991
    hit_test, ///< essai de la sécurité collision
    mmu_usb, ///< M709 S2 : MMU alimenté et muet, mise à jour de son firmware par son USB
    calibration, ///< assistant de calibration (M1989) : le test en cours et « étape/étapes »
    fan_response, ///< M2001
    nozzle_expansion, ///< M2003
};

/// Étape d'un processus, écrite après son nom
enum class Stage : uint8_t {
    busy, ///< le nom seul
    heating,
    probing,
    loading,
    priming,
    step, ///< « 2/5 »
    percent, ///< « 45% »
};

/// État de l'impression, en couleur : orange (en cours), jaune (pause), gris (attente : reprise, chauffe, question ;
/// fin : terminée ou arrêtée, jusqu'à « Accueil » sur l'écran Impression), rouge (erreur)
enum class Tone : uint8_t {
    normal,
    paused,
    waiting,
    error,
    ended,
};

inline constexpr uint8_t no_slot = 0xff;
inline constexpr uint16_t no_time = 0xffff;

struct Face {
    Kind kind = Kind::none;
    Tone tone = Tone::normal;
    uint8_t percent = 0; ///< chargement, impression, processus (Stage::percent)
    uint8_t slot = no_slot; ///< chargement lancé depuis l'atelier, processus d'une bobine : emplacement 0 à 4
    Process process = Process::pa;
    Stage stage = Stage::busy;
    uint8_t step = 0; ///< processus : « step/steps »
    uint8_t steps = 0;
    uint16_t minutes = no_time; ///< impression : temps restant [min]
    int16_t current = 0; ///< préchauffage : buse [°C]
    int16_t target = 0;

    constexpr bool operator==(const Face &) const = default;
};

/// Ce qu'on sait de l'imprimante et de l'écran ouvert
struct Inputs {
    Face process; ///< Kind::process : un de nos processus est en cours
    bool on_process_screen = false;

    bool filament_minimized = false;
    uint8_t filament_percent = 0;
    uint8_t filament_slot = no_slot;

    bool print_fsm = false; ///< machine Printing ouverte (du départ à « Accueil » après la fin)
    bool print_not_started = false; ///< aperçu, questions avant le départ : pas encore de pastille
    bool print_hidden_by_fsm = false; ///< une autre machine à états s'affiche par-dessus (son écran est la priorité)
    bool on_print_screen = false;
    Tone print_tone = Tone::normal;
    uint8_t print_percent = 0;
    uint16_t print_minutes = no_time;

    bool wait_minimized = false;

    bool other_fsm = false; ///< une machine à états autre que l'impression : sa chauffe est la sienne
    bool on_temperatures_screen = false;
    bool nozzle_heating = false; ///< PreheatLatch
    int16_t nozzle = 0;
    int16_t nozzle_target = 0;
};

constexpr Face choose(const Inputs &in) {
    if (in.process.kind == Kind::process) {
        return in.on_process_screen ? Face {} : in.process;
    }
    if (in.print_fsm && !in.print_not_started && !in.print_hidden_by_fsm && !in.on_print_screen) {
        return Face { .kind = Kind::print, .tone = in.print_tone, .percent = in.print_percent > 100 ? uint8_t(100) : in.print_percent, .minutes = in.print_minutes };
    }
    if (in.filament_minimized) {
        return Face { .kind = Kind::filament, .percent = in.filament_percent, .slot = in.filament_slot };
    }
    if (in.wait_minimized) {
        return Face { .kind = Kind::wait };
    }
    if (!in.print_fsm && in.nozzle_heating && !in.other_fsm && !in.on_temperatures_screen) {
        return Face { .kind = Kind::preheat, .current = in.nozzle, .target = in.nozzle_target };
    }
    return {};
}

/// Texte de la pastille, 12 caractères au plus (la place à côté de l'anneau, l'heure sur 12 h) : « 2·45% »,
/// « 47% 1h23 », « 47% 25min », « II 47% » (pause), « ! 47% » (erreur), « 185/215° », et pour un processus son nom
/// puis son étape : « PA 2/5 », « PA chauffe », « Courroies 45% ». name : nom traduit du processus (ou « Patientez »),
/// word : mot traduit de son étape (Stage::heating à Stage::priming)
inline void format_text(const Face &f, char *buf, size_t size, const char *name = "", const char *word = "") {
    constexpr size_t max_chars = 12;
    buf[0] = '\0';
    switch (f.kind) {
    case Kind::none:
        break;
    case Kind::process:
        if (f.stage == Stage::step) {
            snprintf(buf, size, "%s %d/%d", name, f.step, f.steps);
        } else if (f.stage == Stage::percent) {
            snprintf(buf, size, "%s %d%%", name, f.percent);
        } else if (word[0]) {
            snprintf(buf, size, "%s %s", name, word);
        } else {
            snprintf(buf, size, "%s", name);
        }
        break;
    case Kind::wait:
        snprintf(buf, size, "%s", name);
        break;
    case Kind::filament:
        if (f.slot != no_slot) {
            snprintf(buf, size, "%d\xC2\xB7%d%%", f.slot + 1, f.percent);
        } else {
            snprintf(buf, size, "%d%%", f.percent);
        }
        break;
    case Kind::print: {
        const char *prefix = f.tone == Tone::paused ? "II " : (f.tone == Tone::error ? "! " : "");
        const int n = snprintf(buf, size, "%s%d%%", prefix, f.percent);
        if (f.tone != Tone::normal || f.minutes == no_time || n < 0 || size_t(n) >= size) {
            break;
        }
        char time[12];
        if (f.minutes >= 6000) {
            snprintf(time, sizeof(time), " %dh", f.minutes / 60);
        } else if (f.minutes >= 60) {
            snprintf(time, sizeof(time), " %dh%02d", f.minutes / 60, f.minutes % 60);
        } else {
            snprintf(time, sizeof(time), " %dmin", f.minutes);
        }
        if (size_t(n) + strlen(time) <= max_chars && size_t(n) + strlen(time) < size) {
            strcat(buf, time);
        }
        break;
    }
    case Kind::preheat:
        snprintf(buf, size, "%d/%d\xC2\xB0", f.current, f.target);
        break;
    }
}

/// Une opération du filament qui commence (machine Load_unload) s'ouvre-t-elle directement réduite dans la pastille ?
/// Son écran revient de toute façon dès que Prusa attend l'utilisateur.
struct BackgroundInputs {
    bool process = false; ///< un de nos processus charge la bobine (mesure du PA en un clic) : la pastille le montre
    bool chained = false; ///< une opération réduite vient de finir (< 3 s) : la suite d'une même commande (MMU)
    bool printing = false; ///< impression en cours : les changements d'outil du MMU
    bool kept_for_print = false; ///< « Continuer en arrière-plan » déjà choisi pendant cette impression
    bool on_print_screen = false;
};

constexpr bool open_in_background(const BackgroundInputs &in) {
    // Pendant une impression, on ne prend l'écran que de celui qui regarde l'écran Impression (et ne l'a pas refusé)
    return in.process || in.chained || (in.printing && (in.kept_for_print || !in.on_print_screen));
}

/// La buse chauffe-t-elle ? Avec une hystérésis, pour que la pastille n'apparaisse pas à chaque oscillation de la
/// régulation : elle apparaît quand une nouvelle cible est loin au-dessus de la température, et disparaît une fois la
/// cible atteinte, jusqu'à la cible suivante (une buse qui refroidit un peu sous le ventilateur ne la rallume pas).
class PreheatLatch {
public:
    static constexpr int16_t start_below = 5; ///< [°C] sous la cible : elle chauffe vraiment
    static constexpr int16_t reached_below = 2; ///< [°C] sous la cible : atteinte

    constexpr bool update(int16_t current, int16_t target) {
        if (target <= 0) {
            latched_target = 0;
            heating = false;
            return false;
        }
        if (target != latched_target) {
            latched_target = target;
            heating = current < target - start_below;
        } else if (heating && current >= target - reached_below) {
            heating = false;
        }
        return heating;
    }

private:
    int16_t latched_target = 0;
    bool heating = false;
};

} // namespace od3d::island
