/// @file
#include "pairing_od3d.hpp"

#include <atomic>
#include <random/random.h>
#include <timing.h>

namespace od3d::pairing {

namespace {

    constexpr uint32_t duree_ms = 5 * 60 * 1000;
    constexpr uint8_t essais_max = 5;

    // Écrits par l'interface (start, stop), lus par le serveur PrusaLink (check) : d'où les atomiques.
    std::atomic<uint32_t> code_actif { 0 }; ///< 0 : pas d'association en cours
    std::atomic<uint32_t> debut_ms { 0 };
    std::atomic<uint8_t> erreurs { 0 };

} // namespace

uint32_t start() {
    const uint32_t code = 100000 + rand_u() % 900000;
    erreurs = 0;
    debut_ms = ticks_ms();
    code_actif = code;
    return code;
}

void stop() {
    code_actif = 0;
}

bool check(std::string_view recu) {
    uint32_t attendu = code_actif.load();
    if (attendu == 0) {
        return false;
    }
    if (ticks_ms() - debut_ms.load() > duree_ms) {
        code_actif = 0;
        return false;
    }
    uint32_t valeur = 0;
    bool chiffres = recu.size() == 6;
    for (const char c : recu) {
        chiffres &= (c >= '0' && c <= '9');
        valeur = valeur * 10 + static_cast<uint32_t>(c - '0');
    }
    if (!chiffres || valeur != attendu) {
        if (++erreurs >= essais_max) {
            code_actif = 0;
        }
        return false;
    }
    return code_actif.compare_exchange_strong(attendu, 0); // une seule fois, même si deux demandes arrivent ensemble
}

} // namespace od3d::pairing
