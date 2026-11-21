/// @file
/// OneDrop3D (F1, interface G) : textes adaptés à la police de l'imprimante (SPEC_G.md §1.2, §5).
/// Fonctions pures, sur des tampons UTF-8, testées dans tests/unit/feature/od3d.
#pragma once

#include <cstddef>
#include <cstring>

namespace od3d::text {

/// Coupe s à n caractères affichés, « ... » compris (« … » n'est pas dans la police)
inline void ellipsize(char *s, size_t n) {
    size_t chars = 0;
    char *cut = nullptr;
    for (char *p = s; *p; ++p) {
        if ((*p & 0xC0) == 0x80) {
            continue; // suite d'un caractère sur plusieurs octets
        }
        if (chars + 3 == n) {
            cut = p;
        }
        if (++chars > n) {
            if (cut) {
                strcpy(cut, "...");
            } else {
                *s = '\0'; // n < 3 : rien ne tient
            }
            return;
        }
    }
}

/// Titres Prusa en capitales (« RÉGLAGES WI-FI ») → casse de la maquette G (« Réglages Wi-Fi »).
/// Un titre qui a déjà une minuscule est laissé tel quel ; quelques sigles et marques gardent leur casse.
inline void title_case(char *s) {
    for (const char *p = s; *p; ++p) {
        if (*p >= 'a' && *p <= 'z') {
            return;
        }
    }

    // Mot en capitales → forme affichée (même longueur)
    static constexpr const char *kept[][2] = {
        { "MMU", "MMU" },
        { "USB", "USB" },
        { "IP", "IP" },
        { "LAN", "LAN" },
        { "NFC", "NFC" },
        { "LED", "LED" },
        { "WI-FI", "Wi-Fi" },
        { "CONNECT", "Connect" },
        { "PRUSALINK", "PrusaLink" },
        { "OPENPRINTTAG", "OpenPrintTag" },
        { "G-CODE", "G-code" },
    };

    bool first = true;
    for (char *word = s; *word;) {
        size_t len = 0;
        while (word[len] && word[len] != ' ') {
            ++len;
        }

        const char *as_is = nullptr;
        for (const auto &k : kept) {
            if (strlen(k[0]) == len && strncmp(word, k[0], len) == 0) {
                as_is = k[1];
            }
        }
        if (as_is) {
            memcpy(word, as_is, len);
        } else {
            // La première lettre du titre reste en capitale (si elle est accentuée, i = 1 tombe sur son
            // second octet, que rien ne modifie)
            for (size_t i = first ? 1 : 0; i < len; ++i) {
                char &c = word[i];
                if (c >= 'A' && c <= 'Z') {
                    c = static_cast<char>(c - 'A' + 'a');
                } else if (c == '\xC3' && i + 1 < len) {
                    // Lettres accentuées de Latin-1 : U+00C0–U+00DE (sauf ×) → U+00E0–U+00FE
                    if (const unsigned char n = static_cast<unsigned char>(word[i + 1]); n >= 0x80 && n <= 0x9E && n != 0x97) {
                        word[i + 1] = static_cast<char>(n + 0x20);
                    }
                    ++i;
                }
            }
        }

        first = false;
        word += len;
        while (*word == ' ') {
            ++word;
        }
    }
}

} // namespace od3d::text
