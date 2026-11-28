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

/// Minuscule d'une lettre de Latin-1 ou de Latin étendu-A (é, č, ł…), sur 2 octets comme sa capitale
constexpr unsigned lower_latin(unsigned c) {
    if (c >= 0xC0 && c <= 0xDE && c != 0xD7) {
        return c + 0x20; // À–Þ (sauf ×)
    }
    // Latin étendu-A : la capitale puis sa minuscule, paires décalées par ĸ (U+0138) et ŉ (U+0149) ; İ (U+0130) à part
    const bool capital = (c >= 0x100 && c <= 0x137 && c % 2 == 0 && c != 0x130)
        || (c >= 0x139 && c <= 0x148 && c % 2 == 1)
        || (c >= 0x14A && c <= 0x177 && c % 2 == 0)
        || (c >= 0x179 && c <= 0x17E && c % 2 == 1);
    return capital ? c + 1 : c;
}

/// Titres Prusa en capitales (« RÉGLAGES WI-FI ») → casse de la maquette G (« Réglages Wi-Fi »).
/// Un titre qui a déjà une minuscule est laissé tel quel ; quelques sigles et marques gardent leur casse.
/// nouns (allemand) : chaque mot garde sa capitale (« Sprache & Zeit »), sauf les petits mots et verbes de la liste.
inline void title_case(char *s, bool nouns = false) {
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
        { "X", "X" }, // axes (« DÉPLACEMENT DE L'AXE Z », écrans Prusa à l'en-tête G)
        { "Y", "Y" },
        { "Z", "Z" },
        { "XY", "XY" },
    };
    // Allemand : en minuscules malgré la capitale de chaque mot (articles, prépositions, verbes des titres de Prusa)
    static constexpr const char *small_de[] = {
        "DER", "DIE", "DAS", "DES", "DEM", "DEN", "UND", "ODER", "VON", "VOM", "MIT", "FÜR", "IM", "IN", "AM", "AN",
        "AUF", "ZU", "ZUM", "ZUR", "AUS", "BEI", "NACH", "BEWEGEN", "WÄHLEN", "AUSWÄHLEN", "KALIBRIEREN", "ABBRECHEN",
        "WECHSELN",
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
        bool keep_first = first;
        if (nouns) {
            keep_first = true;
            for (const char *w : small_de) {
                if (strlen(w) == len && strncmp(word, w, len) == 0) {
                    keep_first = first;
                }
            }
        }
        if (as_is) {
            memcpy(word, as_is, len);
        } else {
            // La première lettre reste en capitale (si elle est accentuée, i = 1 tombe sur son second octet, que rien ne
            // modifie)
            for (size_t i = keep_first ? 1 : 0; i < len; ++i) {
                char &c = word[i];
                if (nouns && i > 0 && word[i - 1] == '-') {
                    continue; // « Z-Achse », « Erste-Schicht-Kalibrierung »
                }
                if (c >= 'A' && c <= 'Z') {
                    c = static_cast<char>(c - 'A' + 'a');
                } else if ((c == '\xC3' || c == '\xC4' || c == '\xC5') && i + 1 < len) {
                    // Lettre sur 2 octets : U+00C0–U+017F
                    const unsigned l = lower_latin((static_cast<unsigned char>(c) & 0x1F) << 6 | (static_cast<unsigned char>(word[i + 1]) & 0x3F));
                    c = static_cast<char>(0xC0 | l >> 6);
                    word[i + 1] = static_cast<char>(0x80 | (l & 0x3F));
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
