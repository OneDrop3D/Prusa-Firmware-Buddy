/// @file
/// OneDrop3D (lot UI-0, AUDIT_UI.md §6) : mesure de l'interface sur la machine. Chaque dessin de l'écran
/// (Screens::Draw) laisse une ligne dans un anneau en RAM : durée (compteur de cycles DWT), octets SPI, fenêtres et
/// attente du DMA (compteurs du pilote ili9488), latence depuis l'entrée (molette ou toucher) qui l'a demandé. Les
/// dessins sans entrée qui se suivent sur le même écran (animations) sont cumulés dans une ligne, une minute au plus.
/// M2010 ajoute l'anneau à /usb/OD3D_UI.CSV puis le vide ; utils/od3d/decode_ui_csv.py nomme les écrans avec l'ELF de
/// la CI (artefact elf-pile). L'écran est l'adresse de sa fonction de création (ScreenFactory::Screen<T>, une par
/// type) : les vtables des menus sont fusionnées par l'éditeur de liens (une vingtaine à la même adresse dans l'ELF
/// de integ-35). Un dialogue au-dessus est l'adresse de sa vtable. Partie pure (cumul des lignes) testée seule.
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace od3d::perf {

/// Compteurs du pilote de l'écran, cumulés depuis le démarrage (fil de l'interface seulement)
struct Counters {
    uint32_t bytes = 0; ///< octets envoyés sur le SPI (pixels et paramètres des commandes)
    uint32_t windows = 0; ///< fenêtres ouvertes (CASET)
    uint32_t dma_cycles = 0; ///< cycles passés à attendre la fin d'un envoi DMA
};
extern Counters counters;

/// Compteur de cycles du processeur (DWT->CYCCNT)
uint32_t cycles();

enum class Cause : uint8_t {
    none, ///< animation, LOOP, invalidation sans entrée
    knob,
    touch,
    usb, ///< lecture de la clé par l'onglet Imprimer (Step), pas un dessin
    heap, ///< tas libre à l'ouverture de la grille des fichiers
};

/// Demandes à la clé USB (USBH_read), cumulées depuis le démarrage, toutes tâches confondues (AUDIT_PERF_IMPRIMER.md)
struct Usb {
    uint32_t commands = 0; ///< demandes envoyées à la clé
    uint32_t sectors = 0; ///< secteurs lus (512 octets)
    uint32_t us = 0; ///< temps passé à attendre la clé [µs]
    uint32_t split = 0; ///< demandes d'un secteur dues à un tampon décalé (une seule demande aurait suffi)
};
extern Usb usb;

inline Usb operator-(const Usb &a, const Usb &b) {
    return { a.commands - b.commands, a.sectors - b.sectors, a.us - b.us, a.split - b.split };
}

/// Étapes de l'onglet Imprimer : colonne screen des lignes « u » (plus 256 × les demandes d'un secteur de split)
enum class Step : uint8_t {
    heap = 0, ///< ligne « h » : tas libre à l'ouverture de la grille
    folder = 1, ///< lecture du dossier
    card = 2, ///< une carte : ouverture, métadonnées, recherche et bandes de sa vignette
    page = 3, ///< page complète : de son affichage à sa dernière vignette
    preview = 4, ///< « Avant l'impression » : métadonnées et nom, avant son premier dessin
    preview_image = 5, ///< image d'« Avant l'impression »
};

/// Une ligne de l'anneau : un dessin, ou plusieurs dessins sans entrée cumulés
struct Row {
    uint32_t t_ms; ///< début du premier dessin (ticks_ms)
    uint32_t screen; ///< fonction de création de l'écran, ou vtable du dialogue du dessus
    uint32_t draw_us;
    uint32_t bytes;
    uint32_t dma_wait_us;
    uint16_t windows;
    uint16_t span_ms; ///< du début du premier dessin à la fin du dernier
    uint16_t input_10us; ///< entrée → fin du dessin, en 10 µs (655 ms au plus) ; 0 sans entrée
    uint8_t draws;
    Cause cause;
};

inline constexpr uint32_t merge_max_ms = 60000; ///< une ligne d'animation couvre une minute au plus

/// Ajoute next (fin à end_ms) à last si ce sont deux dessins sans entrée du même écran, dans la même minute et sous
/// 255 dessins ; sinon false, et next prend une nouvelle ligne
inline bool merge(Row &last, const Row &next, uint32_t end_ms) {
    if (last.cause != Cause::none || next.cause != Cause::none || last.screen != next.screen || last.draws == UINT8_MAX
        || end_ms - last.t_ms > merge_max_ms) {
        return false;
    }
    last.draw_us += next.draw_us;
    last.bytes += next.bytes;
    last.dma_wait_us += next.dma_wait_us;
    last.windows = uint16_t(std::min<uint32_t>(uint32_t(last.windows) + next.windows, UINT16_MAX));
    last.span_ms = uint16_t(end_ms - last.t_ms);
    ++last.draws;
    return true;
}

/// Ajoute la tranche piece à row si c'est la même étape commencée au même moment (une carte lue en plusieurs tranches) :
/// travail, octets, demandes et attente s'additionnent ; la plus longue tranche (input_10us) et la fin (span_ms) sont
/// gardées. false si row est une autre ligne
inline bool merge_step(Row &row, const Row &piece) {
    if (row.cause != Cause::usb || piece.cause != Cause::usb || row.t_ms != piece.t_ms || (row.screen & 0xff) != (piece.screen & 0xff)
        || row.draws == UINT8_MAX) {
        return false;
    }
    row.screen = (row.screen & 0xff) | (((row.screen >> 8) + (piece.screen >> 8)) << 8);
    row.draw_us += piece.draw_us;
    row.bytes += piece.bytes;
    row.dma_wait_us += piece.dma_wait_us;
    row.windows = uint16_t(std::min<uint32_t>(uint32_t(row.windows) + piece.windows, UINT16_MAX));
    row.span_ms = std::max(row.span_ms, piece.span_ms);
    row.input_10us = std::max(row.input_10us, piece.input_10us);
    ++row.draws;
    return true;
}

/// Une tranche de l'étape step commencée à start_ms : us de travail (l'écran ne répondait pas), delta lu sur la clé.
/// Ajoutée à la ligne de son étape si elle est parmi les dernières de l'anneau, sinon une ligne « u » de plus
void note(Step step, uint32_t start_ms, uint32_t us, const Usb &delta);

/// Ligne « h » : tas libre à l'ouverture de la grille, avant et après ses blocs ; caches : bits 1 fenêtre du dossier,
/// 2 métadonnées, 4 vignettes (thumbs_kb Ko de place)
void note_heap(uint32_t free_before, uint32_t free_after, uint8_t caches, uint8_t thumbs_kb);

/// Entrée reçue (molette ou toucher) : la latence court jusqu'à la fin du prochain dessin
void input(Cause cause);

/// Entrées reçues depuis le démarrage : une animation qui en voit une de plus se termine aussitôt
uint32_t inputs();

/// L'entrée n'a rien demandé à redessiner : pas de latence à mesurer
void drop_input();

void draw_begin();

/// Fin du dessin de screen (voir Row::screen) : une ligne de plus, sauf si rien n'a été envoyé
void draw_end(uint32_t screen);

/// Pointeur de vtable d'un objet polymorphe (son premier mot)
inline uint32_t vtable_of(const void *object) {
    uint32_t vptr;
    memcpy(&vptr, object, sizeof(vptr));
    return vptr;
}

/// Ajoute l'anneau au CSV path (en-tête si le fichier est neuf), puis le vide ; false si l'écriture échoue. Fil de
/// Marlin (M2010), hors impression : les dessins faits pendant l'écriture ne sont pas gardés.
bool dump(const char *path);

} // namespace od3d::perf
