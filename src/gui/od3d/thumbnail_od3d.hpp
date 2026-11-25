/// @file
/// OneDrop3D (F1, interface G) : vignette d'un G-code réduite au décodage (SPEC_G.md §1.13). Les .bgcode n'ont que des
/// vignettes 16×16, 313×173, 440×240 et 480×240 : on lit la 313×173 (celle de l'aperçu Prusa) et on la réduit, au tiers
/// pour la grille des fichiers (104×57), aux 3/5 pour l'accueil (187×103), en faisant la moyenne de chaque case.
#pragma once

#include <array>
#include <cstdint>
#include <gui/qoi_decoder.hpp>
#include <optional>
#include <Rect16.h>
#include <span>
#include <utils/abstract_byte_reader.hpp>
#include <utils/color.hpp>

namespace od3d {

class QoiCache;

/// Relit une image QOI réduite d'un rapport num/factor (moyenne des pixels de chaque case), posée sur un fond
/// (transparence), coins arrondis et lissés. À l'échelle 1, relit une image gardée (QoiCache) en grisant le haut. Le résultat est encore un flux QOI : display::draw_img() l'affiche sans rien garder en
/// mémoire. band() le coupe en bandes de quelques lignes : la source reste ouverte entre deux et l'interface répond
/// pendant la lecture (une vignette de la clé prend ~0,1 s).
class ScaledQoiReader final : public AbstractByteReader {
public:
    static constexpr uint16_t max_width = 192; ///< largeur maximale de l'image réduite [px]

    struct Params {
        uint8_t factor; ///< diviseur : 2 = moitié de la largeur et de la hauteur
        Color back; ///< fond sous les pixels transparents
        uint8_t radius; ///< rayon des coins arrondis [px]
        Color outside; ///< couleur hors des coins arrondis
        uint16_t grey_rows = 0; ///< lignes du haut de l'image réduite en gris à 50 % (écran Impression : pas encore imprimé)
        uint8_t num = 1; ///< multiplicateur, au plus factor : 3 avec factor 5 = 3/5 de la taille (accueil)
    };

    ScaledQoiReader(AbstractByteReader &source, const Params &params);

    /// Lit l'en-tête de la source ; false si l'image est illisible ou trop large. Fait au premier read() sinon.
    bool open();

    uint16_t width() const { return out_w; } ///< après open()
    uint16_t height() const { return out_h; }
    uint16_t row() const { return out_y; } ///< première ligne de la prochaine bande
    bool done() const { return out_y >= out_h; }

    /// Le flux suivant (read) est une image de rows lignes au plus, à partir de row(), à dessiner à la ligne row() de
    /// l'image ; false s'il n'y a plus rien. Sans appel : toute l'image d'un coup.
    bool band(uint16_t rows);

    WritableBytes read(WritableBytes buffer) override;

    QoiCache *record = nullptr; ///< reçoit aussi chaque pixel envoyé (cache des vignettes de la grille)

private:
    /// Prépare le morceau suivant (en-tête ou un pixel) ; false à la fin de la bande
    bool next();

    /// Décode les lignes de la source qui forment la ligne out_y ; false si la source s'arrête avant
    bool decode_band();

    /// Quarts (0 à 4) du pixel (x, y) hors des coins arrondis
    uint8_t corner_quarters(int x, int y) const;

    /// Première ligne (ou colonne) de la source de la ligne réduite i : ceil(i × factor / num)
    uint16_t first_source(uint16_t i) const { return uint16_t((uint32_t(i) * params.factor + params.num - 1) / params.num); }

    AbstractByteReader &source;
    Params params;
    Color transparent; ///< un pixel transparent posé sur le fond, comme Color::mix(back, ..., 0)
    qoi::Decoder decoder;
    std::array<std::byte, 64> input;
    uint8_t input_pos = 0, input_len = 0;
    std::array<std::byte, qoi::Decoder::HEADER_SIZE> header; ///< celui de la source
    std::array<std::byte, qoi::Decoder::HEADER_SIZE> pending; ///< en-tête, puis un pixel QOI_OP_RGB à la fois
    uint8_t pending_pos = 0, pending_len = 0;
    bool opened = false, readable = false, header_sent = false;
    uint8_t rows_in_line = 0; ///< lignes de la source moyennées dans la ligne out_y
    uint16_t src_w = 0, src_x = 0, src_y = 0;
    uint16_t col = 0, col_next = 0; ///< colonne réduite de src_x, et première colonne de la source de la suivante
    uint16_t out_w = 0, out_h = 0, out_x = 0, out_y = 0, band_end = 0;
    std::array<std::array<uint16_t, 3>, max_width> sums; ///< somme R, V, B des pixels de chaque colonne de la ligne en cours
};

/// Images déjà décodées (vignettes réduites de la grille des fichiers), gardées compressées en QOI dans un tampon fourni :
/// une rangée qui revient se redessine sans relire la clé (~2,5 Ko par vignette 104 × 57 au lieu de 11,9 Ko en RGB565).
/// Les plus anciennes sont chassées par les nouvelles. Une image s'enregistre pixel par pixel pendant son premier dessin.
class QoiCache {
public:
    explicit QoiCache(std::span<std::byte> pool)
        : pool(pool) {}

    void clear();

    /// Place pour les images [o]
    size_t capacity() const { return pool.size(); }

    /// Commence l'image key de w × h pixels (une autre en cours est abandonnée, l'ancienne image key oubliée)
    void start(uint32_t key, uint16_t w, uint16_t h);
    void push(Color c); ///< pixel suivant de l'image en cours (rien sans image en cours)
    void finish(); ///< l'image en cours est gardée, si elle est complète et a tenu dans le tampon
    void cancel();

    /// Relit une image gardée en flux QOI (en-tête compris), pour display::draw_img()
    class Reader final : public AbstractByteReader {
    public:
        WritableBytes read(WritableBytes buffer) override;

    private:
        friend class QoiCache;
        Reader(const QoiCache &cache, size_t entry);
        const QoiCache &cache;
        std::array<std::byte, qoi::Decoder::HEADER_SIZE> header;
        size_t start, len, pos = 0; ///< données dans le tampon (circulaire) ; pos compte l'en-tête
    };

    /// L'image key, si elle est gardée
    std::optional<Reader> find(uint32_t key) const;

private:
    struct Entry {
        uint32_t key; ///< 0 : place oubliée (ses octets restent jusqu'à leur tour)
        uint16_t w, h;
        size_t start, len;
    };
    static constexpr size_t max_entries = 16;

    void put(uint8_t byte);
    void flush_run();
    void drop_oldest();

    std::span<std::byte> pool;
    std::array<Entry, max_entries> entries {}; ///< dans l'ordre du tampon : la plus ancienne à first
    size_t first = 0, count = 0;
    size_t used = 0; ///< octets des images gardées et de celle en cours, à partir de la plus ancienne
    // Image en cours
    bool recording = false;
    Entry rec {};
    uint32_t pixels = 0;
    Color prev;
    uint8_t run = 0;
    std::array<uint32_t, 64> index {}; ///< couleurs déjà vues (RGB + alpha 255, 0 : vide), comme le décodeur
};

/// Pixel (x, y) d'une image calculée (bobines, anneau qui tourne, courbes, contours), relatif à l'image
using PixelFn = Color (*)(const void *ctx, int x, int y);

/// Pixels de part (absolu, dans rect) de l'image pixel posée en rect, ligne par ligne, en octets de l'afficheur
/// (bleu, vert, rouge : comme display::fill_rect) dans out
void store_pixels(uint8_t *out, Rect16 rect, Rect16 part, PixelFn pixel, const void *ctx);

/// Image calculée envoyée à l'écran par le tampon de l'afficheur, quelques lignes à la fois, sans effacement préalable
/// (pas de clignotement). Retour du 23/11 (« dans l'Atelier, d'abord les bobines puis le reste ») : elle passait par un
/// flux QOI codé puis décodé et mélangé au fond, les deux tiers du calcul des bobines. Défini dans widgets_od3d.cpp.
void draw_computed(Rect16 rect, PixelFn pixel, const void *ctx);

/// Dessine dans rect l'image f(x, y) → Color (x, y relatifs à rect)
template <class F>
void draw_pixels(Rect16 rect, const F &f) {
    draw_computed(rect, [](const void *ctx, int x, int y) { return (*static_cast<const F *>(ctx))(x, y); }, &f);
}

} // namespace od3d
