/// @file
/// OneDrop3D (F1, interface G) : vignette d'un G-code réduite d'un facteur entier au décodage (SPEC_G.md §1.13).
/// Les .bgcode n'ont que des vignettes 16×16, 313×173, 440×240 et 480×240 : l'accueil lit la 313×173
/// (celle de l'aperçu Prusa) et la réduit de moitié, 156×86, en faisant la moyenne de chaque carré de pixels.
#pragma once

#include <array>
#include <cstdint>
#include <gui/qoi_decoder.hpp>
#include <utils/abstract_byte_reader.hpp>
#include <utils/color.hpp>

namespace od3d {

/// Relit une image QOI réduite d'un facteur entier, posée sur un fond (transparence), coins arrondis.
/// Le résultat est encore un flux QOI : display::draw_img() l'affiche sans rien garder en mémoire.
class ScaledQoiReader final : public AbstractByteReader {
public:
    static constexpr uint16_t max_width = 160; ///< largeur maximale de l'image réduite [px]

    struct Params {
        uint8_t factor; ///< 2 = moitié de la largeur et de la hauteur
        Color back; ///< fond sous les pixels transparents
        uint8_t radius; ///< rayon des coins arrondis [px]
        Color outside; ///< couleur hors des coins arrondis
    };

    ScaledQoiReader(AbstractByteReader &source, const Params &params);

    WritableBytes read(WritableBytes buffer) override;

private:
    /// Prépare le morceau suivant (en-tête ou un pixel) ; false à la fin de l'image
    bool next();

    /// Décode les lignes de la source qui forment la ligne out_y ; false si la source s'arrête avant
    bool decode_band();

    bool outside_corner(int x, int y) const;

    AbstractByteReader &source;
    Params params;
    qoi::Decoder decoder;
    std::array<std::byte, 64> input;
    uint8_t input_pos = 0, input_len = 0;
    std::array<std::byte, qoi::Decoder::HEADER_SIZE> pending; ///< en-tête, puis un pixel QOI_OP_RGB à la fois
    uint8_t pending_pos = 0, pending_len = 0;
    bool started = false;
    uint16_t src_w = 0, src_x = 0, src_y = 0;
    uint16_t out_w = 0, out_h = 0, out_x = 0, out_y = 0;
    std::array<std::array<uint16_t, 3>, max_width> sums; ///< somme R, V, B des pixels de chaque colonne de la ligne en cours
};

} // namespace od3d
