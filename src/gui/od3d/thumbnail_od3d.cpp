/// @file
#include "thumbnail_od3d.hpp"

#include <algorithm>

namespace od3d {

namespace {

    void put_be32(std::byte *p, uint32_t v) {
        p[0] = std::byte(v >> 24);
        p[1] = std::byte(v >> 16);
        p[2] = std::byte(v >> 8);
        p[3] = std::byte(v);
    }

} // namespace

ScaledQoiReader::ScaledQoiReader(AbstractByteReader &source, const Params &params)
    : source(source)
    , params(params)
    , transparent(Color::mix(params.back, params.back, 0))
    , sums {} {}

bool ScaledQoiReader::open() {
    if (opened) {
        return readable;
    }
    opened = true;
    if (params.factor == 0 || params.num == 0 || params.num > params.factor || source.read(header).size() != header.size()) {
        return false;
    }
    src_w = qoi::Decoder::get_image_width(header);
    out_w = uint32_t(src_w) * params.num / params.factor;
    out_h = uint32_t(qoi::Decoder::get_image_height(header)) * params.num / params.factor;
    if (out_w == 0 || out_h == 0 || out_w > max_width) {
        out_w = out_h = 0;
        return false;
    }
    col_next = first_source(1);
    out_x = out_w; // aucune ligne décodée : la première est à faire
    band_end = out_h;
    readable = true;
    return true;
}

bool ScaledQoiReader::band(uint16_t rows) {
    if (!open() || done()) {
        return false;
    }
    band_end = std::min<uint16_t>(out_y + rows, out_h);
    header_sent = false;
    pending_pos = pending_len = 0;
    return true;
}

WritableBytes ScaledQoiReader::read(WritableBytes buffer) {
    size_t n = 0;
    while (n < buffer.size() && (pending_pos < pending_len || next())) {
        buffer[n++] = pending[pending_pos++];
    }
    return buffer.first(n);
}

bool ScaledQoiReader::next() {
    pending_pos = pending_len = 0;

    if (!header_sent) {
        if (!open() || out_y >= band_end) {
            return false;
        }
        header_sent = true;
        pending = header;
        put_be32(&pending[4], out_w);
        put_be32(&pending[8], band_end - out_y);
        pending_len = pending.size();
        return true;
    }

    if (out_x == out_w) {
        if (out_y >= band_end || !decode_band()) {
            return false;
        }
        out_x = 0;
    }

    const unsigned n = (first_source(out_x + 1) - first_source(out_x)) * rows_in_line;
    auto &sum = sums[out_x];
    Color c = Color::from_rgb((sum[0] + n / 2) / n, (sum[1] + n / 2) / n, (sum[2] + n / 2) / n);
    sum = {};
    if (outside_corner(out_x, out_y)) {
        c = params.outside;
    }
    pending[0] = std::byte(0xfe); // QOI_OP_RGB : le pixel en clair
    pending[1] = std::byte(c.r);
    pending[2] = std::byte(c.g);
    pending[3] = std::byte(c.b);
    pending_len = 4;

    if (++out_x == out_w) {
        ++out_y;
    }
    return true;
}

bool ScaledQoiReader::decode_band() {
    const uint16_t next_line = first_source(out_y + 1); // première ligne source de la ligne réduite suivante
    rows_in_line = next_line - src_y;
    const bool grey = out_y < params.grey_rows;
    while (src_y < next_line) {
        while (!decoder.has_pixel()) {
            if (decoder.is_error()) {
                return false;
            }
            if (input_pos == input_len) {
                const WritableBytes got = source.read(input);
                if (got.empty()) {
                    return false;
                }
                input_pos = 0;
                input_len = got.size();
            }
            decoder.push(input[input_pos++]);
        }

        const qoi::Pixel p = decoder.pull_pixel();
        if (col < out_w) {
            // Le fond (transparent) et l'objet (opaque) font presque toute l'image : sans Color::mix, même résultat
            Color c = transparent;
            if (p.a != 0) {
                Color px = Color::from_rgb(p.r, p.g, p.b);
                if (grey) {
                    const uint8_t l = (p.r * 77 + p.g * 150 + p.b * 29) >> 9; // luminance, puis moitié
                    px = Color::from_rgb(l, l, l);
                }
                c = p.a == 255 ? Color::from_rgb((255 * px.r) >> 8, (255 * px.g) >> 8, (255 * px.b) >> 8) : Color::mix(params.back, px, p.a);
            }
            sums[col][0] += c.r;
            sums[col][1] += c.g;
            sums[col][2] += c.b;
        }
        if (++src_x == src_w) {
            src_x = 0;
            ++src_y;
            col = 0;
            col_next = first_source(1);
        } else if (src_x == col_next) {
            ++col;
            col_next = first_source(col + 1);
        }
    }
    return true;
}

bool ScaledQoiReader::outside_corner(int x, int y) const {
    // Même règle que display::draw_rounded_rect() : dedans si dx² + dy² < r²
    const int r = params.radius;
    const int dx = x < r ? r - 1 - x : (x >= out_w - r ? x - (out_w - r) : -1);
    const int dy = y < r ? r - 1 - y : (y >= out_h - r ? y - (out_h - r) : -1);
    return dx >= 0 && dy >= 0 && dx * dx + dy * dy >= r * r;
}

// ---- Image calculée ----

PixelQoiReader::PixelQoiReader(uint16_t w, uint16_t h, Pixel pixel, const void *ctx)
    : pixel(pixel)
    , ctx(ctx)
    , w(w)
    , h(h) {}

WritableBytes PixelQoiReader::read(WritableBytes buffer) {
    size_t n = 0;
    while (n < buffer.size() && (pending_pos < pending_len || next())) {
        buffer[n++] = pending[pending_pos++];
    }
    return buffer.first(n);
}

Color PixelQoiReader::take() {
    Color c = peeked ? *peeked : pixel(ctx, pos % w, pos / w);
    peeked.reset();
    ++pos;
    return c;
}

bool PixelQoiReader::next() {
    pending_pos = pending_len = 0;
    const uint32_t total = uint32_t(w) * h;

    if (!started) {
        started = true;
        if (total == 0) {
            return false;
        }
        constexpr std::array<char, 4> magic { 'q', 'o', 'i', 'f' };
        for (size_t i = 0; i < magic.size(); ++i) {
            pending[i] = std::byte(magic[i]);
        }
        put_be32(&pending[4], w);
        put_be32(&pending[8], h);
        pending[12] = std::byte(3); // RGB
        pending[13] = std::byte(0);
        pending_len = qoi::Decoder::HEADER_SIZE;
        return true;
    }
    if (pos >= total) {
        return false;
    }

    const Color c = take();
    if (last && c == *last) {
        // QOI_OP_RUN : le pixel précédent, 1 à 62 fois
        uint8_t run = 1;
        while (run < 62 && pos < total) {
            peeked = pixel(ctx, pos % w, pos / w);
            if (*peeked != c) {
                break;
            }
            peeked.reset();
            ++pos;
            ++run;
        }
        pending[0] = std::byte(0xc0 | (run - 1));
        pending_len = 1;
        return true;
    }

    // QOI_OP_RGB. L'écran mélange chaque pixel au fond avec une opacité de 255 (Color::mix), ce qui retire 1 à chaque
    // canal : +1 le compense, et le pixel tombe sur la même couleur qu'un fill_rect (les 6 bits hauts de chaque canal).
    last = c;
    const auto up = [](uint8_t v) { return std::byte(v == 255 ? 255 : v + 1); };
    pending[0] = std::byte(0xfe);
    pending[1] = up(c.r);
    pending[2] = up(c.g);
    pending[3] = up(c.b);
    pending_len = 4;
    return true;
}

} // namespace od3d
