/// @file
#include "thumbnail_od3d.hpp"

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
    , sums {} {}

WritableBytes ScaledQoiReader::read(WritableBytes buffer) {
    size_t n = 0;
    while (n < buffer.size() && (pending_pos < pending_len || next())) {
        buffer[n++] = pending[pending_pos++];
    }
    return buffer.first(n);
}

bool ScaledQoiReader::next() {
    pending_pos = pending_len = 0;

    if (!started) {
        started = true;
        std::array<std::byte, qoi::Decoder::HEADER_SIZE> header;
        if (params.factor == 0 || source.read(header).size() != header.size()) {
            return false;
        }
        src_w = qoi::Decoder::get_image_width(header);
        out_w = src_w / params.factor;
        out_h = qoi::Decoder::get_image_height(header) / params.factor;
        if (out_w == 0 || out_h == 0 || out_w > max_width) {
            return false;
        }
        pending = header;
        put_be32(&pending[4], out_w);
        put_be32(&pending[8], out_h);
        pending_len = header.size();
        out_x = out_w; // aucune ligne décodée : la première est à faire
        return true;
    }

    if (out_x == out_w) {
        if (out_y == out_h || !decode_band()) {
            return false;
        }
        out_x = 0;
    }

    const unsigned n = params.factor * params.factor;
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
    const uint32_t next_band = uint32_t(out_y + 1) * params.factor; // première ligne source de la ligne suivante
    while (src_y < next_band) {
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
        if (const unsigned x = src_x / params.factor; x < out_w) {
            Color px = Color::from_rgb(p.r, p.g, p.b);
            if (out_y < params.grey_rows) {
                const uint8_t l = (p.r * 77 + p.g * 150 + p.b * 29) >> 9; // luminance, puis moitié
                px = Color::from_rgb(l, l, l);
            }
            const Color c = Color::mix(params.back, px, p.a);
            sums[x][0] += c.r;
            sums[x][1] += c.g;
            sums[x][2] += c.b;
        }
        if (++src_x == src_w) {
            src_x = 0;
            ++src_y;
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

} // namespace od3d
