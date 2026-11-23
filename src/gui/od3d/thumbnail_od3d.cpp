/// @file
#include "thumbnail_od3d.hpp"
#include "spool_draw.hpp"

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
    if (const uint8_t q = corner_quarters(out_x, out_y)) {
        c = spool::mix_quarters(c, params.outside, q); // coin lissé, comme les cartes (il était en escalier)
    }
    pending[0] = std::byte(0xfe); // QOI_OP_RGB : le pixel en clair
    pending[1] = std::byte(c.r);
    pending[2] = std::byte(c.g);
    pending[3] = std::byte(c.b);
    pending_len = 4;
    if (record) {
        record->push(c);
    }

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
            // Le fond (transparent) et l'objet (opaque) font presque toute l'image : sans Color::mix, même résultat.
            // Une image déjà réduite (cache, relue à l'échelle 1) est opaque : son fond reste le fond, ni gris ni assombri.
            Color c = transparent;
            Color px = Color::from_rgb(p.r, p.g, p.b);
            if (p.a != 0 && !(p.a == 255 && px == transparent)) {
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

uint8_t ScaledQoiReader::corner_quarters(int x, int y) const {
    // Même règle que display::draw_rounded_rect() : (0, 0) au coin extérieur du carré de coin
    const int r = params.radius;
    const int cx = x < r ? x : (x >= out_w - r ? out_w - 1 - x : -1);
    const int cy = y < r ? y : (y >= out_h - r ? out_h - 1 - y : -1);
    return cx >= 0 && cy >= 0 ? spool::corner_beyond_quarters(cx, cy, r) : 0;
}

// ---- Images gardées ----

void QoiCache::clear() {
    count = used = 0;
    recording = false;
}

void QoiCache::start(uint32_t key, uint16_t w, uint16_t h) {
    cancel();
    for (size_t i = 0; i < count; ++i) {
        if (Entry &e = entries[(first + i) % max_entries]; e.key == key) {
            e.key = 0; // ses octets partiront à leur tour
        }
    }
    if (pool.empty()) {
        return;
    }
    rec = { .key = key, .w = w, .h = h, .start = count ? (entries[first].start + used) % pool.size() : 0, .len = 0 };
    if (!count) {
        used = 0; // tampon vide : l'image commence au début
    }
    recording = true;
    pixels = 0;
    prev = Color::from_rgb(0, 0, 0); // comme le décodeur : noir opaque, index vide
    run = 0;
    index = {};
}

void QoiCache::push(Color c) {
    if (!recording) {
        return;
    }
    ++pixels;
    // Encodeur QOI de référence (qoi.h), pixels opaques seulement : série, index, petite différence, luma, RGB
    if (c == prev) {
        if (++run == 62) {
            flush_run();
        }
        return;
    }
    flush_run();
    const uint8_t h = (c.r * 3 + c.g * 5 + c.b * 7 + 255 * 11) & 0x3f;
    const uint32_t packed = c.r | c.g << 8 | c.b << 16 | 0xffu << 24;
    if (index[h] == packed) {
        put(h); // QOI_OP_INDEX
    } else {
        index[h] = packed;
        const int dr = int8_t(c.r - prev.r), dg = int8_t(c.g - prev.g), db = int8_t(c.b - prev.b); // modulo 256
        const int dr_dg = dr - dg, db_dg = db - dg;
        if (dr >= -2 && dr <= 1 && dg >= -2 && dg <= 1 && db >= -2 && db <= 1) {
            put(0x40 | (dr + 2) << 4 | (dg + 2) << 2 | (db + 2)); // QOI_OP_DIFF
        } else if (dg >= -32 && dg <= 31 && dr_dg >= -8 && dr_dg <= 7 && db_dg >= -8 && db_dg <= 7) {
            put(0x80 | (dg + 32)); // QOI_OP_LUMA
            put((dr_dg + 8) << 4 | (db_dg + 8));
        } else {
            put(0xfe); // QOI_OP_RGB
            put(c.r);
            put(c.g);
            put(c.b);
        }
    }
    prev = c;
}

void QoiCache::flush_run() {
    if (run) {
        put(0xc0 | (run - 1)); // QOI_OP_RUN
        run = 0;
    }
}

void QoiCache::put(uint8_t byte) {
    if (!recording) {
        return;
    }
    while (used == pool.size()) {
        if (!count) {
            cancel(); // l'image seule ne tient pas dans le tampon
            return;
        }
        drop_oldest();
    }
    pool[(rec.start + rec.len) % pool.size()] = std::byte(byte);
    ++rec.len;
    ++used;
}

void QoiCache::drop_oldest() {
    used -= entries[first].len;
    first = (first + 1) % max_entries;
    --count;
}

void QoiCache::finish() {
    flush_run();
    if (!recording || pixels != uint32_t(rec.w) * rec.h) {
        cancel(); // la source s'est arrêtée avant la fin
        return;
    }
    if (count == max_entries) {
        drop_oldest();
    }
    entries[(first + count++) % max_entries] = rec;
    recording = false;
}

void QoiCache::cancel() {
    if (recording) {
        recording = false;
        used -= rec.len;
    }
}

std::optional<QoiCache::Reader> QoiCache::find(uint32_t key) const {
    for (size_t i = 0; i < count; ++i) {
        if (entries[(first + i) % max_entries].key == key) {
            return Reader(*this, (first + i) % max_entries);
        }
    }
    return std::nullopt;
}

QoiCache::Reader::Reader(const QoiCache &cache, size_t entry)
    : cache(cache)
    , start(cache.entries[entry].start)
    , len(cache.entries[entry].len) {
    constexpr std::array<char, 4> magic { 'q', 'o', 'i', 'f' };
    for (size_t i = 0; i < magic.size(); ++i) {
        header[i] = std::byte(magic[i]);
    }
    put_be32(&header[4], cache.entries[entry].w);
    put_be32(&header[8], cache.entries[entry].h);
    header[12] = std::byte(3); // RGB
    header[13] = std::byte(0);
}

WritableBytes QoiCache::Reader::read(WritableBytes buffer) {
    size_t n = 0;
    for (; n < buffer.size() && pos < header.size() + len; ++n, ++pos) {
        buffer[n] = pos < header.size() ? header[pos] : cache.pool[(start + pos - header.size()) % cache.pool.size()];
    }
    return buffer.first(n);
}

// ---- Image calculée ----

void store_pixels(uint8_t *out, Rect16 rect, Rect16 part, PixelFn pixel, const void *ctx) {
    for (int y = part.Top(); y <= part.Bottom(); ++y) {
        for (int x = part.Left(); x <= part.Right(); ++x) {
            const Color c = pixel(ctx, x - rect.Left(), y - rect.Top());
            *out++ = c.b;
            *out++ = c.g;
            *out++ = c.r;
        }
    }
}

} // namespace od3d
