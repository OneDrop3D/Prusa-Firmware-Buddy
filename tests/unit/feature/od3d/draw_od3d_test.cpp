// OneDrop3D : dessin d'une passe (images calculées, fond entre les éléments, couleur du focus, cache des vignettes).
#include <gui/od3d/spool_draw.hpp>
#include <gui/od3d/style_od3d.hpp>
#include <gui/od3d/thumbnail_od3d.hpp>

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <iterator>
#include <optional>
#include <vector>

using namespace od3d;

namespace {

/// Source QOI calculée pixel par pixel, pour les tests des vignettes (l'ancien chemin des images calculées du firmware :
/// séries et pixels RGB, chaque canal + 1 car l'écran mélange au fond avec une opacité de 255)
class PixelQoiReader final : public AbstractByteReader {
public:
    using Pixel = Color (*)(const void *ctx, int x, int y);

    PixelQoiReader(uint16_t w, uint16_t h, Pixel pixel, const void *ctx)
        : pixel(pixel)
        , ctx(ctx)
        , w(w)
        , h(h) {}

    WritableBytes read(WritableBytes buffer) override {
        size_t n = 0;
        while (n < buffer.size() && (pending_pos < pending.size() || next())) {
            buffer[n++] = pending[pending_pos++];
        }
        return buffer.first(n);
    }

private:
    bool next() {
        pending.clear();
        pending_pos = 0;
        const uint32_t total = uint32_t(w) * h;
        if (!started) {
            started = true;
            if (total == 0) {
                return false;
            }
            for (char c : { 'q', 'o', 'i', 'f' }) {
                pending.push_back(std::byte(c));
            }
            for (uint32_t v : { uint32_t(w), uint32_t(h) }) {
                for (int shift = 24; shift >= 0; shift -= 8) {
                    pending.push_back(std::byte(v >> shift));
                }
            }
            pending.push_back(std::byte(3)); // RGB
            pending.push_back(std::byte(0));
            return true;
        }
        if (pos >= total) {
            return false;
        }
        const Color c = pixel(ctx, pos % w, pos / w);
        ++pos;
        if (last && c == *last) {
            uint8_t run = 1;
            while (run < 62 && pos < total && pixel(ctx, pos % w, pos / w) == c) {
                ++pos;
                ++run;
            }
            pending.push_back(std::byte(0xc0 | (run - 1)));
            return true;
        }
        last = c;
        const auto up = [](uint8_t v) { return std::byte(v == 255 ? 255 : v + 1); };
        pending.insert(pending.end(), { std::byte(0xfe), up(c.r), up(c.g), up(c.b) });
        return true;
    }

    Pixel pixel;
    const void *ctx;
    uint16_t w, h;
    uint32_t pos = 0;
    std::optional<Color> last;
    bool started = false;
    std::vector<std::byte> pending;
    size_t pending_pos = 0;
};

} // namespace

TEST_CASE("image calculée : les pixels de la partie, ligne par ligne, en octets de l'afficheur") {
    // Image de 10 × 6 posée en (100, 50) ; partie de 3 × 2 qui commence à son pixel (4, 2)
    constexpr Rect16 rect = { 100, 50, 10, 6 };
    constexpr Rect16 part = { 104, 52, 3, 2 };
    std::array<uint8_t, 3 * 3 * 2 + 1> out {};
    out.back() = 0xAA; // rien n'est écrit après la partie
    store_pixels(out.data(), rect, part, [](const void *, int x, int y) { return Color::from_rgb(uint8_t(x), uint8_t(y), 200); }, nullptr);
    size_t i = 0;
    for (int y = 2; y < 4; ++y) {
        for (int x = 4; x < 7; ++x) {
            INFO("x " << x << " y " << y);
            CHECK(out[i++] == 200); // bleu d'abord, comme display::fill_rect
            CHECK(out[i++] == y);
            CHECK(out[i++] == x);
        }
    }
    CHECK(out.back() == 0xAA);
}

TEST_CASE("fond entre les éléments : chaque pixel peint une fois, jamais sous un élément") {
    constexpr Rect16 area = { 0, 0, 100, 50 };
    const std::array<Rect16, 5> rects { {
        { 10, 5, 20, 10 },
        { 50, 0, 30, 50 },
        { 5, 40, 10, 20 }, // dépasse en bas
        { 15, 8, 20, 4 }, // chevauche le premier
        {}, // vide
    } };
    std::array<std::array<int, 100>, 50> painted {};
    style::for_each_gap(area, rects, [&](Rect16 r) {
        for (int y = r.Top(); y <= r.Bottom(); ++y) {
            for (int x = r.Left(); x <= r.Right(); ++x) {
                REQUIRE((x >= 0 && x < 100 && y >= 0 && y < 50));
                ++painted[y][x];
            }
        }
    });
    for (int y = 0; y < 50; ++y) {
        for (int x = 0; x < 100; ++x) {
            bool covered = false;
            for (const Rect16 &r : rects) {
                covered = covered || (!r.IsEmpty() && x >= r.Left() && x <= r.Right() && y >= r.Top() && y <= r.Bottom());
            }
            INFO("x " << x << " y " << y);
            CHECK(painted[y][x] == (covered ? 0 : 1));
        }
    }
}

TEST_CASE("focus : un ton plus clair, la ligne ciblée de la maquette pour une carte") {
    CHECK(style::focused(style::surface) == Color::from_raw(0x34373C));
    CHECK(style::focused(style::accent) != style::accent);
    CHECK(style::focused(COLOR_WHITE) == COLOR_WHITE);
}

namespace {

/// Décode un flux QOI comme display::draw_img() : l'en-tête, puis les pixels
std::vector<qoi::Pixel> decode(AbstractByteReader &reader, uint32_t &w, uint32_t &h) {
    std::array<std::byte, qoi::Decoder::HEADER_SIZE> header;
    w = h = 0;
    if (reader.read(header).size() != header.size()) {
        return {};
    }
    w = qoi::Decoder::get_image_width(header);
    h = qoi::Decoder::get_image_height(header);
    qoi::Decoder decoder;
    std::vector<qoi::Pixel> pixels;
    std::array<std::byte, 5> chunk;
    for (WritableBytes got = reader.read(chunk); !got.empty(); got = reader.read(chunk)) {
        for (std::byte b : got) {
            decoder.push(b);
            while (decoder.has_pixel()) {
                pixels.push_back(decoder.pull_pixel());
            }
        }
    }
    return pixels;
}

Color stripes(int x, int y) {
    return (x + y) % 2 ? COLOR_WHITE : COLOR_BLACK; // damier : la moyenne d'une case paire est un gris moyen
}

Color gradient(int x, int y) {
    return Color::from_rgb(uint8_t(x * 8), uint8_t(y * 20), uint8_t(200 - x));
}

} // namespace

TEST_CASE("vignette réduite : taille au tiers et aux 3/5") {
    PixelQoiReader source(313, 173, [](const void *, int x, int y) { return gradient(x % 30, y % 12); }, nullptr);
    ScaledQoiReader third(source, { .factor = 3, .back = COLOR_BLACK, .radius = 0, .outside = COLOR_BLACK });
    REQUIRE(third.open());
    CHECK(third.width() == 104);
    CHECK(third.height() == 57);

    PixelQoiReader source2(313, 173, [](const void *, int x, int y) { return gradient(x % 30, y % 12); }, nullptr);
    ScaledQoiReader home(source2, { .factor = 5, .back = COLOR_BLACK, .radius = 0, .outside = COLOR_BLACK, .num = 3 });
    uint32_t w, h;
    const auto pixels = decode(home, w, h);
    CHECK(w == 187);
    CHECK(h == 103);
    CHECK(pixels.size() == size_t(w) * h); // toute l'image, d'un coup
}

TEST_CASE("vignette réduite : moyenne de chaque case") {
    PixelQoiReader source(8, 6, [](const void *, int x, int y) { return stripes(x, y); }, nullptr);
    ScaledQoiReader half(source, { .factor = 2, .back = COLOR_BLACK, .radius = 0, .outside = COLOR_BLACK });
    uint32_t w, h;
    const auto pixels = decode(half, w, h);
    REQUIRE(w == 4);
    REQUIRE(h == 3);
    REQUIRE(pixels.size() == 12);
    for (const qoi::Pixel &p : pixels) {
        CHECK(int(p.r) >= 125); // moitié noir, moitié blanc
        CHECK(int(p.r) <= 129);
    }

    // 3/5 : des cases de 2 et de 1 pixels de côté ; une image unie reste unie
    PixelQoiReader plain(10, 5, [](const void *, int, int) { return Color::from_rgb(200, 100, 40); }, nullptr);
    ScaledQoiReader ratio(plain, { .factor = 5, .back = COLOR_BLACK, .radius = 0, .outside = COLOR_BLACK, .num = 3 });
    const auto same = decode(ratio, w, h);
    REQUIRE(w == 6);
    REQUIRE(h == 3);
    REQUIRE(same.size() == 18);
    for (const qoi::Pixel &p : same) {
        CHECK(int(p.r) == 200);
        CHECK(int(p.g) == 100);
        CHECK(int(p.b) == 40);
    }
}

TEST_CASE("vignette réduite : par bandes, les mêmes pixels que d'un coup") {
    const auto make = [](PixelQoiReader &src) {
        return ScaledQoiReader(src, { .factor = 5, .back = COLOR_BLACK, .radius = 4, .outside = COLOR_WHITE, .grey_rows = 7, .num = 3 });
    };
    PixelQoiReader source_a(100, 60, [](const void *, int x, int y) { return gradient(x, y); }, nullptr);
    ScaledQoiReader whole = make(source_a);
    uint32_t w, h;
    const auto reference = decode(whole, w, h);
    REQUIRE(w == 60);
    REQUIRE(h == 36);

    PixelQoiReader source_b(100, 60, [](const void *, int x, int y) { return gradient(x, y); }, nullptr);
    ScaledQoiReader bands = make(source_b);
    std::vector<qoi::Pixel> joined;
    int count = 0;
    while (bands.band(10)) {
        const uint16_t row = bands.row();
        uint32_t bw, bh;
        const auto part = decode(bands, bw, bh);
        CHECK(bw == w);
        CHECK(bh == std::min<uint32_t>(10, h - row));
        CHECK(bands.row() == row + bh);
        joined.insert(joined.end(), part.begin(), part.end());
        ++count;
    }
    CHECK(count == 4); // 10 + 10 + 10 + 6 lignes
    CHECK(bands.done());
    REQUIRE(joined.size() == reference.size());
    for (size_t i = 0; i < joined.size(); ++i) {
        INFO("pixel " << i);
        CHECK(joined[i].r == reference[i].r);
        CHECK(joined[i].g == reference[i].g);
        CHECK(joined[i].b == reference[i].b);
    }
}

TEST_CASE("vignette réduite : image trop large ou illisible") {
    PixelQoiReader wide(400, 10, [](const void *, int, int) { return COLOR_BLACK; }, nullptr);
    ScaledQoiReader too_wide(wide, { .factor = 2, .back = COLOR_BLACK, .radius = 0, .outside = COLOR_BLACK }); // 200 > 192
    CHECK_FALSE(too_wide.open());
    CHECK_FALSE(too_wide.band(5));
    std::array<std::byte, 8> buffer;
    CHECK(too_wide.read(buffer).empty());
}

TEST_CASE("cache des vignettes : relue telle qu'elle a été dessinée, par bandes, sans relire la source") {
    std::vector<std::byte> pool(16 * 1024);
    QoiCache cache(pool);

    // Vignette dessinée par bandes (comme la grille), chaque pixel envoyé aussi au cache
    PixelQoiReader source(313, 173, [](const void *, int x, int y) { return y < 40 ? Color::from_raw(0x22242A) : gradient(x % 31, y % 13); }, nullptr);
    ScaledQoiReader bands(source, { .factor = 3, .back = COLOR_BLACK, .radius = 0, .outside = COLOR_BLACK });
    REQUIRE(bands.open());
    cache.start(7, bands.width(), bands.height());
    bands.record = &cache;
    std::vector<qoi::Pixel> drawn;
    while (bands.band(8)) {
        uint32_t bw, bh;
        const auto part = decode(bands, bw, bh);
        drawn.insert(drawn.end(), part.begin(), part.end());
    }
    CHECK_FALSE(cache.find(7).has_value()); // pas avant la fin
    cache.finish();

    auto image = cache.find(7);
    REQUIRE(image);
    uint32_t w, h;
    const auto again = decode(*image, w, h);
    CHECK(w == 104);
    CHECK(h == 57);
    REQUIRE(again.size() == drawn.size());
    for (size_t i = 0; i < again.size(); ++i) {
        INFO("pixel " << i);
        CHECK(again[i].r == drawn[i].r);
        CHECK(again[i].g == drawn[i].g);
        CHECK(again[i].b == drawn[i].b);
        CHECK(again[i].a == 255);
    }
    CHECK_FALSE(cache.find(8).has_value());
}

TEST_CASE("cache des vignettes : les plus anciennes partent, une image incomplète ou trop grande n'est pas gardée") {
    std::vector<std::byte> pool(1000);
    QoiCache cache(pool);
    // Un pixel sur deux d'une autre couleur : ~4 octets par pixel, 120 pixels ≈ 480 octets ; deux tiennent, pas trois
    const auto record = [&](uint32_t key, uint32_t pixels) {
        cache.start(key, 12, 10);
        for (uint32_t i = 0; i < pixels; ++i) {
            cache.push(Color::from_rgb(uint8_t(key * 40 + i * 7), uint8_t(i * 13), uint8_t(i % 2 ? 250 : 3)));
        }
        cache.finish();
    };
    record(1, 120);
    record(2, 120);
    CHECK(cache.find(1).has_value());
    CHECK(cache.find(2).has_value());
    record(3, 120);
    CHECK_FALSE(cache.find(1).has_value());
    CHECK(cache.find(2).has_value());
    CHECK(cache.find(3).has_value());

    record(4, 50); // incomplète (la source s'est arrêtée)
    CHECK_FALSE(cache.find(4).has_value());
    CHECK(cache.find(3).has_value());

    // Après plusieurs tours du tampon circulaire, l'image se relit toujours entière
    for (uint32_t key = 10; key < 20; ++key) {
        record(key, 120);
    }
    auto image = cache.find(19);
    REQUIRE(image);
    uint32_t w, h;
    const auto pixels = decode(*image, w, h);
    REQUIRE(pixels.size() == 120);
    for (uint32_t i = 0; i < 120; ++i) {
        CHECK(pixels[i].b == uint8_t(i % 2 ? 250 : 3));
    }

    std::vector<std::byte> tiny(100);
    QoiCache small(tiny);
    small.start(5, 12, 10);
    for (int i = 0; i < 120; ++i) {
        small.push(Color::from_rgb(uint8_t(i * 7), uint8_t(i * 13), uint8_t(i % 2 ? 250 : 3)));
    }
    small.finish();
    CHECK_FALSE(small.find(5).has_value());

    record(2, 120);
    cache.clear();
    CHECK_FALSE(cache.find(2).has_value());
}

TEST_CASE("vignette réduite : coins lissés comme les cartes") {
    // Image unie à l'échelle 1, rayon 8 : chaque pixel d'un coin est le mélange en quarts de draw_rounded_rect
    constexpr int w = 30, h = 20, r = 8;
    PixelQoiReader plain(w, h, [](const void *, int, int) { return Color::from_rgb(200, 100, 40); }, nullptr);
    ScaledQoiReader rounded(plain, { .factor = 1, .back = COLOR_BLACK, .radius = r, .outside = COLOR_WHITE });
    uint32_t ow, oh;
    const auto pixels = decode(rounded, ow, oh);
    REQUIRE(pixels.size() == size_t(w * h));
    const Color inside = Color::from_rgb(200, 100, 40);
    int smoothed = 0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const int cx = x < r ? x : (x >= w - r ? w - 1 - x : -1);
            const int cy = y < r ? y : (y >= h - r ? h - 1 - y : -1);
            const uint8_t q = cx >= 0 && cy >= 0 ? spool::corner_beyond_quarters(cx, cy, r) : 0;
            const Color expected = spool::mix_quarters(inside, COLOR_WHITE, q);
            const qoi::Pixel &p = pixels[y * w + x];
            INFO("x " << x << " y " << y);
            CHECK(Color::from_rgb(p.r, p.g, p.b) == expected);
            smoothed += q > 0 && q < 4;
        }
    }
    CHECK(smoothed >= 4 * 4); // des pixels mélangés dans chaque coin, plus d'escalier
}

TEST_CASE("cartes : les colonnes 10 et 11 d'un coin de rayon 12 sont pleines (texte des tuiles à 10 px du bord)") {
    for (int y = 0; y < 12; ++y) {
        CHECK(spool::corner_beyond_quarters(10, y, 12) == 0);
        CHECK(spool::corner_beyond_quarters(11, y, 12) == 0);
    }
}

TEST_CASE("image gardée relue à l'échelle 1 : quelques lignes, le haut grisé, le fond intact") {
    // Comme l'écran Impression : la vignette 156 × 86 gardée en couleur, un pas repeint quelques lignes grisées ou non
    std::vector<std::byte> pool(16 * 1024);
    QoiCache cache(pool);
    const Color back = Color::from_raw(0x22242A);
    PixelQoiReader source(313, 173, [](const void *, int x, int y) { return x > 100 && x < 200 && y > 40 ? gradient(x % 29, y % 11) : COLOR_BLACK; }, nullptr);
    // La source a un fond opaque noir ici : le fond « transparent » est celui des pixels transparents, ajouté à la main
    ScaledQoiReader full(source, { .factor = 2, .back = back, .radius = 0, .outside = back });
    REQUIRE(full.open());
    cache.start(1, full.width(), full.height());
    full.record = &cache;
    uint32_t w, h;
    const auto colour = decode(full, w, h);
    cache.finish();
    REQUIRE(w == 156);
    REQUIRE(h == 86);

    // Le fond d'une image gardée : un pixel exactement « transparent » (le fond mélangé à lui-même)
    const Color transparent = Color::mix(back, back, 0);
    std::vector<std::byte> pool2(16 * 1024);
    QoiCache with_back(pool2);
    with_back.start(2, 6, 4);
    for (int i = 0; i < 24; ++i) {
        with_back.push(i % 3 ? transparent : Color::from_rgb(180, 90, 30));
    }
    with_back.finish();

    constexpr uint16_t grey_rows = 2, first = 1, rows = 2;
    auto image = with_back.find(2);
    REQUIRE(image);
    ScaledQoiReader again(*image, { .factor = 1, .back = back, .radius = 0, .outside = back, .grey_rows = grey_rows });
    REQUIRE(again.open());
    REQUIRE(again.band(first));
    std::array<std::byte, 64> skip;
    while (!again.read(skip).empty()) {
    }
    REQUIRE(again.band(rows));
    uint32_t bw, bh;
    const auto part = decode(again, bw, bh);
    CHECK(bw == 6);
    CHECK(bh == rows);
    REQUIRE(part.size() == 6 * rows);
    for (int i = 0; i < 6 * rows; ++i) {
        const int src = (first + i / 6) * 6 + i % 6;
        const Color p = Color::from_rgb(part[i].r, part[i].g, part[i].b);
        INFO("pixel " << i);
        if (src % 3) {
            CHECK(p == transparent); // ni gris ni assombri : pas de cadre autour de l'image
        } else if (first + i / 6 < grey_rows) {
            CHECK(p.r == p.g); // grisé
            CHECK(p.g == p.b);
        } else {
            CHECK(p.r > p.b); // en couleur
        }
    }

    // L'image gardée relue en entier à l'échelle 1, sans gris : les mêmes lignes que le premier dessin, à 1 près
    auto stored = cache.find(1);
    REQUIRE(stored);
    ScaledQoiReader same(*stored, { .factor = 1, .back = back, .radius = 0, .outside = back });
    const auto replay = decode(same, w, h);
    REQUIRE(replay.size() == colour.size());
    for (size_t i = 0; i < replay.size(); ++i) {
        INFO("pixel " << i);
        CHECK(int(colour[i].r) - int(replay[i].r) <= 1);
        CHECK(int(colour[i].r) - int(replay[i].r) >= 0);
    }
}

TEST_CASE("dessin d'une passe : de haut en bas, chaque élément avec le fond à sa hauteur, une seule fois") {
    constexpr Rect16 area = { 0, 0, 100, 60 };
    const std::array<Rect16, 5> rects { {
        { 0, 0, 100, 10 }, // en-tête
        { 10, 30, 30, 20 }, // carte du bas, inscrite avant celle du haut
        { 10, 14, 30, 12 },
        { 50, 12, 40, 40 },
        { 52, 10, 10, 10 }, // chevauche le précédent, commence plus haut : peint après lui
    } };
    std::vector<int> order; // index d'élément, ou -1 - y d'une bande du fond
    std::array<std::array<int, 100>, 60> painted {};
    style::paint_top_down(
        area, rects, [&](size_t i) { order.push_back(int(i)); },
        [&](Rect16 r) {
            // Les éléments qui commencent à cette hauteur ou au-dessus sont déjà peints
            for (size_t i = 0; i < rects.size(); ++i) {
                if (rects[i].Top() <= r.Top()) {
                    INFO("bande " << r.Top() << ", élément " << i);
                    CHECK(std::find(order.begin(), order.end(), int(i)) != order.end());
                }
            }
            for (int y = r.Top(); y <= r.Bottom(); ++y) {
                for (int x = r.Left(); x <= r.Right(); ++x) {
                    ++painted[y][x];
                }
            }
            order.push_back(-1 - r.Top());
        });

    // Chaque élément une fois ; le haut avant le bas ; un élément après celui, plus ancien, qu'il chevauche
    std::vector<int> elements;
    std::copy_if(order.begin(), order.end(), std::back_inserter(elements), [](int v) { return v >= 0; });
    CHECK(elements == std::vector<int> { 0, 3, 4, 2, 1 });
    // Fond : chaque pixel hors des éléments une fois
    for (int y = 0; y < 60; ++y) {
        for (int x = 0; x < 100; ++x) {
            bool covered = false;
            for (const Rect16 &r : rects) {
                covered = covered || (x >= r.Left() && x <= r.Right() && y >= r.Top() && y <= r.Bottom());
            }
            INFO("x " << x << " y " << y);
            CHECK(painted[y][x] == (covered ? 0 : 1));
        }
    }
}

TEST_CASE("dessin en onde : depuis l'élément touché, chaque élément et chaque pixel du fond une fois") {
    constexpr Rect16 area = { 0, 0, 100, 60 };
    const std::array<Rect16, 5> rects { {
        { 0, 0, 100, 10 }, // en-tête
        { 10, 30, 30, 20 },
        { 10, 14, 30, 12 }, // le plus loin du point
        { 50, 12, 40, 40 }, // contient le point
        { 52, 10, 10, 10 }, // chevauche le précédent, plus récent : peint après lui
    } };
    constexpr point_i16_t from { 80, 50 };
    std::vector<int> order;
    std::vector<int> fill_dist; // distance² de chaque bande du fond, dans l'ordre
    std::array<std::array<int, 100>, 60> painted {};
    style::paint_from(
        area, rects, from, [&](size_t i) { order.push_back(int(i)); },
        [&](Rect16 r) {
            fill_dist.push_back(style::distance2(r, from));
            for (int y = r.Top(); y <= r.Bottom(); ++y) {
                for (int x = r.Left(); x <= r.Right(); ++x) {
                    ++painted[y][x];
                }
            }
        });

    // L'élément touché d'abord, puis anneau par anneau ; 4, qui chevauche 3, après lui ; le plus loin en dernier
    CHECK(order == std::vector<int> { 3, 0, 1, 4, 2 });
    // Les bandes du fond partent du point : anneau après anneau, jamais un plus proche après un plus loin
    const auto ring = [](int d2) {
        int k = 0;
        while (d2 > k * style::wave_step * k * style::wave_step) {
            ++k;
        }
        return k;
    };
    for (size_t i = 1; i < fill_dist.size(); ++i) {
        CHECK(ring(fill_dist[i]) >= ring(fill_dist[i - 1]));
    }
    for (int y = 0; y < 60; ++y) {
        for (int x = 0; x < 100; ++x) {
            bool covered = false;
            for (const Rect16 &r : rects) {
                covered = covered || (x >= r.Left() && x <= r.Right() && y >= r.Top() && y <= r.Bottom());
            }
            INFO("x " << x << " y " << y);
            CHECK(painted[y][x] == (covered ? 0 : 1));
        }
    }
}

TEST_CASE("carte peinte autour de son contenu : chaque pixel hors du contenu une fois, coins seulement dans leurs carrés") {
    for (const uint8_t bw : { uint8_t(0), uint8_t(2) }) {
        constexpr Rect16 card = { 10, 20, 60, 40 };
        constexpr uint8_t r = 12;
        const std::array<Rect16, 3> parts { {
            { 16, 25, 22, 22 }, // tuile
            { 40, 27, 28, 16 }, // titre, jusqu'au bord droit moins le contour
            { 14, 50, 50, 16 }, // déborde sous la carte : coupé
        } };
        std::array<std::array<int, 80>, 70> painted {};
        std::array<std::array<char, 80>, 70> kind {};
        const auto mark = [&](Rect16 p, char k) {
            for (int y = p.Top(); y <= p.Bottom(); ++y) {
                for (int x = p.Left(); x <= p.Right(); ++x) {
                    ++painted[y][x];
                    kind[y][x] = k;
                }
            }
        };
        style::for_each_frame_piece(card, r, bw, parts, [&](Rect16 p, bool inside) { mark(p, inside ? 'f' : 'b'); }, [&](Rect16 p) { mark(p, 'c'); });
        for (int y = 0; y < 70; ++y) {
            for (int x = 0; x < 80; ++x) {
                const bool in_card = x >= card.Left() && x <= card.Right() && y >= card.Top() && y <= card.Bottom();
                bool in_part = false;
                for (const Rect16 &p : parts) {
                    in_part = in_part || (x >= p.Left() && x <= p.Right() && y >= p.Top() && y <= p.Bottom());
                }
                INFO("bw " << int(bw) << " x " << x << " y " << y);
                CHECK(painted[y][x] == (in_card && !in_part ? 1 : 0));
                if (painted[y][x]) {
                    const int cx = std::min(x - card.Left(), card.Right() - x), cy = std::min(y - card.Top(), card.Bottom() - y);
                    CHECK((kind[y][x] == 'c') == (cx < r && cy < r)); // coins lissés : seulement leurs carrés
                    CHECK((kind[y][x] == 'b') == (kind[y][x] != 'c' && (cx < bw || cy < bw))); // contour droit
                }
            }
        }
    }
}

TEST_CASE("températures : l'arrondi ne bouge qu'au-delà de 0,7 °C d'écart") {
    CHECK(style::held_degrees(215.4f, 215) == 215);
    CHECK(style::held_degrees(215.6f, 215) == 215); // oscillation autour de x,5 : rien ne change
    CHECK(style::held_degrees(214.4f, 215) == 215);
    CHECK(style::held_degrees(215.8f, 215) == 216);
    CHECK(style::held_degrees(214.2f, 215) == 214);
    CHECK(style::held_degrees(23.4f, 0) == 23); // première mesure
    CHECK(style::held_degrees(-1.0f, 23) == -1); // plus de mesure
}
