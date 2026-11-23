// OneDrop3D : dessin d'une passe (images calculées, fond entre les éléments, couleur du focus, cache des vignettes).
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
