// OneDrop3D : dessin d'une passe (images calculées en flux QOI, fond entre les éléments, couleur du focus).
#include <gui/od3d/style_od3d.hpp>
#include <gui/od3d/thumbnail_od3d.hpp>

#include <array>
#include <catch2/catch_test_macros.hpp>
#include <vector>

using namespace od3d;

namespace {

Color pattern(int x, int y) {
    if (y == 0) {
        return Color::from_raw(0x26282B); // une ligne d'une seule couleur : séries de plus de 62 pixels
    }
    if (y == 1) {
        return (x / 3) % 2 ? Color::from_raw(0xF8651B) : Color::from_raw(0x000000);
    }
    return Color::from_rgb(uint8_t(x * 3), uint8_t(255 - x), 255); // une couleur par pixel, 255 compris
}

} // namespace

TEST_CASE("image calculée : relue pixel pour pixel par le décodeur QOI de Prusa") {
    constexpr uint16_t w = 70, h = 3;
    PixelQoiReader reader(w, h, [](const void *, int x, int y) { return pattern(x, y); }, nullptr);

    // Comme display::draw_img() : l'en-tête, puis le flux par morceaux
    std::array<std::byte, qoi::Decoder::HEADER_SIZE> header;
    REQUIRE(reader.read(header).size() == header.size());
    CHECK(qoi::Decoder::get_image_width(header) == w);
    CHECK(qoi::Decoder::get_image_height(header) == h);

    qoi::Decoder decoder;
    std::vector<qoi::Pixel> pixels;
    std::array<std::byte, 7> chunk; // petit, pour couper les opérations entre deux lectures
    for (WritableBytes got = reader.read(chunk); !got.empty(); got = reader.read(chunk)) {
        for (std::byte b : got) {
            decoder.push(b);
            while (decoder.has_pixel()) {
                pixels.push_back(decoder.pull_pixel());
            }
        }
    }
    CHECK_FALSE(decoder.is_error());
    REQUIRE(pixels.size() == size_t(w) * h);

    // L'écran mélange chaque pixel au fond avec une opacité de 255 ((255 × v) >> 8 = v - 1) : le flux envoie v + 1
    const auto shown = [](uint8_t v) { return uint8_t((255 * v) >> 8); };
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const Color want = pattern(x, y);
            const qoi::Pixel &got = pixels[y * w + x];
            INFO("x " << x << " y " << y);
            CHECK((shown(got.r) & 0xFC) == (want.r & 0xFC));
            CHECK((shown(got.g) & 0xFC) == (want.g & 0xFC));
            CHECK((shown(got.b) & 0xFC) == (want.b & 0xFC));
        }
    }
}

TEST_CASE("image calculée : vide") {
    PixelQoiReader reader(0, 5, [](const void *, int, int) { return Color::from_raw(0); }, nullptr);
    std::array<std::byte, 32> buffer;
    CHECK(reader.read(buffer).empty());
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
