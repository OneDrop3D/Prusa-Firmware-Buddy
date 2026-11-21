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
