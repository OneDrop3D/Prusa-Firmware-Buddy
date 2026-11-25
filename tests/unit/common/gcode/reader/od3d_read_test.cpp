/// OneDrop3D (AUDIT_PERF_IMPRIMER.md, P-01 et P-03) : un G-code s'ouvre d'abord (une seule recherche dans le dossier) sans
/// changer l'ouverture d'un envoi en cours ; la vignette se lit pareil après le passage au tampon recalé sur les secteurs
#include "test_files.hpp"

#include <gcode_reader_any.hpp>
#include <common/thumbnail_sizes.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {

/// Octets de la vignette d'aperçu (QOI) du fichier. realigned : cherchée avec un tampon de 64 octets, lue ensuite avec
/// 4 Ko recalés sur le secteur, comme la grille des fichiers
std::vector<std::byte> preview_bytes(const char *filename, bool realigned) {
    AnyGcodeFormatReader reader(filename);
    REQUIRE(reader.is_open());
    auto *common = static_cast<GcodeReaderCommon *>(reader.get());
    if (realigned) {
        common->set_read_buffer(64);
    }
    AbstractByteReader *thumbnail = reader->stream_thumbnail_start(thumbnail_sizes::preview_thumbnail_width, thumbnail_sizes::preview_thumbnail_height, IGcodeReader::ImgType::QOI);
    REQUIRE(thumbnail != nullptr);
    if (realigned) {
        REQUIRE(common->set_stream_buffer(4096));
    }
    std::vector<std::byte> bytes;
    std::array<std::byte, 61> chunk; // pas un diviseur de 512 : des lectures à cheval sur deux secteurs
    for (auto got = thumbnail->read(chunk); !got.empty(); got = thumbnail->read(chunk)) {
        bytes.insert(bytes.end(), got.begin(), got.end());
    }
    return bytes;
}

} // namespace

TEST_CASE("od3d: start of the sector of a read position", "[GcodeReader]") {
    STATIC_REQUIRE(GcodeReaderCommon::sector_start(0) == 0);
    STATIC_REQUIRE(GcodeReaderCommon::sector_start(511) == 0);
    STATIC_REQUIRE(GcodeReaderCommon::sector_start(512) == 512);
    STATIC_REQUIRE(GcodeReaderCommon::sector_start(4097) == 4096);
}

TEST_CASE("od3d: a thumbnail reads the same after the buffer is realigned", "[GcodeReader]") {
    for (const char *filename : { NEW_BINARY, NEW_PLAIN }) {
        SECTION(filename) {
            const std::vector<std::byte> plain = preview_bytes(filename, false);
            CHECK(plain.size() > 512);
            CHECK(preview_bytes(filename, true) == plain);
        }
    }
}

TEST_CASE("od3d: a transfer in progress opens its partial file", "[GcodeReader]") {
    // Comme un envoi par le réseau : un dossier au nom du G-code, le fichier partiel dedans
    char name[L_tmpnam + 16];
    REQUIRE(tmpnam(name) != nullptr);
    strcat(name, ".gcode");
    REQUIRE(mkdir(name, 0700) == 0);
    const std::string partial = std::string(name) + "/" + transfers::partial_filename;
    {
        FILE *f = fopen(partial.c_str(), "wb");
        REQUIRE(f != nullptr);
        fputs("G28 ; partial\n", f);
        fclose(f);
    }
    {
        AnyGcodeFormatReader reader(name);
        REQUIRE(reader.is_open());
        REQUIRE(reader->stream_gcode_start() == IGcodeReader::Result_t::RESULT_OK);
        GcodeBuffer buffer;
        REQUIRE(reader->stream_get_line(buffer, IGcodeReader::Continuations::Discard) == IGcodeReader::Result_t::RESULT_OK);
        CHECK(std::string_view(buffer.line.begin, buffer.line.end).starts_with("G28"));
    }
    unlink(partial.c_str());
    rmdir(name);
}

TEST_CASE("od3d: a missing file does not open", "[GcodeReader]") {
    AnyGcodeFormatReader reader("od3d_missing_file.gcode");
    CHECK_FALSE(reader.is_open());
}
