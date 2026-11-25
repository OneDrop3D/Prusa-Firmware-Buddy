// OneDrop3D : écriture du firmware du MMU3 dans son chargeur Caterina (AVR109). Le chargeur simulé suit Caterina.c paquet
// USB par paquet USB (16 octets) : une commande et ses arguments, puis le reste du paquet jeté (Endpoint_ClearOUT).
#include <feature/od3d/avr109_od3d.hpp>
#include <feature/od3d/mmu_flash_od3d.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <deque>
#include <optional>
#include <string_view>
#include <vector>

using namespace od3d::avr109;

namespace {

struct Caterina final : Port {
    std::vector<uint8_t> flash = std::vector<uint8_t>(app_size, 0x55);
    std::vector<uint8_t> image;
    std::deque<std::vector<uint8_t>> packets;
    std::deque<uint8_t> answer;
    size_t address = 0;
    std::vector<std::pair<size_t, uint8_t>> written; ///< (page, premier octet) dans l'ordre d'écriture
    std::optional<size_t> broken; ///< page qui se relit fausse
    bool mmctl = false; ///< chargeur « MMCTL20 » : « P » suivi de son nom
    bool mute = false;
    size_t done = 0, total = 0;

    bool send(std::span<const uint8_t> bytes) override {
        for (size_t i = 0; i < bytes.size(); i += 16) {
            packets.emplace_back(bytes.begin() + i, bytes.begin() + std::min(bytes.size(), i + 16));
        }
        process();
        return true;
    }

    bool receive(std::span<uint8_t> bytes, uint32_t) override {
        if (mute || answer.size() < bytes.size()) {
            return false;
        }
        for (auto &b : bytes) {
            b = answer.front();
            answer.pop_front();
        }
        return true;
    }

    void drain() override { answer.clear(); }

    bool read_page(size_t index, std::span<uint8_t, page_size> page) override {
        std::fill(page.begin(), page.end(), 0xFF);
        for (size_t i = 0; i < page_size && index * page_size + i < image.size(); ++i) {
            page[i] = image[index * page_size + i];
        }
        return true;
    }

    void progress(size_t d, size_t t) override {
        done = d;
        total = t;
    }

    void process() {
        for (;;) {
            std::vector<uint8_t> r;
            for (const auto &p : packets) {
                r.insert(r.end(), p.begin(), p.end());
            }
            if (r.empty()) {
                return;
            }
            size_t used = 1;
            std::vector<uint8_t> reply;
            const std::string_view name = mmctl ? "MMCTL20" : "CATERIN";
            switch (r[0]) {
            case 0x1b:
                break;
            case 'S':
                reply.assign(name.begin(), name.end());
                break;
            case 'b':
                reply = { 'Y', 0, 128 };
                break;
            case 's':
                reply = { 0x87, 0x95, 0x1E };
                break;
            case 'L':
            case 'E':
                reply = { '\r' };
                break;
            case 'P':
                if (mmctl && r.size() < 8) {
                    return;
                }
                used = mmctl ? 8 : 1;
                reply = { uint8_t(!mmctl || std::equal(r.begin() + 1, r.begin() + 8, name.begin()) ? '\r' : '?') };
                break;
            case 'A':
                if (r.size() < 3) {
                    return;
                }
                address = size_t(r[1]) << 9 | size_t(r[2]) << 1;
                used = 3;
                reply = { '\r' };
                break;
            case 'B': {
                const size_t n = r.size() >= 4 ? size_t(r[1]) << 8 | r[2] : 0;
                if (r.size() < 4 || r.size() < 4 + n) {
                    return;
                }
                std::fill_n(flash.begin() + address, page_size, 0xFF); // BootloaderAPI_ErasePage
                std::copy_n(r.begin() + 4, n, flash.begin() + address);
                if (broken == address / page_size) {
                    flash[address + 5] ^= 1;
                }
                written.emplace_back(address / page_size, flash[address]);
                used = 4 + n;
                reply = { '\r' };
                break;
            }
            case 'g': {
                if (r.size() < 4) {
                    return;
                }
                const size_t n = size_t(r[1]) << 8 | r[2];
                reply.assign(flash.begin() + address, flash.begin() + address + n);
                address += n;
                used = 4;
                break;
            }
            default:
                reply = { '?' };
            }
            // la commande et ses arguments, puis le reste de son dernier paquet
            while (used) {
                used -= std::min(used, packets.front().size());
                packets.pop_front();
            }
            answer.insert(answer.end(), reply.begin(), reply.end());
        }
    }
};

/// Vecteurs en page 0, des données en page 4 et dans la dernière page du firmware
std::vector<uint8_t> image() {
    std::vector<uint8_t> v(0x6E84, 0xFF);
    v[0] = 0x0C;
    v[1] = 0x94;
    std::fill_n(v.begin() + 0x200, 16, 9);
    std::fill_n(v.begin() + 0x6E80, 4, 3);
    return v;
}

constexpr size_t pages_of(size_t bytes) {
    return (bytes + page_size - 1) / page_size;
}

} // namespace

TEST_CASE("avr109: one command per USB packet, as Caterina reads them") {
    Caterina mmu;
    const uint8_t esc_s[] = { 0x1b, 'S' };
    mmu.send(esc_s); // Échap et « S » dans le même paquet : « S » est jeté, rien ne répond (PrintForge avant le 22/11)
    CHECK(mmu.answer.empty());
    CHECK(mmu.packets.empty());
}

TEST_CASE("avr109: page 0 erased first and written last, everything read back") {
    Caterina mmu;
    mmu.image = image();
    const size_t pages = pages_of(mmu.image.size());
    const auto report = flash(mmu, pages);
    REQUIRE(report.result == Result::ok);
    CHECK(std::equal(mmu.image.begin(), mmu.image.end(), mmu.flash.begin()));
    REQUIRE(mmu.written.size() == pages + 1);
    CHECK(mmu.written.front() == std::pair<size_t, uint8_t> { 0, 0xFF });
    CHECK(mmu.written.back() == std::pair<size_t, uint8_t> { 0, 0x0C });
    CHECK(mmu.flash[pages * page_size] == 0x55); // au-delà du firmware, rien n'est touché
    CHECK((mmu.done == 2 * pages && mmu.total == 2 * pages));
    CHECK((mmu.packets.empty() && mmu.answer.empty()));
}

TEST_CASE("avr109: a wrong read-back stops with page 0 still blank") {
    Caterina mmu;
    mmu.image = image();
    mmu.broken = 4;
    const auto report = flash(mmu, pages_of(mmu.image.size()));
    CHECK(report.result == Result::verify_failed);
    CHECK(report.address == 0x200);
    CHECK((mmu.flash[0] == 0xFF && mmu.flash[1] == 0xFF)); // Caterina reste dans son chargeur : on peut recommencer
}

TEST_CASE("avr109: MMCTL20 bootloader gets P followed by its name") {
    Caterina mmu;
    mmu.mmctl = true;
    mmu.image = image();
    CHECK(flash(mmu, pages_of(mmu.image.size())).result == Result::ok);
    CHECK(mmu.flash[0] == 0x0C);
}

TEST_CASE("avr109: a silent bootloader is reported on S, nothing written") {
    Caterina mmu;
    mmu.mute = true;
    mmu.image = image();
    const auto report = flash(mmu, pages_of(mmu.image.size()));
    CHECK(report.result == Result::no_answer);
    CHECK(report.command == 'S');
    CHECK(mmu.written.empty());
}

TEST_CASE("mmu_flash: only the MMU firmware and its Caterina bootloader are taken") {
    using od3d::mmu_flash::Device;
    using od3d::mmu_flash::usb_device;
    CHECK(usb_device(0x2c99, 0x0004) == Device::firmware);
    CHECK(usb_device(0x2c99, 0x0003) == Device::bootloader); // mode de secours : MMU entré par son bouton reset
    CHECK(usb_device(0x2c99, 0x0007) == Device::none); // chargeur d'un autre appareil Prusa
    CHECK(usb_device(0x2341, 0x0036) == Device::none); // chargeur Caterina d'Arduino : même protocole, autre carte
    CHECK(usb_device(0x0000, 0x0004) == Device::none);
}

TEST_CASE("mmu_flash: the IN data toggle follows Caterina through its empty packets") {
    // Caterina bascule DATA0/DATA1 à chaque paquet envoyé, paquet vide compris ; la HAL de ST en DMA ne compte que les
    // paquets de données (stm32f4xx_hal_hcd.c, XFRC) ; fill() ajoute le paquet vide
    using od3d::mmu_flash::ended_by_empty_packet;
    constexpr size_t packet = 16, asked = 160; // CDC_TXRX_EPSIZE de Caterina, ce que demande fill()
    size_t device = 0, host = 0; // paquets comptés de chaque côté : leur parité est le basculement
    for (const size_t n : { 0, 7, 3, 1, 3, 1, 1, 1, 128, 1, 1 }) { // Échap (paquet vide seul), S, b, P, s, A, B, A, g, L, E
        device += n / packet + 1; // une réponse finit par un paquet court, vide après un paquet plein
        host += (n + packet - 1) / packet + ended_by_empty_packet(n, asked, packet);
        CHECK(host % 2 == device % 2);
    }
    CHECK_FALSE(ended_by_empty_packet(asked, asked, packet)); // tout reçu : le paquet vide vient seul au transfert suivant
    CHECK(ended_by_empty_packet(0, 128, 64));
}
