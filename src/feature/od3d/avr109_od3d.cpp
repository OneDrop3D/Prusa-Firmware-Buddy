/// @file
#include "avr109_od3d.hpp"

#include <algorithm>
#include <array>
#include <cstring>

namespace od3d::avr109 {

namespace {

    constexpr uint32_t timeout_ms = 1'000; ///< Caterina répond en quelques ms, une page écrite comprise

    struct Session {
        Port &port;
        Report report {};

        bool fail(Result result, char command, size_t address = 0) {
            report = { result, command, uint16_t(address) };
            return false;
        }

        bool send(std::span<const uint8_t> bytes) {
            return port.send(bytes) || fail(Result::no_answer, char(bytes[0]));
        }

        bool receive(std::span<uint8_t> bytes, char command) {
            return port.receive(bytes, timeout_ms) || fail(Result::no_answer, command);
        }

        /// Commande à laquelle le chargeur répond par un retour chariot
        bool command(std::span<const uint8_t> bytes) {
            uint8_t answer = 0;
            return send(bytes) && receive({ &answer, 1 }, char(bytes[0])) && (answer == '\r' || fail(Result::refused, char(bytes[0])));
        }

        bool ask(char c, std::span<uint8_t> answer) {
            const uint8_t byte = c;
            return send({ &byte, 1 }) && receive(answer, c);
        }

        /// « A » : l'adresse en mots de 16 bits, poids fort d'abord
        bool address(size_t byte) {
            const uint16_t word = byte / 2;
            const std::array<uint8_t, 3> a { 'A', uint8_t(word >> 8), uint8_t(word) };
            return command(a);
        }
    };

    /// Le nom du chargeur (7 lettres), demandé comme avrdude : Échap seul, vidage, puis « S », tant qu'il ne répond pas
    /// ou répond « ? ». Un chargeur qui vient d'apparaître peut manquer les premières demandes.
    bool identify(Session &s, std::array<uint8_t, 7> &name) {
        for (int attempt = 0; attempt < 5; ++attempt) {
            const uint8_t esc = 0x1b;
            if (!s.send({ &esc, 1 })) {
                return false;
            }
            s.port.drain();
            const uint8_t ask = 'S';
            if (!s.send({ &ask, 1 })) {
                return false;
            }
            if (s.port.receive({ name.data(), 1 }, timeout_ms) && name[0] != '?') {
                return s.receive({ name.data() + 1, name.size() - 1 }, 'S');
            }
        }
        return s.fail(Result::no_answer, 'S');
    }

    /// Nom, pages de 128 octets, mode programmation et signature de l'ATmega32U4
    bool present(Session &s) {
        std::array<uint8_t, 7> name;
        std::array<uint8_t, 3> block;
        std::array<uint8_t, 3> signature;
        if (!identify(s, name) || !s.ask('b', block)) {
            return false;
        }
        if (block[0] != 'Y' || size_t(block[1] << 8 | block[2]) < page_size) {
            return s.fail(Result::not_paged, 'b');
        }
        // Le chargeur « MMCTL20 » des MMU3 récents veut « P » suivi de son nom (src/mmu2/mmu2_bootloader.cpp) ; dans le même
        // envoi, un chargeur qui ne lit que « P » jette le reste
        static constexpr uint8_t mmctl[] = { 'P', 'M', 'M', 'C', 'T', 'L', '2', '0' };
        const bool is_mmctl = std::equal(name.begin(), name.end(), mmctl + 1);
        if (!s.command(is_mmctl ? std::span<const uint8_t>(mmctl) : std::span<const uint8_t>(mmctl, 1)) || !s.ask('s', signature)) {
            return false;
        }
        return signature == std::array<uint8_t, 3> { 0x87, 0x95, 0x1E } || s.fail(Result::wrong_chip, 's');
    }

    bool write_page(Session &s, size_t index, std::span<const uint8_t, page_size> page) {
        std::array<uint8_t, 4 + page_size> block { 'B', 0, page_size, 'F' }; // Caterina efface la page puis l'écrit
        std::copy(page.begin(), page.end(), block.begin() + 4);
        return s.address(index * page_size) && s.command(block);
    }

    bool verify_page(Session &s, size_t index, std::span<const uint8_t, page_size> page) {
        static constexpr uint8_t read[] = { 'g', 0, page_size, 'F' };
        std::array<uint8_t, page_size> got;
        if (!s.address(index * page_size) || !s.send(read) || !s.receive(got, 'g')) {
            return false;
        }
        return std::equal(got.begin(), got.end(), page.begin()) || s.fail(Result::verify_failed, 'g', index * page_size);
    }

} // namespace

Report flash(Port &port, size_t pages) {
    Session s { port };
    std::array<uint8_t, page_size> page;
    size_t done = 0;
    const size_t total = 2 * pages;

    // Les pages [from, to) écrites, puis relues
    const auto pass = [&](size_t from, size_t to) {
        for (int verify = 0; verify < 2; ++verify) {
            for (size_t i = from; i < to; ++i) {
                if (!port.read_page(i, page)) {
                    return s.fail(Result::file_error, 0, i * page_size);
                }
                if (!(verify ? verify_page(s, i, page) : write_page(s, i, page))) {
                    return false;
                }
                port.progress(++done, total);
            }
        }
        return true;
    };

    page.fill(0xFF);
    if (pages == 0 || !present(s) || !write_page(s, 0, page) || !pass(1, pages) || !pass(0, 1)) {
        return pages ? s.report : Report { Result::file_error };
    }
    // Firmware écrit et relu : un « E » sans réponse n'y change rien, Caterina le lance de toute façon après 8 s
    static constexpr uint8_t leave[] = { 'L', 'E' };
    if (s.command({ leave, 1 })) {
        s.command({ leave + 1, 1 });
    }
    return {};
}

} // namespace od3d::avr109
