/// @file
/// OneDrop3D : écriture du firmware du MMU3 dans son chargeur Caterina (AVR109, comme `avrdude -c avr109 -p
/// atmega32u4`), sans dépendance au matériel pour être testée. Caterina traite une commande par paquet USB et jette le
/// reste du paquet : chaque commande part dans son propre envoi. La page 0 est effacée d'abord et écrite en dernier : tant
/// qu'elle est vide, Caterina ne lance aucun firmware et reste dans son chargeur, si bien qu'une écriture interrompue
/// se relance.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace od3d::avr109 {

inline constexpr size_t page_size = 128; ///< SPM_PAGESIZE de l'ATmega32U4
inline constexpr size_t app_size = 0x7000; ///< 28 Ko pour le firmware, les 4 Ko du haut sont au chargeur

class Port {
public:
    /// Un envoi = un transfert USB (une commande et ses arguments)
    virtual bool send(std::span<const uint8_t> bytes) = 0;
    /// Remplit `bytes` en entier, faux après `timeout_ms` sans réponse
    virtual bool receive(std::span<uint8_t> bytes, uint32_t timeout_ms) = 0;
    /// Oublie ce qui a été reçu (butterfly_drain d'avrdude)
    virtual void drain() = 0;
    /// Page `index` du firmware (0xFF au-delà de sa fin)
    virtual bool read_page(size_t index, std::span<uint8_t, page_size> page) = 0;
    virtual void progress(size_t done, size_t total) = 0;

protected:
    ~Port() = default;
};

enum class Result : uint8_t {
    ok,
    no_answer, ///< `command` sans réponse
    refused, ///< `command` refusée
    not_paged, ///< le chargeur n'écrit pas par pages de 128 octets
    wrong_chip, ///< signature autre que l'ATmega32U4
    file_error, ///< page du firmware illisible
    verify_failed, ///< relecture différente à `address`
};

struct Report {
    Result result = Result::ok;
    char command = 0;
    uint16_t address = 0;
};

/// Présentation (nom, pages, signature), écriture et relecture de `pages` pages, puis sortie du chargeur
Report flash(Port &port, size_t pages);

} // namespace od3d::avr109
