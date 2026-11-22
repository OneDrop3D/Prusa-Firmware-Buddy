/// @file
/// OneDrop3D (F43) : filament chargé jusqu'à la buse retenu après un redémarrage ou un flash, sans lecture de
/// l'imprimante (testé dans tests/unit/feature/od3d).
///
/// Prusa oublie l'emplacement chargé à chaque démarrage du MMU (MMU2::Start : extruder = MMU2_NO_TOOL) ; il ne le
/// réapprend qu'en fin de changement d'outil (ToolChangeCommon). Le MMU, lui, le garde dans son EEPROM (et suppose le
/// filament dans la buse), mais ne le rapporte pas : son sélecteur répond 0xff tant qu'il n'a pas refait son origine,
/// ce qu'il ne fait pas avec un filament dedans. On garde donc notre propre enregistrement.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace od3d::filament_memory {

inline constexpr uint8_t slots = 5; ///< MMU3
inline constexpr uint8_t no_slot = 0xff;
inline constexpr uint8_t selector_unknown = 0xff; ///< sélecteur pas encore à l'origine (filament dedans au démarrage)
inline constexpr uint32_t settle_ms = 3000; ///< après la connexion du MMU : registres relus plusieurs fois (100 ms)

/// Ce qui est gardé (config_store, « OD3D Loaded Filament »). Taille fixe : pour changer la structure, créer un nouvel
/// élément (nouveau nom), sans réutiliser ce nom.
struct Record {
    uint32_t time = 0; ///< date du chargement [s depuis 1970], 0 si l'heure n'était pas connue
    uint8_t slot = no_slot; ///< emplacement chargé jusqu'à la buse, no_slot : aucun
    std::array<char, 11> material {}; ///< nom de la matière de l'emplacement au chargement (« PLA »)

    bool operator==(const Record &) const = default;
};
static_assert(sizeof(Record) == 16);

/// Ce qui est vu au démarrage du MMU
struct BootView {
    Record stored;
    bool material_matches = false; ///< l'emplacement a toujours la matière enregistrée
    bool finda = false; ///< FINDA : filament dans le sélecteur
    bool extruder = false; ///< capteur de l'extrudeur : filament présent (sûr)
    uint8_t selector = selector_unknown; ///< emplacement du sélecteur rapporté par le MMU
    bool mmu_busy = false; ///< commande en cours ou erreur du MMU
    uint32_t active_ms = 0; ///< depuis la connexion du MMU
};

enum class Boot : uint8_t {
    wait, ///< pas encore : registres pas relus, MMU occupé ou en erreur
    restore, ///< filament retrouvé : Prusa le croit chargé, comme après un changement d'outil réussi
    forget, ///< comportement de Prusa (rien de chargé), l'enregistrement s'efface
};

/// Restaurer seulement si les deux capteurs voient du filament, si le MMU ne contredit pas l'emplacement et si la
/// matière n'a pas changé ; jamais avant que les registres du MMU soient relus (FINDA vaut 0 avant la 1re lecture)
constexpr Boot boot_decision(const BootView &v) {
    if (v.stored.slot >= slots) {
        return Boot::forget;
    }
    if (v.mmu_busy || v.active_ms < settle_ms) {
        return Boot::wait;
    }
    const bool selector_agrees = v.selector == v.stored.slot || v.selector == selector_unknown;
    return v.finda && v.extruder && selector_agrees && v.material_matches ? Boot::restore : Boot::forget;
}

/// Emplacement dans la buse : celui que Prusa croit chargé, seulement si FINDA et le capteur de l'extrudeur voient du
/// filament (jamais « chargé » quand les capteurs ne sont pas d'accord)
constexpr std::optional<uint8_t> confirmed_slot(uint8_t prusa_slot, bool finda, bool extruder) {
    return prusa_slot < slots && finda && extruder ? std::optional<uint8_t>(prusa_slot) : std::nullopt;
}

/// Ni rechargement ni purge : l'emplacement demandé est déjà dans la buse, avec la même matière
constexpr bool skip_reload(uint8_t requested, std::optional<uint8_t> loaded, bool same_material) {
    return loaded == requested && same_material;
}

/// Emplacement à garder maintenant. Pendant une commande du MMU : aucun (une coupure à ce moment ne doit rien faire
/// restaurer). Pendant une impression, rien n'est écrit (pas d'écriture à chaque changement d'outil : la 1re commande
/// a déjà effacé l'enregistrement). Sinon : celui que Prusa croit chargé.
constexpr std::optional<uint8_t> slot_to_keep(bool command_in_progress, bool printing, uint8_t prusa_slot) {
    if (command_in_progress) {
        return no_slot;
    }
    if (printing) {
        return std::nullopt;
    }
    return prusa_slot < slots ? prusa_slot : no_slot;
}

/// Enregistrement pour slot et sa matière ; la date de stored est gardée si rien ne change
constexpr Record make_record(const Record &stored, uint8_t slot, const char *material, uint32_t now) {
    Record r;
    r.slot = slot < slots ? slot : no_slot;
    for (size_t i = 0; r.slot != no_slot && material && material[i] && i + 1 < r.material.size(); ++i) {
        r.material[i] = material[i];
    }
    const bool same = r.slot == stored.slot && r.material == stored.material;
    r.time = same ? stored.time : (r.slot == no_slot ? 0 : now);
    return r;
}

} // namespace od3d::filament_memory
