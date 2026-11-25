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

/// Emplacement que Prusa croit chargé (MMU2::get_current_tool), seulement si aucun changement d'outil n'est resté
/// inachevé (MMU2::get_tool_change_tool) ; no_slot sinon. Un arrêt de l'impression pendant ToolChangeCommon laisse
/// l'ancien emplacement dans get_current_tool() alors que le MMU finit de charger le nouveau : Prusa ne s'en servait
/// plus, il déchargeait tout à l'arrêt (retour du 25/11 : « PETG chargé » alors que ce n'était pas le dernier filament)
constexpr uint8_t certain_slot(uint8_t prusa_slot, uint8_t changing_slot) {
    return prusa_slot < slots && (changing_slot >= slots || changing_slot == prusa_slot) ? prusa_slot : no_slot;
}

/// Emplacement dans la buse : celui que Prusa croit chargé, seulement si FINDA et le capteur de l'extrudeur voient du
/// filament (jamais « chargé » quand les capteurs ne sont pas d'accord)
constexpr std::optional<uint8_t> confirmed_slot(uint8_t prusa_slot, bool finda, bool extruder) {
    return prusa_slot < slots && finda && extruder ? std::optional<uint8_t>(prusa_slot) : std::nullopt;
}

/// Capteur de filament de la tête : unknown s'il est coupé, pas calibré ou en panne
enum class Sensor : uint8_t {
    unknown,
    has,
    none,
};

/// Ce que l'imprimante sait du filament dans la buse
struct NozzleView {
    bool mmu_configured = false; ///< MMU activé dans les réglages
    bool mmu_active = false; ///< il répond (MMU2::xState::Active)
    uint8_t prusa_slot = no_slot; ///< outil de Prusa (VirtualToolIndex::currently_selected), no_slot : aucun
    uint8_t changing_slot = no_slot; ///< changement d'outil commencé (MMU2::get_tool_change_tool), no_slot : aucun
    bool finda = false;
    Sensor extruder = Sensor::unknown;
};

/// Emplacement dans la buse, jamais supposé (retour du 23/11 : PA imprimé sans filament). MMU actif : celui de Prusa
/// s'il est sûr (certain_slot) et si FINDA et la tête voient du filament. MMU activé qui ne répond pas (à flasher,
/// M709 S2, erreur de communication) : aucun, Prusa rend alors l'outil 0 sans rien savoir. Sans MMU : l'outil unique,
/// sauf si la tête voit sûrement qu'il n'y a rien (capteur coupé : comme Prusa, on le croit chargé)
constexpr std::optional<uint8_t> nozzle_slot(const NozzleView &v) {
    if (v.mmu_active) {
        return confirmed_slot(certain_slot(v.prusa_slot, v.changing_slot), v.finda, v.extruder == Sensor::has);
    }
    if (v.mmu_configured || v.prusa_slot >= slots || v.extruder == Sensor::none) {
        return std::nullopt;
    }
    return v.prusa_slot;
}

/// Ni rechargement ni purge : l'emplacement demandé est déjà dans la buse, avec la même matière
constexpr bool skip_reload(uint8_t requested, std::optional<uint8_t> loaded, bool same_material) {
    return loaded == requested && same_material;
}

/// Emplacement à garder maintenant. Pendant une commande du MMU : aucun (une coupure à ce moment ne doit rien faire
/// restaurer). Pendant une impression, rien n'est écrit (pas d'écriture à chaque changement d'outil : la 1re commande
/// a déjà effacé l'enregistrement). Sinon : celui que Prusa croit chargé, s'il est sûr (l'appelant passe certain_slot).
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

/// Enregistrement à écrire pour l'emplacement keep (slot_to_keep), nullopt s'il ne change pas. Matière et date restent
/// celles du chargement : une matière modifiée ensuite sur l'emplacement chargé (le filament dans la buse, lui, n'a pas
/// changé) ne suit plus, et fait oublier le filament au démarrage suivant (boot_decision, material_matches)
constexpr std::optional<Record> record_update(const Record &stored, uint8_t keep, const char *material, uint32_t now) {
    const uint8_t slot = keep < slots ? keep : no_slot;
    if (slot == stored.slot) {
        return std::nullopt;
    }
    return make_record(stored, slot, material, now);
}

/// Premier T d'une impression partie du début du fichier (MMU2::tool_change, F43) : que faire du filament gardé
enum class StartLoad : uint8_t {
    keep, ///< le même, sûr et vu par les deux capteurs : reculé de start_load_mm, rien n'est rechargé
    unload_first, ///< un autre, ou un filament dont Prusa n'est pas sûr : déchargé avec ramming, puis le T de Prusa
    load, ///< rien dans la buse : le T de Prusa
};

/// requested : emplacement du T ; prusa_slot, changing_slot : comme certain_slot ; finda, extruder : les capteurs
constexpr StartLoad start_load(uint8_t requested, uint8_t prusa_slot, uint8_t changing_slot, bool finda, bool extruder) {
    if (confirmed_slot(certain_slot(prusa_slot, changing_slot), finda, extruder) == requested) {
        return StartLoad::keep;
    }
    // Filament inconnu vu par un capteur : à décharger, comme Prusa le faisait à l'arrêt (sinon le MMU attend en vain
    // que l'extrudeur le relâche, ou le croit déjà là si c'est le même emplacement)
    return prusa_slot < slots || finda || extruder ? StartLoad::unload_first : StartLoad::load;
}

} // namespace od3d::filament_memory
