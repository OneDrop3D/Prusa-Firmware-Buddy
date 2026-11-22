#pragma once

#include <bitset>

#include <filament.hpp>
#include "../../../../../../../src/feature/od3d/filament_presets_math_od3d.hpp"

struct ConfigStoreStub {
    template <typename T>
    struct Item {
        T get() { return T(); }
        T get(auto) { return T(); }

        void set(auto) {}
        void set(auto, auto) {}

        void apply(auto) {}
    };

    Item<std::bitset<max_preset_filament_type_count>> visible_preset_filament_types;
    Item<std::bitset<max_preset_filament_type_count>> visible_user_filament_types;

    Item<FilamentTypeParameters_EEPROM1> adhoc_filament_parameters;
    Item<FilamentTypeParameters_EEPROM5> adhoc_filament_parameters_5;

    Item<FilamentTypeParameters_EEPROM1> user_filament_parameters;

    Item<od3d::filament_presets::Override> od3d_preset_overrides; // OneDrop3D (UI-13)
};

ConfigStoreStub &config_store() {
    static ConfigStoreStub stub;
    return stub;
}
