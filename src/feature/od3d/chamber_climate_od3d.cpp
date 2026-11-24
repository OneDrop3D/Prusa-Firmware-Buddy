/// @file
#include "chamber_climate_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <variant>

#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <feature/chamber/chamber.hpp>
#include <feature/safety_timer/safety_timer.hpp>
#include <filament.hpp>
#include <marlin_server.hpp>
#include <module/temperature.h>
#include <timing.h>
#include <tool_index.hpp>

namespace od3d::chamber_climate {

namespace {

    constexpr uint32_t step_interval_ms = 1000;

    // Thread de Marlin seulement
    Cooldown cooldown;
    uint32_t last_step_ms = 0;
    bool print_wants_soft_cooldown = false; ///< une matière de l'impression en cours demande la descente
    int16_t print_bed_target = 0; ///< consigne la plus haute du plateau pendant l'impression
    /// Plafond posé une fois pour cette impression : ensuite la consigne choisie à l'écran est respectée, « Arrêt »
    /// compris (retour du 24/11 : « dans une impression on peut pas gérer la température de la chambre »).
    /// shortcut: le maximum de la première matière ; à reposer au changement de matière si un mélange le demande
    bool print_ceiling_done = false;

    // Lus par l'interface
    std::atomic<bool> stop_requested { false };
    std::atomic<bool> status_active { false };
    std::atomic<int16_t> status_target { 0 };
    std::atomic<uint16_t> status_next_s { 0 };
    std::atomic<uint16_t> status_steps_left { 0 };

    Recipe recipe_for(const FilamentType &type, bool follow_name = true) {
        if (const auto *preset = std::get_if<PresetFilamentType>(&type)) {
            return preset_recipe(static_cast<uint8_t>(*preset));
        }
        if (const auto *user = std::get_if<UserFilamentType>(&type)) {
            return user_recipe(user->index);
        }
        if (follow_name && type.is_adhoc()) {
            // Filament ad hoc (MMU, OpenPrintTag) : la recette de la matière du même nom (« PLA »)
            const FilamentTypeParameters p = type.parameters();
            return recipe_for(FilamentType::from_name(std::string_view(p.name)), false);
        }
        return Recipe::none;
    }

    bool any_nozzle_heating() {
        for (const auto tool : PhysicalToolIndex::all()) {
            if (Temperature::degTargetHotend(tool) > 0) {
                return true;
            }
        }
        return false;
    }

    void publish_status(uint32_t now) {
        status_active = cooldown.active();
        status_target = cooldown.target();
        status_next_s = static_cast<uint16_t>(cooldown.ms_to_next_step(now) / 1000);
        status_steps_left = cooldown.steps_left();
    }

    /// Pose une consigne du plateau (setTargetBed la borne aux limites de Prusa) et retient celle qui est en place
    void set_bed(int16_t target) {
        thermalManager.setTargetBed(target);
        cooldown.confirm(thermalManager.degTargetBed());
    }

    /// Pendant l'impression : matières utilisées, consigne du plateau, plafond de la chambre
    void step_printing() {
        print_bed_target = std::max<int16_t>(print_bed_target, thermalManager.degTargetBed());

        const FilamentType type = FilamentType::for_current_tool_heuristic();
        if (!type) {
            return;
        }
        const Recipe recipe = recipe_for(type);
        print_wants_soft_cooldown |= has_soft_cooldown(recipe);
        if (!has_ceiling(recipe)) {
            return;
        }

        // Pas pendant une pause (minuteur de sécurité actif) : poser une cible de chambre relancerait les buses
        if (marlin_vars().print_state != marlin_server::State::Printing || buddy::safety_timer().is_active()) {
            return;
        }
        auto &chamber = buddy::chamber();
        const auto max_c = type.parameters().chamber_max_temperature;
        if (print_ceiling_done) {
            return;
        }
        if (const auto target = ceiling_target(recipe, max_c, chamber.target_temperature().has_value(), chamber.capabilities().cooling)) {
            chamber.set_target_temperature(buddy::Temperature(*target));
            print_ceiling_done = true;
        }
    }

} // namespace

// ---- Recettes et réglages ----

Recipe preset_recipe(uint8_t preset) {
    return recipe_get<uint32_t>(config_store().od3d_climate_preset_recipes.get(), preset);
}

void set_preset_recipe(uint8_t preset, Recipe r) {
    config_store().od3d_climate_preset_recipes.set(recipe_set<uint32_t>(config_store().od3d_climate_preset_recipes.get(), preset, r));
}

Recipe user_recipe(uint8_t index) {
    return recipe_get<uint16_t>(config_store().od3d_climate_user_recipes.get(), index);
}

void set_user_recipe(uint8_t index, Recipe r) {
    config_store().od3d_climate_user_recipes.set(recipe_set<uint16_t>(config_store().od3d_climate_user_recipes.get(), index, r));
}

CooldownParams params() {
    return CooldownParams {
        .step_c = config_store().od3d_cooldown_step.get(),
        .interval_min = config_store().od3d_cooldown_interval.get(),
        .floor_c = config_store().od3d_cooldown_floor.get(),
    }
        .clamped();
}

void set_params(const CooldownParams &p) {
    const CooldownParams c = p.clamped();
    config_store().od3d_cooldown_step.set(c.step_c);
    config_store().od3d_cooldown_interval.set(c.interval_min);
    config_store().od3d_cooldown_floor.set(c.floor_c);
}

// ---- Thread de Marlin ----

void step() {
    const uint32_t now = ticks_ms();
    if (ticks_diff(now, last_step_ms) < int32_t(step_interval_ms)) {
        return;
    }
    last_step_ms = now;

    if (marlin_server::is_printing()) {
        step_printing();
    }

    if (stop_requested.exchange(false) && cooldown.active()) {
        cooldown.stop();
        thermalManager.setTargetBed(0); // arrêt demandé : refroidissement normal
    }

    const Cooldown::Step s = cooldown.step({
        .now_ms = now,
        .bed_current = thermalManager.degBed(),
        .bed_target = thermalManager.degTargetBed(),
        .nozzle_heating = any_nozzle_heating() || buddy::safety_timer().is_active(),
    });
    switch (s.action) {
    case Cooldown::Action::none:
    case Cooldown::Action::abort:
        break;
    case Cooldown::Action::set_target:
        set_bed(s.target);
        break;
    case Cooldown::Action::finish:
        thermalManager.setTargetBed(0);
        break;
    }
    publish_status(now);
}

void on_print_start() {
    cooldown.stop(); // la consigne en place reste : l'impression pose la sienne, sans attendre
    stop_requested = false;
    print_wants_soft_cooldown = false;
    print_bed_target = 0;
    print_ceiling_done = false;
    publish_status(ticks_ms());
}

void on_print_finished() {
    const bool wanted = print_wants_soft_cooldown;
    const int16_t bed = print_bed_target;
    print_wants_soft_cooldown = false;
    print_bed_target = 0;
    if (!wanted || any_nozzle_heating()) {
        return;
    }
    if (const auto first = cooldown.start(params(), bed, thermalManager.degBed(), ticks_ms())) {
        set_bed(*first);
    }
    publish_status(ticks_ms());
}

bool start_now() {
    if (marlin_server::is_printing() || any_nozzle_heating()) {
        return false;
    }
    const float current = thermalManager.degBed();
    const auto first = cooldown.start(params(), static_cast<int16_t>(current), current, ticks_ms());
    if (first) {
        set_bed(*first);
    }
    publish_status(ticks_ms());
    return first.has_value();
}

// ---- Tous threads ----

void request_stop() {
    stop_requested = true;
}

Status status() {
    return Status {
        .active = status_active,
        .target = status_target,
        .next_step_s = status_next_s,
        .steps_left = status_steps_left,
        .floor_c = params().floor_c,
    };
}

} // namespace od3d::chamber_climate
