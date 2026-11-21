/// @file
#include "spool_usage_math_od3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace od3d::spool_usage {

namespace {

    struct Density {
        std::string_view prefix;
        float g_cm3;
    };

    // Densités courantes des matières (fiches des fabricants, valeurs moyennes) ; les noms des préréglages de Prusa
    // (filament.hpp) plus quelques noms usuels. Une variante chargée (« PETG-CF ») prend celle de sa base.
    constexpr Density densities[] = {
        { "PLA", 1.24f },
        { "PETG", 1.27f },
        { "PCTG", 1.23f },
        { "CPE", 1.25f },
        { "ASA", 1.07f },
        { "ABS", 1.04f },
        { "PC", 1.20f },
        { "PVB", 1.08f },
        { "HIPS", 1.04f },
        { "PP", 0.90f },
        { "FLEX", 1.21f },
        { "TPU", 1.21f },
        { "PA", 1.14f },
        { "PPS", 1.35f },
        { "PPA", 1.18f },
        { "PVA", 1.23f },
        { "BVOH", 1.14f },
    };

    bool starts_with_nocase(std::string_view text, std::string_view prefix) {
        if (text.size() < prefix.size()) {
            return false;
        }
        for (size_t i = 0; i < prefix.size(); ++i) {
            char c = text[i];
            if (c >= 'a' && c <= 'z') {
                c = char(c - 'a' + 'A');
            }
            if (c != prefix[i]) {
                return false;
            }
        }
        return true;
    }

    constexpr float pi = 3.14159265f;

} // namespace

float density(std::string_view material) {
    float best = default_density;
    size_t best_len = 0;
    for (const Density &d : densities) {
        if (d.prefix.size() > best_len && starts_with_nocase(material, d.prefix)) {
            best = d.g_cm3;
            best_len = d.prefix.size();
        }
    }
    return best;
}

float grams_per_mm(float density) {
    // Section [mm²] × densité [g/cm³ = mg/mm³] → mg/mm, puis g/mm
    constexpr float section_mm2 = pi / 4 * filament_diameter_mm * filament_diameter_mm;
    return section_mm2 * density / 1000;
}

float grams(float length_mm, float density) {
    return length_mm * grams_per_mm(density);
}

float meters(float weight_g, float density) {
    return weight_g / grams_per_mm(density) / 1000;
}

Record new_spool(const Record &previous, uint16_t net_g) {
    Record r = previous;
    r.net_g = net_g;
    r.remaining_g = net_g;
    r.used_g = 0;
    r.last_print_g = 0;
    return r;
}

Record set_net(const Record &previous, uint16_t net_g) {
    if (!is_tracked(previous)) {
        return new_spool(previous, net_g);
    }
    Record r = previous;
    r.net_g = net_g;
    r.remaining_g = std::min<float>(r.remaining_g, net_g);
    return r;
}

Record consume(const Record &previous, float grams) {
    Record r = previous;
    if (!(grams > 0)) {
        return r;
    }
    r.remaining_g = std::max(0.0f, r.remaining_g - grams);
    r.used_g += grams;
    return r;
}

uint8_t fill_percent(float remaining_g, float net_g) {
    if (!(net_g > 0) || !(remaining_g > 0)) {
        return 0;
    }
    return uint8_t(std::clamp<long>(std::lround(remaining_g * 100 / net_g), 0, 100));
}

bool is_low(float remaining_g, float alert_g) {
    return remaining_g <= alert_g;
}

bool crossed_alert(float before_g, float after_g, float alert_g) {
    return !is_low(before_g, alert_g) && is_low(after_g, alert_g);
}

bool not_enough(float planned_g, float remaining_g) {
    // Comparer avec NAN donne toujours faux : une valeur inconnue n'alerte pas (comme le contrôle de Prusa)
    return planned_g > remaining_g;
}

bool should_commit(float pending_g, bool printing, bool active_tool, uint32_t still_ms) {
    if (!(pending_g >= 0.1f)) {
        return false;
    }
    if (pending_g >= flush_g) {
        return true;
    }
    if (printing) {
        return !active_tool && pending_g >= tool_change_g;
    }
    return still_ms >= idle_ms;
}

void format_weight(char *buf, size_t size, uint16_t grams) {
    if (grams >= 1000 && grams % 1000 == 0) {
        snprintf(buf, size, "%u kg", unsigned(grams / 1000));
    } else {
        snprintf(buf, size, "%u g", unsigned(grams));
    }
}

} // namespace od3d::spool_usage
