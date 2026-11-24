/// @file
#include "print_info_od3d.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace od3d::print_info {

namespace {

    /// Appelle f(index, élément) pour chaque élément de la liste, espaces de tête retirés
    template <class F>
    void for_each_item(std::string_view list, char separator, F &&f) {
        for (uint8_t i = 0; i < max_tools && !list.empty(); ++i) {
            const size_t end = list.find(separator);
            std::string_view item = list.substr(0, end);
            while (!item.empty() && item.front() == ' ') {
                item.remove_prefix(1);
            }
            f(i, item);
            list = end == std::string_view::npos ? std::string_view {} : list.substr(end + 1);
        }
    }

    float to_float(std::string_view item) {
        char buf[16];
        const size_t n = std::min(item.size(), sizeof(buf) - 1);
        memcpy(buf, item.data(), n);
        buf[n] = '\0';
        return strtof(buf, nullptr);
    }

} // namespace

float FileInfo::total_grams() const {
    float total = 0;
    for (float g : grams) {
        total += g;
    }
    return total;
}

void parse_metadata(FileInfo &info, std::string_view name, std::string_view value) {
    if (name == "estimated printing time (normal mode)") {
        info.seconds = parse_duration(value);
    } else if (name == "filament used [g]") {
        for_each_item(value, ',', [&](uint8_t i, std::string_view item) {
            info.grams[i] = to_float(item);
            if (info.grams[i] > 0.0003f) {
                info.used |= 1 << i;
            }
        });
    } else if (name == "filament used [mm]") {
        info.millimeters = 0;
        for_each_item(value, ',', [&](uint8_t, std::string_view item) {
            info.millimeters += to_float(item);
        });
    } else if (name == "temperature") {
        for_each_item(value, ',', [&](uint8_t i, std::string_view item) {
            info.nozzle[i] = int16_t(to_float(item));
        });
    } else if (name == "filament_type") {
        for_each_item(value, ';', [&](uint8_t i, std::string_view item) {
            const size_t n = std::min(item.size(), info.types[i].size() - 1);
            memcpy(info.types[i].data(), item.data(), n);
            info.types[i][n] = '\0';
        });
    }
}

uint32_t parse_duration(std::string_view text) {
    uint32_t total = 0, number = 0;
    for (char c : text) {
        if (c >= '0' && c <= '9') {
            number = number * 10 + (c - '0');
            continue;
        }
        switch (c) {
        case 'd':
            total += number * 86400;
            break;
        case 'h':
            total += number * 3600;
            break;
        case 'm':
            total += number * 60;
            break;
        case 's':
            total += number;
            break;
        default:
            continue; // espaces : le nombre continue jusqu'à son unité
        }
        number = 0;
    }
    return total;
}

void format_hm(uint32_t seconds, char *buf, size_t size) {
    snprintf(buf, size, "%u h %02u", unsigned(seconds / 3600), unsigned(seconds / 60 % 60));
}

int corner_inset(int y, int h, int r) {
    const int d = y < r ? r - 1 - y : (y >= h - r ? y - (h - r) : -1);
    if (d < 0) {
        return 0;
    }
    int inside = 0;
    while (inside < r && inside * inside + d * d < r * r) {
        ++inside;
    }
    return r - inside;
}

namespace {
    constexpr int strip_track_top = 4, strip_track_h = 12, strip_track_r = 6;

    int strip_done(int w, int percent) {
        return w * std::min(percent, 100) / 100;
    }

    int strip_cursor(int w, int percent) {
        return std::clamp(strip_done(w, percent) - 1, 0, w - 3);
    }
} // namespace

StripPart strip_part(int x, int y, int w, int h, int percent) {
    // Piste arrondie, partie faite orange, curseur blanc de 3 px
    const int done = strip_done(w, percent);
    const int cursor = strip_cursor(w, percent);
    if (x >= cursor && x < cursor + 3) {
        return (x != cursor + 1 && (y == 0 || y == h - 1)) ? StripPart::outside : StripPart::cursor; // coins de 1 px
    }
    const int ty = y - strip_track_top;
    const int inset = ty >= 0 && ty < strip_track_h ? corner_inset(ty, strip_track_h, strip_track_r) : w;
    if (x < inset || x >= w - inset) {
        return StripPart::outside;
    }
    return x < done - inset ? StripPart::done : StripPart::track;
}

std::pair<int, int> strip_changed(int w, int from, int to) {
    // Ce qui bouge à un pourcentage donné : le bout arrondi de la partie faite (jusqu'au rayon en deçà) et le curseur ;
    // entre deux pourcentages, tout ce qui est entre les deux
    const auto first = [w](int p) { return std::min(strip_cursor(w, p), strip_done(w, p) - strip_track_r); };
    const auto end = [w](int p) { return std::max(strip_cursor(w, p) + 3, strip_done(w, p)); };
    return { std::max(0, std::min(first(from), first(to))), std::min(w, std::max(end(from), end(to))) };
}

} // namespace od3d::print_info
