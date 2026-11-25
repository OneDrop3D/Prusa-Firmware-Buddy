/// @file
#include "live_od3d.hpp"
#include "force_probe_od3d.hpp" // csv_printf

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdio>

namespace od3d::live {

namespace {

    struct Slot {
        std::atomic<const char *> msgid = nullptr;
        std::atomic<int16_t> a = 0, b = 0, c = 0;

        void store(const char *m, int va, int vb, int vc) {
            const auto clip = [](int v) { return int16_t(std::clamp(v, -32767, 32767)); };
            a = clip(va);
            b = clip(vb);
            c = clip(vc);
            msgid = m; // en dernier : la phrase arrive avec ses nombres
        }

        Line load() const {
            return Line { msgid.load(), a.load(), b.load(), c.load() };
        }
    };

    Slot line_now;
    Slot line_value;

} // namespace

void set(const char *msgid, int a, int b, int c) {
    line_now.store(msgid, a, b, c);
}

void set_value(const char *msgid, int a, int b, int c) {
    line_value.store(msgid, a, b, c);
}

void clear() {
    line_now.store(nullptr, 0, 0, 0);
    line_value.store(nullptr, 0, 0, 0);
}

Line now() {
    return line_now.load();
}

Line value() {
    return line_value.load();
}

} // namespace od3d::live

// Défini ici, compilé sur toutes les imprimantes : force_probe_od3d.cpp ne l'est qu'avec la cellule de charge et le MMU,
// et l'INDX écrit aussi ses CSV (courroies, accéléromètre, mesure de l'écran)
namespace od3d::force_probe {

int csv_printf(FILE *file, const char *format, ...) {
    static char line[256]; // hors de la pile de Marlin (seul fil qui écrit les CSV)
    va_list args;
    va_start(args, format);
    const int n = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (n < 0) {
        return -1;
    }
    const size_t length = std::min(size_t(n), sizeof(line) - 1);
    return fwrite(line, 1, length, file) == length ? int(length) : -1;
}

} // namespace od3d::force_probe
