/// @file
#include "live_od3d.hpp"

#include <algorithm>
#include <atomic>

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
