/// @file
/// Givrage (frost_od3d.hpp) : l'afficheur ILI9488
#include "frost_od3d.hpp"

#include <display.hpp>
#include <ili9488.hpp>
#include <screen.hpp>
#include <ScreenHandler.hpp>

namespace od3d::frost {

namespace {

    Area area_of(const Rect16 &r) {
        return Area { r.Left(), r.Top(), r.Width(), r.Height() };
    }

    uint8_t *read(Area a) {
        return ili9488_read_rect(a.x, a.y, a.w, a.h);
    }

    void write(Area a, const uint8_t *) {
        display::draw_from_buffer(point_ui16(a.x, a.y), a.w, a.h); // depuis le tampon de l'afficheur, que read a rendu
    }

} // namespace

void run(const Rect16 &area, const Rect16 &hole) {
    run(area_of(area), area_of(hole), ili9488_buffer_size() / 3, read, write);
}

void close() {
    if (screen_t *screen = Screens::Access()->Get()) {
        screen->Invalidate();
    }
}

} // namespace od3d::frost
