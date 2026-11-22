/// @file
/// Givrage (frost_od3d.hpp) : l'afficheur ILI9488 et la boucle de l'interface
#include "frost_od3d.hpp"

#include <display.hpp>
#include <ili9488.hpp>
#include <screen.hpp>
#include <ScreenHandler.hpp>

namespace od3d::frost {

namespace {

    Job job;
    const window_t *owner = nullptr;

    Area area_of(const Rect16 &r) {
        return Area { r.Left(), r.Top(), r.Width(), r.Height() };
    }

    uint8_t *read(Area a) {
        return ili9488_read_rect(a.x, a.y, a.w, a.h);
    }

    void write(Area a, const uint8_t *) {
        display::draw_from_buffer(point_ui16(a.x, a.y), a.w, a.h); // depuis le tampon de l'afficheur, que read a rempli
    }

    /// owner est ce qui est dessiné tout au-dessus : rien ne s'est ouvert par-dessus (son fond serait relu)
    bool on_top() {
        screen_t *screen = Screens::Access()->Get();
        if (!screen) {
            return false;
        }
        const window_t *top = screen;
        window_t *last = screen->GetLastDialog();
        for (window_t *d = screen->GetFirstDialog(); d; d = d == last ? nullptr : d->GetNext()) {
            if (d->IsVisible()) {
                top = d;
            }
        }
        return top == owner;
    }

} // namespace

void start(const window_t &who, const Rect16 &area, const Rect16 &hole) {
    owner = &who;
    job.start(area_of(area), area_of(hole));
}

void stop(const window_t &who) {
    if (owner == &who) {
        owner = nullptr;
        job.stop();
    }
}

void close(const window_t &dialog) {
    stop(dialog);
    if (screen_t *screen = Screens::Access()->Get()) {
        screen->Invalidate();
    }
}

bool step() {
    if (!owner || job.done()) {
        return false;
    }
    if (Screens::Access()->is_change_pending()) {
        return false; // l'écran change : son propriétaire va s'arrêter
    }
    if (!on_top()) {
        owner = nullptr; // un autre dialogue le couvre : son fond n'est plus ce qu'il faut givrer
        job.stop();
        return false;
    }
    job.step(ili9488_buffer_size() / 3, read, write);
    return !job.done();
}

} // namespace od3d::frost
