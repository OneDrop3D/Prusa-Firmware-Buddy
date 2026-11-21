/// @file
#include "screen_move_od3d.hpp"

#include "menu_od3d.hpp"
#include "screen_mmu_od3d.hpp"

#include <algorithm>
#include <common/marlin_vars.hpp>
#include <config_store/store_c_api.h>
#include <cstdio>
#include <cstring>
#include <display.hpp>
#include <gcode/inject_queue_actions.hpp>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <utils/string_builder.hpp>
#include <window_msgbox.hpp>

// Marlin en dernier : ses macros ne doivent pas toucher les en-têtes de l'interface
#include <inc/MarlinConfig.h>
#include <module/motion.h>

namespace od3d {

namespace {

    // ---- Géométrie (SPEC_G.md §3.17) ----
    // Colonnes ajustées à la police de l'imprimante : « Rétracter » (81 px) tient dans la colonne E,
    // pris sur la croix et la carte, qui garde presque les proportions du plateau.
    // Sous l'en-tête de 40 px : croix et carte de 144 px (152 avant), touches un peu moins hautes.
    constexpr Rect16 cross_rect = { 8, style::content.Top(), 140, 144 };
    constexpr Rect16 x_minus_body = { 12, 94, 40, 44 };
    constexpr Rect16 x_plus_body = { 104, 94, 40, 44 };
    constexpr Rect16 y_plus_body = { 54, 48, 48, 42 };
    constexpr Rect16 y_minus_body = { 54, 142, 48, 42 };
    constexpr Rect16 origin_xy_body = { 56, 94, 44, 44 };
    // Diagonales dans les coins de la croix (retour de l'utilisateur, 21/11 : « plusieurs axes à la fois »)
    constexpr Rect16 up_left_body = { 12, 49, 38, 38 };
    constexpr Rect16 up_right_body = { 106, 49, 38, 38 };
    constexpr Rect16 down_left_body = { 12, 144, 38, 38 };
    constexpr Rect16 down_right_body = { 106, 144, 38, 38 };
    constexpr Rect16 z_value_rect = { 148, style::content.Top(), 66, 16 };
    constexpr Rect16 z_plus_body = { 154, 62, 54, 60 };
    constexpr Rect16 z_minus_body = { 154, 127, 54, 60 };
    constexpr Rect16 e_value_rect = { 214, style::content.Top(), 84, 16 };
    constexpr Rect16 extrude_body = { 214, 62, 84, 60 };
    constexpr Rect16 retract_body = { 214, 127, 84, 60 };
    constexpr Rect16 map_rect = { 304, style::content.Top(), 168, 144 };
    constexpr Rect16 step_title_rect = { 8, 203, 36, 16 };
    constexpr Rect16 position_rect = { 296, 203, 176, 16 };
    // Trois boutons en bas : origine, moteurs coupés, axes du MMU
    constexpr Rect16 origin_all_body = { 8, 236, 150, 36 };
    constexpr Rect16 motors_body = { 165, 236, 150, 36 };
    constexpr Rect16 mmu_axes_body = { 322, 236, 150, 36 };

    /// 58 px : « 0.1 mm » (54 px) tient dans le contour de 2 px du pas choisi
    constexpr Rect16 step_body(int i) {
        return Rect16(46 + 62 * i, 196, 58, 30);
    }

    static_assert(style::inside(style::content, cross_rect) && style::inside(style::content, map_rect) && style::inside(style::content, style::with_ring(mmu_axes_body)));
    static_assert(style::with_ring(origin_all_body).Right() < style::with_ring(motors_body).Left() && style::with_ring(motors_body).Right() < style::with_ring(mmu_axes_body).Left());
    static_assert(style::inside(cross_rect, style::with_ring(up_left_body)) && style::inside(cross_rect, style::with_ring(up_right_body)) && style::inside(cross_rect, style::with_ring(down_left_body)) && style::inside(cross_rect, style::with_ring(down_right_body)));
    static_assert(style::with_ring(up_left_body).Right() < style::with_ring(y_plus_body).Left() && style::with_ring(y_plus_body).Right() < style::with_ring(up_right_body).Left());
    static_assert(style::with_ring(up_left_body).Bottom() < style::with_ring(x_minus_body).Top() && style::with_ring(x_minus_body).Bottom() < style::with_ring(down_left_body).Top());
    static_assert(style::inside(cross_rect, style::with_ring(x_minus_body)) && style::inside(cross_rect, style::with_ring(x_plus_body)) && style::inside(cross_rect, style::with_ring(y_plus_body)) && style::inside(cross_rect, style::with_ring(y_minus_body)));
    static_assert(z_value_rect.Bottom() < style::with_ring(z_plus_body).Top() && cross_rect.Right() < z_value_rect.Left() && z_value_rect.Right() < e_value_rect.Left());
    static_assert(style::with_ring(retract_body).Right() < map_rect.Left() && cross_rect.Bottom() < style::with_ring(step_body(0)).Top());
    static_assert(style::with_ring(step_body(3)).Right() < position_rect.Left() && style::with_ring(step_body(0)).Bottom() < style::with_ring(origin_all_body).Top());
    static_assert(style::with_ring(y_plus_body).Bottom() < style::with_ring(origin_xy_body).Top() && style::with_ring(origin_xy_body).Bottom() < style::with_ring(y_minus_body).Top());
    static_assert(style::with_ring(z_plus_body).Bottom() < style::with_ring(z_minus_body).Top() && step_title_rect.Right() < style::with_ring(step_body(0)).Left());

    // Carte : le centre du point (Ø14) reste à 11 px du trait (rayon des coins) pour que son carré, effacé
    // quand il bouge, ne touche jamais un coin arrondi ; le réticule s'arrête à 2 px du bord
    constexpr Rect16 map_inner = { map_rect.Left() + 1, map_rect.Top() + 1, map_rect.Width() - 2, map_rect.Height() - 2 };
    constexpr int16_t map_radius = 11; ///< coins de map_inner
    constexpr int16_t dot_d = 14;
    constexpr Rect16 head_area = { map_inner.Left() + map_radius, map_inner.Top() + map_radius, map_inner.Width() - 2 * map_radius, map_inner.Height() - 2 * map_radius };
    static_assert(2 * (dot_d / 2) * (dot_d / 2) < map_radius * map_radius, "coin du carré du point dans l'arrondi");
    constexpr float bed_x = X_BED_SIZE;
    constexpr float bed_y = Y_BED_SIZE;

    constexpr int16_t pixel_x(float mm) {
        return head_area.Left() + static_cast<int16_t>(std::clamp(mm, 0.f, bed_x) * (head_area.Width() - 1) / bed_x + 0.5f);
    }
    constexpr int16_t pixel_y(float mm) {
        return head_area.Bottom() - static_cast<int16_t>(std::clamp(mm, 0.f, bed_y) * (head_area.Height() - 1) / bed_y + 0.5f);
    }
    constexpr float mm_x(int16_t x) {
        return std::clamp<int>(x - head_area.Left(), 0, head_area.Width() - 1) * bed_x / (head_area.Width() - 1);
    }
    constexpr float mm_y(int16_t y) {
        return std::clamp<int>(head_area.Bottom() - y, 0, head_area.Height() - 1) * bed_y / (head_area.Height() - 1);
    }
    static_assert(pixel_x(0) == head_area.Left() && pixel_x(bed_x) == head_area.Right() && pixel_y(0) == head_area.Bottom() && pixel_y(bed_y) == head_area.Top());
    static_assert(pixel_x(mm_x(400)) == 400 && pixel_y(mm_y(100)) == 100 && mm_x(0) == 0 && mm_y(479) == 0, "toucher -> mm -> pixel");

    constexpr Rect16 crosshair_column(int16_t x) {
        return Rect16(x, map_inner.Top() + 2, 1, map_inner.Height() - 4);
    }
    constexpr Rect16 crosshair_row(int16_t y) {
        return Rect16(map_inner.Left() + 2, y, map_inner.Width() - 4, 1);
    }
    constexpr Rect16 dot_rect(point_i16_t c) {
        return Rect16(c.x - dot_d / 2, c.y - dot_d / 2, dot_d, dot_d);
    }

    constexpr Color map_back = Color::from_raw(0x22252A);
    constexpr Color map_grid = Color::from_raw(0x2C3036);
    constexpr Color crosshair_color = Color::from_raw(0x824223); ///< #F8651B à 45 % sur le fond de la carte
    constexpr Color hint_color = Color::from_raw(0x7D8188);
    constexpr Color selected_back = Color::from_raw(0x3A2618); ///< choix sélectionné (SPEC_G.md §1.1)

    // ---- Pas ----

    constexpr float step_mm[] = { 0.1f, 1.f, 10.f, 50.f };
    constexpr const char *step_labels[] = { "0.1 mm", "1 mm", "10 mm", "50 mm" };
    uint8_t step_index = 2; ///< 10 mm comme la maquette, gardé d'une ouverture à l'autre (un seul écran à la fois)
    float e_moved = 0; ///< filament poussé (+) ou tiré (-) depuis l'ouverture de l'écran [mm]

    // ---- G-codes ----

    // Vitesses du menu Prusa (MANUAL_FEEDRATE, mm/min), recopiées dans les G-codes ci-dessous
    constexpr float manual_feedrate[] = MANUAL_FEEDRATE;
    static_assert(manual_feedrate[X_AXIS] == 3000 && manual_feedrate[Y_AXIS] == 3000 && manual_feedrate[Z_AXIS] == 480 && manual_feedrate[E_AXIS] == 240);

    /// Un pas relatif puis retour en absolu, en une seule entrée de la file d'injection : rien ne s'intercale.
    /// G1 garde les butées logicielles de Marlin (axes qui ont leur origine).
    constexpr GCodeLiteral step_gcodes[] = {
        { "G91\nG1 X%.2f F3000\nG90" },
        { "G91\nG1 Y%.2f F3000\nG90" },
        { "G91\nG1 Z%.2f F480\nG90" },
        { "M83\nG1 E%.2f F240\nM82" },
    };

    /// X ou Y sans origine : comme la prise d'origine, arrêt au contact d'un bord (M1990)
    constexpr GCodeLiteral free_gcodes[] = {
        { "M1990 X%.2f" },
        { "M1990 Y%.2f" },
    };

    // Diagonales : un G1 X et Y d'une ligne (même distance sur les deux axes), une chaîne par pas et par sens, car un
    // GCodeLiteral n'a qu'un paramètre. Sens : 0 haut-gauche (X- Y+), 1 haut-droite, 2 bas-gauche, 3 bas-droite.
#define OD3D_DIAG(sx, sy, d) GCodeLiteral { "G91\nG1 X" sx d " Y" sy d " F3000\nG90" }
#define OD3D_DIAG_FREE(sx, sy, d) GCodeLiteral { "M1990 X" sx d " Y" sy d }
#define OD3D_DIAG_STEPS(M, sx, sy) { M(sx, sy, "0.1"), M(sx, sy, "1"), M(sx, sy, "10"), M(sx, sy, "50") }
    constexpr GCodeLiteral diag_gcodes[4][4] = {
        OD3D_DIAG_STEPS(OD3D_DIAG, "-", ""), OD3D_DIAG_STEPS(OD3D_DIAG, "", ""), OD3D_DIAG_STEPS(OD3D_DIAG, "-", "-"), OD3D_DIAG_STEPS(OD3D_DIAG, "", "-")
    };
    constexpr GCodeLiteral diag_free_gcodes[4][4] = {
        OD3D_DIAG_STEPS(OD3D_DIAG_FREE, "-", ""), OD3D_DIAG_STEPS(OD3D_DIAG_FREE, "", ""), OD3D_DIAG_STEPS(OD3D_DIAG_FREE, "-", "-"), OD3D_DIAG_STEPS(OD3D_DIAG_FREE, "", "-")
    };
#undef OD3D_DIAG_STEPS
#undef OD3D_DIAG_FREE
#undef OD3D_DIAG

    /// Rien pendant une impression : la liste de Prusa n'avance pas non plus tant que Marlin travaille
    bool not_printing() {
        if (marlin_client::is_printing()) {
            MsgBoxWarning(_("Not while printing."), Responses_Ok);
            return false;
        }
        return true;
    }

    void home_all() {
        if (not_printing()) {
            marlin_client::gcode("G28 P I"); // comme « Prise d'origine Auto » de Prusa hors impression
        }
    }

    void home_xy() {
        if (not_printing()) {
            marlin_client::gcode("G28 X Y I");
        }
    }

    void disable_motors() {
        if (not_printing()) {
            marlin_client::gcode("M18"); // comme « Désactiver les moteurs » de Prusa sur la CORE One
        }
    }

    /// Lu dans les octets de Marlin (autre tâche) : au pire en retard d'un cycle
    bool known(uint8_t axes) {
        return !axes_need_homing(axes);
    }

    /// Règle du menu Prusa ; Marlin refuserait aussi l'extrusion à froid
    bool hot_enough() {
        return marlin_vars().allow_cold_extrude || marlin_vars().active_hotend().temp_nozzle >= marlin_vars().extrude_min_temp;
    }

    /// Sans origine, propose la prise d'origine (pas pendant qu'elle se fait) au lieu de bouger
    bool homed(uint8_t axes) {
        if (known(axes)) {
            return true;
        }
        if (!marlin_vars().is_processing.get() && MsgBoxQuestion(_("Axes not homed. Home them now?"), Responses_YesNo) == Response::Yes) {
            (axes & _BV(Z_AXIS)) ? home_all() : home_xy(); // Z se prend au milieu du plateau : il faut X et Y
        }
        return false;
    }

    void move_by(AxisEnum axis, float mm) {
        if (!not_printing()) {
            return;
        }
        GCodeLiteral gcode = step_gcodes[axis];
        if (axis == E_AXIS) {
            if (!hot_enough()) {
                // shortcut: boîte de dialogue au lieu du toast de la maquette, à remplacer par le toast commun (UI-2)
                MsgBoxWarning(_("Nozzle too cold to extrude: heat it first."), Responses_Ok);
                return;
            }
        } else if (axis == Z_AXIS) {
            if (!homed(_BV(Z_AXIS))) {
                return; // Z sans origine : le plateau pourrait toucher la buse, on propose la prise d'origine
            }
        } else if (!known(_BV(axis))) {
            gcode = free_gcodes[axis]; // X ou Y sans origine : arrêt au contact d'un bord
        }
        // Comme le menu Prusa : le pas d'E par défaut pousse un peu moins que demandé, compensé ici seulement
        gcode.parameter = axis == E_AXIS ? mm * CALIBRATED_AXIS_STEPS_PER_UNIT_E0 / get_steps_per_unit_e() : mm;
        if (marlin_client::inject(gcode) && axis == E_AXIS) {
            e_moved += mm;
        }
    }

    template <AxisEnum axis, int dir>
    void step() {
        move_by(axis, dir * step_mm[step_index]);
    }

    /// X et Y ensemble ; un seul des deux sans origine : l'un après l'autre
    template <uint8_t direction>
    void step_diagonal() {
        if (!not_printing()) {
            return;
        }
        const bool x_known = known(_BV(X_AXIS)), y_known = known(_BV(Y_AXIS));
        if (x_known == y_known) {
            (void)marlin_client::inject((x_known ? diag_gcodes : diag_free_gcodes)[direction][step_index]);
            return;
        }
        const float d = step_mm[step_index];
        move_by(X_AXIS, (direction == 0 || direction == 2) ? -d : d);
        move_by(Y_AXIS, direction < 2 ? d : -d);
    }

    void open_mmu_axes() {
        open_screen<ScreenMMUMotors>();
    }

    /// Va au point touché comme la liste de Prusa (G123 : vitesse manuelle, arrêté par l'ordre suivant) ;
    /// la carte ne couvre que le plateau, à l'intérieur des butées
    void go_to(float x, float y) {
        if (!not_printing() || !homed(_BV(X_AXIS) | _BV(Y_AXIS))) {
            return;
        }
        ArrayStringBuilder<32> gcode;
        gcode.append_printf("G123 X%.1f Y%.1f", static_cast<double>(x), static_cast<double>(y));
        (void)marlin_client::gcode_try(gcode.str()); // file pleine : le toucher est perdu, comme un clic de trop chez Prusa
    }

    template <uint8_t i>
    void select_step() {
        step_index = i;
    }
    constexpr Card::Action step_actions[] = { select_step<0>, select_step<1>, select_step<2>, select_step<3> };

    /// « X 125.0 », « X -- » tant que l'axe n'a pas d'origine
    void axis_text(char (&out)[12], char axis, float mm, bool is_known) {
        if (is_known) {
            snprintf(out, sizeof(out), "%c %.1f", axis, static_cast<double>(mm));
        } else {
            snprintf(out, sizeof(out), "%c --", axis);
        }
    }

    /// Chevron de 13 × 7 px en trait de 2 px, centré sur c
    void draw_chevron(point_i16_t c, MoveKey::Arrow arrow, Color color) {
        if (arrow >= MoveKey::Arrow::up_left) {
            // Chevron d'angle de 9 px, trait de 2 px, pointe vers le coin
            const int16_t sx = (arrow == MoveKey::Arrow::up_left || arrow == MoveKey::Arrow::down_left) ? -1 : 1;
            const int16_t sy = (arrow == MoveKey::Arrow::up_left || arrow == MoveKey::Arrow::up_right) ? -1 : 1;
            const int16_t tip_x = c.x + 4 * sx, tip_y = c.y + 4 * sy;
            for (int16_t t = 0; t < 2; ++t) {
                display::draw_line(point_ui16(tip_x - t * sx, tip_y), point_ui16(tip_x - t * sx, tip_y - 8 * sy), color);
                display::draw_line(point_ui16(tip_x, tip_y - t * sy), point_ui16(tip_x - 8 * sx, tip_y - t * sy), color);
            }
            return;
        }
        const bool vertical = arrow == MoveKey::Arrow::up || arrow == MoveKey::Arrow::down;
        const int16_t s = (arrow == MoveKey::Arrow::down || arrow == MoveKey::Arrow::right) ? 1 : -1; // côté de la pointe
        for (int16_t t = 0; t < 2; ++t) {
            for (int16_t side = -1; side <= 1; side += 2) {
                if (vertical) {
                    display::draw_line(point_ui16(c.x + 6 * side, c.y - 3 * s + t), point_ui16(c.x, c.y + 3 * s + t), color);
                } else {
                    display::draw_line(point_ui16(c.x - 3 * s + t, c.y + 6 * side), point_ui16(c.x + 3 * s + t, c.y), color);
                }
            }
        }
    }

    /// Disque de diamètre d, ligne par ligne : les coins du carré ne sont pas touchés (même règle que spool::Disc)
    void fill_disc(point_i16_t at, int16_t d, Color color) {
        for (int16_t y = 0; y < d; ++y) {
            const int dy = 2 * y + 1 - d;
            int16_t x = 0;
            while (x < d / 2 && (2 * x + 1 - d) * (2 * x + 1 - d) + dy * dy >= d * d) {
                ++x;
            }
            display::fill_rect(Rect16(at.x + x, at.y + y, d - 2 * x, 1), color);
        }
    }

} // namespace

// ---- Touches ----

MoveKey::MoveKey(window_t *parent, Rect16 body, Action action, Color back, Arrow arrow, const string_view_utf8 &label, bool arrow_below, Color arrow_color)
    : Card(parent, style::with_ring(body), action, arrow == Arrow::none ? style::line : style::secondary, back, style::radius)
    , label(label)
    , arrow(arrow)
    , arrow_below(arrow_below)
    , arrow_color(arrow_color) {}

void MoveKey::set_dimmed(bool set) {
    if (set != dimmed) {
        dimmed = set;
        invalidate_content(); // chevron et libellé seuls, sans repeindre la touche
    }
}

void MoveKey::draw_content(Rect16 body) {
    const int16_t cx = body.Left() + body.Width() / 2;
    if (arrow == Arrow::none) {
        draw_icon({ int16_t(cx - 8), int16_t(body.Top() + (body.Height() - 16) / 2) }, img::home_full_16x16, fill());
        return;
    }
    if (arrow >= Arrow::up_left) {
        draw_chevron({ cx, int16_t(body.Top() + body.Height() / 2) }, arrow, dimmed ? style::text_faint : arrow_color);
        return;
    }
    // Chevron (18 px) et libellé (16 px) centrés ensemble, 4 px entre eux
    const int16_t top = body.Top() + (body.Height() - 38) / 2;
    const int16_t arrow_top = arrow_below ? top + 20 : top;
    const int16_t label_top = arrow_below ? top : top + 22;
    draw_chevron({ cx, int16_t(arrow_top + 9) }, arrow, dimmed ? style::text_faint : arrow_color);
    draw_text(Rect16(body.Left() + 1, label_top, body.Width() - 2, height(Font::small)), label, Font::small, dimmed ? style::text_faint : style::text_strong, fill(), Align_t::Center());
}

StepChip::StepChip(window_t *parent, uint8_t index)
    : Card(parent, style::with_ring(step_body(index)), step_actions[index], style::surface, style::screen, 15)
    , index(index) {}

void StepChip::unconditionalDraw() {
    // Choisi : fond brun et contour orange de 2 px (SPEC_G.md §1.5), d'une passe ; ciblé : un ton plus clair
    const bool chosen = index == step_index;
    const Color base = chosen ? selected_back : style::surface;
    const Color bg = IsFocused() ? style::focused(base) : base;
    draw_frame(GetRect(), back, style::accent, bg, radius, chosen ? 2 : 0);
    const Rect16 b = body();
    draw_text(Rect16(b.Left() + 2, b.Top() + (b.Height() - height(Font::small)) / 2, b.Width() - 4, height(Font::small)), string_view_utf8::MakeCPUFLASH(step_labels[index]), Font::small, style::text_strong, bg, Align_t::Center());
}

// ---- Textes ----

LiveText::LiveText(window_t *parent, Rect16 rect, Align_t align)
    : window_t(parent, rect)
    , align(align) {}

void LiveText::set(const char *value, Color value_color) {
    if (value_color == color && strcmp(value, text.data()) == 0) {
        return;
    }
    strlcpy(text.data(), value, text.size());
    color = value_color;
    Invalidate();
}

void LiveText::unconditionalDraw() {
    draw_text(GetRect(), string_view_utf8::MakeRAM(text.data()), Font::small, color, style::screen, align);
}

CrossPanel::CrossPanel(window_t *parent, Rect16 rect)
    : window_t(parent, rect) {}

void CrossPanel::unconditionalDraw() {
    display::draw_rounded_rect(GetRect(), style::screen, style::surface, style::radius, MIC_ALL_CORNERS);
}

// ---- Carte du plateau ----

BedMap::BedMap(window_t *parent)
    : window_t(parent, map_rect)
    , hint(_("tap to go there")) {
    // Légende en bas à gauche, à sa longueur (17 caractères au plus) : elle ne touche pas les coins arrondis
    const uint16_t chars = std::min<uint16_t>(hint.computeNumUtf8Chars(), 17);
    hint_rect = Rect16(map_inner.Left() + 6, map_inner.Bottom() - 19, chars * width(Font::small), height(Font::small));
}

void BedMap::set(std::optional<point_i16_t> p) {
    if (p == head) {
        return;
    }
    head = p;
    // Comme HeaterCard : le fond reste valide si rien d'autre ne l'a invalidé (dialogue refermé)
    const bool background_valid = HasValidBackground();
    Invalidate();
    if (background_valid) {
        ValidateBackground();
    }
}

void BedMap::draw_grid(Rect16 r) const {
    // Tous les 50 mm en X et 55 mm en Y (SPEC_G.md §3.17)
    for (int mm = 50; mm < bed_x; mm += 50) {
        if (const int16_t x = pixel_x(mm); x >= r.Left() && x <= r.Right()) {
            display::fill_rect(Rect16(x, r.Top(), 1, r.Height()), map_grid);
        }
    }
    for (int mm = 55; mm < bed_y; mm += 55) {
        if (const int16_t y = pixel_y(mm); y >= r.Top() && y <= r.Bottom()) {
            display::fill_rect(Rect16(r.Left(), y, r.Width(), 1), map_grid);
        }
    }
    if (r.HasIntersection(hint_rect)) {
        draw_text(hint_rect, hint, Font::small, hint_color, map_back);
    }
}

void BedMap::paint(Rect16 r) const {
    display::fill_rect(r, map_back);
    draw_grid(r);
}

void BedMap::unconditionalDraw() {
    if (!HasValidBackground()) {
        // Trait et fond d'une passe : les coins du trait suivent l'arrondi (map_radius à l'intérieur)
        draw_rounded_frame(map_rect, style::screen, style::line, map_back, map_radius + 1, 1);
        draw_grid(map_inner);
    } else if (drawn) {
        // Seuls l'ancien point et les traits du réticule qui ont bougé s'effacent : pas de clignotement
        if (!head || head->x != drawn->x) {
            paint(crosshair_column(drawn->x));
        }
        if (!head || head->y != drawn->y) {
            paint(crosshair_row(drawn->y));
        }
        paint(dot_rect(*drawn));
    }
    drawn = head;
    if (head) {
        display::fill_rect(crosshair_column(head->x), crosshair_color);
        display::fill_rect(crosshair_row(head->y), crosshair_color);
        const Rect16 dot = dot_rect(*head);
        fill_disc({ dot.Left(), dot.Top() }, dot_d, style::text_strong);
        fill_disc({ int16_t(dot.Left() + 2), int16_t(dot.Top() + 2) }, dot_d - 4, style::accent);
    }
}

// ---- Écran ----

ScreenMove::ScreenMove()
    : Screen(true)
    , header(this, _("Move axes"), true)
    , cross(this, cross_rect)
    , x_minus(this, x_minus_body, step<X_AXIS, -1>, style::surface, MoveKey::Arrow::left, string_view_utf8::MakeCPUFLASH("X-"))
    , x_plus(this, x_plus_body, step<X_AXIS, 1>, style::surface, MoveKey::Arrow::right, string_view_utf8::MakeCPUFLASH("X+"))
    , y_plus(this, y_plus_body, step<Y_AXIS, 1>, style::surface, MoveKey::Arrow::up, string_view_utf8::MakeCPUFLASH("Y+"))
    , y_minus(this, y_minus_body, step<Y_AXIS, -1>, style::surface, MoveKey::Arrow::down, string_view_utf8::MakeCPUFLASH("Y-"), true)
    , origin_xy(this, origin_xy_body, home_xy, style::surface, MoveKey::Arrow::none)
    , xy_up_left(this, up_left_body, step_diagonal<0>, style::surface, MoveKey::Arrow::up_left)
    , xy_up_right(this, up_right_body, step_diagonal<1>, style::surface, MoveKey::Arrow::up_right)
    , xy_down_left(this, down_left_body, step_diagonal<2>, style::surface, MoveKey::Arrow::down_left)
    , xy_down_right(this, down_right_body, step_diagonal<3>, style::surface, MoveKey::Arrow::down_right)
    , z_value(this, z_value_rect, Align_t::Center())
    , z_plus(this, z_plus_body, step<Z_AXIS, 1>, style::screen, MoveKey::Arrow::up, string_view_utf8::MakeCPUFLASH("Z+"))
    , z_minus(this, z_minus_body, step<Z_AXIS, -1>, style::screen, MoveKey::Arrow::down, string_view_utf8::MakeCPUFLASH("Z-"), true)
    , e_value(this, e_value_rect, Align_t::Center())
    , extrude(this, extrude_body, step<E_AXIS, 1>, style::screen, MoveKey::Arrow::down, _("Extrude"), false, style::accent)
    , retract(this, retract_body, step<E_AXIS, -1>, style::screen, MoveKey::Arrow::up, _("Retract"))
    , map(this)
    , step_title(this, step_title_rect, Align_t::LeftCenter())
    , steps { { this, 0 }, { this, 1 }, { this, 2 }, { this, 3 } }
    , position(this, position_rect, Align_t::LeftCenter())
    , origin_all(this, origin_all_body, home_all, _("Home XYZ"), &img::home_full_16x16, style::secondary, style::text_strong)
    , motors(this, motors_body, disable_motors, _("Motors off"), nullptr, style::secondary, style::text_strong)
    , mmu_axes(this, mmu_axes_body, open_mmu_axes, _("MMU axes"), &img::spool_16x16, style::secondary, style::text_strong)
    , tabs(this, section_tab())
    , shown_step(step_index) {
    ClrMenuTimeoutClose(); // comme la liste de Prusa : l'écran reste ouvert pendant les déplacements
    e_moved = 0;
    char title[8];
    _("Step").copyToRAM(title, sizeof(title));
    step_title.set(title);
    update();
    x_plus.SetFocus();
}

void ScreenMove::update() {
    const bool xy_known = known(_BV(X_AXIS) | _BV(Y_AXIS));
    const float x = marlin_vars().native_pos[X_AXIS].get();
    const float y = marlin_vars().native_pos[Y_AXIS].get();

    char a[12], b[12], text[24];
    axis_text(a, 'X', x, xy_known);
    axis_text(b, 'Y', y, xy_known);
    snprintf(text, sizeof(text), "%s  %s", a, b);
    position.set(text, style::text_soft);

    axis_text(a, 'Z', marlin_vars().native_pos[Z_AXIS].get(), known(_BV(Z_AXIS)));
    z_value.set(a);

    const bool cold = !hot_enough();
    if (cold) {
        _("E cold").copyToRAM(text, sizeof(text));
    } else {
        snprintf(text, sizeof(text), "E %.1f", static_cast<double>(e_moved));
    }
    e_value.set(text, cold ? style::warning : style::text_muted);
    extrude.set_dimmed(cold);
    retract.set_dimmed(cold);

    map.set(xy_known ? std::optional<point_i16_t>(point_i16_t { pixel_x(x), pixel_y(y) }) : std::nullopt);
}

void ScreenMove::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::HELD_RELEASED:
        if (GetFirstDialog()) {
            return; // sous une boîte de dialogue, l'appui long ne referme pas l'écran
        }
        break;
    case GUI_event_t::TOUCH_CLICK:
        // La carte n'est pas active (hors de la molette) : son toucher est lu ici
        if (const point_ui16_t p = event_conversion_union { .pvoid = param }.point; map_rect.Contain(p)) {
            go_to(mm_x(p.x), mm_y(p.y));
            return;
        }
        break;
    case GUI_event_t::LOOP:
        update();
        break;
    default:
        break;
    }
    Screen::windowEvent(sender, event, param);

    if (shown_step != step_index) { // un pas vient d'être choisi : seules l'ancienne et la nouvelle puce changent
        steps[shown_step].Invalidate();
        steps[step_index].Invalidate();
        shown_step = step_index;
    }
}

} // namespace od3d
