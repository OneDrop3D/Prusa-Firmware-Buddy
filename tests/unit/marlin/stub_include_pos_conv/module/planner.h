#pragma once

#include <core/macros.h>
#include <core/types.h>

inline struct Planner {
    static constexpr bool leveling_active = true;

    static inline float z_fade_height = 0;
    static inline float inverse_z_fade_height = 0;

    // OneDrop3D (F15) : équerrage XY (valeurs de la CORE One)
    static inline float skew_xy = 0.0f;
    static constexpr float skew_y_center = 110.0f;
    static constexpr float skew_x_min = -2.0f, skew_x_max = 252.0f;

    static void set_z_fade_height(const float zfh) {
        z_fade_height = zfh > 0 ? zfh : 0;
        inverse_z_fade_height = RECIPROCAL(z_fade_height);
    }
} planner;

inline struct UBL {

    static inline float z_correction = 0;

    static inline float get_z_correction(const xy_pos_t &) {
        return z_correction;
    }

} ubl;
