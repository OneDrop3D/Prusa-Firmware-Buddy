/// @file
#include <module/motion.h>
#include <module/planner.h>

#include <algorithm>

#if HAS_LEVELING
    #include <feature/bedlevel/bedlevel.h>
#endif

// This unit has UNITTESTS in machine_pos_conv_tests.cpp

MachinePosXYZ to_machine_pos(const xyz_pos_t &pos) {
    MachinePosXYZ result = pos.to_tag<MachinePosTag>();

#if HAS_MESH
    if (planner.leveling_active) {
    #if ENABLED(OD3D_SKEW_CORRECTION_XY)
        // OneDrop3D (F15) : équerrage XY (cisaillement autour du milieu du plateau en Y), avant le MBL : la hauteur du
        // maillage se lit là où la tête va vraiment. Jamais au-delà des courses de X, sauf si la position y est déjà.
        if (Planner::skew_xy != 0.0f) {
            const float x = pos.x - (pos.y - Planner::skew_y_center) * Planner::skew_xy;
            result.x = std::clamp(x, std::min(pos.x, Planner::skew_x_min), std::max(pos.x, Planner::skew_x_max));
        }
    #endif
        const float mbl_correction = ubl.get_z_correction(result.xy());

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        if (Planner::z_fade_height != 0.0f) {
            result.z += mbl_correction * std::clamp<float>(1 - pos.z * Planner::inverse_z_fade_height, 0, 1);
        } else
    #endif
        {
            // No fade height, apply verbatim
            result.z += mbl_correction;
        }
    }
#endif

    return result;
}

xyz_pos_t to_native_pos(const MachinePosXYZ &pos) {
    xyz_pos_t result = pos.to_tag<NativePosTag>();

#if HAS_MESH
    if (planner.leveling_active) {
        const float mbl_correction = ubl.get_z_correction(result.xy());

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        if (Planner::z_fade_height != 0) {
            if (pos.z >= Planner::z_fade_height) {
                // Above fade height - correction does not apply

            } else if (mbl_correction >= Planner::z_fade_height) {
                // The function stops being monotonous and math breaks down. Shouldn't happen in practice.

            } else if (pos.z < mbl_correction) {
                // Full correction applied - mirroring the clamp in to_machine_pos
                result.z = pos.z - mbl_correction;

            } else {
                // machine_z = native_z + mbl_correction * fade(native_z)
                // fade(z) = 1 - native_z/fade_height
                //
                // So:
                // - machine_z = native_z + mbl_correction * (1 - native_z / fade_height)
                // - machine_z = native_z + mbl_correction - mbl_correction * native_z / fade_height
                // - machine_z = native_z * (1 - mbl_correction / fade_height) + mbl_correction
                //
                // Solving for native_z:
                //   native_z = (machine_z - mbl_correction) / (1 - mbl_correction / fade_height)
                result.z = (pos.z - mbl_correction) / (1.0f - mbl_correction * Planner::inverse_z_fade_height);
            }
        } else
    #endif
        {
            // No fade
            result.z -= mbl_correction;
        }
    #if ENABLED(OD3D_SKEW_CORRECTION_XY)
        // OneDrop3D (F15) : inverse de l'équerrage (Y n'est pas modifié) ; une position bornée ne revient pas exactement
        if (Planner::skew_xy != 0.0f) {
            result.x = pos.x + (pos.y - Planner::skew_y_center) * Planner::skew_xy;
        }
    #endif
    }
#endif

    return result;
}
