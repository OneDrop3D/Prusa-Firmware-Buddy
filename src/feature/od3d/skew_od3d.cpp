/// @file
#include "skew_od3d.hpp"

#include <cmath>

namespace od3d::skew {

namespace {
    constexpr double min_length_mm = 10.0;
    constexpr double max_length_mm = 400.0;
    constexpr double pi = 3.14159265358979323846;

    bool length_ok(double v) {
        return std::isfinite(v) && v >= min_length_mm && v <= max_length_mm;
    }
} // namespace

Result factor_from_measures(float ac_f, float bd_f, float ad_f) {
    // En double : une seule fois, à la saisie ; les carrés (~20 000) se soustraient pour un écart de quelques dizaines
    const double ac = ac_f, bd = bd_f, ad = ad_f;
    if (!length_ok(ac) || !length_ok(bd) || !length_ok(ad)) {
        return { Status::inconsistent, 0.0f };
    }

    // Côté AB du parallélogramme (identité des diagonales : AC² + BD² = 2·AB² + 2·AD²)
    const double ab_sq4 = 2 * ac * ac + 2 * bd * bd - 4 * ad * ad;
    if (!(ab_sq4 > 0)) {
        return { Status::inconsistent, 0.0f };
    }
    const double ab = std::sqrt(ab_sq4) * 0.5;
    if (ab < 0.5 * ad || ab > 2 * ad) {
        return { Status::inconsistent, 0.0f }; // pas la pièce de mesure (ou AD faux)
    }

    const double cos_a = (ac * ac - ab * ab - ad * ad) / (2 * ad * ab);
    if (!(cos_a > -1 && cos_a < 1)) {
        return { Status::inconsistent, 0.0f };
    }
    const double k = std::tan(pi * 0.5 - std::acos(cos_a));
    if (!std::isfinite(k)) {
        return { Status::inconsistent, 0.0f };
    }
    if (std::fabs(k) > double(max_factor)) {
        return { Status::too_large, 0.0f };
    }
    return { Status::ok, float(k) };
}

float factor_to_degrees(float factor) {
    return float(std::atan(double(factor)) * 180.0 / pi);
}

bool is_valid_factor(float factor) {
    return std::isfinite(factor) && std::fabs(factor) <= max_factor;
}

} // namespace od3d::skew
