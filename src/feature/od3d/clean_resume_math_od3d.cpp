/// @file
#include "clean_resume_math_od3d.hpp"
#include "feature_marks_od3d.hpp"

#include <algorithm>
#include <initializer_list>
#include <limits>

namespace od3d::clean_resume {

namespace {

    float dist(Point a, Point b) {
        return std::hypot(b.x - a.x, b.y - a.y);
    }

    /// Point du tracé à l'abscisse s (s_at : abscisse de chaque point gardé)
    Point point_at(const Run &run, const std::array<float, Run::max_points> &s_at, float s) {
        for (size_t i = 1; i < run.count; ++i) {
            if (s <= s_at[i] || i + 1 == run.count) {
                const float len = s_at[i] - s_at[i - 1];
                const float t = len > 0 ? std::clamp((s - s_at[i - 1]) / len, 0.0f, 1.0f) : 0.0f;
                const Point a = run.at(i - 1), b = run.at(i);
                return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t };
            }
        }
        return run.at(0);
    }

} // namespace

Settings Settings::clamped() const {
    return Settings {
        .enabled = enabled,
        .purge_tenths = std::min(purge_tenths, purge_max),
        .retract_tenths = std::min(retract_tenths, retract_max),
    };
}

void Run::push(Point p) {
    if (count < max_points) {
        ring[(first + count) % max_points] = p;
        ++count;
    } else {
        // Plein : le plus ancien point est remplacé, le suivant devient le premier
        ring[first] = p;
        first = uint8_t((first + 1) % max_points);
    }
}

void Run::clear() {
    first = 0;
    count = 0;
    z = NAN;
    total_mm = 0;
    total_e = 0;
    object = -1;
}

float Run::kept_mm() const {
    float mm = 0;
    for (size_t i = 1; i < count; ++i) {
        mm += dist(at(i - 1), at(i));
    }
    return mm;
}

void Tracker::close_run() {
    if (current_.count >= 2 && current_.kept_mm() >= min_run_mm) {
        last_ = current_;
    }
    current_.clear();
}

void Tracker::reset() {
    feature_ = 0;
    current_.clear();
    last_.clear();
    layer_z_ = NAN;
    prev_layer_z_ = NAN;
}

void Tracker::set_feature(uint8_t feature) {
    if (feature != feature_) {
        close_run();
    }
    feature_ = feature;
}

bool Tracker::move(Point from, float z0, float e0, Point to, float z1, float e1) {
    const float de = e1 - e0;
    const float mm = dist(from, to);
    // Seuls comptent les mouvements qui extrudent dans le plan : pas les déplacements, les rétractions, ni les
    // mouvements qui changent Z (mode vase)
    if (!(de > 0) || !std::isfinite(de) || !(mm > 0) || !std::isfinite(mm) || !(std::fabs(z1 - z0) <= z_eps)) {
        return false;
    }
    if (!std::isfinite(layer_z_) || z1 < layer_z_ - z_eps) {
        // Première couche, ou Z qui redescend (objets imprimés l'un après l'autre) : les couches repartent d'ici
        prev_layer_z_ = NAN;
        layer_z_ = z1;
    } else if (z1 > layer_z_ + z_eps) {
        prev_layer_z_ = layer_z_;
        layer_z_ = z1;
    }
    if (feature_ != feature_marks::infill) {
        return false;
    }
    // À la suite du tracé précédent s'il part de son dernier point, à la même hauteur ; sinon une nouvelle suite
    const bool follows = current_.count > 0 && std::fabs(current_.z - z1) <= z_eps && dist(current_.at(current_.count - 1), from) <= join_mm;
    if (!follows) {
        close_run();
        current_.z = z1;
        current_.push(from);
    }
    current_.push(to);
    current_.total_mm += mm;
    current_.total_e += de;
    return !follows;
}

// Pas en ligne : ses variables (~150 octets) quittent la pile de Marlin avant les mouvements de la reprise
[[gnu::noinline]] Refusal plan_infill(const Settings &settings, const Tracker &tracker, float resume_z, float resume_e, float current_e,
    const Box &limits, bool (*cancelled)(int16_t), InfillPlan &out) {
    // Remis à zéro sur place : pas d'InfillPlan temporaire sur la pile
    out.purge = {};
    out.z = NAN;
    out.purge_e = 0;
    out.line_count = 0;
    out.e_per_mm = 0;
    out.retract = 0;
    out.e_after = NAN;
    const Settings s = settings.clamped();
    if (!s.enabled) {
        return Refusal::disabled;
    }
    const float unretract = resume_e - current_e;
    if (!std::isfinite(unretract) || unretract < 0 || unretract > max_unretract_mm) {
        return Refusal::e_position;
    }
    if (!std::isfinite(tracker.prev_layer_z()) || !std::isfinite(resume_z)) {
        return Refusal::first_layer;
    }

    // La suite la plus récente assez longue
    const Run *run = nullptr;
    for (const Run *r : { &tracker.current(), &tracker.last() }) {
        if (r->count >= 2 && r->kept_mm() >= min_run_mm) {
            run = r;
            break;
        }
    }
    if (!run) {
        return Refusal::no_infill;
    }
    // Couche en cours ou précédente, jamais au-dessus de la reprise ni trop loin dessous
    if (!(run->z >= tracker.prev_layer_z() - z_eps) || !(run->z <= resume_z + z_eps) || !(resume_z - run->z <= max_layer_gap_mm)) {
        return Refusal::too_far;
    }
    if (cancelled && run->object >= 0 && cancelled(run->object)) {
        return Refusal::cancelled;
    }
    const float e_per_mm = run->e_per_mm();
    if (!(e_per_mm >= min_e_per_mm && e_per_mm <= max_e_per_mm)) {
        return Refusal::extrusion;
    }
    const size_t n = run->count;
    for (size_t i = 0; i < n; ++i) {
        const Point p = run->at(i);
        if (!(p.x >= limits.x_min && p.x <= limits.x_max && p.y >= limits.y_min && p.y <= limits.y_max)) {
            return Refusal::limits;
        }
    }

    // Abscisse de chaque point le long du tracé, et barycentre des segments (pondéré par leur longueur)
    std::array<float, Run::max_points> s_at {};
    float cx = 0, cy = 0;
    for (size_t i = 1; i < n; ++i) {
        const Point a = run->at(i - 1), b = run->at(i);
        const float len = dist(a, b);
        s_at[i] = s_at[i - 1] + len;
        cx += (a.x + b.x) * 0.5f * len;
        cy += (a.y + b.y) * 0.5f * len;
    }
    const float total = s_at[n - 1];
    cx /= total;
    cy /= total;

    // Purge : milieux des segments et points intérieurs, assez loin des bouts ; le plus proche du barycentre (au milieu
    // d'une longue ligne plutôt que sur un raccord le long d'un périmètre) ; à égalité, le plus central le long du tracé
    float best_s = NAN;
    float best_d2 = std::numeric_limits<float>::infinity();
    Point best;
    const auto consider = [&](float at, Point p) {
        const float margin = std::min(at, total - at);
        if (margin < edge_margin_mm) {
            return;
        }
        const float d2 = (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy);
        if (d2 < best_d2 - 1e-6f || (std::fabs(d2 - best_d2) <= 1e-6f && margin > std::min(best_s, total - best_s))) {
            best_s = at;
            best_d2 = d2;
            best = p;
        }
    };
    for (size_t i = 1; i < n; ++i) {
        const Point a = run->at(i - 1), b = run->at(i);
        consider((s_at[i - 1] + s_at[i]) * 0.5f, Point { (a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f });
        if (i + 1 < n) {
            consider(s_at[i], b);
        }
    }
    if (!std::isfinite(best_s)) {
        return Refusal::short_run;
    }
    out.purge = best;

    // Cordon : le long du tracé, du côté où il en reste le plus, jamais à moins de edge_margin_mm d'un bout
    const float ahead_mm = total - edge_margin_mm - best_s;
    const float behind_mm = best_s - edge_margin_mm;
    const bool ahead = ahead_mm >= behind_mm;
    const float length = std::min(prime_line_mm, ahead ? ahead_mm : behind_mm);
    if (length >= min_line_mm) {
        const float end_s = ahead ? best_s + length : best_s - length;
        constexpr float eps = 1e-4f;
        if (ahead) {
            for (size_t i = 0; i < n && out.line_count + 1u < InfillPlan::max_line; ++i) {
                if (s_at[i] > best_s + eps && s_at[i] < end_s - eps) {
                    out.line[out.line_count++] = run->at(i);
                }
            }
        } else {
            for (size_t i = n; i-- > 0 && out.line_count + 1u < InfillPlan::max_line;) {
                if (s_at[i] < best_s - eps && s_at[i] > end_s + eps) {
                    out.line[out.line_count++] = run->at(i);
                }
            }
        }
        out.line[out.line_count++] = point_at(*run, s_at, end_s);
    }

    out.z = resume_z;
    out.purge_e = unretract + s.purge_tenths / 10.0f;
    out.e_per_mm = e_per_mm;
    out.retract = s.retract_tenths / 10.0f;
    out.e_after = resume_e - out.retract;
    return Refusal::none;
}

} // namespace od3d::clean_resume
