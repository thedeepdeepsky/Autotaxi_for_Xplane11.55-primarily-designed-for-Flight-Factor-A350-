// Compatibility layer for the CDDL-licensed BetterPushback v1.10 segment planner.
#pragma once
#include "Geo.h"
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <vector>
namespace bpgeometry {
using vect2_t = autotaxi::Vec2;
using bool_t = int;
constexpr bool_t B_TRUE = 1, B_FALSE = 0;
constexpr int SEG_TYPE_STRAIGHT = 0, SEG_TYPE_TURN = 1;
struct seg_t {
    int type = 0;
    vect2_t start_pos, end_pos;
    double start_hdg = 0, end_hdg = 0;
    bool_t backward = 0;
    double len = 0;
    struct { double r = 0; bool_t right = 0; } turn;
};
struct vehicle_t { double wheelbase = 28.35, max_steer = 45; };
struct list_t { std::vector<seg_t *> items; };
inline seg_t *safe_calloc(std::size_t, std::size_t size) {
    return static_cast<seg_t *>(std::calloc(1, size));
}
inline void list_insert_tail(list_t *list, seg_t *item) { list->items.push_back(item); }
inline seg_t *list_remove_tail(list_t *list) {
    auto *item = list->items.back(); list->items.pop_back(); return item;
}
inline vect2_t hdg2dir(double h) { return autotaxi::direction(h); }
inline double dir2hdg(vect2_t v) { return autotaxi::heading(v); }
inline double rel_hdg(double a, double b) { return autotaxi::wrap180(b - a); }
inline vect2_t vect2_add(vect2_t a, vect2_t b) { return a + b; }
inline vect2_t vect2_sub(vect2_t a, vect2_t b) { return a - b; }
inline vect2_t vect2_scmul(vect2_t a, double s) { return a * s; }
inline vect2_t vect2_neg(vect2_t a) { return a * -1; }
inline vect2_t vect2_mean(vect2_t a, vect2_t b) { return (a + b) * .5; }
inline double vect2_abs(vect2_t a) { return autotaxi::length(a); }
inline double vect2_dist(vect2_t a, vect2_t b) { return autotaxi::length(a - b); }
inline double vect2_dotprod(vect2_t a, vect2_t b) { return autotaxi::dot(a, b); }
inline vect2_t vect2_set_abs(vect2_t a, double size) { return a * (size / vect2_abs(a)); }
inline vect2_t vect2_norm(vect2_t a, int right) { return right ? vect2_t{a.y, -a.x} : vect2_t{-a.y, a.x}; }
inline vect2_t vect2_rot(vect2_t a, double angle) {
    double c = std::cos(angle * autotaxi::rad), s = std::sin(angle * autotaxi::rad);
    return {a.x * c + a.y * s, a.y * c - a.x * s};
}
inline vect2_t vect2vect_isect(vect2_t a, vect2_t originA, vect2_t b, vect2_t originB, bool_t confined) {
    double cross = a.x * b.y - a.y * b.x;
    vect2_t delta = originB - originA;
    if (std::abs(cross) > 1e-12) {
        double u = (delta.x * b.y - delta.y * b.x) / cross;
        double v = (delta.x * a.y - delta.y * a.x) / cross;
        if (!confined || (u >= 0 && u <= 1 && v >= 0 && v <= 1))
            return originA + a * u;
    }
    return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
}
int compute_segs(const vehicle_t *, vect2_t, double, vect2_t, double, list_t *);
} // namespace bpgeometry
#define MAX(a, b) std::max<double>((a), (b))
#define MIN(a, b) std::min<double>((a), (b))
#define ABS(a) std::abs(a)
#define DEG2RAD(a) ((a) * autotaxi::rad)
#define RAD2DEG(a) ((a) / autotaxi::rad)
#define IS_NULL_VECT(a) (!std::isfinite((a).x) || !std::isfinite((a).y))
