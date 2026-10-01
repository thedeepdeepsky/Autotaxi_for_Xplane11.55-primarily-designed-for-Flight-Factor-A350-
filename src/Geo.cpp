#include "Geo.h"
#include <algorithm>
namespace autotaxi {
double dot(Vec2 a, Vec2 b) {
    return a.x * b.x + a.y * b.y;
}
double length(Vec2 a) {
    return std::hypot(a.x, a.y);
}
double wrap180(double d) {
    return d - 360.0 * std::floor((d + 180.0) / 360.0);
}
bool valid(GeoPoint p) {
    return std::isfinite(p.lat) && std::isfinite(p.lon) && std::abs(p.lat) <= 90 && std::abs(p.lon) <= 180;
}
Vec2 direction(double h) {
    return {std::sin(h * rad), std::cos(h * rad)};
}
double heading(Vec2 v) {
    double h = std::atan2(v.x, v.y) / rad;
    return h < 0 ? h + 360 : h;
}
double distance(GeoPoint a, GeoPoint b) {
    double s = std::sin((b.lat - a.lat) * rad / 2), t = std::sin(wrap180(b.lon - a.lon) * rad / 2);
    double v = s * s + std::cos(a.lat * rad) * std::cos(b.lat * rad) * t * t;
    return 6371000.0 * 2 * std::asin(std::sqrt(std::clamp(v, 0.0, 1.0)));
}
Vec2 project(GeoPoint o, GeoPoint p) {
    return {wrap180(p.lon - o.lon) * rad * 6371000.0 * std::cos(o.lat * rad),
            (p.lat - o.lat) * rad * 6371000.0};
}
GeoPoint unproject(GeoPoint o, Vec2 p) {
    return {o.lat + p.y / (rad * 6371000.0),
            wrap180(o.lon + p.x / (rad * 6371000.0 * std::cos(o.lat * rad)))};
}
Projection onSegment(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 d = b - a;
    double n = dot(d, d), t = n > 1e-9 ? std::clamp(dot(p - a, d) / n, 0.0, 1.0) : 0;
    Vec2 q = a + d * t;
    return {q, t, length(p - q)};
}
} // namespace autotaxi
