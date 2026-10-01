#pragma once
#include <cmath>
namespace autotaxi {
constexpr double pi = 3.14159265358979323846;
constexpr double rad = pi / 180.0;
struct GeoPoint {
    double lat = 0, lon = 0;
};
struct Vec2 {
    double x = 0, y = 0;
    Vec2 operator+(Vec2 b) const {
        return {x + b.x, y + b.y};
    }
    Vec2 operator-(Vec2 b) const {
        return {x - b.x, y - b.y};
    }
    Vec2 operator*(double s) const {
        return {x * s, y * s};
    }
};
double dot(Vec2 a, Vec2 b);
double length(Vec2 a);
double wrap180(double degrees);
double distance(GeoPoint a, GeoPoint b);
bool valid(GeoPoint p);
Vec2 direction(double heading);
double heading(Vec2 v);
Vec2 project(GeoPoint origin, GeoPoint point);
GeoPoint unproject(GeoPoint origin, Vec2 point);
struct Projection {
    Vec2 point;
    double t = 0, distance = 0;
};
Projection onSegment(Vec2 p, Vec2 a, Vec2 b);
} // namespace autotaxi
