#pragma once
#include "AptDatabase.h"
#include <algorithm>
namespace autotaxi {
// A local projection avoids repeated trigonometry during curve and detour probes.
class PavementQuery {
    struct Ring {
        std::vector<Vec2> points;
        Vec2 low{1e30, 1e30}, high{-1e30, -1e30};
        bool contains(Vec2 p) const {
            if (points.size() < 3 || p.x < low.x - .5 || p.x > high.x + .5 || p.y < low.y - .5 ||
                p.y > high.y + .5)
                return false;
            bool inside = false;
            Vec2 previous = points.back() - p;
            for (auto vertex : points) {
                Vec2 current = vertex - p;
                if (onSegment({}, previous, current).distance < .5)
                    return true;
                if ((current.y > 0) != (previous.y > 0) &&
                    0 < (previous.x - current.x) * (-current.y) / (previous.y - current.y) + current.x)
                    inside = !inside;
                previous = current;
            }
            return inside;
        }
    };
    GeoPoint origin_;
    std::vector<std::vector<Ring>> surfaces_;

  public:
    PavementQuery(const Airport &airport, GeoPoint origin) : origin_(origin) {
        for (const auto &surface : airport.pavements) {
            std::vector<Ring> rings;
            for (const auto &source : surface.rings) {
                Ring ring;
                for (auto geo : source) {
                    Vec2 p = project(origin, geo);
                    ring.points.push_back(p);
                    ring.low = {std::min(ring.low.x, p.x), std::min(ring.low.y, p.y)};
                    ring.high = {std::max(ring.high.x, p.x), std::max(ring.high.y, p.y)};
                }
                rings.push_back(std::move(ring));
            }
            if (!rings.empty())
                surfaces_.push_back(std::move(rings));
        }
    }
    bool contains(GeoPoint position) const {
        Vec2 p = project(origin_, position);
        for (const auto &rings : surfaces_)
            if (rings.front().contains(p) && std::none_of(rings.begin() + 1, rings.end(),
                                                          [&](const Ring &hole) { return hole.contains(p); }))
                return true;
        return false;
    }
    bool connection(GeoPoint from, GeoPoint to) const {
        Vec2 delta = project(from, to);
        int count = std::max(1, static_cast<int>(std::ceil(length(delta) / 4)));
        for (int i = 0; i <= count; ++i)
            if (!contains(unproject(from, delta * (static_cast<double>(i) / count))))
                return false;
        return true;
    }
};
} // namespace autotaxi
