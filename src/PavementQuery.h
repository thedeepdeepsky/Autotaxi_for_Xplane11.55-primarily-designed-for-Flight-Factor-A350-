#pragma once
#include "AptDatabase.h"
#include <algorithm>
namespace autotaxi {
// A local projection avoids repeated trigonometry during curve and detour probes.
class PavementQuery {
    struct Ring {
        std::vector<Vec2> points;
        Vec2 low{1e30, 1e30}, high{-1e30, -1e30};
        std::vector<std::vector<std::size_t>> rows;
        double rowHeight = 1;
        std::size_t row(double y) const {
            return static_cast<std::size_t>(
                std::clamp((y - low.y + .5) / rowHeight, 0.0, static_cast<double>(rows.size() - 1)));
        }
        void index() {
            const auto count =
                static_cast<std::size_t>(std::clamp(std::ceil((high.y - low.y + 1) / 8), 1., 512.));
            rowHeight = std::max(1., high.y - low.y + 1) / count;
            rows.resize(count);
            if (points.empty())
                return;
            Vec2 previous = points.back();
            for (std::size_t i = 0; i < points.size(); ++i) {
                const auto current = points[i];
                for (auto r = row(std::min(previous.y, current.y) - .5);
                     r <= row(std::max(previous.y, current.y) + .5); ++r)
                    rows[r].push_back(i);
                previous = current;
            }
        }
        bool contains(Vec2 p) const {
            if (points.size() < 3 || p.x < low.x - .5 || p.x > high.x + .5 || p.y < low.y - .5 ||
                p.y > high.y + .5)
                return false;
            bool inside = false;
            // Only edges crossing this Y range can hit the boundary or the horizontal ray.
            for (auto i : rows[row(p.y)]) {
                Vec2 current = points[i] - p, previous = points[i ? i - 1 : points.size() - 1] - p;
                Vec2 delta = current - previous;
                const double squared = dot(delta, delta);
                const double t = squared > 1e-9 ? std::clamp(-dot(previous, delta) / squared, 0., 1.) : 0;
                const Vec2 nearest = previous + delta * t;
                if (dot(nearest, nearest) < .25)
                    return true;
                if ((current.y > 0) != (previous.y > 0) &&
                    0 < (previous.x - current.x) * (-current.y) / (previous.y - current.y) + current.x)
                    inside = !inside;
            }
            return inside;
        }
    };
    GeoPoint origin_;
    std::vector<std::vector<Ring>> surfaces_;

  public:
    PavementQuery(const Airport &airport, GeoPoint origin, bool includeRunways = false) : origin_(origin) {
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
                ring.index();
                rings.push_back(std::move(ring));
            }
            if (!rings.empty())
                surfaces_.push_back(std::move(rings));
        }
        if (includeRunways)
            for (const auto &runway : airport.runways) {
                Vec2 a = project(origin, runway.ends[0].position),
                     b = project(origin, runway.ends[1].position);
                double span = length(b - a);
                if (span < 1 || runway.width <= 0)
                    continue;
                Vec2 side{(b.y - a.y) / span * runway.width / 2, -(b.x - a.x) / span * runway.width / 2};
                Ring ring;
                ring.points = {a + side, b + side, b - side, a - side};
                for (auto point : ring.points) {
                    ring.low = {std::min(ring.low.x, point.x), std::min(ring.low.y, point.y)};
                    ring.high = {std::max(ring.high.x, point.x), std::max(ring.high.y, point.y)};
                }
                ring.index();
                surfaces_.push_back({std::move(ring)});
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
    bool wheelEnvelope(GeoPoint cg, double yaw, double wheelbase, double mainAxleAft, double halfSpan,
                       double margin = 0) const {
        Vec2 rear = project(origin_, cg) - direction(yaw) * mainAxleAft;
        Vec2 lateral{direction(yaw).y * (halfSpan + margin), -direction(yaw).x * (halfSpan + margin)};
        return contains(unproject(origin_, rear + lateral)) && contains(unproject(origin_, rear - lateral)) &&
               contains(unproject(origin_, rear + direction(yaw) * wheelbase));
    }
};
} // namespace autotaxi
