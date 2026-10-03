#include "PavementQuery.h"
#include <iostream>
#include <random>
#include <stdexcept>
using namespace autotaxi;
namespace {
void check(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}
bool referenceRing(Vec2 p, const std::vector<Vec2> &ring) {
    if (ring.size() < 3)
        return false;
    bool inside = false;
    Vec2 previous = ring.back() - p;
    for (auto vertex : ring) {
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
void indexedBoundary() {
    const GeoPoint origin{31, 121};
    Airport airport;
    Pavement surface;
    std::vector<GeoPoint> boundary;
    for (int i = 0; i < 1800; ++i) {
        const double angle = i * 2 * pi / 1800, radius = 150 + 45 * std::cos(7 * angle);
        boundary.push_back(unproject(origin, {radius * std::cos(angle), radius * std::sin(angle)}));
    }
    surface.rings.push_back(boundary);
    surface.rings.push_back({unproject(origin, {-20, -20}), unproject(origin, {20, -20}),
                             unproject(origin, {20, 20}), unproject(origin, {-20, 20})});
    // Degenerate rings and duplicate/tiny edges must retain the old boundary tolerance.
    surface.rings[0].insert(surface.rings[0].begin() + 5, surface.rings[0][4]);
    surface.rings[0].insert(surface.rings[0].begin() + 6,
                            unproject(origin, project(origin, surface.rings[0][5]) + Vec2{1e-6, 1e-6}));
    airport.pavements.push_back(surface);
    airport.pavements.push_back({{{origin}}});
    airport.pavements.push_back({{{}}});
    const PavementQuery query(airport, origin);
    std::vector<std::vector<Vec2>> rings;
    for (const auto &ring : surface.rings) {
        rings.push_back({});
        for (auto p : ring)
            rings.back().push_back(project(origin, p));
    }
    auto compare = [&](Vec2 p) {
        const auto geo = unproject(origin, p);
        p = project(origin, geo);
        const bool expected = referenceRing(p, rings[0]) && !referenceRing(p, rings[1]);
        check(query.contains(geo) == expected, "Indexed boundary differs from full polygon scan");
    };
    std::mt19937 random(1768);
    std::uniform_real_distribution<double> coordinate(-220, 220);
    for (int i = 0; i < 15000; ++i)
        compare({coordinate(random), coordinate(random)});
    for (auto p : rings[0])
        for (Vec2 offset : {Vec2{}, {.49, 0}, {.51, 0}, {0, .49}, {0, .51}, {0, -.49}, {0, -.51}})
            compare(p + offset);
    for (double x : {-20.51, -20.49, 0., 20.49, 20.51})
        for (double y : {-20.51, -20.49, 0., 20.49, 20.51})
            compare({x, y});
    check(!query.connection(unproject(origin, {-60, 0}), unproject(origin, {60, 0})),
          "Pavement hole was bypassed by indexed connection");
}
void runwayAndMargins() {
    const GeoPoint origin{31, 121};
    Airport airport;
    Runway runway;
    runway.width = 60;
    runway.ends[0].position = origin;
    runway.ends[1].position = unproject(origin, {0, 1000});
    airport.runways.push_back(runway);
    const PavementQuery query(airport, origin, true);
    check(query.contains(unproject(origin, {30.49, 500})), "Runway boundary tolerance lost");
    check(!query.contains(unproject(origin, {30.51, 500})), "Runway boundary tolerance enlarged");
    check(query.wheelEnvelope(unproject(origin, {0, 500}), 0, 28.35, 2.2, 6.5), "Runway wheel envelope lost");
    check(!query.wheelEnvelope(unproject(origin, {29, 500}), 0, 28.35, 2.2, 6.5),
          "Wheel crossing was missed");
}
} // namespace
int main() {
    try {
        indexedBoundary();
        runwayAndMargins();
        std::cout << "Indexed pavement matches full scan, including holes, boundaries and runway wheels\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
