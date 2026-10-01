#include "CenterlineNetwork.h"
#include "RoutePlanner.h"
#include <algorithm>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
using namespace autotaxi;
namespace {
void check(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}
std::string point(double x, double y) {
    auto p = unproject({0, 0}, {x, y});
    std::ostringstream out;
    out.precision(15);
    out << p.lat << ' ' << p.lon;
    return out.str();
}
Airport curved(char width = 'E') {
    std::ostringstream apt;
    apt << "1 0 0 0 TEST Painted test\n"
        << "1201 " << point(0, 8) << " both 1 A\n1201 " << point(100, 108) << " both 2 B\n"
        << "1202 1 2 oneway taxiway_" << width << " A\n"
        << "120 Taxi centerline\n112 " << point(0, 0) << ' ' << point(60, 0) << " 51 101\n"
        << "116 " << point(100, 100) << ' ' << point(100, 140) << "\n"
        << "120 Hold line\n111 " << point(30, 10) << " 54\n115 " << point(50, 10) << "\n"
        << "120 Boundary\n111 " << point(40, -20) << " 2\n115 " << point(90, -20) << "\n";
    std::istringstream input(apt.str());
    return parseAirport(input);
}
void curvesAndRestrictions() {
    std::istringstream malformed("1 0 0 0 BAD Bad marking\n120 Centerline\n111 0 0 51\n"
                                 "112 bad data\n115 0 .001\n");
    check(parseAirport(malformed).groundLines.empty(),
          "A malformed marking must not fabricate a shortcut across missing coordinates");
    auto a = curved();
    check(a.groundLines.size() == 3, "Read yellow, holding and boundary markings separately");
    const auto &line = a.groundLines.front();
    check(line.centerline() && line.points.size() > 20, "Sample the actual Bezier centerline");
    check(!a.groundLines[1].centerline() && !a.groundLines[2].centerline(),
          "Holding and broken boundary lines must not become taxi routes");
    double error = 1e9;
    for (auto p : line.points)
        error = std::min(error, length(project({0, 0}, p) - Vec2{72.5, 35}));
    check(error < 3, "Incoming Bezier handle must be reflected, as in WED");
    check(a.atcFallbackEdges == 0, "Connected painted geometry replaces the offset ATC chord");
    check(std::abs(project({0, 0}, a.nodes.at(2).position).y - 100) < .01,
          "Correct an ATC node to the painted line");
    RouteOptions options;
    options.maxJoinDistance = 12;
    options.maxApronJoinDistance = 12;
    auto route = planRoute(a, a.nodes.at(1).position, 90, {DestinationKind::Node, "B", 2, 0}, options);
    check(!route.requiresPushback && route.points.size() > 20,
          "Route retains curve samples rather than a node chord");
    for (auto p : route.points) {
        auto geo = unproject(route.origin, p);
        double nearest = 1e9;
        for (std::size_t i = 1; i < line.points.size(); ++i)
            nearest = std::min(
                nearest,
                onSegment({}, project(geo, line.points[i - 1]), project(geo, line.points[i])).distance);
        check(nearest < .4, "Planned curve must coincide with the actual painted centerline");
    }
    bool rejected = false;
    try {
        planRoute(a, a.nodes.at(2).position, 180, {DestinationKind::Node, "A", 1, 0}, options);
    } catch (const std::exception &) {
        rejected = true;
    }
    check(rejected, "Painted graph must preserve ATC one-way restrictions");
    a = curved('C');
    rejected = false;
    try {
        planRoute(a, a.nodes.at(1).position, 90, {DestinationKind::Node, "B", 2, 0}, options);
    } catch (const std::exception &) {
        rejected = true;
    }
    check(rejected, "A painted line must not bypass an ATC class C restriction");
}
void apronJunction() {
    std::ostringstream apt;
    apt << "1 0 0 0 TEST Apron test\n"
        << "1201 " << point(0, 12) << " both 1 A\n1201 " << point(200, 12) << " both 2 B\n"
        << "1202 1 2 twoway taxiway_E A\n"
        << "120 Main\n111 " << point(0, 0) << " 1\n115 " << point(200, 0) << "\n"
        << "120 Apron\n111 " << point(100, 100) << " 51\n115 " << point(100, 0) << "\n";
    std::istringstream input(apt.str());
    auto a = parseAirport(input);
    std::map<int, int> degree;
    for (const auto &edge : a.edges) {
        ++degree[edge.from];
        ++degree[edge.to];
    }
    int junction = -1;
    for (const auto &n : a.nodes)
        if (length(project({0, 0}, n.second.position) - Vec2{100, 0}) < .5 && degree[n.first] >= 3)
            junction = n.first;
    check(junction >= 0, "An apron feature ending on a long centerline creates a real T junction");
    RouteOptions options;
    options.maxJoinDistance = 8;
    options.maxApronJoinDistance = 8;
    auto route = planRoute(a, unproject({0, 0}, {100, 95}), 180,
                           {DestinationKind::Node, "Junction", junction, 0}, options);
    check(!route.requiresPushback, "Apron yellow line is routable without a long tow to ATC nodes");
    for (auto p : route.points)
        check(std::abs(project({0, 0}, unproject(route.origin, p)).x - 100) < .5,
              "Apron route stays on its painted centerline");
}
Airport paintedEntry(double offset = 0, double along = 100, bool disconnected = false, char width = 'E') {
    Airport a;
    a.id = "TEST";
    a.runways.push_back({45, {{"01", {0, 0}, 0}, {"19", unproject({0, 0}, {0, 2000}), 0}}});
    auto geo = [&](double x, double y) { return unproject({0, 0}, {x - offset, y}); };
    a.nodes[1] = {1, geo(-200, along - 80), "Start"};
    a.nodes[2] = {2, geo(-160, along - 80), "ATC end"};
    a.edges.push_back({1, 2, false, false, width, "A", {}, {}, false});
    GroundLine approach{51, "Painted entry", {geo(-200, along - 80), geo(-160, along - 80)}};
    if (disconnected) {
        a.groundLines.push_back(approach);
        approach.points.clear();
    }
    approach.points.push_back(geo(-80, along - 80));
    for (int i = 1; i <= 40; ++i) {
        double theta = i * pi / 80;
        approach.points.push_back(geo(-80 + 80 * std::sin(theta), along - 80 * std::cos(theta)));
    }
    a.groundLines.push_back(approach);
    buildCenterlineNetwork(a);
    return a;
}
void entriesWithoutRunwayAtc() {
    RouteOptions options;
    options.runwayClearance = true;
    options.maxJoinDistance = 12;
    options.maxApronJoinDistance = 12;
    Destination departure{DestinationKind::Runway, "RWY 01", 0, 0};
    auto a = paintedEntry(.8);
    check(!a.paintedRunwayEntries.empty(), "Recognize painted lineup without ATC runway edges");
    check(std::none_of(a.edges.begin(), a.edges.end(), [](const TaxiEdge &e) { return e.runway; }),
          "A painted entry must not invent a runway graph or shortcuts");
    auto route = planRoute(a, a.nodes.at(1).position, 90, departure, options);
    check(!route.requiresPushback && route.points.size() > 30, "Follow the connected painted entry curve");
    check(std::abs(wrap180(route.finalHeading)) < .01, "Line up in the selected runway direction");
    check(std::abs(project({0, 0}, unproject(route.origin, route.points.back())).x) < .01,
          "Alignment ends on the surveyed runway axis");
    auto rejects = [&](const Airport &airport, const RouteOptions &settings) {
        try {
            planRoute(airport, airport.nodes.at(1).position, 90, departure, settings);
        } catch (const std::exception &) {
            return true;
        }
        return false;
    };
    options.runwayClearance = false;
    check(rejects(a, options), "Painted entries still require runway clearance");
    options.runwayClearance = true;
    a = paintedEntry(8);
    check(a.paintedRunwayEntries.empty() && rejects(a, options),
          "Nearby paint must not fabricate an entry across an unpainted gap");
    a = paintedEntry(0, 100, true);
    check(!a.paintedRunwayEntries.empty() && rejects(a, options),
          "A disconnected painted entry must not become reachable");
    a = paintedEntry(0, 100, false, 'C');
    check(rejects(a, options), "Painted entries must preserve known taxiway width restrictions");
    a = paintedEntry(0, 1000);
    check(rejects(a, options), "A far painted intersection is excluded by default");
    options.allowIntersectionDeparture = true;
    route = planRoute(a, a.nodes.at(1).position, 90, departure, options);
    check(route.runwayRemaining < 800, "Intersection departure reports reduced remaining runway");
    // This line runs beside the axis but never approaches from outside the runway strip.
    a = paintedEntry(8);
    a.groundLines.push_back(
        {51, "Parallel marking", {unproject({0, 0}, {1, 100}), unproject({0, 0}, {1, 200})}});
    buildCenterlineNetwork(a);
    check(a.paintedRunwayEntries.empty(), "A nearby parallel marking alone is not an entrance");
}
} // namespace
int main() {
    try {
        curvesAndRestrictions();
        apronJunction();
        entriesWithoutRunwayAtc();
        std::cout << "Centerline checks passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
}
