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
    // This compact Bezier fixture tests paint extraction; use an aircraft that fits without oversteer.
    options.wheelbase = 5;
    options.cockpitAheadNose = 1;
    options.mainAxleAft = 1;
    options.minimumTurnRadius = 3;
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
void lineTypeSemantics() {
    std::istringstream input(
        "1 0 0 0 TYPES Line types\n"
        "120 Yellow\n111 0 0 1 101\n115 0 .001\n"
        "120 LightOnly\n111 0 .002 101\n115 0 .003\n"
        "120 Safety\n111 0 .004 105 0\n115 0 .005\n"
        "120 Road\n111 0 .006 20\n115 0 .007\n"
        "120 BrokenRoad\n111 0 .008 22\n115 0 .009\n"
        "120 Hold\n111 0 .010 4\n115 0 .011\n");
    const auto airport = parseAirport(input);
    check(airport.groundLines.size() == 6, "Read each line type as a separate feature");
    check(airport.groundLines[0].centerline() && airport.groundLines[1].centerline() &&
              airport.groundLines[2].centerline(),
          "Painted and lighted centerline types are routable");
    check(!airport.groundLines[3].centerline() && !airport.groundLines[4].centerline() &&
              !airport.groundLines[5].centerline(),
          "Road and hold markings remain display-only");

    std::ostringstream stand;
    stand << "1 0 0 0 STAND White lead-in\n"
          << "1201 " << point(0, 0) << " both 1 A\n"
          << "1201 " << point(0, 100) << " both 2 B\n"
          << "1202 1 2 twoway taxiway_E A\n"
          << "120 White stand lead-in\n"
          << "111 " << point(0, 80) << " 20\n"
          << "115 " << point(0, 100) << "\n"
          << "1300 " << point(0, 100) << " 0 gate jets Gate 1\n"
          << "1301 C airline\n";
    std::istringstream standInput(stand.str());
    const auto withStand = parseAirport(standInput);
    check(withStand.groundLines.size() == 1 && !withStand.groundLines.front().centerline() &&
              !withStand.groundLines.front().standLeadIn,
          "White 20/22 roadway markings remain display-only");

    stand.str("");
    stand.clear();
    stand << "1 0 0 0 VENDOR Vendor stand line\n"
          << "1201 " << point(0, 0) << " both 1 A\n"
          << "1201 " << point(0, 100) << " both 2 B\n"
          << "1202 1 2 twoway taxiway_E A\n"
          << "120 Vendor white stand line\n"
          << "111 " << point(0, 60) << " 30\n"
          << "115 " << point(0, 140) << "\n"
          << "1300 " << point(0, 100) << " 0 gate jets Gate 2\n"
          << "1301 C airline\n";
    std::istringstream vendorInput(stand.str());
    const auto vendor = parseAirport(vendorInput);
    check(vendor.groundLines.size() == 1 && !vendor.groundLines.front().standLeadIn,
          "White vendor roadway markings remain display-only");
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
void standProximity() {
    auto geo = [](double x, double y) { return unproject({0, 0}, {x, y}); };
    Airport airport;
    airport.ramps.push_back({geo(10, 0), 0, 'E', "Near middle"});
    airport.ramps.push_back({geo(0, 0), 0, 'E', "At stop"});
    airport.ramps.push_back({geo(45, 300), 0, 'E', "At distance limit"});
    airport.groundLines = {
        {20, "Long white lead-in", {geo(0, -100), geo(0, 100)}},
        {22, "Nearby interior", {geo(20, -100), geo(20, 100)}},
        {31, "Distance limit", {geo(0, 200), geo(0, 400)}},
        {30, "Too far", {geo(80, -100), geo(80, 100)}},
        {20, "Crosswise road", {geo(-100, 0), geo(100, 0)}},
        {20, "Compact stand line", {geo(0, 0), geo(0, 35)}},
        {51, "Yellow stand lead-in", {geo(0, 0), geo(0, 35)}},
        {51, "Yellow taxiway main", {geo(0, 35), geo(0, 100)}},
        {4, "Hold line", {geo(0, -100), geo(0, 100)}},
        {52, "Pavement edge", {geo(0, -100), geo(0, 100)}}};
    buildCenterlineNetwork(airport);
    check(!airport.groundLines[0].standLeadIn && !airport.groundLines[1].standLeadIn,
          "Long lines passing a stand remain display-only");
    check(!airport.groundLines[2].standLeadIn,
          "A line without a stand-connected endpoint remains display-only");
    check(!airport.groundLines[5].standLeadIn && airport.groundLines[5].standLeadInRamp == -1,
          "A white compact service line remains display-only");
    check(airport.groundLines[6].standLeadInRamp == 1,
          "A yellow centerline terminating at a stand is identified as lead-in");
    check(!airport.groundLines[7].standLeadIn,
          "The taxiway main line connected to a stand spur is not itself lead-in");
    airport.ramps.push_back({geo(46, 300), 0, 'E', "Beyond distance limit"});
    airport.groundLines.push_back({20, "Beyond limit", {geo(0, 500), geo(0, 700)}});
    buildCenterlineNetwork(airport);
    check(!airport.groundLines.back().standLeadIn,
          "Lines beyond the lead-in proximity limit remain display-only");
    for (std::size_t i : {3u, 4u, 8u, 9u})
        check(!airport.groundLines[i].centerline(),
              "Far, crosswise, holding and edge markings cannot become stand guidance");
    airport.ramps.clear();
    buildCenterlineNetwork(airport);
    check(!airport.groundLines[5].standLeadIn && airport.groundLines[5].standLeadInRamp == -1 &&
              !airport.groundLines[6].standLeadIn && airport.groundLines[6].standLeadInRamp == -1,
          "Rebuilding clears obsolete stand associations");
}
Airport paintedEntry(double offset = 0, double along = 100, bool disconnected = false, char width = 'E',
                     double runwayLength = 2000) {
    Airport a;
    a.id = "TEST";
    a.runways.push_back({45, {{"01", {0, 0}, 0}, {"19", unproject({0, 0}, {0, runwayLength}), 0}}});
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
    a.pavements.push_back(
        {{{geo(-260, along - 130), geo(60, along - 130), geo(60, along + 150), geo(-260, along + 150)}}});
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
    route = planRoute(a, a.nodes.at(1).position, 90, departure, options);
    check(!a.paintedRunwayEntries.empty() && route.cockpitGuidance,
          "A converging paint curve may end before the axis and continue smoothly");
    check(std::any_of(a.paintedRunwayEntries.begin(), a.paintedRunwayEntries.end(),
                      [](const PaintedRunwayEntry &entry) { return !entry.continuation.empty(); }),
          "Unpainted continuations must remain distinguishable from observed yellow paint");
    const auto paintEnd = a.groundLines.back().points.back();
    double endGap = 1e30, maxTurn = 0;
    for (std::size_t i = 0; i < route.points.size(); ++i) {
        endGap = std::min(endGap, distance(paintEnd, unproject(route.origin, route.points[i])));
        if (i > 0 && i + 1 < route.points.size())
            maxTurn = std::max(maxTurn, std::abs(wrap180(heading(route.points[i + 1] - route.points[i]) -
                                                         heading(route.points[i] - route.points[i - 1]))));
    }
    check(endGap < .5 && maxTurn < 10,
          "Retain the final observed paint point and avoid abrupt ATC-chord lineup");
    a = paintedEntry(8, 690, false, 'E', 3000);
    route = planRoute(a, a.nodes.at(1).position, 90, departure, options);
    check(route.runway && !route.requiresPushback && !a.paintedRunwayEntries.empty(),
          "Departure-end policy uses the observed paint endpoint even when continuation passes 700 m");
    check(std::any_of(a.paintedRunwayEntries.begin(), a.paintedRunwayEntries.end(),
                      [](const PaintedRunwayEntry &entry) {
                          return !entry.continuation.empty() &&
                                 project({0, 0}, entry.continuation.back()).y > 700;
                      }),
          "Boundary fixture must continue beyond the departure-end limit");
    a = paintedEntry(25);
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
    a = paintedEntry(25);
    a.groundLines.push_back(
        {51, "Parallel marking", {unproject({0, 0}, {1, 100}), unproject({0, 0}, {1, 200})}});
    buildCenterlineNetwork(a);
    check(a.paintedRunwayEntries.empty(), "A nearby parallel marking alone is not an entrance");
}
Airport brokenLine(double gap = 18, double offset = 0, bool hole = false, bool paved = true,
                   bool conflictingDirection = false, bool oneWay = false) {
    Airport a;
    a.nodes[1] = {1, unproject({0, 0}, {0, 0}), "Start"};
    a.nodes[2] = {2, unproject({0, 0}, {200 + gap, offset}), "End"};
    a.edges.push_back({1, 2, oneWay, false, 'E', "A", {}, {}, false});
    a.groundLines.push_back({51, "A", {unproject({0, 0}, {0, 0}), unproject({0, 0}, {100, 0})}});
    a.groundLines.push_back(
        {51, "A", {unproject({0, 0}, {100 + gap, offset}), unproject({0, 0}, {200 + gap, offset})}});
    if (paved) {
        Pavement surface;
        surface.rings.push_back({unproject({0, 0}, {-20, -40}), unproject({0, 0}, {260, -40}),
                                 unproject({0, 0}, {260, 40}), unproject({0, 0}, {-20, 40})});
        if (hole)
            surface.rings.push_back({unproject({0, 0}, {105, -10}), unproject({0, 0}, {114, -10}),
                                     unproject({0, 0}, {114, 10}), unproject({0, 0}, {105, 10})});
        a.pavements.push_back(std::move(surface));
    }
    if (conflictingDirection) {
        a.nodes[3] = {3, unproject({0, 0}, {100, 0}), {}};
        a.nodes[4] = {4, unproject({0, 0}, {100 + gap, offset}), {}};
        a.edges.clear();
        a.edges.push_back({1, 3, true, false, 'E', "A", {}, {}, false});
        a.edges.push_back({2, 4, true, false, 'E', "A", {}, {}, false});
    }
    buildCenterlineNetwork(a);
    return a;
}
void shortGaps() {
    auto a = brokenLine();
    check(a.centerlineGapLinks == 1, "An aligned 18 m paved paint break is connected");
    auto bridge =
        std::find_if(a.edges.begin(), a.edges.end(), [](const TaxiEdge &e) { return e.inferredGap; });
    check(bridge != a.edges.end() && bridge->geometry.size() > 20 && bridge->width == 'E',
          "The inferred bridge preserves width rules and sampled tangent geometry");
    RouteOptions o;
    o.maxJoinDistance = o.maxApronJoinDistance = 8;
    const auto route = planRoute(a, a.nodes.at(1).position, 90, {DestinationKind::Node, "End", 2}, o);
    check(route.length < 240 && route.atcFallbackCount == 0 && route.inferredGapCount == 1,
          "A short paint gap follows the local centerline rather than an ATC chord or detour");
    check(brokenLine(21).centerlineGapLinks == 0, "Do not bridge long unmarked gaps");
    check(brokenLine(18, 1).centerlineGapLinks == 1, "A shallow offset retains tangent continuity");
    check(brokenLine(18, 3).centerlineGapLinks == 0, "Reject a gap curve with insufficient turn radius");
    check(brokenLine(18, 12).centerlineGapLinks == 0, "Do not jump across parallel lanes");
    check(brokenLine(18, 0, true).centerlineGapLinks == 0, "A pavement hole blocks gap repair");
    check(brokenLine(18, 0, false, false).centerlineGapLinks == 0,
          "Unknown pavement cannot establish an inferred paint connection");
    check(brokenLine(18, 0, false, true, true).centerlineGapLinks == 0,
          "Two incoming one-way ends must remain disconnected");
    a = brokenLine(18, 0, false, true, false, true);
    bridge = std::find_if(a.edges.begin(), a.edges.end(), [](const TaxiEdge &e) { return e.inferredGap; });
    check(bridge != a.edges.end() && bridge->oneWay &&
              project({0, 0}, a.nodes.at(bridge->to).position).x >
                  project({0, 0}, a.nodes.at(bridge->from).position).x,
          "Gap repair inherits the permitted ATC direction");
    bool reverseRejected = false;
    try {
        planRoute(a, a.nodes.at(2).position, 270, {DestinationKind::Node, "Start", 1}, o);
    } catch (const std::exception &) {
        reverseRejected = true;
    }
    check(reverseRejected, "An inferred gap cannot open a reverse one-way route");
    a = brokenLine();
    for (auto &e : a.edges)
        if (e.inferredGap) {
            e.oneWay = true;
            if (project({0, 0}, a.nodes.at(e.from).position).x >
                project({0, 0}, a.nodes.at(e.to).position).x) {
                std::swap(e.from, e.to);
                std::reverse(e.geometry.begin(), e.geometry.end());
            }
            e.activeRunways = {"01/19"};
        }
    bool rejected = false;
    try {
        planRoute(a, a.nodes.at(1).position, 90, {DestinationKind::Node, "End", 2}, o);
    } catch (const std::exception &) {
        rejected = true;
    }
    check(rejected, "An inferred bridge still requires its active-runway clearance");
}
Airport terminalMerge(bool sharp = false, bool reverseMain = false, bool hole = false) {
    Airport a;
    auto geo = [](Vec2 p) { return unproject({0, 0}, p); };
    a.nodes[1] = {1, geo({-100, 0}), {}};
    a.nodes[2] = {2, geo({100, 0}), {}};
    a.nodes[3] = {3, geo(sharp ? Vec2{0, -100} : Vec2{-100, -50}), {}};
    a.nodes[4] = {4, geo(sharp ? Vec2{0, -2} : Vec2{-2, -1}), {}};
    a.edges.push_back(
        {reverseMain ? 2 : 1, reverseMain ? 1 : 2, reverseMain, false, 'F', "B", {"02/20"}, {}, false});
    a.edges.push_back({3, 4, true, false, 'E', "A", {"01/19"}, {}, false});
    a.groundLines.push_back({51, "B", {a.nodes.at(1).position, a.nodes.at(2).position}});
    a.groundLines.push_back({51, "A", {a.nodes.at(3).position, a.nodes.at(4).position}});
    Pavement surface;
    surface.rings.push_back({geo({-150, -120}), geo({150, -120}), geo({150, 50}), geo({-150, 50})});
    if (hole)
        surface.rings.push_back({geo({-5, -3}), geo({1, -3}), geo({1, 1}), geo({-5, 1})});
    a.pavements.push_back(std::move(surface));
    buildCenterlineNetwork(a);
    return a;
}
void shallowMergeGaps() {
    auto a = terminalMerge();
    auto bridge =
        std::find_if(a.edges.begin(), a.edges.end(), [](const TaxiEdge &e) { return e.inferredGap; });
    check(a.centerlineGapLinks == 1 && bridge != a.edges.end() && bridge->oneWay && bridge->width == 'E' &&
              bridge->activeRunways == std::vector<std::string>({"01/19", "02/20"}),
          "A shallow endpoint-to-line merge preserves both sides' width, direction and runway rules");
    RouteOptions o;
    o.maxJoinDistance = o.maxApronJoinDistance = 8;
    o.runwayClearance = true;
    o.via = {"A", "B"};
    auto route =
        planRoute(a, a.nodes.at(3).position, heading(project(a.nodes.at(3).position, a.nodes.at(4).position)),
                  {DestinationKind::Node, "End", 2}, o);
    check(route.length < 240 && route.inferredGapCount == 1 && route.atcFallbackCount == 0,
          "Named taxiways join automatically across a short tangent extension");
    check(terminalMerge(true).centerlineGapLinks == 0, "Do not invent a right-angle turn to nearby paint");
    check(terminalMerge(false, true).centerlineGapLinks == 0,
          "A tangent extension cannot connect incompatible one-way flows");
    check(terminalMerge(false, false, true).centerlineGapLinks == 0,
          "A tangent extension cannot cross a pavement hole");
}
} // namespace
int main() {
    try {
        lineTypeSemantics();
        standProximity();
        curvesAndRestrictions();
        apronJunction();
        entriesWithoutRunwayAtc();
        shortGaps();
        shallowMergeGaps();
        std::cout << "Centerline checks passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
}
