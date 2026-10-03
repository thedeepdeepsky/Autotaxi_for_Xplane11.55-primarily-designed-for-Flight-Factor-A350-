#include "Config.h"
#include "DsfLoader.h"
#include "PavementQuery.h"
#include "RouteTiming.h"
#include "TurnGuidance.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
using namespace autotaxi;
namespace {
const GeoPoint origin{31, 121};
void check(bool value, const std::string &message) {
    if (!value)
        throw std::runtime_error(message);
}
template <class F> void rejects(F operation, const char *message) {
    bool failed = false;
    try {
        operation();
    } catch (const std::exception &) {
        failed = true;
    }
    check(failed, message);
}
void node(Airport &a, int id, Vec2 point) {
    a.nodes[id] = {id, unproject(origin, point), {}};
}
void edge(Airport &a, int from, int to, std::string name, const std::vector<Vec2> &shape = {},
          bool runway = false) {
    TaxiEdge e;
    e.from = from;
    e.to = to;
    e.name = std::move(name);
    e.width = 'E';
    e.oneWay = true;
    e.runway = runway;
    e.painted = !shape.empty();
    for (auto point : shape)
        e.geometry.push_back(unproject(origin, point));
    a.edges.push_back(std::move(e));
}
void widePavement(Airport &a, double halfWidth = 400) {
    Pavement p;
    p.rings.push_back({});
    for (Vec2 point : {Vec2{-halfWidth, -400}, {halfWidth, -400}, {halfWidth, 2400}, {-halfWidth, 2400}})
        p.rings[0].push_back(unproject(origin, point));
    a.pavements.push_back(std::move(p));
}
RouteOptions options() {
    RouteOptions o;
    o.maxApronJoinDistance = o.maxJoinDistance = 8;
    return o;
}
void orderedRoute() {
    Airport a;
    node(a, 1, {0, 0});
    node(a, 2, {0, 100});
    node(a, 3, {70, 170});
    node(a, 4, {70, 270});
    node(a, 5, {0, 200});
    node(a, 6, {0, 350});
    edge(a, 1, 2, "A");
    edge(a, 2, 5, "SHORT");
    edge(a, 5, 6, "SHORT");
    edge(a, 2, 3, "B");
    edge(a, 3, 4, "C");
    edge(a, 4, 6, "D");
    widePavement(a);
    auto o = options();
    auto direct = planRoute(a, origin, 0, {DestinationKind::Node, "end", 6}, o);
    o.via = parseRouteVia("b > 4; d");
    check(o.via == std::vector<std::string>({"B", "#4", "D"}), "Parse ordered taxiways and nodes");
    auto specified = planRoute(a, origin, 0, {DestinationKind::Node, "end", 6}, o);
    check(specified.length > direct.length + 30 &&
              std::find(specified.nodeIds.begin(), specified.nodeIds.end(), 4) != specified.nodeIds.end(),
          "ATC waypoints force the longer branch in order");
    o.via = parseRouteVia("A > B > C > D");
    auto intersections = planRoute(a, origin, 0, {DestinationKind::Node, "end", 6}, o);
    check(intersections.nodeIds == std::vector<int>({2, 3, 4, 6}),
          "Taxiway names alone automatically resolve their connecting intersections");
    o.via = {"D", "B"};
    rejects([&] { planRoute(a, origin, 0, {DestinationKind::Node, "end", 6}, o); },
            "An unreachable taxiway order must not be reordered or ignored");
    o.via = {"MISSING"};
    rejects([&] { planRoute(a, origin, 0, {DestinationKind::Node, "end", 6}, o); },
            "Unknown ATC instruction must fail visibly");
}
void multipleIntersections() {
    Airport a;
    node(a, 1, {0, 0});
    node(a, 2, {0, 100});
    node(a, 3, {0, 200});
    node(a, 4, {100, 100});
    node(a, 5, {100, 200});
    node(a, 6, {200, 200});
    edge(a, 1, 2, "A");
    edge(a, 2, 3, "A");
    edge(a, 2, 4, "B"); // The first A/B intersection leads to a dead end.
    edge(a, 3, 5, "B");
    edge(a, 5, 6, "C");
    widePavement(a);
    auto o = options();
    o.via = parseRouteVia("a b c");
    const auto route = planRoute(a, origin, 0, {DestinationKind::Node, "End", 6}, o);
    check(route.nodeIds == std::vector<int>({2, 3, 5, 6}),
          "Resolve the globally connected intersection rather than the nearest name match");
}
Route gateRoute(bool allowOversteer, bool curved) {
    Airport a;
    std::vector<Vec2> paint{{0, 0}, {150, 0}};
    if (curved) {
        for (int i = 1; i <= 50; ++i) {
            double angle = i * pi / 100;
            paint.push_back({150 + 60 * std::sin(angle), 60 - 60 * std::cos(angle)});
        }
        paint.push_back({210, 200});
    } else
        paint.push_back({350, 0});
    node(a, 1, paint.front());
    node(a, 2, paint.back());
    edge(a, 1, 2, "LEAD-IN", paint);
    widePavement(a);
    const Vec2 axleStop = curved ? Vec2{210, 165} : Vec2{300, 0};
    const double yaw = curved ? 0 : 90;
    a.ramps.push_back({unproject(origin, axleStop + direction(yaw) * 2.2), yaw, 'E', "Test stand"});
    auto o = options();
    o.allowOversteer = allowOversteer;
    auto route = planRoute(a, origin, 90, {DestinationKind::Ramp, "Test stand", 0}, o);
    check(route.ramp && !route.requiresPushback, "Gate arrival stays a forward route");
    check(route.points.size() > (curved ? 50u : 2u), "Gate route retains the painted lead-in");
    check(length(route.cockpitStop - (axleStop + direction(yaw) * 2.2)) < .01,
          "The stand metadata projected onto paint is the cockpit stop");
    check(route.cockpitGuidance && !route.mainAxleOversteer && route.maximumOversteer < .1,
          "A wide painted approach retains cockpit guidance without unnecessary oversteer");
    for (const auto &point : route.points) {
        if (curved && point.y > 3 && point.y < 55)
            check(point.x > 150, "Gate arrival must not cut across the apron chord");
    }
    o.via = {"#2"};
    rejects([&] { planRoute(a, origin, 90, {DestinationKind::Ramp, "Test stand", 0}, o); },
            "A node beyond the truncated stand stop has not been visited");
    return route;
}
void simulateGate(const Route &route) {
    ControllerConfig config;
    TaxiController controller;
    controller.start(route, config);
    double yaw = route.initialHeading, speed = 0, cte = 0, arrivalCap = -1;
    Vec2 axle = direction(yaw) * -config.mainAxleAft;
    for (int step = 0; step < 24000 && controller.active(); ++step) {
        auto out = controller.update(
            {unproject(route.origin, axle + direction(yaw) * config.mainAxleAft), yaw, speed, true}, .05);
        check(out.phase != TaxiPhase::Fault, out.reason);
        if (out.remaining <= config.apronApproachDistance ||
            (route.apronArrivalDistance >= 0 && out.progress >= route.apronArrivalDistance)) {
            if (out.remaining < config.apronApproachDistance / 2)
                check(out.targetSpeed <= config.apronSpeed + 1e-6,
                      "Stand arrival must settle below the apron speed cap");
            if (arrivalCap >= 0)
                check(out.targetSpeed <= arrivalCap + 1e-6,
                      "Apron target speed must not increase after a bend");
            arrivalCap = out.targetSpeed;
        }
        cte = std::max(cte, out.crossTrack);
        speed = std::max(0., speed + (.39 - 1.5 * out.brake) * .05);
        yaw = wrap180(yaw + speed / config.wheelbase * std::tan(out.steerDegrees * rad) * .05 / rad);
        axle = axle + direction(yaw) * (speed * .05);
    }
    check(controller.phase() == TaxiPhase::Complete,
          "Gate braking must complete without stalling two metres short");
    Vec2 cockpit = axle + direction(yaw) * (config.wheelbase + config.cockpitAheadNose);
    check(length(cockpit - route.cockpitStop) < 1 && std::abs(wrap180(yaw - route.finalHeading)) < 2,
          "Cockpit, rather than CG or axle, stops at the displayed stand target");
    check(dot(cockpit - route.cockpitStop, direction(route.finalHeading)) < .5,
          "Cockpit must not overshoot the stand stop");
    std::cout << "Synthetic gate arrival max reference CTE: " << cte << " m\n";
}
Airport exits(bool atcAxis, bool paint) {
    Airport a;
    a.runways.push_back({60, {{"18", unproject(origin, {0, 2200})}, {"36", unproject(origin, {0, -200})}}});
    node(a, 1, {0, -200});
    node(a, 2, {0, 400});
    node(a, 3, {0, 2200});
    node(a, 4, {180, 280});
    node(a, 5, {80, 280});
    node(a, 6, {0, 200});
    if (atcAxis) {
        edge(a, 1, 2, "18/36", {}, true);
        edge(a, 2, 3, "18/36", {}, true);
    }
    edge(a, 2, 4, "UNMARKED");
    if (paint) {
        std::vector<Vec2> curve;
        for (int i = 0; i <= 60; ++i) {
            double angle = i * pi / 120;
            curve.push_back({80 - 80 * std::cos(angle), 200 + 80 * std::sin(angle)});
        }
        edge(a, 6, 5, "EXIT", curve);
        edge(a, 5, 4, "RAMP", {{80, 280}, {180, 280}});
        a.paintedRunwayEntries.push_back({6, 0, {}});
    }
    widePavement(a);
    return a;
}
void runwayExit() {
    auto o = options();
    o.runwayClearance = true;
    for (bool atcAxis : {false, true}) {
        std::cout << "Exit test ATC axis=" << atcAxis << '\n';
        auto a = exits(atcAxis, true);
        auto route = planRoute(a, origin, 0, {DestinationKind::Node, "Apron", 4}, o);
        check(!route.requiresPushback &&
                  std::find(route.nodeIds.begin(), route.nodeIds.end(), 5) != route.nodeIds.end(),
              "Runway exit follows painted ramp with or without an ATC runway axis");
        for (const auto &marker : route.taxiwaysAlongRoute)
            check(marker.label != "UNMARKED", "Reject unpainted ATC exit chord");
        a.runways.push_back(
            {60, {{"09", unproject(origin, {180, 280})}, {"27", unproject(origin, {2180, 280})}}});
        a.paintedRunwayEntries.push_back({4, 1, {}});
        auto onward = planRoute(a, origin, 0, {DestinationKind::Runway, "RWY 09", 1, 0}, o);
        check(!onward.requiresPushback &&
                  std::find(onward.nodeIds.begin(), onward.nodeIds.end(), 5) != onward.nodeIds.end(),
              "Runway-to-runway routes use the same painted exit as stand arrivals");
    }
    auto a = exits(true, false);
    edge(a, 1, 2, "UNMARKED ALIGNED TAXIWAY");
    rejects([&] { planRoute(a, origin, 0, {DestinationKind::Node, "Apron", 4}, o); },
            "Without a painted exit, do not invent a right-angle exit");
    o.paintedRunwayExitsOnly = false;
    node(a, 4, {180, 400});
    check(planRoute(a, origin, 0, {DestinationKind::Node, "Apron", 4}, o).points.size() >= 3,
          "Explicit legacy exit policy remains configurable");
}
void truncatedRunwayAxis() {
    auto airport = exits(true, true);
    node(airport, 1, {0, 100});
    node(airport, 3, {0, 2000});
    auto o = options();
    o.runwayClearance = true;
    const Destination goal{DestinationKind::Node, "Apron", 4};
    for (double along : {0., 25., 80.}) {
        auto start = unproject(origin, {0, -200 + along});
        auto route = planRoute(airport, start, 0, goal, o);
        check(!route.requiresPushback && !route.apronDeparture,
              "A truncated runway axis needs a forward runway join, not an apron connector or tow");
        for (auto point : route.points) {
            auto absolute = project(origin, unproject(route.origin, point));
            if (absolute.y < 100)
                check(std::abs(absolute.x) < .5, "The terminal gap follows the surveyed runway axis");
        }
        for (const auto &marker : route.taxiwaysAlongRoute)
            check(marker.label != "UNMARKED", "Terminal runway gaps do not permit unpainted ATC exits");
    }
    rejects([&] { planRoute(airport, unproject(origin, {0, -230}), 0, goal, o); },
            "Do not extend the surveyed runway beyond its physical endpoint");
    rejects([&] { planRoute(airport, unproject(origin, {0, 2100}), 180, goal, o); },
            "A terminal extension cannot reverse the existing one-way runway axis");
    o.runwayClearance = false;
    rejects([&] { planRoute(airport, unproject(origin, {0, -175}), 0, goal, o); },
            "Terminal axis joins still require runway clearance");
}
void disconnectedStand() {
    Airport a;
    node(a, 1, {-180, 0});
    node(a, 2, {180, 0});
    node(a, 3, {0, 5});
    node(a, 4, {0, 160});
    edge(a, 1, 2, "APRON", {{-180, 0}, {180, 0}});
    edge(a, 3, 4, "LEAD-IN", {{0, 5}, {0, 160}});
    a.ramps.push_back({unproject(origin, {0, 132.2}), 0, 'E', "Gap stand"});
    widePavement(a);
    auto o = options();
    auto route = planRoute(a, a.nodes.at(1).position, 90, {DestinationKind::Ramp, "Gap stand", 0}, o);
    check(route.inferredGapCount > 0 && !route.requiresPushback,
          "A short perpendicular stand gap has a smooth inferred entry");
    auto end = project(origin, unproject(route.origin, route.cockpitStop));
    check(length(end - Vec2{0, 132.2}) < .01, "Keep the cockpit stop on its own yellow lead-in");
    for (auto point : route.points) {
        auto local = project(origin, unproject(route.origin, point));
        // The turn may need compensation; the terminal straight must recover the painted lead-in.
        if (local.y > end.y - 10)
            check(std::abs(local.x) < .01, "Retain the painted final stand approach");
    }
    simulateGate(route);
    a.pavements.clear();
    rejects([&] { planRoute(a, a.nodes.at(1).position, 90, {DestinationKind::Ramp, "Gap", 0}, o); },
            "Unknown pavement cannot authorize an inferred stand entry");
}
void roundedJunctionNoise() {
    Airport a;
    node(a, 1, {0, 0});
    node(a, 2, {0, 200});
    edge(a, 1, 2, "PAINT", {{0, 0}, {0, 99}, {.25, 100}, {0, 101}, {0, 200}});
    widePavement(a);
    auto route = planRoute(a, origin, 0, {DestinationKind::Node, "end", 2}, options());
    check(route.length < 205, "Weld tolerance must not turn a straight painted junction into a detour");
    a.edges.clear();
    std::vector<Vec2> tight;
    for (int i = 0; i <= 24; ++i) {
        double t = i * pi / 48;
        tight.push_back({14 * std::sin(t), 14 - 14 * std::cos(t)});
    }
    node(a, 2, tight.back());
    edge(a, 1, 2, "TIGHT", tight);
    rejects([&] { planRoute(a, origin, 90, {DestinationKind::Node, "end", 2}, options()); },
            "Curvature tolerance must still reject a genuinely undersized painted arc");
}
void policies() {
    Airport a;
    node(a, 1, {0, 0});
    node(a, 2, {0, 160});
    edge(a, 1, 2, "NARROW", {{0, 0}, {0, 160}});
    a.edges[0].width = 'C';
    widePavement(a, 7);
    a.pavementCoverageComplete = true;
    auto o = options();
    rejects([&] { planRoute(a, origin, 0, {DestinationKind::Node, "end", 2}, o); },
            "Strict aircraft width rejects undersized taxiway");
    o.ignorePavementLimits = true;
    auto route = planRoute(a, origin, 0, {DestinationKind::Node, "end", 2}, o);
    check(std::any_of(route.pavementRisk.begin(), route.pavementRisk.end(), [](bool risk) { return risk; }),
          "Relaxed narrow segment retains orange risk");
    a.ramps.push_back({unproject(origin, {0, 102.2}), 0, 'C', "Small stand"});
    rejects([&] { planRoute(a, origin, 0, {DestinationKind::Ramp, "Small stand", 0}, o); },
            "Stand size is independently enforced");
    o.ignoreStandSize = true;
    check(planRoute(a, origin, 0, {DestinationKind::Ramp, "Small stand", 0}, o).destinationRisk,
          "Undersized stand retains terminal risk");
    o.allowOversteer = false;
    o.ignorePavementLimits = false;
    check(!turnClearance(40, 10, o).feasible,
          "Disabled oversteer rejects turns that need outward compensation");
}
void holdAndSpeed() {
    Route route;
    route.origin = origin;
    route.points = {{0, 0}, {0, 500}};
    route.length = 500;
    ControllerConfig config;
    config.mainAxleAft = 0;
    TaxiController controller;
    controller.start(route, config);
    AircraftState state{origin, 0, 4, true};
    auto before = controller.update(state, .05);
    controller.setTaxiSpeed(2);
    auto slowed = controller.update(state, .05);
    check(slowed.targetSpeed < before.targetSpeed && slowed.brake > 0,
          "Live speed changes reach brake controller");
    controller.setEmergencyBrake(true);
    state.position = unproject(origin, {30, 20});
    auto held = controller.update(state, .05);
    check(held.phase == TaxiPhase::Hold && held.brake == 1 && controller.active(),
          "Emergency holds control and route despite braking drift");
    state.position = unproject(origin, {0, 20});
    state.speed = 0;
    controller.setEmergencyBrake(false);
    auto resumed = controller.update(state, .05);
    check(resumed.phase == TaxiPhase::Taxi && resumed.remaining > 400 && resumed.targetSpeed <= 2,
          "Resume preserves route and selected speed");
    controller.setEmergencyBrake(true);
    state.onGround = false;
    check(controller.update(state, .05).phase == TaxiPhase::Fault,
          "Emergency does not suppress airborne telemetry interlock");
}
void dsfContours(const std::filesystem::path &fixtures) {
    std::istringstream text(
        "POLYGON_DEF pavement.pol\nPOLYGON_DEF road.pol\nPOLYGON_DEF photo.pol\n"
        "BEGIN_POLYGON 0 0 2\nBEGIN_WINDING\nPOLYGON_POINT 121 31\nPOLYGON_POINT 121.002 31\n"
        "POLYGON_POINT 121.002 31.002\nPOLYGON_POINT 121 31.002\nEND_WINDING\n"
        "BEGIN_WINDING\nPOLYGON_POINT 121.0008 31.0008\nPOLYGON_POINT 121.0012 31.0008\n"
        "POLYGON_POINT 121.0012 31.0012\nPOLYGON_POINT 121.0008 31.0012\nEND_WINDING\nEND_POLYGON\n"
        "BEGIN_POLYGON 1 0 2\nBEGIN_WINDING\nPOLYGON_POINT 121.003 31\nPOLYGON_POINT 121.004 31\n"
        "POLYGON_POINT 121.004 31.001\nEND_WINDING\nEND_POLYGON\n"
        "BEGIN_POLYGON 2 0 2\nBEGIN_WINDING\nPOLYGON_POINT 121.005 31\nPOLYGON_POINT 121.006 31\n"
        "POLYGON_POINT 121.006 31.001\nEND_WINDING\nEND_POLYGON\n");
    Airport a;
    auto result = parseDsfGeometry(text, a, fixtures);
    check(result.contours == 3 && result.pavements == 1 && !a.pavementCoverageComplete,
          "Airport pavement metadata imports only identified surfaces, never road paint or orthophoto "
          "footprints");
    PavementQuery query(a, origin);
    check(query.contains({31.0004, 121.0004}) && !query.contains({31.001, 121.001}),
          "DSF winding holes remain unpaved");
    check(query.wheelEnvelope({31.0004, 121.0004}, 0, 28.35, 2.2, 6.5) &&
              !query.wheelEnvelope({31.001, 121.001}, 0, 28.35, 2.2, 6.5),
          "Actual wheel footprint detects a polygon hole");
    text.clear();
    text.seekg(0);
    a = {};
    check(parseDsfGeometry(text, a, fixtures, "photo.pol").pavements == 2,
          "Explicit verified resources may supplement incomplete scenery metadata");
}
void dsfCurves(const std::filesystem::path &fixtures) {
    auto vertex = [](Vec2 point, Vec2 control) {
        auto p = unproject(origin, point), c = unproject(origin, control);
        std::ostringstream row;
        row.precision(15);
        row << "POLYGON_POINT " << p.lon << ' ' << p.lat << ' ' << c.lon << ' ' << c.lat << '\n';
        return row.str();
    };
    std::istringstream text("POLYGON_DEF Single_Yellow_solid.lin\nBEGIN_POLYGON 0 0 4\nBEGIN_WINDING\n" +
                            vertex({0, 0}, {60, 0}) + vertex({100, 100}, {100, 140}) +
                            vertex({100, 100}, {100, 100}) + vertex({100, 100}, {140, 100}) +
                            vertex({200, 100}, {200, 100}) + "END_WINDING\nEND_POLYGON\n");
    Airport a;
    check(parseDsfGeometry(text, a, fixtures).lines == 1 && a.groundLines[0].points.size() > 50,
          "DSF Bezier line imports sampled curvature with split handles");
    double gap = 1e30;
    for (auto p : a.groundLines[0].points)
        gap = std::min(gap, length(project(origin, p) - Vec2{72.5, 35}));
    check(gap < 2 && distance(a.groundLines[0].points.back(), unproject(origin, {200, 100})) < .01,
          "DSF incoming handle reflection follows the official WED curve rather than the anchor chord");
    std::istringstream malformed(
        "POLYGON_DEF Single_Yellow_solid.lin\nBEGIN_POLYGON 0 0 4\n"
        "BEGIN_WINDING\nPOLYGON_POINT 121 31\nPOLYGON_POINT 121.001 31\nEND_WINDING\nEND_POLYGON\n");
    a = {};
    check(parseDsfGeometry(malformed, a, fixtures).lines == 0,
          "Missing curve controls must not silently produce a straight route");
    std::istringstream uv(
        "POLYGON_DEF photo.pol\nBEGIN_POLYGON 0 65535 4\nBEGIN_WINDING\n"
        "POLYGON_POINT 121 31 0 0\nPOLYGON_POINT 121.001 31 1 0\nPOLYGON_POINT 121.001 31.001 1 1\n"
        "END_WINDING\nEND_POLYGON\n");
    a = {};
    check(parseDsfGeometry(uv, a, fixtures).contours == 1 && a.sceneryContours[0].rings[0].size() == 4,
          "Four-dimensional orthophoto UVs are not Bezier coordinates");
}
void aircraftConfig(const std::filesystem::path &configPath) {
    auto config = loadConfig(configPath);
    check(!config.requireA350 && config.controller.maxTaxiSpeed > 10 && config.route.allowOversteer,
          "Shipped profile supports other aircraft and a 20 kt configurable ceiling");
    auto customPath = std::filesystem::temp_directory_path() / "autotaxi-custom-geometry-test.ini";
    {
        std::ifstream original(configPath);
        std::ofstream custom(customPath);
        custom << original.rdbuf()
               << "\nwheelbase_m=20\ncockpit_ahead_nose_m=3\nmain_axle_aft_m=4\nmain_gear_half_track_m=4.5\n";
    }
    auto custom = loadConfig(customPath);
    std::filesystem::remove(customPath);
    check(custom.controller.mainGearHalfTrack == 4.5 && custom.route.wheelbase == 20 &&
              custom.route.cockpitAheadNose == 3 && custom.route.mainAxleAft == 4,
          "User INI overrides both map and route geometry without consulting aircraft ACF");
    AircraftState aircraft{origin, 0, 0, true};
    check(length(cockpitPosition(origin, custom.controller, aircraft) - Vec2{0, 19}) < .001 &&
              length(mainAxlePosition(origin, custom.controller, aircraft) - Vec2{0, -4}) < .001,
          "Configured geometry must move the actual cockpit and axle markers");
}
void localScenery(const std::filesystem::path &aptPath) {
    std::ifstream input(aptPath);
    auto airport = parseAirport(input, aptPath.u8string());
    RouteOptions o;
    o.runwayClearance = true;
    if (airport.nodes.count(39711)) {
        auto start = airport.nodes.at(39711).position;
        for (auto destination : destinations(airport)) {
            bool test = destination.kind == DestinationKind::Runway;
            if (destination.kind == DestinationKind::Ramp) {
                const auto &name = airport.ramps[destination.index].name;
                for (const auto *number : {"21", "23", "18", "90", "93"})
                    test = test || name == number || name == std::string("Gate ") + number ||
                           name == std::string("Gate") + number;
            }
            if (!test)
                continue;
            try {
                auto route = planRoute(airport, start, 162, destination, o);
                check(!route.requiresPushback,
                      "Runway departure must not require a tow: " + destination.label);
                std::cout << "Origin39711 " << destination.label << ": " << route.length
                          << " m, inferred=" << route.inferredGapCount << '\n';
                if (destination.label == "Gate 93") {
                    check(route.length < 5840, "Gate93 must not retain the painted-junction detour");
                    check(route.cockpitGuidance && !route.mainAxleOversteer && route.maximumOversteer > 0,
                          "Gate93 must remain cockpit-guided when oversteer is needed");
                    TaxiController controller;
                    ControllerConfig config;
                    controller.start(route, config);
                    auto prediction = forecastTaxiTiming(controller, {start, 162, 0, true}, {.39, 1.5});
                    check(prediction.totalSeconds > 0 && !prediction.cockpitPath.empty(),
                          "Node39711 to Gate93 must have a complete simulated cockpit trajectory");
                    check(length(prediction.cockpitPath.back().position - route.cockpitStop) < 1,
                          "Gate93 cockpit prediction must stop at the displayed target");
                    std::cout << "Gate93 predicted stop: " << prediction.totalSeconds << " s, error "
                              << length(prediction.cockpitPath.back().position - route.cockpitStop) << " m\n";
                    double along = 0;
                    for (std::size_t k = 1; k < route.points.size(); ++k) {
                        Vec2 delta = route.points[k] - route.points[k - 1];
                        along += length(delta);
                        if (along > route.length - 2600 && along < route.length - 1950)
                            check(std::abs(wrap180(heading(delta) - 71.56)) < 45,
                                  "The reported Gate93 junctions must not reverse into the detour arcs");
                    }
                    std::ofstream dump(std::filesystem::temp_directory_path() / "autotaxi-gate93-map.csv");
                    dump << "kind,id,x,y\n";
                    for (std::size_t i = 0; i < airport.groundLines.size(); ++i)
                        if (airport.groundLines[i].centerline())
                            for (auto geo : airport.groundLines[i].points) {
                                auto point = project(start, geo);
                                dump << "paint," << i << ',' << point.x << ',' << point.y << '\n';
                            }
                    for (auto point : route.points)
                        dump << "route,0," << point.x << ',' << point.y << '\n';
                }
            } catch (const std::exception &e) {
                if (destination.kind != DestinationKind::Ramp ||
                    airport.ramps[destination.index].name != "Gate 90")
                    throw;
                auto relaxed = o;
                relaxed.ignoreStandSize = true;
                auto route = planRoute(airport, start, 162, destination, relaxed);
                check(route.destinationRisk, "Relaxed Gate90 must retain its undersized stand warning");
                std::cout << "Origin39711 " << destination.label << ": " << e.what() << "; relaxed size "
                          << route.length << " m\n";
            }
        }
    }
    for (const std::string stand : {"590", "S1-122"}) {
        auto ramp = std::find_if(airport.ramps.begin(), airport.ramps.end(),
                                 [&](const Ramp &r) { return r.name == stand; });
        check(ramp != airport.ramps.end(), "Local arrival stand missing: " + stand);
        int goalMatches = 0;
        for (const auto &e : airport.edges)
            if (e.painted && !e.runway) {
                double gap = 1e30, yaw = 0;
                auto stop = unproject(ramp->position, direction(ramp->heading) * -o.mainAxleAft);
                for (std::size_t i = 1; i < e.geometry.size(); ++i) {
                    auto snap = onSegment({}, project(stop, e.geometry[i - 1]), project(stop, e.geometry[i]));
                    if (snap.distance < gap) {
                        gap = snap.distance;
                        yaw = heading(project(e.geometry[i - 1], e.geometry[i]));
                    }
                }
                double angle = std::abs(wrap180(yaw - ramp->heading));
                if (gap < 15 && (angle < 30 || (!e.oneWay && 180 - angle < 30))) {
                    ++goalMatches;
                    std::cout << "Local stand " << stand << " lead-in gap=" << gap
                              << " class=" << (e.width ? e.width : '?') << " one-way=" << e.oneWay
                              << " heading difference=" << angle << '\n';
                }
            }
        std::cout << "Local " << stand << " matching directed lead-ins: " << goalMatches << '\n';
        Destination destination{DestinationKind::Ramp, stand, static_cast<int>(ramp - airport.ramps.begin())};
        std::vector<std::pair<double, int>> candidates;
        for (const auto &n : airport.nodes) {
            double gap = distance(n.second.position, ramp->position);
            if (gap >= 50 && gap <= 1000)
                candidates.push_back({gap, n.first});
        }
        std::sort(candidates.begin(), candidates.end());
        std::optional<Route> arrival;
        std::string lastError;
        o.maxJoinDistance = o.maxApronJoinDistance = 8;
        for (std::size_t candidate = 0; candidate < std::min<std::size_t>(128, candidates.size()) && !arrival;
             ++candidate) {
            int id = candidates[candidate].second;
            for (const auto &e : airport.edges) {
                if (!e.painted || (e.from != id && (e.to != id || e.oneWay)))
                    continue;
                auto start = airport.nodes.at(id).position;
                double yaw = e.from == id
                                 ? heading(project(e.geometry[0], e.geometry[1]))
                                 : heading(project(e.geometry.back(), e.geometry[e.geometry.size() - 2]));
                try {
                    auto route = planRoute(airport, start, yaw, destination, o);
                    if (!route.requiresPushback)
                        arrival = std::move(route);
                } catch (const std::exception &e) {
                    lastError = e.what();
                }
                if (arrival)
                    break;
            }
        }
        check(arrival.has_value(), "Cannot construct local approach fixture: " + stand + " / " + lastError);
        check(!arrival->cockpitPath.empty(), "Local gate has an expected cockpit trajectory: " + stand);
        simulateGate(*arrival);
        std::cout << "Local " << stand << " painted lead-in and stop OK\n";
    }
    bool testedExit = false;
    for (const auto &entry : airport.paintedRunwayEntries) {
        if (entry.continuation.size() < 2)
            continue;
        const auto &rw = airport.runways.at(entry.runway);
        auto join = entry.continuation.back();
        Vec2 forward = project(entry.continuation[entry.continuation.size() - 2], join) * -1;
        double yaw = heading(forward);
        auto start = unproject(join, direction(yaw) * -100);
        for (const auto &e : airport.edges) {
            if (!e.painted || e.runway || (e.from != entry.node && e.to != entry.node))
                continue;
            int target = e.from == entry.node ? e.to : e.from;
            try {
                auto route =
                    planRoute(airport, start, yaw, {DestinationKind::Node, "Painted exit", target}, o);
                if (route.requiresPushback)
                    continue;
                check(std::none_of(route.taxiwaysAlongRoute.begin(), route.taxiwaysAlongRoute.end(),
                                   [](const RouteMarker &marker) { return marker.label == "UNMARKED"; }),
                      "Local exit must not use fabricated taxiway");
                std::cout << "Local RWY " << rw.ends[0].name << '/' << rw.ends[1].name
                          << " tangent-continuous painted exit OK\n";
                testedExit = true;
                break;
            } catch (const std::exception &) {
            }
        }
        if (testedExit)
            break;
    }
    check(testedExit, "No local painted runway exit fixture could be planned");
}
} // namespace
int main(int argc, char **argv) {
    try {
        if (argc != 3 && argc != 4)
            throw std::runtime_error("Usage: route_feature_tests fixture-dir config.ini [local-apt.dat]");
        orderedRoute();
        multipleIntersections();
        std::cout << "Ordered route OK\n";
        runwayExit();
        truncatedRunwayAxis();
        disconnectedStand();
        roundedJunctionNoise();
        std::cout << "Painted exits OK\n";
        policies();
        holdAndSpeed();
        dsfContours(argv[1]);
        dsfCurves(argv[1]);
        aircraftConfig(argv[2]);
        simulateGate(gateRoute(true, true));
        simulateGate(gateRoute(false, false));
        auto axleGate = gateRoute(true, false);
        axleGate.cockpitGuidance = false;
        axleGate.mainAxleOversteer = true;
        axleGate.points.back() = axleGate.cockpitStop - direction(axleGate.finalHeading) * 30.15;
        axleGate.length = 0;
        for (std::size_t i = 1; i < axleGate.points.size(); ++i)
            axleGate.length += length(axleGate.points[i] - axleGate.points[i - 1]);
        // Use a straight reference fixture; the last cockpit-guidance samples are beyond its axle stop.
        axleGate.points = {{0, 0}, axleGate.points.back()};
        axleGate.length = length(axleGate.points.back());
        simulateGate(axleGate);
        if (argc == 4)
            localScenery(argv[3]);
        std::cout << "Route, policy, emergency and DSF contour checks passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
}
