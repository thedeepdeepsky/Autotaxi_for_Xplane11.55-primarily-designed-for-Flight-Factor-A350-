#include "AptDatabase.h"
#include "Config.h"
#include "RouteTiming.h"
#include "TaxiController.h"
#include "TurnGuidance.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace autotaxi;
namespace {
void check(bool yes, const char *why) {
    if (!yes)
        throw std::runtime_error(why);
}
template <class F> void rejects(F f, const char *why) {
    bool failed = false;
    try {
        f();
    } catch (const std::exception &) {
        failed = true;
    }
    check(failed, why);
}
std::string fixture(const std::string &name = "Test airport", const std::string &rampName = "Gate One") {
    return "1 0 0 0 TEST " + name +
           "\n"
           "100 60 1 0 0.25 1 3 0 09 0 0 0 0 1 1 1 0 27 0 0.02 0 0 1 1 1 0\n"
           "1201 -0.001 0 both 10 Start\n"
           "1201 0 0 both 20 Entry\n"
           "1201 0 0.001 both 30 Centerline\n"
           "1201 -0.001 0.001 both 40 Narrow\n"
           "1202 10 20 oneway taxiway_E Alpha\n"
           "1202 20 30 twoway runway 09/27\n"
           "1204 departure 09,27\n"
           "1202 10 40 twoway taxiway_C Bravo\n"
           "1300 -0.001 0 90 gate heavy " +
           rampName +
           "\n"
           "1301 E airline test\n"
           "1 0 0 0 NEXT Next airport\n";
}
void parserAndRouting() {
    std::istringstream in(fixture());
    auto a = parseAirport(in);
    check(a.id == "TEST" && a.nodes.size() == 4 && a.edges.size() == 3, "Airport boundary / taxi records");
    check(a.runways.size() == 1 && a.runways[0].ends[1].name == "27", "Runway row offsets");
    check(a.edges[0].oneWay && a.edges[0].width == 'E', "Direction and wingspan class");
    check(a.edges[1].activeRunways.size() == 2, "Active runway list");
    check(a.ramps[0].name == "Gate One" && a.ramps[0].width == 'E', "Ramp parsing");
    RouteOptions o;
    GeoPoint start{-0.00095, 0};
    auto choices = destinations(a);
    check(choices.size() == 7, "Destination list");
    auto node = planRoute(a, start, 0, {DestinationKind::Node, "Node20", 20, 0}, o);
    check(node.nodeIds.back() == 20, "Forward directed taxi route");
    rejects([&] { planRoute(a, {0, 0}, 180, {DestinationKind::Node, "Node10", 10, 0}, o); },
            "Reverse one-way must fail");
    rejects([&] { planRoute(a, start, 0, {DestinationKind::Node, "Node40", 40, 0}, o); },
            "A350 width rejects class C");
    rejects([&] { planRoute(a, start, 0, choices[0], o); }, "Lineup needs runway clearance");
    o.runwayClearance = true;
    auto runway = planRoute(a, start, 0, choices[0], o);
    check(runway.runway && std::abs(runway.finalHeading - 90) < 0.1, "Runway true heading");
    auto end = unproject(runway.origin, runway.points.back());
    check(std::abs(end.lat) < 1e-8, "Lineup stop on centerline");
    check(runway.points.size() >= 4, "Lineup includes approach and straight alignment");
    check(wrap180(1 - 359) == 2, "Heading wrap");
    check(distance({0, 179.999}, {0, -179.999}) < 230, "Date line distance");
}
void databasePriority(const std::filesystem::path &dir) {
    auto high = dir / "Custom Scenery" / "High priority" / "Earth nav data";
    auto disabled = dir / "Custom Scenery" / "Disabled" / "Earth nav data";
    auto global = dir / "Custom Scenery" / "Global Airports" / "Earth nav data";
    auto global12 = dir / "Global Scenery" / "Global Airports" / "Earth nav data";
    auto unlisted = dir / "Custom Scenery" / "Unlisted" / "Earth nav data";
    auto base = dir / "Resources" / "default scenery" / "default apt dat" / "Earth nav data";
    std::filesystem::create_directories(high);
    std::filesystem::create_directories(disabled);
    std::filesystem::create_directories(global);
    std::filesystem::create_directories(global12);
    std::filesystem::create_directories(unlisted);
    std::filesystem::create_directories(base);
    std::ofstream(high / "apt.dat") << fixture("Priority airport");
    std::ofstream(disabled / "apt.dat") << fixture("Disabled airport");
    std::ofstream(global / "apt.dat") << fixture("Global airport", "539");
    std::ofstream(global12 / "apt.dat") << fixture("XP12 global airport");
    std::ofstream(unlisted / "apt.dat") << fixture("Unlisted airport");
    auto other = fixture("Second airport");
    other.replace(other.find("TEST"), 4, "OTHR");
    std::ofstream(base / "apt.dat") << fixture("Base airport") << other;
    std::ofstream(dir / "Custom Scenery" / "scenery_packs.ini")
        << "\xEF\xBB\xBF"
        << "SCENERY_PACK " << (dir / "Custom Scenery" / "High priority").generic_u8string()
        << "/\nSCENERY_PACK_DISABLED Custom Scenery/Disabled/\nSCENERY_PACK Custom Scenery/High priority/\n"
           "SCENERY_PACK Custom Scenery/Global Airports/\nSCENERY_PACK_DISABLED *GLOBAL_AIRPORTS*\n";
    auto files = discoverAptFiles(dir);
    check(files.size() == 3 && files[0] == high / "apt.dat" && files[1] == global / "apt.dat" &&
              files[2] == base / "apt.dat",
          "BOM, absolute/spaced paths, deduplication, disabled/unlisted packs and XP11 base order");
    AptDatabase db;
    db.scan(dir);
    auto a = db.nearest({0, 0});
    check(a.name == "Priority airport", "Scenery priority / disabled exclusion");
    check(a.source == (high / "apt.dat").u8string(), "Geometry comes from the highest priority source");
    auto otherIndex = std::find_if(db.index().begin(), db.index().end(),
                                   [](const AirportIndex &entry) { return entry.id == "OTHR"; });
    check(otherIndex != db.index().end() && otherIndex->file == base / "apt.dat",
          "Priority is selected independently for every airport ID");
    check(a.ramps[0].aliases.size() == 1 && a.ramps[0].aliases[0] == "539",
          "Stand aliases preserve the active scenery network");
    rejects([&] { db.nearest({50, 50}); }, "Faraway airport must fail");
    std::ofstream(dir / "Custom Scenery" / "scenery_packs.ini")
        << "SCENERY_PACK *GLOBAL_AIRPORTS*\nSCENERY_PACK Custom Scenery/High priority/\n";
    db.scan(dir);
    check(db.nearest({0, 0}).name == "XP12 global airport", "XP12 placeholder retains its listed priority");
    std::ofstream(dir / "Custom Scenery" / "scenery_packs.ini")
        << "SCENERY_PACK Custom Scenery/High priority/\nSCENERY_PACK *GLOBAL_AIRPORTS*\n";
    db.scan(dir);
    check(db.nearest({0, 0}).name == "Priority airport", "Custom scenery above XP12 placeholder wins");
    std::ofstream(dir / "Custom Scenery" / "scenery_packs.ini")
        << "SCENERY_PACK_DISABLED *GLOBAL_AIRPORTS*\nSCENERY_PACK_DISABLED Custom Scenery/Global Airports/\n";
    files = discoverAptFiles(dir);
    check(files.size() == 1 && files.front() == base / "apt.dat",
          "Disabled global airports must not be reintroduced by fallback");
    auto ini = dir / "test.ini";
    std::ofstream(ini) << "steering_mode=direct\nsteering_index=-1\n";
    rejects([&] { loadConfig(ini); }, "Direct steering cannot use scalar index");
}
void gateDeparture() {
    Airport a;
    a.id = "GATE";
    GeoPoint origin{0, 0};
    auto geo = [&](double x, double y) { return unproject(origin, {x, y}); };
    a.nodes[1] = {1, geo(0, 0), "A"};
    a.nodes[2] = {2, geo(0, 400), "B"};
    a.nodes[3] = {3, geo(400, 400), "C"};
    a.edges = {{1, 2, false, false, 'E', "A", {}}, {2, 3, false, false, 'E', "B", {}}};
    a.ramps.push_back({geo(90, 100), 90, 'E', "Gate 539", {}});
    a.pavements.push_back({{{geo(-50, -50), geo(500, -50), geo(500, 500), geo(-50, 500)}}});
    RouteOptions options;
    Destination goal{DestinationKind::Node, "Node 3", 3, 0};
    auto route = planRoute(a, geo(90, 100), 0, goal, options);
    check(route.apronDeparture && !route.requiresPushback && route.nodeIds.back() == 3,
          "Forward departure from paved apron beyond 35 m");
    check(route.taxiwaysAlongRoute.size() == 2 &&
              std::abs(route.taxiwaysAlongRoute[1].distance - route.nodesAlongRoute[0].distance) < 1e-6,
          "Next taxiway ETA uses its entry node, not its exit node");
    check(std::abs(route.nodesAlongRoute.back().distance - route.length) < 1e-6,
          "Marker distances follow the retained route geometry");
    check(route.points.size() > 20 && std::abs(wrap180(heading(route.points[1]))) < 5,
          "Apron departure curve starts along the aircraft heading");
    for (std::size_t i = 1; i < route.points.size(); ++i)
        check(pavedConnection(a, unproject(route.origin, route.points[i - 1]),
                              unproject(route.origin, route.points[i])),
              "Departure stays on pavement");
    auto reverse = planRoute(a, geo(90, 100), 90, goal, options);
    check(reverse.requiresPushback, "Nose-in stand departure waits for pushback");
    TaxiController controller;
    rejects([&] { controller.start(reverse, {}); },
            "Taxi must not propel an aircraft along a reverse gate exit");
    a.pavements[0].rings.push_back({geo(30, 70), geo(60, 70), geo(60, 130), geo(30, 130)});
    check(!pavedConnection(a, geo(90, 100), geo(0, 100)), "Apron connection cannot cross a polygon hole");
    a.pavements.clear();
    rejects([&] { planRoute(a, geo(90, 100), 0, goal, options); },
            "Long connectors require pavement geometry");
}
void alternateTurns() {
    Airport a;
    a.id = "TURNS";
    GeoPoint origin{0, 0};
    auto node = [&](int id, double x, double y) { a.nodes[id] = {id, unproject(origin, {x, y}), {}}; };
    node(1, 0, 0);
    node(2, 0, 100);
    node(3, 80, 120);
    node(4, 20, 100);
    node(5, 0, 230);
    node(6, 180, 230);
    node(7, 180, 100);
    auto edge = [&](int x, int y) { a.edges.push_back({x, y, true, false, 'E', "A", {}}); };
    edge(1, 2);
    edge(2, 3);
    edge(3, 4);
    edge(2, 5);
    edge(5, 6);
    edge(6, 7);
    edge(7, 4);
    auto route = planRoute(a, unproject(origin, {0, 10}), 0, {DestinationKind::Node, "Goal", 4, 0}, {});
    check(std::find(route.nodeIds.begin(), route.nodeIds.end(), 5) != route.nodeIds.end(),
          "Choose a longer route when the shortest has a forbidden turn");
}
void controllerSimulation(bool turning) {
    ControllerConfig c;
    Route r;
    r.origin = {31, 121};
    r.runway = true;
    r.finalHeading = turning ? 90 : 0;
    r.points = turning ? std::vector<Vec2>{{0, 0}, {0, 180}, {150, 180}, {370, 180}}
                       : std::vector<Vec2>{{0, 0}, {0, 220}, {0, 440}};
    TaxiController ctrl;
    ctrl.start(r, c);
    Vec2 axle{0, 0};
    double h = 0, speed = 0, maxError = 0, steer = 0;
    ControlOutput out;
    constexpr double dt = 0.05;
    for (int step = 0; step < 16000 && ctrl.active(); ++step) {
        AircraftState state{unproject(r.origin, axle + direction(h) * c.mainAxleAft), h, speed, true};
        out = ctrl.update(state, dt);
        check(out.phase != TaxiPhase::Fault, out.reason.c_str());
        check(std::abs(out.steerDegrees - steer) <= c.steerRate * dt + 1e-8, "Steering slew limit");
        steer = out.steerDegrees;
        maxError = std::max(maxError, out.crossTrack);
        speed = std::max(0.0, speed + (.39 - out.brake * 1.5) * dt);
        h = wrap180(h + speed / c.wheelbase * std::tan(out.steerDegrees * rad) * dt / rad);
        axle = axle + direction(h) * (speed * dt);
    }
    check(ctrl.phase() == TaxiPhase::Complete, "Kinematic simulation must complete");
    check(std::abs(wrap180(h - r.finalHeading)) < 2, "Runway heading tolerance at stop");
    check(length(axle - r.points.back()) < 4.5, "Stop at runway target");
    std::cout << (turning ? "90-degree lineup" : "Straight lineup") << ": CTE max " << maxError
              << " m, heading " << h << "\n";
}
void routeTiming() {
    ControllerConfig config;
    config.taxiSpeed = 20 * .514444;
    Route straight;
    straight.points = {{0, 0}, {0, 750}};
    auto rolling = estimateRouteTiming(straight, 0, config.taxiSpeed, config);
    auto stopped = estimateRouteTiming(straight, 0, 0, config);
    check(stopped.totalSeconds > rolling.totalSeconds + 5, "ETA includes acceleration from rest");
    check(rolling.totalSeconds > 750 / config.taxiSpeed, "ETA includes endpoint deceleration and settling");
    check(stopped.secondsTo(100) > 0 && stopped.secondsTo(300) > stopped.secondsTo(100),
          "Node/taxiway ETA integrates each intervening part of the route");
    Route bends;
    bends.points = {{0, 0}, {0, 250}, {250, 250}, {250, 500}};
    auto turning = estimateRouteTiming(bends, 0, 0, config);
    check(turning.totalSeconds > stopped.totalSeconds + 20, "Equal-length route with bends takes longer");
    config.turnSpeed = .8;
    auto slowTurns = estimateRouteTiming(bends, 0, 0, config);
    check(slowTurns.totalSeconds > turning.totalSeconds, "ETA follows configured turn speed");
    config.taxiSpeed = 10 * .514444;
    check(estimateRouteTiming(straight, 0, 0, config).totalSeconds > stopped.totalSeconds,
          "Selected cruise speed remains an upper limit");
    config.taxiSpeed = 20 * .514444;
    Route lineup;
    lineup.points = {{0, 0}, {0, 60}};
    auto taxi = estimateRouteTiming(lineup, 0, 0, config);
    lineup.runway = true;
    check(estimateRouteTiming(lineup, 0, 0, config).totalSeconds > taxi.totalSeconds,
          "Runway alignment observes the controller's 2 m/s cap");
    Route shortRoute;
    shortRoute.points = {{0, 0}, {0, 4}};
    auto shortTime = estimateRouteTiming(shortRoute, 0, 0, config);
    check(std::isfinite(shortTime.totalSeconds) && shortTime.totalSeconds > 1 && shortTime.totalSeconds < 15,
          "Short stopped routes use a finite acceleration/braking profile");
    for (std::size_t i = 1; i < stopped.samples.size(); ++i) {
        const auto &before = stopped.samples[i - 1];
        const auto &after = stopped.samples[i];
        double distance = after.distance - before.distance;
        double change = after.speed * after.speed - before.speed * before.speed;
        check(change <= 2 * .25 * distance + 1e-6 && change >= -2 * 1.25 * distance - 1e-6,
              "Forecast speed respects the modeled thrust and full brake authority");
    }
    straight.requiresPushback = true;
    check(estimateRouteTiming(straight, 0, 0, config).totalSeconds < 0,
          "Tow time is not falsely estimated at the taxi speed");
    std::cout << "750 m ETA at 20 kt: straight " << stopped.totalSeconds << " s, bends "
              << turning.totalSeconds << " s\n";
}
void controllerGeometryCache() {
    ControllerConfig config;
    config.mainAxleAft = 0;
    config.steerRate = 100000;
    config.taxiSpeed = 20 * .514444;
    TaxiController controller;
    // Reusing the controller must replace cached geometry, including degenerate segments.
    for (const auto &points :
         std::vector<std::vector<Vec2>>{{{0, 0}, {0, 120}, {90, 120}, {90, 240}, {160, 330}, {160, 330}},
                                        {{0, 0}, {0, 35}},
                                        {{0, 0}, {0, 100}, {0, 160}, {0, 161}, {100, 161}}}) {
        Route route;
        route.origin = {31, 121};
        route.points = points;
        route.finalHeading = heading(points.back() - points[points.size() - 2]);
        for (std::size_t i = 1; i < points.size(); ++i)
            route.length += length(points[i] - points[i - 1]);
        for (bool runway : {false, true}) {
            route.runway = runway;
            controller.start(route, config);
            for (std::size_t segment = 0; segment + 1 < points.size(); ++segment) {
                const auto delta = points[segment + 1] - points[segment];
                if (length(delta) == 0)
                    continue;
                for (double fraction : {.05, .5, .9}) {
                    const auto position = points[segment] + delta * fraction;
                    AircraftState state{unproject(route.origin, position), heading(delta), 3, true};
                    const auto projection = onSegment(project(route.origin, state.position), points[segment],
                                                      points[segment + 1]);
                    double remaining = length(points[segment + 1] - projection.point);
                    for (std::size_t i = segment + 2; i < points.size(); ++i)
                        remaining += length(points[i] - points[i - 1]);
                    // Isolate cached geometry from the live speed-setpoint history.
                    controller.start(route, config);
                    controller.seekProgress(route.length - remaining);
                    const auto out = controller.update(state, .02);
                    check(out.phase != TaxiPhase::Fault, "Cached geometry keeps valid routes active");
                    check(std::abs(out.remaining - remaining) < 1e-7,
                          "Cached distance matches the original sum throughout the route");
                    check(std::abs(out.progress - std::max(0.0, route.length - remaining)) < 1e-7,
                          "Cached distance preserves route progress");
                    const auto expected = routeSpeedLimit(route, config, segment, projection.point, remaining,
                                                          out.steerDegrees);
                    check(std::abs(out.targetSpeed - expected) < 1e-7,
                          "Cached bends preserve turn, lineup and stopping speed limits");
                }
            }
            controller.stop();
        }
    }
}
void paintedCockpitSimulation() {
    ControllerConfig config;
    Route route;
    route.origin = {31, 121};
    route.runway = route.cockpitGuidance = true;
    route.finalHeading = 90;
    for (int i = 0; i <= 60; ++i)
        route.points.push_back({0, i * 2.0});
    for (int i = 1; i <= 80; ++i) {
        double angle = i * pi / 160;
        route.points.push_back({80 - 80 * std::cos(angle), 120 + 80 * std::sin(angle)});
    }
    route.points.push_back({480, 200});
    TaxiController controller;
    controller.start(route, config);
    double yaw = 0, speed = 0, error = 0, innerCut = 0;
    double front = config.wheelbase + config.cockpitAheadNose;
    Vec2 axle{0, -front};
    constexpr double dt = .05;
    for (int step = 0; step < 20000 && controller.active(); ++step) {
        AircraftState state{unproject(route.origin, axle + direction(yaw) * config.mainAxleAft), yaw, speed,
                            true};
        auto out = controller.update(state, dt);
        check(out.phase != TaxiPhase::Fault, out.reason.c_str());
        error = std::max(error, out.crossTrack);
        Vec2 cockpit = axle + direction(yaw) * front;
        if (cockpit.x > 20 && cockpit.x < 60 && cockpit.y > 140)
            innerCut = std::max(innerCut, 80 - length(axle - Vec2{80, 120}));
        speed = std::max(0., speed + (.39 - out.brake * 1.5) * dt);
        yaw = wrap180(yaw + speed / config.wheelbase * std::tan(out.steerDegrees * rad) * dt / rad);
        axle = axle + direction(yaw) * (speed * dt);
    }
    check(controller.phase() == TaxiPhase::Complete, "Painted cockpit guidance must complete lineup");
    check(error < 4 && innerCut > 3, "Cockpit follows paint while the main axle cuts inside the curve");
    check(length(axle + direction(yaw) * front - route.points.back()) < 4.5 &&
              std::abs(wrap180(yaw - 90)) < 2,
          "Cockpit endpoint and runway heading tolerance");
    std::cout << "Cockpit curve CTE " << error << " m; main axle inner cut " << innerCut << " m\n";
}
void cockpitTowHandoff() {
    ControllerConfig config;
    Route route;
    route.origin = {31, 121};
    route.cockpitGuidance = true;
    route.points = {{0, 0}, {0, -.8}};
    for (int i = 1; i <= 200; ++i)
        route.points.push_back({0, i * 2.});
    route.length = 401.6;
    TaxiController controller;
    controller.start(route, config);
    AircraftState state{route.origin, 0, 0, true};
    auto output = controller.update(state, .05);
    check(output.phase == TaxiPhase::Taxi && output.crossTrack < .01 && output.progress > 25,
          "Cockpit reference must initialize past the short reverse CG prefix at tow handoff");
    check(std::abs(output.steerDegrees) < .01 && output.throttle == 0 && output.brake == 0,
          "Aligned tow handoff starts forward without a spurious turn or cross-track fault");
    controller.stop();
    controller.start(route, config);
    state.position =
        unproject(route.origin, {0, -config.wheelbase - config.cockpitAheadNose + config.mainAxleAft});
    output = controller.update(state, .05);
    check(output.phase == TaxiPhase::Taxi && output.progress < 2,
          "Restart initializes from actual telemetry rather than assuming cockpit is always ahead on route");
}
void oversteerGeometry() {
    RouteOptions options;
    auto wide = turnClearance(100, 30, options), narrow = turnClearance(100, 11, options);
    check(wide.outwardOffset < .001 && wide.offtracking > 4,
          "Wide fillets allow cockpit-over-centerline taxi with normal main wheel offtracking");
    check(narrow.feasible && narrow.outwardOffset > 3,
          "Insufficient inner wheel clearance automatically requires outward compensation");
    options.maxOversteer = 1;
    check(!turnClearance(100, 11, options).feasible, "Do not exceed configured oversteer allowance");
    options.maxOversteer = 12;
    check(turnClearance(100, -1, options).outwardOffset < .001,
          "Unknown pavement must not invent a need for clearance-based oversteer");
    Route route;
    route.origin = {31, 121};
    route.cockpitGuidance = true;
    for (int i = 0; i <= 75; ++i)
        route.points.push_back({0, i * 2.});
    for (int i = 1; i <= 100; ++i) {
        double angle = i * pi / 200;
        route.points.push_back({100 - 100 * std::cos(angle), 150 + 100 * std::sin(angle)});
    }
    for (int i = 1; i <= 200; ++i)
        route.points.push_back({100 + i * 2., 250});
    Airport airport;
    airport.pavementCoverageComplete = true;
    Pavement annulus;
    for (double radius : {135., 89.}) {
        std::vector<GeoPoint> ring;
        for (int i = 0; i < 360; ++i) {
            double angle = i * pi / 180;
            ring.push_back(
                unproject(route.origin, {100 + radius * std::cos(angle), 150 + radius * std::sin(angle)}));
        }
        annulus.rings.push_back(std::move(ring));
    }
    airport.pavements.push_back(annulus);
    auto rectangle = [&](double x1, double y1, double x2, double y2) {
        airport.pavements.push_back(
            {{{unproject(route.origin, {x1, y1}), unproject(route.origin, {x2, y1}),
               unproject(route.origin, {x2, y2}), unproject(route.origin, {x1, y2})}}});
    };
    rectangle(-25, -100, 25, 160);
    rectangle(90, 220, 600, 280);
    const Route marked = route;
    auto misaligned = marked;
    misaligned.initialHeading = 30;
    rejects([&] { applyTurnGuidance(airport, misaligned, options); },
            "Do not begin cockpit guidance when the front reference cannot join the planned apron prefix");
    applyTurnGuidance(airport, route, options);
    check(route.turnClearanceKnown && route.maximumOversteer > 1 && route.oversteerOffsets.front() < .1 &&
              route.oversteerOffsets.back() < .1,
          "Pavement-aware route applies oversteer and smoothly recovers the centerline");
    auto checkReferences = [&](Route &planned) {
        buildReferencePaths(planned, options);
        check(planned.cockpitPath.size() > 2 && planned.mainAxlePath.size() == planned.cockpitPath.size(),
              "Both aircraft references need corresponding fixed planning paths");
        for (std::size_t i = 0; i < planned.cockpitPath.size(); ++i)
            check(std::abs(length(planned.cockpitPath[i].position - planned.mainAxlePath[i].position) -
                           options.wheelbase - options.cockpitAheadNose) < .001 &&
                      planned.cockpitPath[i].distance == planned.mainAxlePath[i].distance,
                  "Planned reference positions must preserve aircraft geometry at shared progress");
    };
    checkReferences(route);
    // Give axle guidance positive clearance beyond the pavement query's 0.5 m boundary tolerance.
    auto &innerRing = airport.pavements.front().rings[1];
    innerRing.clear();
    for (int i = 0; i < 360; ++i) {
        double angle = i * pi / 180;
        innerRing.push_back(
            unproject(marked.origin, {100 + 85 * std::cos(angle), 150 + 85 * std::sin(angle)}));
    }
    auto axleRoute = marked;
    applyAxleOversteer(airport, axleRoute, options);
    check(axleRoute.mainAxleOversteer && !axleRoute.cockpitGuidance && axleRoute.maximumOversteer > 4 &&
              axleRoute.turnClearanceKnown,
          "Main-axle mode estimates cockpit outset and validates known wheel footprint");
    checkReferences(axleRoute);
    auto cockpitOversteer = marked;
    applyCockpitOversteer(airport, cockpitOversteer, options);
    check(cockpitOversteer.cockpitGuidance && !cockpitOversteer.mainAxleOversteer &&
              cockpitOversteer.maximumOversteer > 0 &&
              cockpitOversteer.maximumOversteer < axleRoute.maximumOversteer,
          "Kinematic oversteer must control a cockpit trajectory instead of selecting axle pursuit");
    TaxiController oversteerController;
    oversteerController.start(cockpitOversteer, {});
    check(forecastTaxiTiming(oversteerController, {marked.origin, 0, 0, true}, {.39, 1.5}).totalSeconds > 0,
          "Outward cockpit trajectory must be driveable with the real steering and brake controller");
    auto broadAirport = airport;
    broadAirport.pavements.front().rings.resize(1);
    auto broadRoute = marked;
    auto noOversteer = options;
    noOversteer.allowOversteer = false;
    noOversteer.maxOversteer = 0;
    applyCockpitOversteer(broadAirport, broadRoute, noOversteer);
    check(broadRoute.cockpitGuidance && broadRoute.maximumOversteer < .01,
          "A feasible wide painted bend must remain on paint even in the transient fallback");
    rejects(
        [&] {
            auto narrowRoute = marked;
            applyCockpitOversteer(airport, narrowRoute, noOversteer);
        },
        "Disabled oversteer must not be bypassed by the transient fallback");
    airport.pavements.clear();
    rejects(
        [&] {
            auto invalid = marked;
            applyAxleOversteer(airport, invalid, options);
        },
        "Complete pavement data must reject a known off-pavement oversteer path");
    airport.pavementCoverageComplete = false;
    axleRoute = marked;
    applyAxleOversteer(airport, axleRoute, options);
    check(!axleRoute.turnClearanceKnown, "Incomplete custom pavement must be reported as unverified");
}
void standVisibility(const std::filesystem::path &dir) {
    auto text = fixture();
    auto boundary = text.find("1 0 0 0 NEXT");
    text.insert(boundary, "1300 -0.001 0.001 90 gate jets Small Gate\n1301 C airline test\n"
                          "15 -0.001 0.002 90 Legacy Gate\n");
    std::istringstream input(text);
    auto airport = parseAirport(input);
    auto choices = destinations(airport);
    check(airport.ramps.size() == 3 && choices.size() == 9,
          "All modern and legacy ramps remain visible regardless of wingspan class");
    check(airport.ramps.back().name == "Legacy Gate" && airport.ramps.back().width == 0,
          "Legacy startup location retains position, name and unknown width");
    rejects([&] { planRoute(airport, {-.001, 0}, 0, {DestinationKind::Ramp, "Small Gate", 1, 0}, {}); },
            "Showing a small stand must not allow A350 routing into it");
    auto nav = dir / "legacy" / "Resources/default scenery/default apt dat/Earth nav data";
    std::filesystem::create_directories(nav);
    std::ofstream(nav / "apt.dat") << "1 0 0 0 LGCY Legacy airport\n15 31 121 90 Legacy Stand\n99\n";
    AptDatabase database;
    database.scan(dir / "legacy");
    check(database.nearest({31, 121}).ramps.size() == 1,
          "Legacy startup locations participate in airport indexing");
}
} // namespace
int main(int argc, char **argv) {
    try {
        parserAndRouting();
        gateDeparture();
        alternateTurns();
        auto temp = std::filesystem::temp_directory_path() / "autotaxi_core_tests";
        databasePriority(temp);
        standVisibility(temp);
        routeTiming();
        controllerGeometryCache();
        paintedCockpitSimulation();
        cockpitTowHandoff();
        oversteerGeometry();
        controllerSimulation(false);
        controllerSimulation(true);
        if (argc == 3 && std::string(argv[1]) == "--airport") {
            AptDatabase db;
            db.scan(std::filesystem::u8path(argv[2]));
            auto a = db.nearest({31.143, 121.802});
            std::cout << "Real apt.dat: " << a.id << ", " << a.nodes.size() << " nodes, " << a.edges.size()
                      << " edges; " << a.source << "\n";
            int successes = 0;
            auto choices = destinations(a);
            RouteOptions options;
            options.runwayClearance = true;
            for (const auto &choice : choices) {
                if (choice.kind != DestinationKind::Runway)
                    continue;
                GeoPoint threshold = a.runways[choice.index].ends[choice.end].position;
                std::vector<std::pair<double, const TaxiEdge *>> edges;
                for (const auto &edge : a.edges)
                    if (!edge.runway && (!edge.width || edge.width >= 'E'))
                        edges.push_back({distance(threshold, a.nodes.at(edge.from).position), &edge});
                std::sort(edges.begin(), edges.end(),
                          [](const auto &x, const auto &y) { return x.first < y.first; });
                bool found = false;
                for (std::size_t i = 0; i < std::min<std::size_t>(30, edges.size()) && !found; ++i) {
                    const auto &edge = *edges[i].second;
                    auto from = a.nodes.at(edge.from).position, to = a.nodes.at(edge.to).position;
                    auto vector = project(from, to);
                    auto midpoint = unproject(from, vector * 0.5);
                    for (int reverse = 0; reverse < (edge.oneWay ? 1 : 2); ++reverse) {
                        try {
                            auto route =
                                planRoute(a, midpoint, heading(vector) + reverse * 180, choice, options);
                            check(route.runway && route.points.size() >= 3, "Real runway route structure");
                            std::cout << choice.label << ": " << route.length << " m, heading "
                                      << route.finalHeading << "\n";
                            ++successes;
                            found = true;
                            break;
                        } catch (const std::exception &) {
                        }
                    }
                }
            }
            check(successes > 0, "At least one real scenery runway route must be reachable");
        }
        std::cout << "All core checks passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAILED: " << e.what() << "\n";
        return 1;
    }
}
