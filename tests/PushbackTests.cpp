#include "PushbackPlanner.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace autotaxi;
namespace {
void check(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
void simulateTaxi(const PushbackPlan &plan) {
    ControllerConfig config;
    TaxiController controller;
    controller.start(plan.taxi, config);
    double yaw = plan.heading, speed = 0;
    Vec2 axle = project(plan.taxi.origin, plan.handoff) - direction(yaw) * config.mainAxleAft;
    constexpr double dt = .05;
    for (int step = 0; step < 100000 && controller.active(); ++step) {
        AircraftState state{unproject(plan.taxi.origin, axle + direction(yaw) * config.mainAxleAft), yaw,
                            speed, true};
        auto out = controller.update(state, dt);
        if (out.phase == TaxiPhase::Fault) {
            std::cerr << "Handoff node " << plan.nodeId << " yaw " << plan.heading << " fault axle " << axle.x
                      << ',' << axle.y << " yaw " << yaw << " speed " << speed << " CTE " << out.crossTrack
                      << " remaining " << out.remaining << '\n';
            std::cerr << "Route points:";
            for (std::size_t i = 0; i < std::min<std::size_t>(35, plan.taxi.points.size()); ++i)
                std::cerr << ' ' << plan.taxi.points[i].x << ',' << plan.taxi.points[i].y;
            std::cerr << '\n';
            throw std::runtime_error(plan.taxi.label + ": " + out.reason);
        }
        speed = std::max(0.0, speed + (.39 - out.brake * 1.5) * dt);
        yaw = wrap180(yaw + speed / config.wheelbase * std::tan(out.steerDegrees * rad) * dt / rad);
        axle = axle + direction(yaw) * (speed * dt);
    }
    check(controller.phase() == TaxiPhase::Complete,
          "Forward taxi must finish from the actual planned tow endpoint");
    check(std::abs(wrap180(yaw - plan.taxi.finalHeading)) < 2, "Runway alignment after automatic pushback");
    Vec2 reference =
        axle + direction(yaw) * (plan.taxi.cockpitGuidance ? config.wheelbase + config.cockpitAheadNose : 0);
    check(length(reference - plan.taxi.points.back()) < 4.5, "Runway endpoint after automatic pushback");
}
void validate(const PushbackPlan &plan, const Airport &airport, const AircraftState &aircraft) {
    check(plan.nodeId >= 0 && !plan.segments.empty(), "A reverse route must end at a real taxi node");
    check(plan.preview.requiresPushback && !plan.taxi.requiresPushback,
          "Tow and taxi routes must be separate");
    check(plan.preview.nodesAlongRoute.size() == plan.taxi.nodesAlongRoute.size(),
          "Tow preview retains forward taxi node markers");
    check(plan.preview.cockpitPath.size() == plan.taxi.cockpitPath.size(),
          "Tow preview retains the expected forward cockpit path");
    check(plan.preview.mainAxlePath.size() == plan.taxi.mainAxlePath.size(),
          "Tow preview retains the expected main-axle path");
    for (std::size_t i = 0; i < plan.taxi.mainAxlePath.size(); ++i)
        check(distance(unproject(plan.preview.origin, plan.preview.mainAxlePath[i].position),
                       unproject(plan.taxi.origin, plan.taxi.mainAxlePath[i].position)) < .01 &&
                  std::abs(plan.preview.mainAxlePath[i].distance - plan.taxi.mainAxlePath[i].distance -
                           plan.length) < 1e-6,
              "Main-axle planning path must retain location and progress across towing origins");
    for (std::size_t i = 0; i < plan.taxi.cockpitPath.size(); ++i) {
        check(distance(unproject(plan.preview.origin, plan.preview.cockpitPath[i].position),
                       unproject(plan.taxi.origin, plan.taxi.cockpitPath[i].position)) < .01,
              "Preview and taxi cockpit paths agree geographically despite different origins");
        check(std::abs(plan.preview.cockpitPath[i].distance - plan.taxi.cockpitPath[i].distance -
                       plan.length) < 1e-6,
              "Cockpit preview progress includes towing distance");
    }
    for (std::size_t i = 0; i < plan.taxi.nodesAlongRoute.size(); ++i)
        check(std::abs(plan.preview.nodesAlongRoute[i].distance - plan.taxi.nodesAlongRoute[i].distance -
                       plan.length) < 1e-6,
              "Preview node markers include the reverse route distance");
    for (std::size_t i = 0; i < plan.taxi.taxiwaysAlongRoute.size(); ++i)
        check(std::abs(plan.preview.taxiwaysAlongRoute[i].distance -
                       plan.taxi.taxiwaysAlongRoute[i].distance - plan.length) < 1e-6,
              "Preview taxiway entry markers include the reverse route distance");
    check(distance(airport.nodes.at(plan.nodeId).position, plan.segments.back().end) < 0.5,
          "Tow ends at node");
    check(std::abs(wrap180(plan.segments.front().startHeading - aircraft.trueHeading)) < 0.01,
          "Tow starts at aircraft heading");
    for (const auto &segment : plan.segments) {
        check(segment.backward, "Tow route must reverse rather than drive through the stand");
        check(segment.type == 0 || segment.radius > 25, "A350 tow must respect turning radius");
        if (segment.type == 0)
            check(dot(project(segment.start, segment.end), direction(segment.startHeading)) < 0,
                  "Straight tow geometry must move behind the nose");
        else {
            double turn = wrap180(segment.endHeading - segment.startHeading);
            check((turn > 0) != segment.right, "Reverse arc must use the correct steering side");
        }
    }
    for (std::size_t i = 1; i < plan.segments.size(); ++i) {
        check(distance(plan.segments[i - 1].end, plan.segments[i].start) < .01,
              "Tow segments must be connected");
        check(std::abs(wrap180(plan.segments[i - 1].endHeading - plan.segments[i].startHeading)) < .01,
              "Tow headings must be continuous");
    }
    for (std::size_t i = 1; i < plan.points.size(); ++i)
        check(pavedConnection(airport, unproject(plan.origin, plan.points[i - 1]),
                              unproject(plan.origin, plan.points[i])),
              "Reverse path must remain paved");
}
void obstacleDetour() {
    GeoPoint origin{31, 121};
    auto geo = [&](double x, double y) { return unproject(origin, {x, y}); };
    Airport airport;
    airport.nodes[1] = {1, geo(118, 193), "Join"};
    airport.nodes[2] = {2, geo(-120, 116), "Goal"};
    airport.edges = {{1, 2, true, false, 'E', "A", {}}};
    airport.pavements.push_back({{{geo(-200, -250), geo(500, -250), geo(500, 500), geo(-200, 500)},
                                  {geo(10, -40), geo(120, -40), geo(120, 80), geo(10, 80)}}});
    AircraftState aircraft{origin, 90, 0, true};
    auto plan = planPushback(airport, aircraft, {DestinationKind::Node, "Goal", 2, 0}, {}, {});
    validate(plan, airport, aircraft);
    check(plan.nodeId == 1 && plan.segments.size() >= 4,
          "Connected nodes ahead of a stand require a continuous multi-stage reverse detour");
    check(plan.length <= 450, "Apron detours remain bounded");
}
void synthetic() {
    GeoPoint origin{0, 0};
    auto geo = [&](double x, double y) { return unproject(origin, {x, y}); };
    Airport airport;
    airport.id = "TOW";
    airport.nodes[1] = {1, geo(0, 0), "A"};
    airport.nodes[2] = {2, geo(0, 200), "B"};
    airport.nodes[3] = {3, geo(300, 200), "C"};
    airport.edges = {{1, 2, false, false, 'E', "A", {}}, {2, 3, false, false, 'E', "B", {}}};
    airport.pavements.push_back({{{geo(-200, -200), geo(500, -200), geo(500, 500), geo(-200, 500)}}});
    AircraftState aircraft{geo(90, 100), 90, 0, true};
    auto plan = planPushback(airport, aircraft, {DestinationKind::Node, "Goal", 3, 0}, {}, {});
    validate(plan, airport, aircraft);
    std::ostringstream serialized;
    writePushbackCache(serialized, {plan.segments});
    std::istringstream input(serialized.str());
    auto loaded = readPushbackCache(input);
    check(loaded.size() == 1 && loaded[0].size() == plan.segments.size(), "Cache round trip");
    check(distance(loaded[0].back().end, plan.segments.back().end) < 0.001, "Cache endpoint precision");
    std::istringstream bad("route\nseg 1 0 0 0 0 0 90 1 -5 0 1\n");
    bool rejected = false;
    try {
        std::cout.setf(std::ios::unitbuf);
        readPushbackCache(bad);
    } catch (const std::exception &) {
        rejected = true;
    }
    check(rejected, "Invalid cache cannot be overwritten");
    auto path = std::filesystem::temp_directory_path() / "autotaxi-pushback-tests" / "routes.dat";
    std::filesystem::create_directories(path.parent_path());
    auto other = plan.segments;
    for (auto &segment : other) {
        segment.start.lat += 0.1;
        segment.end.lat += 0.1;
    }
    {
        std::ofstream out(path);
        writePushbackCache(out, {other, plan.segments});
    }
    storePushbackRoute(path, plan);
    std::ifstream updated(path);
    auto preserved = readPushbackCache(updated);
    check(preserved.size() == 2, "Preserve unrelated routes and replace the same stand route");
    check(std::filesystem::exists(path.string() + ".autotaxi.bak"), "Original cache backup");
    airport.pavements.clear();
    rejected = false;
    try {
        planPushback(airport, aircraft, {DestinationKind::Node, "Goal", 3, 0}, {}, {});
    } catch (const std::exception &) {
        rejected = true;
    }
    check(rejected, "No automatic towing through unknown pavement");
}
} // namespace
int main(int argc, char **argv) {
    try {
        synthetic();
        obstacleDetour();
        if (argc >= 2) {
            std::ifstream file(std::filesystem::u8path(argv[1]));
            auto airport = parseAirport(file, argv[1]);
            AircraftState aircraft{{31.135482, 121.802825}, 72.3, 0, true};
            std::string stand = argc >= 3 ? argv[2] : "539";
            if (stand != "539") {
                auto ramp = std::find_if(airport.ramps.begin(), airport.ramps.end(),
                                         [&](const Ramp &r) { return r.name == stand; });
                check(ramp != airport.ramps.end(), "Requested stand must exist in active apt.dat");
                aircraft.position = ramp->position;
                aircraft.trueHeading = ramp->heading;
            }
            std::cout << "Stand " << stand << " heading " << aircraft.trueHeading << " / markings "
                      << airport.groundLines.size() << " / links " << airport.edges.size() << " / fallback "
                      << airport.atcFallbackEdges << " / pavement(CG) "
                      << pavedConnection(airport, aircraft.position, aircraft.position)
                      << " / pavement(axle) "
                      << pavedConnection(airport,
                                         unproject(aircraft.position, direction(aircraft.trueHeading) * -2.2),
                                         unproject(aircraft.position, direction(aircraft.trueHeading) * -2.2))
                      << '\n';
            RouteOptions options;
            options.runwayClearance = true;
            int reached = 0;
            for (const auto &destination : destinations(airport)) {
                if (destination.kind != DestinationKind::Runway)
                    continue;
                auto plan = planPushback(airport, aircraft, destination, options, {});
                validate(plan, airport, aircraft);
                simulateTaxi(plan);
                std::cout << destination.label << " -> Node " << plan.nodeId << " tow " << plan.length
                          << " m / heading " << plan.heading << " / taxi " << plan.taxi.length
                          << " m / fallback " << plan.taxi.atcFallbackCount << '\n';
                if (stand == "590")
                    check(plan.length < 180, "Stand 590 should use the nearby painted apron network");
                ++reached;
            }
            check(reached == 10, "All ten ZSPD runway directions must have a tow-to-node departure");
        }
        std::cout << "Pushback planning and cache checks passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
}
