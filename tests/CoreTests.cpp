#include "AptDatabase.h"
#include "Config.h"
#include "TaxiController.h"
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
        speed = std::max(0.0, speed + (out.throttle * 4.0 - 0.06 - out.brake * 1.5) * dt);
        h = wrap180(h + speed / c.wheelbase * std::tan(out.steerDegrees * rad) * dt / rad);
        axle = axle + direction(h) * (speed * dt);
    }
    check(ctrl.phase() == TaxiPhase::Complete, "Kinematic simulation must complete");
    check(std::abs(wrap180(h - r.finalHeading)) < 2, "Runway heading tolerance at stop");
    check(length(axle - r.points.back()) < 4.5, "Stop at runway target");
    std::cout << (turning ? "90-degree lineup" : "Straight lineup") << ": CTE max " << maxError
              << " m, heading " << h << "\n";
}
} // namespace
int main(int argc, char **argv) {
    try {
        parserAndRouting();
        gateDeparture();
        alternateTurns();
        auto temp = std::filesystem::temp_directory_path() / "autotaxi_core_tests";
        databasePriority(temp);
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
