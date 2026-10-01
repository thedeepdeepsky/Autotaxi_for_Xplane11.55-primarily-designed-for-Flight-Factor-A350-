#include "TaxiController.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
using namespace autotaxi;
namespace {
void check(bool condition, const std::string &message) {
    if (!condition)
        throw std::runtime_error(message);
}
void simulate(const Route &route, double initialHeading) {
    ControllerConfig config;
    TaxiController controller;
    controller.start(route, config);
    Vec2 axle = direction(initialHeading) * -config.mainAxleAft;
    double yaw = initialHeading, speed = 0, maxError = 0;
    constexpr double dt = 0.05;
    for (int step = 0; step < 100000 && controller.active(); ++step) {
        AircraftState state{unproject(route.origin, axle + direction(yaw) * config.mainAxleAft), yaw, speed,
                            true};
        auto output = controller.update(state, dt);
        if (output.phase == TaxiPhase::Fault)
            std::cerr << "\nAt " << axle.x << "," << axle.y << " yaw " << yaw << " speed " << speed << " CTE "
                      << output.crossTrack << " remaining " << output.remaining << "\n";
        check(output.phase != TaxiPhase::Fault, route.label + ": " + output.reason);
        maxError = std::max(maxError, output.crossTrack);
        speed = std::max(0.0, speed + (output.throttle * 4 - 0.06 - output.brake * 1.5) * dt);
        yaw = wrap180(yaw + speed / config.wheelbase * std::tan(output.steerDegrees * rad) * dt / rad);
        axle = axle + direction(yaw) * (speed * dt);
    }
    check(controller.phase() == TaxiPhase::Complete, route.label + ": simulation did not complete");
    check(std::abs(wrap180(yaw - route.finalHeading)) < 2, route.label + ": final heading");
    check(length(axle - route.points.back()) < 4.5, route.label + ": final position");
    std::cout << " / simulation CTE " << maxError << " m";
}
} // namespace
int main(int argc, char **argv) {
    try {
        check(argc >= 2, "Usage: scenery_diagnostics apt.dat [latitude longitude heading]");
        std::ifstream in(std::filesystem::u8path(argv[1]));
        check(in.good(), "Cannot open apt.dat");
        auto airport = parseAirport(in, argv[1]);
        GeoPoint position =
            argc >= 4 ? GeoPoint{std::stod(argv[2]), std::stod(argv[3])} : GeoPoint{31.135482, 121.802825};
        double yaw = argc >= 5 ? std::stod(argv[4]) : 0;
        std::cout << airport.id << " stand position " << position.lat << "," << position.lon << " heading "
                  << yaw << " / " << departureLabel(airport, position) << "\n";
        RouteOptions options;
        options.runwayClearance = true;
        const std::set<std::string> expected{"RWY 17R", "RWY 35L", "RWY 16L", "RWY 34R", "RWY 16R",
                                             "RWY 34L", "RWY 17L", "RWY 35R", "RWY 15",  "RWY 33"};
        std::set<std::string> reached;
        bool stand539 = airport.id == "ZSPD" && distance(position, {31.135482, 121.802825}) < 5;
        for (const auto &destination : destinations(airport)) {
            if (destination.kind != DestinationKind::Runway)
                continue;
            Route route;
            try {
                route = planRoute(airport, position, yaw, destination, options);
            } catch (const std::exception &e) {
                std::cout << destination.label << " unavailable: " << e.what() << "\n";
                check(!stand539 || !expected.count(destination.label),
                      destination.label + " must be reachable from stand 539");
                continue;
            }
            reached.insert(destination.label);
            if (stand539 && (std::abs(wrap180(yaw)) < 1 || std::abs(wrap180(yaw - 342)) < 1))
                check(!route.requiresPushback,
                      destination.label + ": north-facing departure must be forward");
            if (stand539 && std::abs(wrap180(yaw - 72.3)) < 1)
                check(route.requiresPushback, destination.label + ": nose-in stand must wait for a tow");
            std::cout << destination.label << " " << route.length << " m / "
                      << (route.requiresPushback ? "pushback needed" : "forward departure");
            if (!route.requiresPushback)
                simulate(route, yaw);
            else {
                // A tow moves the aircraft to the first network segment before taxi control starts.
                check(route.points.size() > 2, "Pushback route needs a network segment");
                Vec2 first = route.points[2] - route.points[1];
                double offset = std::min(20.0, length(first) * 0.25);
                auto handoff = unproject(route.origin, route.points[1] + first * (offset / length(first)));
                auto forward = planRoute(airport, handoff, heading(first), destination, options);
                check(!forward.requiresPushback, "Forward handoff must not request another pushback");
                simulate(forward, heading(first));
            }
            std::cout << "\n";
        }
        if (stand539)
            check(reached == expected,
                  "ZSPD must support all ten runway directions, including painted-only 15/33 entries");
        else
            check(!reached.empty(), "No runway routes found");
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAILED: " << e.what() << "\n";
        return 1;
    }
}
