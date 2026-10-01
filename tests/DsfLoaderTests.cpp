#include "AptDatabase.h"
#include "CenterlineNetwork.h"
#include "DsfLoader.h"
#include "PushbackPlanner.h"
#include "RoutePlanner.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace autotaxi;
int main(int argc, char **argv) {
    try {
        if (argc != 4)
            throw std::runtime_error("Usage: dsf_loader_tests apt.dat X-Plane-root DSFTool.exe");
        std::ifstream input(std::filesystem::u8path(argv[1]));
        auto airport = parseAirport(input, argv[1], false);
        auto before = airport.groundLines.size();
        buildCenterlineNetwork(airport);
        RouteOptions baselineOptions;
        baselineOptions.runwayClearance = true;
        AircraftState aircraft{{31.13382579, 121.81707839}, 72.41, 0, true};
        auto baseline =
            planPushback(airport, aircraft, {DestinationKind::Runway, "RWY 33", 0, 1}, baselineOptions, {});
        if (baseline.taxi.points.size() < 2)
            throw std::runtime_error("Baseline route has no geometry");
        input.close();
        input.open(std::filesystem::u8path(argv[1]));
        airport = parseAirport(input, argv[1], false);
        before = airport.groundLines.size();
        auto result = loadDsfPaintedLines(airport, std::filesystem::u8path(argv[2]), argv[3]);
        if (result.files != 1 || result.lines <= 0 || airport.groundLines.size() <= before)
            throw std::runtime_error("DSF loader did not import single-yellow lines");
        buildCenterlineNetwork(airport);
        RouteOptions options;
        options.runwayClearance = true;
        auto after = planPushback(airport, aircraft, {DestinationKind::Runway, "RWY 33", 0, 1}, options, {});
        if (after.taxi.points.size() < 2)
            throw std::runtime_error("DSF-augmented route has no geometry");
        std::cout << "DSF loader imported " << result.lines << " line chains; " << airport.groundLines.size()
                  << " total ground lines\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
