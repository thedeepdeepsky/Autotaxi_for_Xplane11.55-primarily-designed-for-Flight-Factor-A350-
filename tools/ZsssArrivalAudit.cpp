#include "RoutePlanner.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace autotaxi;

namespace {
std::string clean(std::string value) {
    for (char &c : value)
        if (c == '\t' || c == '\r' || c == '\n')
            c = ' ';
    return value;
}

} // namespace

int main(int argc, char **argv) {
    try {
        if (argc < 5 || argc > 6)
            throw std::runtime_error("Usage: zsss_arrival_audit apt.dat output.tsv airport-id runway-name");
        std::ifstream input(std::filesystem::u8path(argv[1]), std::ios::binary);
        if (!input)
            throw std::runtime_error("Cannot open apt.dat");
        // The audit input is normally a one-airport extracted apt.dat.  Keep
        // the airport argument for the same invocation shape as the plugin's
        // airport loader and reject a mismatched header explicitly.
        Airport airport = parseAirport(input, argv[3]);
        const std::string runwayName = argv[4];
        auto runway = std::find_if(airport.runways.begin(), airport.runways.end(), [&](const Runway &r) {
            return r.ends[0].name == runwayName || r.ends[1].name == runwayName;
        });
        if (runway == airport.runways.end())
            throw std::runtime_error("Runway not found: " + runwayName);
        const int end = runway->ends[0].name == runwayName ? 0 : 1;
        const int opposite = end == 0 ? 1 : 0;
        const GeoPoint origin = runway->ends[end].position;
        const double trueHeading = heading(project(origin, runway->ends[opposite].position));

        RouteOptions options;
        options.runwayClearance = true;
        options.allowUnknownWidth = true;
        std::ofstream output(std::filesystem::u8path(argv[2]), std::ios::binary);
        if (!output)
            throw std::runtime_error("Cannot create audit output");
        output << std::setprecision(9);
        output << "kind\tid\tname\tx\ty\theading\tstyle\tlead\treachable\treason\tlength\toversteer\n";
        for (std::size_t lineIndex = 0; lineIndex < airport.groundLines.size(); ++lineIndex) {
            const auto &line = airport.groundLines[lineIndex];
            for (const auto &point : line.points) {
                const Vec2 xy = project(origin, point);
                output << "line\t" << lineIndex << '\t' << clean(line.name) << '\t' << xy.x << '\t' << xy.y << "\t0\t"
                       << line.style << '\t' << (line.standLeadIn ? 1 : 0) << "\t0\t\t0\t0\n";
            }
            output << "line_end\t" << lineIndex << "\t\t0\t0\t0\t" << line.style << '\t'
                   << (line.standLeadIn ? 1 : 0)
                   << "\t0\t\t0\t0\n";
        }
        output << "runway\t" << runwayName << "\t\t0\t0\t" << trueHeading << "\t0\t0\t1\t\t0\t0\n";
        int unreachable = 0;
        for (std::size_t i = 0; i < airport.ramps.size(); ++i) {
            const Ramp &ramp = airport.ramps[i];
            const Destination destination{DestinationKind::Ramp, rampLabel(ramp), static_cast<int>(i), 0};
            const Vec2 xy = project(origin, ramp.position);
            bool reachable = false;
            std::string reason;
            double routeLength = 0, oversteer = 0;
            try {
                const Route route = planRoute(airport, origin, trueHeading, destination, options);
                reachable = !route.requiresPushback;
                routeLength = route.length;
                oversteer = route.maximumOversteer;
                if (!reachable)
                    reason = "requires pushback";
            } catch (const std::exception &e) {
                reason = clean(e.what());
            }
            if (!reachable)
                ++unreachable;
            output << "ramp\t" << i << '\t' << clean(rampLabel(ramp)) << '\t' << xy.x << '\t' << xy.y << '\t'
                   << ramp.heading << "\t0\t0\t" << (reachable ? 1 : 0) << '\t' << reason << '\t'
                   << routeLength << '\t' << oversteer << '\n';
        }
        std::cout << "airport=" << airport.id << " runway=" << runwayName << " ramps=" << airport.ramps.size()
                  << " unreachable=" << unreachable << "\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
