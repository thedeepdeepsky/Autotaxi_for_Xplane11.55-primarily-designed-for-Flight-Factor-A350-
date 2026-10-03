#include "PlanningWorker.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace autotaxi;
int main(int argc, char **argv) {
    try {
        if (argc != 4)
            throw std::runtime_error("Usage: planning_benchmark apt.dat stand runway");
        std::ifstream input(std::filesystem::u8path(argv[1]));
        if (!input)
            throw std::runtime_error("Cannot open airport");
        const auto airport = parseAirport(input, argv[1]);
        const auto ramp = std::find_if(airport.ramps.begin(), airport.ramps.end(),
                                       [&](const Ramp &r) { return r.name == argv[2]; });
        if (ramp == airport.ramps.end())
            throw std::runtime_error("Stand not found");
        const auto choices = destinations(airport);
        const auto destination = std::find_if(choices.begin(), choices.end(), [&](const Destination &d) {
            return d.kind == DestinationKind::Runway &&
                   airport.runways.at(d.index).ends[d.end].name == argv[3];
        });
        if (destination == choices.end())
            throw std::runtime_error("Runway not found");
        std::cout.setf(std::ios::unitbuf);
        const AircraftState state{ramp->position, ramp->heading, 0, true};
        Config config;
        config.route.runwayClearance = true;
        const auto start = std::chrono::steady_clock::now();
        std::cout << "Planning stand " << argv[2] << " to runway " << argv[3] << '\n';
        const auto plan = planDeparture(airport, state, *destination, config, config.route);
        const auto seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::cout << "Planning took " << seconds << " s, taxi " << plan.route.length << " m";
        if (plan.pushback)
            std::cout << ", tow " << plan.pushback->length << " m";
        std::cout << '\n';
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
