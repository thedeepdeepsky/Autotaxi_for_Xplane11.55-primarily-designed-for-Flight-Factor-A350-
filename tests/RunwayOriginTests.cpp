#include "Config.h"
#include "PavementQuery.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace autotaxi;
namespace {
Airport readAirport(const std::string &path, const std::string &id) {
    std::ifstream input(std::filesystem::u8path(path));
    for (std::string row; input;) {
        auto offset = input.tellg();
        if (!std::getline(input, row))
            break;
        std::istringstream header(row);
        int code, height, tower, obsolete;
        std::string airport;
        if (header >> code >> height >> tower >> obsolete >> airport && code == 1 && airport == id) {
            input.seekg(offset);
            return parseAirport(input, path);
        }
    }
    throw std::runtime_error("Airport not found: " + id);
}
void localOrigins(const Airport &airport, RouteOptions options) {
    options.runwayClearance = true;
    options.ignoreStandSize = options.ignorePavementLimits = true;
    const auto destinationsList = destinations(airport);
    auto gate = std::find_if(destinationsList.begin(), destinationsList.end(), [](const Destination &d) {
        return d.label == "Gate 271 T2";
    });
    int failures = 0;
    for (std::size_t index = 0; index < airport.runways.size(); ++index) {
        const auto &runway = airport.runways[index];
        for (int end = 0; end < 2; ++end) {
            Vec2 delta = project(runway.ends[end].position, runway.ends[1 - end].position);
            Vec2 axis = delta * (1 / length(delta));
            double startAlong = 1e30, lastAlong = -1e30;
            for (const auto &edge : airport.edges)
                if (edge.runway && edge.name.find(runway.ends[end].name) != std::string::npos)
                    for (int node : {edge.from, edge.to}) {
                        double along = dot(project(runway.ends[end].position, airport.nodes.at(node).position), axis);
                        startAlong = std::min(startAlong, along);
                        lastAlong = std::max(lastAlong, along);
                    }
            std::cout << "RWY " << runway.ends[end].name << " axis " << startAlong << ".." << lastAlong
                      << " / surveyed " << length(delta) << " / displaced " << runway.ends[end].displaced << " m\n";
            for (double offset : {0., 25., 80.}) {
                auto position = unproject(runway.ends[end].position, axis * offset);
                for (const auto &target : destinationsList) {
                    if (target.kind != DestinationKind::Runway &&
                        (gate == destinationsList.end() || target.label != gate->label))
                        continue;
                    try {
                        auto route = planRoute(airport, position, heading(axis), target, options);
                        if (route.requiresPushback)
                            throw std::runtime_error("A runway start must not request towing");
                        std::cout << "  +" << offset << " -> " << target.label << ": " << route.length << " m\n";
                    } catch (const std::exception &error) {
                        std::cout << "  +" << offset << " -> " << target.label << ": " << error.what() << '\n';
                        ++failures;
                    }
                }
            }
        }
    }
    if (failures)
        throw std::runtime_error(std::to_string(failures) + " runway-origin routes failed");
}
} // namespace
int main(int argc, char **argv) {
    try {
        std::cout.setf(std::ios::unitbuf);
        if (argc != 3)
            throw std::runtime_error("Usage: runway_origin_tests apt.dat config.ini");
        const auto airport = readAirport(argv[1], "ZSSS");
        std::cout << airport.id << " nodes=" << airport.nodes.size() << " edges=" << airport.edges.size()
                  << " runways=" << airport.runways.size() << " entries=" << airport.paintedRunwayEntries.size() << '\n';
        localOrigins(airport, loadConfig(argv[2]).route);
        std::cout << "Runway origin checks passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
