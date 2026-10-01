#include "AptDatabase.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
using namespace autotaxi;
namespace {
struct Usage {
    int instances = 0, anchors = 0, curved = 0, unmatched = 0;
    double maxGap = 0;
};
bool candidate(const std::string &definition) {
    return definition.find("Single_Yellow_solid") != std::string::npos ||
           definition.find("Double_Yellow") != std::string::npos ||
           definition.find("RWY_W_Centre.lin") != std::string::npos;
}
int candidateKind(const std::string &definition) {
    if (definition.find("RWY_W_Centre.lin") != std::string::npos)
        return 3;
    if (definition.find("Double_Yellow") != std::string::npos)
        return 2;
    return definition.find("Single_Yellow_solid") != std::string::npos ? 1 : 0;
}
} // namespace
int main(int argc, char **argv) {
    try {
        if (argc != 3)
            throw std::runtime_error("Usage: dsf_audit apt.dat DSFTool-output.txt");
        std::ifstream apt(std::filesystem::u8path(argv[1])), dsf(std::filesystem::u8path(argv[2]));
        if (!apt || !dsf)
            throw std::runtime_error("Cannot open local scenery input");
        auto airport = parseAirport(apt, argv[1], false);
        GeoPoint origin = airport.runways.empty() ? GeoPoint{} : airport.runways[0].ends[0].position;
        std::vector<std::pair<Vec2, Vec2>> yellow;
        std::vector<std::pair<Vec2, Vec2>> taxiways;
        for (const auto &edge : airport.edges)
            if (!edge.runway && airport.nodes.count(edge.from) && airport.nodes.count(edge.to))
                taxiways.push_back({project(origin, airport.nodes.at(edge.from).position),
                                    project(origin, airport.nodes.at(edge.to).position)});
        for (const auto &line : airport.groundLines)
            for (std::size_t i = 1; i < line.points.size(); ++i) {
                auto segment =
                    std::make_pair(project(origin, line.points[i - 1]), project(origin, line.points[i]));
                if (line.centerline() || line.style == 2 || line.style == 3 || line.style == 4 ||
                    line.style == 54)
                    yellow.push_back(segment);
            }
        std::vector<std::string> definitions;
        std::map<std::string, Usage> usage;
        std::string active;
        int objects = 0, networks = 0;
        for (std::string row; std::getline(dsf, row);) {
            std::istringstream record(row);
            std::string command;
            record >> command;
            if (command == "POLYGON_DEF") {
                std::string definition;
                std::getline(record >> std::ws, definition);
                definitions.push_back(definition);
            } else if (command == "OBJECT_DEF") {
                ++objects;
            } else if (command == "NETWORK_DEF") {
                ++networks;
            } else if (command == "BEGIN_POLYGON") {
                int index, parameter, depth;
                if (!(record >> index >> parameter >> depth) || index < 0 ||
                    static_cast<std::size_t>(index) >= definitions.size())
                    throw std::runtime_error("Invalid polygon definition reference");
                active = definitions[index];
                auto &entry = usage[active];
                ++entry.instances;
                if (depth == 4 && std::filesystem::path(active).extension() == ".lin")
                    ++entry.curved;
            } else if (command == "END_POLYGON") {
                active.clear();
            } else if (command == "POLYGON_POINT" && !active.empty()) {
                GeoPoint point;
                if (!(record >> point.lon >> point.lat) || !valid(point))
                    throw std::runtime_error("Invalid polygon coordinate");
                auto &entry = usage[active];
                ++entry.anchors;
                if (!candidate(active))
                    continue;
                Vec2 position = project(origin, point);
                double gap = std::numeric_limits<double>::infinity();
                if (candidateKind(active) == 3) {
                    for (const auto &runway : airport.runways)
                        gap = std::min(gap, onSegment(position, project(origin, runway.ends[0].position),
                                                      project(origin, runway.ends[1].position))
                                                .distance);
                } else {
                    const auto &segments = candidateKind(active) == 2 ? yellow : taxiways;
                    for (const auto &segment : segments)
                        gap = std::min(gap, onSegment(position, segment.first, segment.second).distance);
                }
                entry.maxGap = std::max(entry.maxGap, gap);
                if (gap > 3)
                    ++entry.unmatched;
            }
        }
        if (definitions.empty())
            throw std::runtime_error("No polygon definitions in DSFTool input");
        std::cout
            << "Airport: " << airport.id << "\nAPT source: " << airport.source
            << "\nDSF definitions: " << objects << " objects, " << definitions.size() << " polygons, "
            << networks << " networks\n"
            << "Single-yellow anchors use apt.dat 1202 taxiway edges; double-yellow anchors use apt.dat "
               "yellow styles;\n"
            << "runway anchors use row 100 axes. A 3 m gap threshold is only a geometry diagnostic.\n";
        std::cout << std::fixed << std::setprecision(2);
        for (const auto &item : usage) {
            if (std::filesystem::path(item.first).extension() != ".lin" &&
                item.first.find("Stand590.pol") == std::string::npos)
                continue;
            const auto &entry = item.second;
            std::cout << item.first << ": instances=" << entry.instances << " anchors=" << entry.anchors
                      << " curves=" << entry.curved;
            if (candidate(item.first))
                std::cout << " unmatched(>3m)=" << entry.unmatched << " max-gap=" << entry.maxGap << "m";
            std::cout << '\n';
        }
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
