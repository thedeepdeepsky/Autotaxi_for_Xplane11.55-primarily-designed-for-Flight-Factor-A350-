#include "Config.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
using namespace autotaxi;
namespace {
Airport readAirport(std::ifstream &input, const std::string &id, const std::string &source) {
    for (std::string row; input;) {
        const auto offset = input.tellg();
        if (!std::getline(input, row))
            break;
        if (row.rfind("1 ", 0) != 0)
            continue;
        std::istringstream header(row);
        int code, height, tower, obsolete;
        std::string airport;
        if (header >> code >> height >> tower >> obsolete >> airport && airport == id) {
            input.seekg(offset);
            return parseAirport(input, source);
        }
    }
    throw std::runtime_error("Airport not found: " + id);
}
double alignedGap(const std::vector<GeoPoint> &points, const Ramp &ramp, double &angle) {
    double gap = 1e30;
    angle = 180;
    for (std::size_t k = 1; k < points.size(); ++k) {
        const auto a = project(ramp.position, points[k - 1]);
        const auto b = project(ramp.position, points[k]);
        if (length(b - a) < .01)
            continue;
        const double yaw = std::abs(wrap180(heading(b - a) - ramp.heading));
        const double turn = std::min(yaw, 180.0 - yaw);
        const auto snap = onSegment({}, a, b);
        if (turn <= 35 && snap.distance < gap) {
            gap = snap.distance;
            angle = turn;
        }
    }
    return gap;
}
} // namespace
int main(int argc, char **argv) {
    try {
        if (argc < 3 || argc > 4)
            throw std::runtime_error("Usage: stand_lead_in_audit apt.dat airport-id [stand-name]");
        std::ifstream input(std::filesystem::u8path(argv[1]), std::ios::binary);
        if (!input)
            throw std::runtime_error("Cannot open airport file");
        const auto airport = readAirport(input, argv[2], argv[1]);
        std::cout << std::fixed << std::setprecision(1) << airport.id << " stands=" << airport.ramps.size()
                  << " lines=" << airport.groundLines.size() << '\n';
        std::map<int, std::pair<int, int>> styles;
        for (const auto &line : airport.groundLines) {
            ++styles[line.style].first;
            styles[line.style].second += line.centerline();
        }
        for (const auto &entry : styles)
            std::cout << "Style " << entry.first << ": " << entry.second.first << " features, "
                      << entry.second.second << " navigable\n";

        // Undirected paint components report topology only, not clearance or
        // aircraft turn feasibility. Keep that distinction explicit in output.
        std::unordered_map<int, std::vector<int>> adjacent;
        for (const auto &edge : airport.edges)
            if (edge.painted && !edge.runway) {
                adjacent[edge.from].push_back(edge.to);
                adjacent[edge.to].push_back(edge.from);
            }
        std::unordered_map<int, int> component;
        std::vector<std::size_t> sizes;
        for (const auto &entry : adjacent) {
            if (component.count(entry.first))
                continue;
            const int id = static_cast<int>(sizes.size());
            sizes.push_back(0);
            std::queue<int> pending;
            pending.push(entry.first);
            component[entry.first] = id;
            while (!pending.empty()) {
                const int node = pending.front();
                pending.pop();
                ++sizes[id];
                for (int next : adjacent[node])
                    if (component.emplace(next, id).second)
                        pending.push(next);
            }
        }
        int matched = 0, missing = 0;
        bool requestedFound = argc != 4;
        for (const auto &ramp : airport.ramps) {
        if (argc == 4 && ramp.name != argv[3])
                continue;
            requestedFound = true;
            double best = 45, angle = 180;
            int style = -1;
            for (const auto &line : airport.groundLines) {
                if (!line.centerline())
                    continue;
                double turn;
                const double gap = alignedGap(line.points, ramp, turn);
                if (gap < best) {
                    best = gap;
                    angle = turn;
                    style = line.style;
                }
            }
            double graphGap = 45;
            std::size_t componentSize = 0;
            for (const auto &edge : airport.edges) {
                if (!edge.painted || edge.runway)
                    continue;
                double turn;
                const double gap = alignedGap(edge.geometry, ramp, turn);
                if (gap < graphGap) {
                    graphGap = gap;
                    const auto found = component.find(edge.from);
                    componentSize = found == component.end() ? 0 : sizes[found->second];
                }
            }
            if (style >= 0) {
                ++matched;
                std::cout << ramp.name << ": MATCH style=" << style << " gap=" << best
                          << " m angle=" << angle << " deg paint-component-nodes=" << componentSize << '\n';
            } else {
                ++missing;
                std::cout << ramp.name << ": NO ALIGNED LEAD-IN within 45 m\n";
            }
            if (argc == 4) {
                std::vector<std::pair<double, std::size_t>> nearby;
                for (std::size_t i = 0; i < airport.groundLines.size(); ++i) {
                    const auto &line = airport.groundLines[i];
                    double nearest = 1e30;
                    for (std::size_t k = 1; k < line.points.size(); ++k)
                        nearest = std::min(nearest, onSegment({}, project(ramp.position, line.points[k - 1]),
                                                               project(ramp.position, line.points[k])).distance);
                    if (nearest < 80)
                        nearby.push_back({nearest, i});
                }
                std::sort(nearby.begin(), nearby.end());
                for (const auto &entry : nearby) {
                    const auto &line = airport.groundLines[entry.second];
                    double turn;
                    const double gap = alignedGap(line.points, ramp, turn);
                    const double endpointGap = std::min(distance(line.points.front(), ramp.position),
                                                        distance(line.points.back(), ramp.position));
                    double lineLength = 0;
                    for (std::size_t k = 1; k < line.points.size(); ++k)
                        lineLength += distance(line.points[k - 1], line.points[k]);
                    std::cout << "  nearby style=" << line.style << " gap=" << entry.first
                              << " m aligned-gap=" << (gap < 1e20 ? gap : -1)
                              << " m endpoint-gap=" << endpointGap
                              << " m length=" << lineLength
                              << " m navigable=" << line.centerline() << " stand-lead-in=" << line.standLeadIn
                              << " ramp-index=" << line.standLeadInRamp
                              << " points=" << line.points.size() << " name=" << line.name << '\n';
                    if (argc == 4 && line.style == 51) {
                        std::cout << "    geometry:";
                        for (const auto &point : line.points) {
                            const auto xy = project(ramp.position, point);
                            std::cout << " (" << xy.x << "," << xy.y << ")";
                        }
                        std::cout << '\n';
                        double firstJoin = 1e9, lastJoin = 1e9;
                        for (const auto &other : airport.groundLines) {
                            if (&other == &line || !other.paintedCenterline())
                                continue;
                            const Vec2 first = project(line.points.front(), line.points.front());
                            const Vec2 last = project(line.points.back(), line.points.back());
                            for (std::size_t k = 1; k < other.points.size(); ++k) {
                                firstJoin = std::min(firstJoin, onSegment(first,
                                                                          project(line.points.front(), other.points[k - 1]),
                                                                          project(line.points.front(), other.points[k])).distance);
                                lastJoin = std::min(lastJoin, onSegment(last,
                                                                         project(line.points.back(), other.points[k - 1]),
                                                                         project(line.points.back(), other.points[k])).distance);
                            }
                        }
                        std::cout << "    endpoint joins=" << firstJoin << "," << lastJoin << '\n';
                    }
                }
                const auto config = loadConfig("A350AutoTaxi.ini");
                auto options = config.route;
                options.runwayClearance = options.ignoreStandSize = options.ignorePavementLimits = true;
                std::vector<std::pair<double, std::size_t>> starts;
                for (std::size_t k = 0; k < airport.edges.size(); ++k) {
                    const auto &edge = airport.edges[k];
                    const double gap = distance(airport.nodes.at(edge.from).position, ramp.position);
                    if (edge.painted && !edge.runway && edge.geometry.size() > 1 && gap > 150 && gap < 500)
                        starts.push_back({gap, k});
                }
                std::sort(starts.begin(), starts.end());
                bool success = false;
                std::string lastError;
                for (std::size_t k = 0; k < std::min<std::size_t>(starts.size(), 16); ++k) {
                    const auto &edge = airport.edges[starts[k].second];
                    try {
                        const auto route = planRoute(airport, edge.geometry.front(),
                                                     heading(project(edge.geometry[0], edge.geometry[1])),
                                                     {DestinationKind::Ramp, ramp.name,
                                                      static_cast<int>(&ramp - airport.ramps.data())}, options);
                        if (route.requiresPushback)
                            continue;
                        std::cout << "Local approach OK from node " << edge.from << ": " << route.length
                                  << " m, cockpit=" << route.cockpitGuidance << '\n';
                        success = true;
                        break;
                    } catch (const std::exception &error) {
                        lastError = error.what();
                    }
                }
                if (!success)
                    std::cout << "Local approach unavailable: " << lastError << '\n';
            }
        }
        if (argc == 3) {
            std::cout << "Unclassified stands:\n";
            for (std::size_t i = 0; i < airport.ramps.size(); ++i) {
                const auto &ramp = airport.ramps[i];
                const bool classified = std::any_of(airport.groundLines.begin(), airport.groundLines.end(),
                                                    [&](const GroundLine &line) {
                                                        return line.standLeadInRamp == static_cast<int>(i);
                                                    });
                if (!classified)
                    std::cout << "  " << ramp.name << " heading=" << ramp.heading << '\n';
            }
        }
        if (!requestedFound)
            throw std::runtime_error("Stand not found");
        std::cout << "Summary: " << matched << " matched, " << missing << " missing. "
                  << "Paint connectivity is not proof of a feasible route.\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
