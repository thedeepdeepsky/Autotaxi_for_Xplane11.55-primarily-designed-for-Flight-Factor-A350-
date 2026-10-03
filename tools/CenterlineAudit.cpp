#include "PavementQuery.h"
#include "RoutePlanner.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
using namespace autotaxi;
int main(int argc, char **argv) {
    try {
        if (argc != 5 && argc != 6)
            throw std::runtime_error(
                "Usage: centerline_audit apt.dat runway start-node heading [max-length]");
        std::ifstream input(std::filesystem::u8path(argv[1]));
        if (!input)
            throw std::runtime_error("Cannot open airport");
        auto a = parseAirport(input, argv[1]);
        std::cout << "Repaired gaps: " << a.centerlineGapLinks << '\n';
        Destination destination;
        bool found = false;
        for (auto d : destinations(a))
            if (d.kind == DestinationKind::Runway && a.runways[d.index].ends[d.end].name == argv[2]) {
                destination = d;
                found = true;
            }
        if (!found)
            throw std::runtime_error("Runway not found");
        auto threshold = a.runways[destination.index].ends[destination.end].position;
        for (const auto &e : a.edges)
            if (e.inferredGap && distance(threshold, a.nodes.at(e.from).position) < 800) {
                auto p = project(threshold, a.nodes.at(e.from).position);
                auto q = project(threshold, a.nodes.at(e.to).position);
                std::cout << "Repaired " << e.from << " -> " << e.to << " offset " << p.x << ',' << p.y
                          << " -> " << q.x << ',' << q.y << '\n';
            }
        PavementQuery pavement(a, threshold, true);
        std::map<int, std::vector<int>> incident;
        for (std::size_t i = 0; i < a.edges.size(); ++i)
            if (a.edges[i].painted && !a.edges[i].runway) {
                incident[a.edges[i].from].push_back(static_cast<int>(i));
                incident[a.edges[i].to].push_back(static_cast<int>(i));
            }
        std::cout << std::fixed << std::setprecision(2);
        for (const auto &[id, edges] : incident) {
            const auto geo = a.nodes.at(id).position;
            if (edges.size() != 1 || distance(threshold, geo) > 800)
                continue;
            const auto &edge = a.edges[edges.front()];
            const auto &shape = edge.geometry;
            const auto before = edge.from == id ? shape[1] : shape[shape.size() - 2];
            Vec2 tangent = project(before, geo);
            double best = 20, angle = 0, advance = 0;
            int nearest = -1;
            GeoPoint target;
            for (std::size_t i = 0; i < a.edges.size(); ++i) {
                const auto &other = a.edges[i];
                if (!other.painted || other.runway || other.from == id || other.to == id)
                    continue;
                for (std::size_t k = 1; k < other.geometry.size(); ++k) {
                    Vec2 p = project(geo, other.geometry[k - 1]), q = project(geo, other.geometry[k]);
                    const auto snap = onSegment({}, p, q);
                    if (snap.distance < best) {
                        best = snap.distance;
                        nearest = static_cast<int>(i);
                        angle = std::abs(wrap180(heading(q - p) - heading(tangent)));
                        angle = std::min(angle, 180 - angle);
                        advance = std::abs(wrap180(heading(snap.point) - heading(tangent)));
                        target = unproject(geo, snap.point);
                    }
                }
            }
            if (nearest >= 0)
                std::cout << "Endpoint " << id << " " << edge.name << " -> " << a.edges[nearest].name
                          << " gap=" << best << " tangent=" << angle << " advance=" << advance
                          << " paved=" << pavement.connection(geo, target) << '\n';
        }
        int id = std::stoi(argv[3]);
        if (a.nodeAliases.count(id))
            id = a.nodeAliases.at(id);
        auto start = a.nodes.at(id).position;
        std::cout << "Start node " << id << " at " << std::setprecision(8) << start.lat << ',' << start.lon
                  << '\n';
        RouteOptions o;
        o.runwayClearance = true;
        o.maxJoinDistance = o.maxApronJoinDistance = 8;
        const auto route = planRoute(a, start, std::stod(argv[4]), destination, o);
        std::cout << "Route " << route.length << " m, pushback=" << route.requiresPushback << " names:";
        for (auto marker : route.taxiwaysAlongRoute)
            std::cout << ' ' << marker.label;
        std::cout << '\n';
        std::cout << "Route inferred gaps: " << route.inferredGapCount
                  << ", ATC fallback: " << route.atcFallbackCount << '\n';
        for (std::size_t i = 1; i < route.nodeIds.size(); ++i)
            for (const auto &e : a.edges)
                if (!e.painted && !e.runway &&
                    ((e.from == route.nodeIds[i - 1] && e.to == route.nodeIds[i]) ||
                     (!e.oneWay && e.to == route.nodeIds[i - 1] && e.from == route.nodeIds[i])))
                    std::cout << "Fallback " << e.name << ' ' << e.from << " -> " << e.to << " length "
                              << distance(a.nodes.at(e.from).position, a.nodes.at(e.to).position) << '\n';
        auto painted = a;
        painted.edges.erase(std::remove_if(painted.edges.begin(), painted.edges.end(),
                                           [](const TaxiEdge &e) { return !e.painted && !e.runway; }),
                            painted.edges.end());
        try {
            auto paintRoute = planRoute(painted, start, std::stod(argv[4]), destination, o);
            std::cout << "Paint-only route " << paintRoute.length << " m, inferred gaps "
                      << paintRoute.inferredGapCount << '\n';
        } catch (const std::exception &e) {
            std::cout << "Paint-only unavailable: " << e.what() << '\n';
        }
        if (argc == 6 && (route.length > std::stod(argv[5]) || route.requiresPushback))
            throw std::runtime_error("Local paint-gap regression: route detoured or requested pushback");
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
