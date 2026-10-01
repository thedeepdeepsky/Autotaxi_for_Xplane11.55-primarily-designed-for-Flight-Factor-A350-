#include "CenterlineNetwork.h"
#include <algorithm>
#include <map>
#include <queue>
#include <set>
#include <unordered_set>
namespace autotaxi {
namespace {
using Cell = std::pair<int, int>;
Cell cell(Vec2 p, double size) {
    return {static_cast<int>(std::floor(p.x / size)), static_cast<int>(std::floor(p.y / size))};
}
struct Segment {
    Vec2 a, b;
    int runwaySource = -1;
    std::string sourceName;
    std::vector<double> cuts{0, 1};
};
double cross(Vec2 a, Vec2 b) {
    return a.x * b.y - a.y * b.x;
}
void split(Segment &a, Segment &b) {
    Vec2 u = a.b - a.a, v = b.b - b.a;
    double determinant = cross(u, v);
    if (std::abs(determinant) > 1e-9) {
        double t = cross(b.a - a.a, v) / determinant, s = cross(b.a - a.a, u) / determinant;
        if (t >= 0 && t <= 1 && s >= 0 && s <= 1) {
            a.cuts.push_back(t);
            b.cuts.push_back(s);
        }
    }
    // WED feature endpoints may differ by rounding; connect only sub-metre gaps.
    for (auto point : {a.a, a.b}) {
        auto p = onSegment(point, b.a, b.b);
        if (p.distance < .3)
            b.cuts.push_back(p.t);
    }
    for (auto point : {b.a, b.b}) {
        auto p = onSegment(point, a.a, a.b);
        if (p.distance < .3)
            a.cuts.push_back(p.t);
    }
}
template <class F> void cells(Vec2 a, Vec2 b, double padding, double size, F f) {
    auto lo = cell({std::min(a.x, b.x) - padding, std::min(a.y, b.y) - padding}, size);
    auto hi = cell({std::max(a.x, b.x) + padding, std::max(a.y, b.y) + padding}, size);
    for (int x = lo.first; x <= hi.first; ++x)
        for (int y = lo.second; y <= hi.second; ++y)
            f(Cell{x, y});
}
bool sameRules(const TaxiEdge &a, const TaxiEdge &b) {
    return a.oneWay == b.oneWay && a.runway == b.runway && a.width == b.width && a.name == b.name &&
           a.activeRunways == b.activeRunways && a.painted == b.painted;
}
} // namespace
void buildCenterlineNetwork(Airport &airport) {
    airport.atcFallbackEdges = static_cast<std::size_t>(std::count_if(
        airport.edges.begin(), airport.edges.end(), [](const TaxiEdge &edge) { return !edge.runway; }));
    if (airport.nodes.empty() || airport.edges.empty() ||
        std::none_of(airport.groundLines.begin(), airport.groundLines.end(),
                     [](const GroundLine &line) { return line.centerline(); }))
        return;
    GeoPoint origin = airport.nodes.begin()->second.position;
    const auto atc = airport.edges;
    const auto originalNodes = airport.nodes;
    std::vector<Segment> segments;
    for (const auto &line : airport.groundLines)
        if (line.centerline())
            for (std::size_t i = 1; i < line.points.size(); ++i) {
                Vec2 a = project(origin, line.points[i - 1]), b = project(origin, line.points[i]);
                if (length(b - a) > .01) {
                    int count = std::max(1, static_cast<int>(std::ceil(length(b - a) / 20)));
                    for (int k = 0; k < count; ++k)
                        segments.push_back({a + (b - a) * (static_cast<double>(k) / count),
                                            a + (b - a) * (static_cast<double>(k + 1) / count),
                                            -1,
                                            line.name,
                                            {0, 1}});
                }
            }
    // Runway navigation uses row 100's surveyed axis, keeping ATC's entry topology.
    for (std::size_t i = 0; i < atc.size(); ++i)
        if (atc[i].runway) {
            Vec2 a = project(origin, originalNodes.at(atc[i].from).position);
            Vec2 b = project(origin, originalNodes.at(atc[i].to).position);
            for (const auto &runway : airport.runways) {
                if (atc[i].name.find(runway.ends[0].name) == std::string::npos &&
                    atc[i].name.find(runway.ends[1].name) == std::string::npos)
                    continue;
                Vec2 p = project(origin, runway.ends[0].position),
                     q = project(origin, runway.ends[1].position);
                a = onSegment(a, p, q).point;
                b = onSegment(b, p, q).point;
                break;
            }
            int count = std::max(1, static_cast<int>(std::ceil(length(b - a) / 20)));
            for (int k = 0; k < count; ++k)
                segments.push_back({a + (b - a) * (static_cast<double>(k) / count),
                                    a + (b - a) * (static_cast<double>(k + 1) / count),
                                    static_cast<int>(i),
                                    {},
                                    {0, 1}});
        }
    std::map<Cell, std::vector<int>> spatial;
    for (std::size_t i = 0; i < segments.size(); ++i) {
        std::set<int> nearby;
        cells(segments[i].a, segments[i].b, .3, 20, [&](Cell c) {
            auto it = spatial.find(c);
            if (it != spatial.end())
                nearby.insert(it->second.begin(), it->second.end());
        });
        for (int other : nearby)
            split(segments[i], segments[other]);
        cells(segments[i].a, segments[i].b, .3, 20,
              [&](Cell c) { spatial[c].push_back(static_cast<int>(i)); });
    }
    std::unordered_map<int, std::vector<int>> incidentAtc;
    std::vector<Vec2> oldFrom, oldTo;
    std::map<Cell, std::vector<int>> rules;
    int nextId = 0;
    for (const auto &n : originalNodes)
        nextId = std::max(nextId, n.first + 1);
    const int firstVirtualId = nextId;
    for (std::size_t i = 0; i < atc.size(); ++i) {
        incidentAtc[atc[i].from].push_back(static_cast<int>(i));
        incidentAtc[atc[i].to].push_back(static_cast<int>(i));
        oldFrom.push_back(project(origin, originalNodes.at(atc[i].from).position));
        oldTo.push_back(project(origin, originalNodes.at(atc[i].to).position));
        cells(oldFrom.back(), oldTo.back(), 45, 80, [&](Cell c) { rules[c].push_back(static_cast<int>(i)); });
    }
    std::map<int, Vec2> aliases;
    for (const auto &entry : incidentAtc) {
        Vec2 p = project(origin, originalNodes.at(entry.first).position);
        bool runwayNode =
            std::any_of(entry.second.begin(), entry.second.end(), [&](int i) { return atc[i].runway; });
        double best = 40;
        int chosen = -1;
        Projection match;
        for (std::size_t i = 0; i < segments.size(); ++i) {
            if (runwayNode && segments[i].runwaySource < 0)
                continue;
            auto candidate = onSegment(p, segments[i].a, segments[i].b);
            if (candidate.distance < best) {
                best = candidate.distance;
                chosen = static_cast<int>(i);
                match = candidate;
            }
        }
        if (chosen >= 0) {
            segments[chosen].cuts.push_back(match.t);
            aliases[entry.first] = match.point;
        }
    }
    std::map<Cell, std::vector<int>> vertices;
    std::vector<Vec2> positions;
    std::vector<int> ids;
    auto vertex = [&](Vec2 p, int preferred = -1) {
        Cell c = cell(p, 1);
        for (int x = c.first - 1; x <= c.first + 1; ++x)
            for (int y = c.second - 1; y <= c.second + 1; ++y) {
                auto it = vertices.find({x, y});
                if (it == vertices.end())
                    continue;
                for (int index : it->second)
                    if (length(positions[index] - p) < .3)
                        return index;
            }
        int index = static_cast<int>(positions.size()), id = preferred >= 0 ? preferred : nextId++;
        vertices[c].push_back(index);
        positions.push_back(p);
        ids.push_back(id);
        airport.nodes[id] = {id, unproject(origin, p),
                             preferred >= 0 ? originalNodes.at(id).name : "Painted centerline"};
        return index;
    };
    for (const auto &alias : aliases) {
        int index = vertex(alias.second, alias.first);
        airport.nodeAliases[alias.first] = ids[index];
    }
    std::unordered_set<int> entryNodes;
    airport.paintedRunwayEntries.clear();
    // Some scenery has painted lineup curves but no ATC runway edges. Accept only
    // actual feature endpoints near the surveyed axis, approached from outside the strip.
    for (const auto &line : airport.groundLines) {
        if (!line.centerline() || line.points.size() < 2)
            continue;
        for (std::size_t runwayIndex = 0; runwayIndex < airport.runways.size(); ++runwayIndex) {
            const auto &runway = airport.runways[runwayIndex];
            Vec2 p = project(origin, runway.ends[0].position), q = project(origin, runway.ends[1].position);
            double runwayLength = length(q - p);
            if (runwayLength < 1 || runway.width <= 0)
                continue;
            Vec2 axis = (q - p) * (1 / runwayLength);
            bool approach = std::any_of(line.points.begin(), line.points.end(), [&](GeoPoint point) {
                Vec2 offset = project(origin, point) - p;
                double along = dot(offset, axis);
                return along >= 0 && along <= runwayLength &&
                       std::abs(cross(offset, axis)) > runway.width / 2 + 5;
            });
            if (!approach)
                continue;
            for (bool last : {false, true}) {
                Vec2 end = project(origin, last ? line.points.back() : line.points.front());
                Vec2 before = project(origin, last ? line.points[line.points.size() - 2] : line.points[1]);
                Vec2 tangent = end - before;
                if (onSegment(end, p, q).distance > 2 || length(tangent) < .01 ||
                    std::abs(dot(tangent * (1 / length(tangent)), axis)) < std::cos(35 * rad))
                    continue;
                int node = ids[vertex(end)];
                bool duplicate = std::any_of(
                    airport.paintedRunwayEntries.begin(), airport.paintedRunwayEntries.end(),
                    [&](const PaintedRunwayEntry &entry) {
                        return entry.node == node && entry.runway == static_cast<int>(runwayIndex);
                    });
                if (!duplicate)
                    airport.paintedRunwayEntries.push_back({node, static_cast<int>(runwayIndex)});
                entryNodes.insert(node);
            }
        }
    }
    std::vector<TaxiEdge> physical;
    std::set<std::pair<int, int>> used;
    for (auto &segment : segments) {
        std::sort(segment.cuts.begin(), segment.cuts.end());
        for (std::size_t k = 1; k < segment.cuts.size(); ++k) {
            Vec2 a = segment.a + (segment.b - segment.a) * segment.cuts[k - 1];
            Vec2 b = segment.a + (segment.b - segment.a) * segment.cuts[k];
            int from = vertex(a), to = vertex(b);
            if (from == to || !used.insert(std::minmax(from, to)).second)
                continue;
            TaxiEdge e{ids[from], ids[to], false, false, 0, "Painted apron", {}, {}, true};
            Vec2 middle = (a + b) * .5;
            int best = -1;
            double bestDistance = 40;
            auto nearby = rules.find(cell(middle, 80));
            if (nearby != rules.end())
                for (int i : nearby->second) {
                    auto p = onSegment(middle, oldFrom[i], oldTo[i]);
                    double angle = std::abs(wrap180(heading(b - a) - heading(oldTo[i] - oldFrom[i])));
                    angle = std::min(angle, 180 - angle);
                    if (p.distance < bestDistance && angle < 35) {
                        bestDistance = p.distance;
                        best = i;
                    }
                    if (p.distance < 35)
                        e.activeRunways.insert(e.activeRunways.end(), atc[i].activeRunways.begin(),
                                               atc[i].activeRunways.end());
                }
            if (segment.runwaySource >= 0)
                best = segment.runwaySource;
            if (best < 0 && !segment.sourceName.empty())
                e.name = segment.sourceName;
            if (best >= 0) {
                e.width = atc[best].width;
                e.name = atc[best].name;
                e.oneWay = atc[best].oneWay;
                e.runway = segment.runwaySource >= 0;
                if (e.oneWay && dot(b - a, oldTo[best] - oldFrom[best]) < 0) {
                    std::swap(e.from, e.to);
                    std::swap(a, b);
                }
            }
            for (const auto &runway : airport.runways) {
                Vec2 p = project(origin, runway.ends[0].position),
                     q = project(origin, runway.ends[1].position);
                auto proximity = onSegment(middle, p, q);
                if (proximity.distance <= runway.width / 2 + 10)
                    e.activeRunways.push_back(runway.ends[0].name + "/" + runway.ends[1].name);
            }
            std::sort(e.activeRunways.begin(), e.activeRunways.end());
            e.activeRunways.erase(std::unique(e.activeRunways.begin(), e.activeRunways.end()),
                                  e.activeRunways.end());
            e.geometry = {airport.nodes.at(e.from).position, airport.nodes.at(e.to).position};
            physical.push_back(std::move(e));
        }
    }
    std::unordered_map<int, std::vector<int>> incident;
    for (std::size_t i = 0; i < physical.size(); ++i) {
        incident[physical[i].from].push_back(static_cast<int>(i));
        incident[physical[i].to].push_back(static_cast<int>(i));
    }
    auto canonical = [&](int id) {
        auto it = airport.nodeAliases.find(id);
        return it == airport.nodeAliases.end() ? id : it->second;
    };
    auto hasPaintedPath = [&](int edge) {
        int from = canonical(atc[edge].from), to = canonical(atc[edge].to);
        if (from == to)
            return true;
        std::queue<int> queue;
        std::unordered_set<int> visited{from};
        queue.push(from);
        while (!queue.empty() && visited.size() < 6000) {
            int id = queue.front();
            queue.pop();
            auto neighbors = incident.find(id);
            if (neighbors == incident.end())
                continue;
            for (int index : neighbors->second) {
                const auto &e = physical[index];
                int next = e.from == id ? e.to : e.from;
                if (e.oneWay && e.from != id)
                    continue;
                if (next == to)
                    return true;
                Vec2 p = project(origin, airport.nodes.at(next).position);
                if (!e.runway && onSegment(p, oldFrom[edge], oldTo[edge]).distance < 45 &&
                    visited.insert(next).second)
                    queue.push(next);
            }
        }
        return false;
    };
    std::vector<TaxiEdge> fallback;
    for (std::size_t i = 0; i < atc.size(); ++i)
        if (!atc[i].runway && !hasPaintedPath(static_cast<int>(i))) {
            TaxiEdge e = atc[i];
            e.from = canonical(e.from);
            e.to = canonical(e.to);
            if (e.from != e.to)
                fallback.push_back(std::move(e));
        }
    airport.atcFallbackEdges = fallback.size();
    // Retain junctions and ATC IDs; collapse dense curve probes into short curved links.
    std::vector<bool> consumed(physical.size());
    auto protectedVertex = [&](int id) {
        return id < firstVirtualId || incident[id].size() != 2 || entryNodes.count(id);
    };
    std::vector<int> order;
    for (std::size_t i = 0; i < physical.size(); ++i)
        if (protectedVertex(physical[i].from) || protectedVertex(physical[i].to))
            order.push_back(static_cast<int>(i));
    for (std::size_t i = 0; i < physical.size(); ++i)
        order.push_back(static_cast<int>(i));
    airport.edges.clear();
    for (int index : order) {
        if (consumed[index])
            continue;
        TaxiEdge e = physical[index];
        consumed[index] = true;
        if (!e.oneWay && !protectedVertex(e.from) && protectedVertex(e.to)) {
            std::swap(e.from, e.to);
            std::reverse(e.geometry.begin(), e.geometry.end());
        }
        double extent = distance(e.geometry.front(), e.geometry.back());
        while (!protectedVertex(e.to) && extent < 35) {
            int candidate = -1;
            for (int n : incident[e.to])
                if (!consumed[n])
                    candidate = n;
            if (candidate < 0)
                break;
            auto next = physical[candidate];
            if (!sameRules(e, next) || (next.oneWay && next.from != e.to))
                break;
            if (next.to == e.to) {
                std::swap(next.from, next.to);
                std::reverse(next.geometry.begin(), next.geometry.end());
            }
            consumed[candidate] = true;
            extent += distance(next.geometry.front(), next.geometry.back());
            e.to = next.to;
            e.geometry.push_back(next.geometry.back());
            if (e.to == e.from)
                break;
        }
        if (e.from != e.to)
            airport.edges.push_back(std::move(e));
    }
    airport.edges.insert(airport.edges.end(), fallback.begin(), fallback.end());
    std::unordered_set<int> retained;
    for (const auto &e : airport.edges) {
        retained.insert(e.from);
        retained.insert(e.to);
    }
    for (auto it = airport.nodes.begin(); it != airport.nodes.end();) {
        if (it->first >= firstVirtualId && !retained.count(it->first))
            it = airport.nodes.erase(it);
        else
            ++it;
    }
}
} // namespace autotaxi
