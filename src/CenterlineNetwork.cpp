#include "CenterlineNetwork.h"
#include "PavementQuery.h"
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
    int standLeadInRamp = -1;
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
           a.activeRunways == b.activeRunways && a.painted == b.painted && a.inferredGap == b.inferredGap &&
           a.standLeadInRamp == b.standLeadInRamp;
}
template <class NodeAt>
void stitchGaps(Airport &airport, GeoPoint origin, std::vector<TaxiEdge> &physical, NodeAt nodeAt) {
    struct End {
        int node, edge;
        Vec2 point, outward;
    };
    struct Join {
        int first, second;
        double length;
        TaxiEdge edge;
    };
    std::unordered_map<int, std::vector<int>> incident;
    for (std::size_t i = 0; i < physical.size(); ++i) {
        incident[physical[i].from].push_back(static_cast<int>(i));
        incident[physical[i].to].push_back(static_cast<int>(i));
    }
    std::vector<End> ends;
    for (const auto &[id, edges] : incident) {
        if (edges.size() != 1 || physical[edges.front()].runway)
            continue;
        const auto &e = physical[edges.front()];
        Vec2 p = project(origin, airport.nodes.at(id).position);
        Vec2 before = project(origin, airport.nodes.at(id == e.from ? e.to : e.from).position);
        Vec2 tangent = p - before;
        if (length(tangent) > .01)
            ends.push_back({id, edges.front(), p, tangent * (1 / length(tangent))});
    }
    std::sort(ends.begin(), ends.end(), [](const End &a, const End &b) { return a.node < b.node; });
    std::map<Cell, std::vector<int>> spatial;
    PavementQuery pavement(airport, origin, true);
    std::vector<Join> joins;
    // Repair only short, forward-facing breaks. Do not merge nearby parallel lanes.
    for (std::size_t i = 0; i < ends.size(); ++i) {
        const auto &a = ends[i];
        cells(a.point, a.point, 20, 20, [&](Cell c) {
            const auto found = spatial.find(c);
            if (found == spatial.end())
                return;
            for (int j : found->second) {
                const auto &b = ends[j];
                Vec2 delta = b.point - a.point;
                double span = length(delta);
                if (span < .3 || span > 20 || a.edge == b.edge ||
                    dot(a.outward, delta * (1 / span)) < std::cos(25 * rad) ||
                    dot(b.outward, delta * (-1 / span)) < std::cos(25 * rad))
                    continue;
                const auto &first = physical[a.edge], &second = physical[b.edge];
                bool forward =
                    (!first.oneWay || first.to == a.node) && (!second.oneWay || second.from == b.node);
                bool reverse =
                    (!first.oneWay || first.from == a.node) && (!second.oneWay || second.to == b.node);
                if (!forward && !reverse)
                    continue;
                TaxiEdge e;
                e.from = a.node;
                e.to = b.node;
                e.oneWay = !forward || !reverse;
                e.width = !first.width    ? second.width
                          : !second.width ? first.width
                                          : std::min(first.width, second.width);
                e.name = first.name;
                e.activeRunways = first.activeRunways;
                e.activeRunways.insert(e.activeRunways.end(), second.activeRunways.begin(),
                                       second.activeRunways.end());
                e.standLeadInRamp = first.standLeadInRamp >= 0 ? first.standLeadInRamp : second.standLeadInRamp;
                std::sort(e.activeRunways.begin(), e.activeRunways.end());
                e.activeRunways.erase(std::unique(e.activeRunways.begin(), e.activeRunways.end()),
                                      e.activeRunways.end());
                e.painted = true;
                e.inferredGap = true;
                Vec2 p = a.point + a.outward * (span / 3), q = b.point + b.outward * (span / 3);
                bool feasible = true;
                double extent = 0;
                Vec2 previous = a.point;
                int count = static_cast<int>(std::ceil(span * 3));
                for (int k = 0; k <= count; ++k) {
                    double t = static_cast<double>(k) / count, u = 1 - t;
                    Vec2 point = a.point * (u * u * u) + p * (3 * u * u * t) + q * (3 * u * t * t) +
                                 b.point * (t * t * t);
                    Vec2 v =
                        (p - a.point) * (3 * u * u) + (q - p) * (6 * u * t) + (b.point - q) * (3 * t * t);
                    Vec2 acceleration = (q - p * 2 + a.point) * (6 * u) + (b.point - q * 2 + p) * (6 * t);
                    double speed = length(v);
                    if (speed < .01 || std::abs(cross(v, acceleration)) / std::pow(speed, 3) > 1 / 35. ||
                        !pavement.contains(unproject(origin, point))) {
                        feasible = false;
                        break;
                    }
                    extent += length(point - previous);
                    previous = point;
                    e.geometry.push_back(unproject(origin, point));
                }
                if (!feasible)
                    continue;
                if (!forward) {
                    std::swap(e.from, e.to);
                    std::reverse(e.geometry.begin(), e.geometry.end());
                }
                joins.push_back({static_cast<int>(i), j, extent, std::move(e)});
            }
        });
        spatial[cell(a.point, 20)].push_back(static_cast<int>(i));
    }
    std::sort(joins.begin(), joins.end(), [](const Join &a, const Join &b) {
        if (a.length != b.length)
            return a.length < b.length;
        return std::make_pair(a.first, a.second) < std::make_pair(b.first, b.second);
    });
    std::unordered_set<int> joined;
    for (auto &join : joins)
        if (!joined.count(join.first) && !joined.count(join.second)) {
            joined.insert(join.first);
            joined.insert(join.second);
            physical.push_back(std::move(join.edge));
            ++airport.centerlineGapLinks;
        }
    spatial.clear();
    auto indexEdge = [&](int index) {
        const auto &e = physical[index];
        if (!e.runway && !e.inferredGap)
            cells(project(origin, e.geometry.front()), project(origin, e.geometry.back()), .3, 20,
                  [&](Cell c) { spatial[c].push_back(index); });
    };
    for (std::size_t i = 0; i < physical.size(); ++i)
        indexEdge(static_cast<int>(i));
    // A feature can stop just before a shallow merge. Extend its observed tangent
    // to actual paint; the planner still checks the aircraft's turn at that junction.
    for (std::size_t i = 0; i < ends.size(); ++i) {
        if (joined.count(static_cast<int>(i)))
            continue;
        auto &end = ends[i];
        double best = 20;
        int target = -1;
        Vec2 hit;
        bool forward = false, reverse = false;
        std::set<int> nearby;
        cells(end.point, end.point + end.outward * 20, .3, 20, [&](Cell c) {
            auto found = spatial.find(c);
            if (found != spatial.end())
                nearby.insert(found->second.begin(), found->second.end());
        });
        const auto source = physical[end.edge];
        for (int index : nearby) {
            const auto &e = physical[index];
            if (e.from == end.node || e.to == end.node || e.runway || e.inferredGap)
                continue;
            Vec2 a = project(origin, e.geometry.front()), b = project(origin, e.geometry.back());
            Vec2 delta = b - a;
            const double span = length(delta), determinant = cross(end.outward, delta);
            if (span < .01 || std::abs(determinant) < 1e-9 ||
                std::abs(dot(end.outward, delta * (1 / span))) < std::cos(35 * rad))
                continue;
            double advance = cross(a - end.point, delta) / determinant;
            double along = cross(a - end.point, end.outward) / determinant;
            if (advance < .3 || advance >= best || along < 0 || along > 1)
                continue;
            bool sameDirection = dot(end.outward, delta) > 0;
            bool canForward = (!source.oneWay || source.to == end.node) && (!e.oneWay || sameDirection);
            bool canReverse = (!source.oneWay || source.from == end.node) && (!e.oneWay || !sameDirection);
            if (!canForward && !canReverse)
                continue;
            Vec2 candidate = end.point + end.outward * advance;
            bool paved = true;
            const int count = static_cast<int>(std::ceil(advance * 2));
            for (int k = 0; k <= count; ++k)
                if (!pavement.contains(unproject(origin, end.point + end.outward * (advance * k / count)))) {
                    paved = false;
                    break;
                }
            if (!paved)
                continue;
            best = advance;
            target = index;
            hit = candidate;
            forward = canForward;
            reverse = canReverse;
        }
        if (target < 0)
            continue;
        const auto original = physical[target];
        int junction = nodeAt(hit);
        if (junction == end.node)
            continue;
        if (junction != original.from && junction != original.to) {
            auto second = original;
            second.from = junction;
            second.geometry.front() = airport.nodes.at(junction).position;
            physical[target].to = junction;
            physical[target].geometry.back() = airport.nodes.at(junction).position;
            const int newIndex = static_cast<int>(physical.size());
            physical.push_back(std::move(second));
            for (auto &other : ends)
                if (other.edge == target && other.node == original.to)
                    other.edge = newIndex;
            indexEdge(newIndex);
        }
        TaxiEdge e;
        e.from = end.node;
        e.to = junction;
        e.oneWay = !forward || !reverse;
        e.width = !source.width     ? original.width
                  : !original.width ? source.width
                                    : std::min(source.width, original.width);
        e.name = source.name;
        e.activeRunways = source.activeRunways;
        e.activeRunways.insert(e.activeRunways.end(), original.activeRunways.begin(),
                               original.activeRunways.end());
        std::sort(e.activeRunways.begin(), e.activeRunways.end());
        e.activeRunways.erase(std::unique(e.activeRunways.begin(), e.activeRunways.end()),
                              e.activeRunways.end());
        e.geometry = {airport.nodes.at(e.from).position, airport.nodes.at(e.to).position};
        e.painted = e.inferredGap = true;
        if (!forward) {
            std::swap(e.from, e.to);
            std::reverse(e.geometry.begin(), e.geometry.end());
        }
        physical.push_back(std::move(e));
        joined.insert(static_cast<int>(i));
        for (std::size_t j = 0; j < ends.size(); ++j)
            if (ends[j].node == junction)
                joined.insert(static_cast<int>(j));
        ++airport.centerlineGapLinks;
    }
}
struct EntryCurve {
    int runway;
    std::vector<Vec2> points;
};
std::vector<EntryCurve> runwayContinuations(const Airport &airport, GeoPoint origin) {
    std::vector<EntryCurve> entries;
    for (const auto &line : airport.groundLines) {
        if (!line.centerline() || line.points.size() < 3)
            continue;
        for (std::size_t index = 0; index < airport.runways.size(); ++index) {
            const auto &runway = airport.runways[index];
            Vec2 threshold = project(origin, runway.ends[0].position);
            Vec2 delta = project(origin, runway.ends[1].position) - threshold;
            double extent = length(delta);
            if (extent < 600 || runway.width < 10)
                continue;
            Vec2 axis = delta * (1 / extent);
            bool approach = std::any_of(line.points.begin(), line.points.end(), [&](GeoPoint point) {
                Vec2 p = project(origin, point) - threshold;
                return dot(p, axis) >= 0 && dot(p, axis) <= extent &&
                       std::abs(cross(p, axis)) > runway.width / 2 + 5;
            });
            if (!approach)
                continue;
            for (bool last : {false, true}) {
                Vec2 end = project(origin, last ? line.points.back() : line.points.front());
                Vec2 before = project(origin, last ? line.points[line.points.size() - 2] : line.points[1]);
                Vec2 tangent = end - before;
                if (length(tangent) < .01)
                    continue;
                tangent = tangent * (1 / length(tangent));
                double along = dot(end - threshold, axis), gap = std::abs(cross(end - threshold, axis));
                if (along < 0 || along > extent || gap > std::min(15., runway.width / 2 - 3) ||
                    std::abs(dot(tangent, axis)) < std::cos(60 * rad))
                    continue;
                Vec2 forward = axis * (dot(tangent, axis) >= 0 ? 1 : -1);
                // Preserve every observed paint point, then continue its tangent inside the runway.
                for (double advance : {40., 60., 90., 120.}) {
                    Vec2 target = threshold + axis * along + forward * advance;
                    double targetAlong = dot(target - threshold, axis);
                    if (targetAlong < 0 || targetAlong > extent)
                        continue;
                    double handle = advance * .4;
                    Vec2 first = end + tangent * handle, second = target - forward * handle;
                    std::vector<Vec2> shape{end};
                    bool feasible = true;
                    int count = static_cast<int>(std::ceil(advance / 2));
                    for (int i = 1; i <= count; ++i) {
                        double t = static_cast<double>(i) / count, u = 1 - t;
                        Vec2 p = end * (u * u * u) + first * (3 * u * u * t) + second * (3 * u * t * t) +
                                 target * (t * t * t);
                        Vec2 v = (first - end) * (3 * u * u) + (second - first) * (6 * u * t) +
                                 (target - second) * (3 * t * t);
                        Vec2 a =
                            (second - first * 2 + end) * (6 * u) + (target - second * 2 + first) * (6 * t);
                        double speed = length(v), curvature = std::abs(cross(v, a)) / std::pow(speed, 3);
                        double at = dot(p - threshold, axis);
                        if (speed < .01 || curvature > 1 / 35. || at < 0 || at > extent ||
                            std::abs(cross(p - threshold, axis)) > runway.width / 2 - 1) {
                            feasible = false;
                            break;
                        }
                        shape.push_back(p);
                    }
                    if (feasible) {
                        entries.push_back({static_cast<int>(index), std::move(shape)});
                        break;
                    }
                }
            }
        }
    }
    return entries;
}
} // namespace
void buildCenterlineNetwork(Airport &airport) {
    // Stand lead-ins in the supported scenery are painted centerlines (usually
    // style 51). White/custom linear features are roadway or service markings;
    // they remain display-only because their geometry does not identify an
    // aircraft guidance line reliably.
    for (auto &line : airport.groundLines) {
        if (line.points.size() < 2 || !line.paintedCenterline())
            continue;
        // Rebuilding the airport after a scenery refresh must not retain an
        // earlier, more permissive stand association.
        line.standLeadIn = false;
        line.standLeadInRamp = -1;
        constexpr double standLeadInDistance = 45.0;
        constexpr double standLeadInEndpointDistance = 40.0;
        constexpr double standLeadInStandJoinDistance = 8.0;
        // Commercial apt.dat files sometimes leave a long gap between the
        // stand spur and the painted taxiway. Keep the stand end terminal,
        // but allow the opposite end to join across that scenery gap.
        constexpr double standLeadInNetworkJoinDistance = 60.0;
        auto joinDistance = [&](const GeoPoint &endpoint) {
            double best = 1e9;
            for (const auto &other : airport.groundLines) {
                if (&other == &line || !other.paintedCenterline())
                    continue;
                for (std::size_t i = 1; i < other.points.size(); ++i) {
                    const Vec2 a = project(endpoint, other.points[i - 1]);
                    const Vec2 b = project(endpoint, other.points[i]);
                    if (length(b - a) > .01)
                        best = std::min(best, onSegment({}, a, b).distance);
                }
            }
            return best;
        };
        const double firstJoin = joinDistance(line.points.front());
        const double lastJoin = joinDistance(line.points.back());
        bool matched = false;
        int matchedRamp = -1;
        double matchedGap = standLeadInDistance;
        for (std::size_t rampIndex = 0; rampIndex < airport.ramps.size(); ++rampIndex) {
            const auto &ramp = airport.ramps[rampIndex];
            const double endpointGap = std::min(distance(ramp.position, line.points.front()),
                                                distance(ramp.position, line.points.back()));
            if (endpointGap > standLeadInEndpointDistance)
                continue;
            const bool firstAtStand = distance(ramp.position, line.points.front()) <=
                                      distance(ramp.position, line.points.back());
            // A real lead-in is a terminal spur: its stand end must not also
            // be a taxiway junction, while its opposite end must meet the
            // painted taxiway network. This prevents a main taxiway that merely
            // passes a stand from being promoted as the stand lead-in.
            const double standJoin = firstAtStand ? firstJoin : lastJoin;
            const double networkJoin = firstAtStand ? lastJoin : firstJoin;
            if (standJoin < standLeadInStandJoinDistance || networkJoin > standLeadInNetworkJoinDistance)
                continue;
            for (std::size_t i = 1; i < line.points.size(); ++i) {
                const Vec2 a = project(ramp.position, line.points[i - 1]);
                const Vec2 b = project(ramp.position, line.points[i]);
                if (length(b - a) < .01)
                    continue;
                const auto snap = onSegment({}, a, b);
                if (snap.distance > standLeadInDistance)
                    continue;
                const double lineHeading = heading(b - a);
                double turn = std::abs(wrap180(lineHeading - ramp.heading));
                turn = std::min(turn, 180.0 - turn);
                if (turn <= 35.0 && (matchedRamp < 0 || snap.distance < matchedGap)) {
                    matchedGap = snap.distance;
                    matched = true;
                    matchedRamp = static_cast<int>(rampIndex);
                }
            }
        }
        line.standLeadIn = matched;
        line.standLeadInRamp = matchedRamp;
    }
    airport.centerlineGapLinks = 0;
    airport.atcFallbackEdges = static_cast<std::size_t>(std::count_if(
        airport.edges.begin(), airport.edges.end(), [](const TaxiEdge &edge) { return !edge.runway; }));
    if (airport.nodes.empty() || airport.edges.empty() ||
        std::none_of(airport.groundLines.begin(), airport.groundLines.end(),
                     [](const GroundLine &line) { return line.centerline(); }))
        return;
    GeoPoint origin = airport.nodes.begin()->second.position;
    const auto atc = airport.edges;
    const auto originalNodes = airport.nodes;
    const auto continuations = runwayContinuations(airport, origin);
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
                                            {0, 1},
                                            line.standLeadInRamp});
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
    for (const auto &entry : continuations) {
        int node = ids[vertex(entry.points.front())];
        PaintedRunwayEntry candidate{node, entry.runway, {}};
        for (auto point : entry.points)
            candidate.continuation.push_back(unproject(origin, point));
        airport.paintedRunwayEntries.push_back(std::move(candidate));
        entryNodes.insert(node);
    }
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
                if (std::any_of(continuations.begin(), continuations.end(), [&](const EntryCurve &entry) {
                        return entry.runway == static_cast<int>(runwayIndex) &&
                               length(entry.points.front() - end) < .3;
                    }))
                    continue;
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
                    airport.paintedRunwayEntries.push_back({node, static_cast<int>(runwayIndex), {}});
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
            e.standLeadInRamp = segment.standLeadInRamp;
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
    stitchGaps(airport, origin, physical, [&](Vec2 p) { return ids[vertex(p)]; });
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
