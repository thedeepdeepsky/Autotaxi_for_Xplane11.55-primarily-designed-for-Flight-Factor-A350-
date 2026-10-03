#include "RoutePlanner.h"
#include "PavementQuery.h"
#include "TurnGuidance.h"
#include <algorithm>
#include <cctype>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
namespace autotaxi {
namespace {
std::string upper(std::string value) {
    for (char &c : value)
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return value;
}
bool namedRunway(std::string names, const std::string &wanted) {
    for (char &c : names)
        if (c == '/' || c == ',' || c == '-')
            c = ' ';
    std::istringstream in(names);
    for (std::string n; in >> n;)
        if (n == wanted)
            return true;
    return false;
}
bool allowed(const TaxiEdge &e, const RouteOptions &o) {
    if ((e.runway || !e.activeRunways.empty()) && !o.runwayClearance)
        return false;
    return e.runway || o.ignorePavementLimits || (e.width ? e.width >= o.minimumWidth : o.allowUnknownWidth);
}
bool insideRing(GeoPoint p, const std::vector<GeoPoint> &ring) {
    if (ring.size() < 3)
        return false;
    bool inside = false;
    Vec2 previous = project(p, ring.back());
    for (auto point : ring) {
        Vec2 current = project(p, point);
        if (onSegment({}, previous, current).distance < 0.5)
            return true;
        if ((current.y > 0) != (previous.y > 0) &&
            0 < (previous.x - current.x) * (-current.y) / (previous.y - current.y) + current.x)
            inside = !inside;
        previous = current;
    }
    return inside;
}
bool onPavement(const Airport &a, GeoPoint p) {
    for (const auto &surface : a.pavements)
        if (!surface.rings.empty() && insideRing(p, surface.rings[0])) {
            bool hole = false;
            for (std::size_t i = 1; i < surface.rings.size(); ++i)
                if (insideRing(p, surface.rings[i])) {
                    hole = true;
                    break;
                }
            if (!hole)
                return true;
        }
    return false;
}
struct Link {
    int from, to, edge;
    Vec2 a, b;
    double length, cost;
    std::vector<Vec2> shape;
    bool painted;
};
struct Start {
    int link;
    Vec2 snap;
    double cost, proximity;
    bool pushback, apron;
    std::vector<Vec2> connector;
    double progress;
};
struct ShapeSnap {
    Vec2 point, tangent;
    double distance = std::numeric_limits<double>::infinity(), progress = 0;
};
ShapeSnap snapShape(const std::vector<Vec2> &shape, Vec2 position) {
    ShapeSnap best;
    double along = 0;
    for (std::size_t i = 1; i < shape.size(); ++i) {
        Vec2 delta = shape[i] - shape[i - 1];
        double gap = length(delta);
        auto candidate = onSegment(position, shape[i - 1], shape[i]);
        if (gap > .01 && candidate.distance < best.distance)
            best = {candidate.point, delta * (1 / gap), candidate.distance, along + gap * candidate.t};
        along += gap;
    }
    return best;
}
ShapeSnap alongShape(const std::vector<Vec2> &shape, double progress) {
    double along = 0;
    for (std::size_t i = 1; i < shape.size(); ++i) {
        Vec2 delta = shape[i] - shape[i - 1];
        double gap = length(delta);
        if (gap > .01 && (along + gap >= progress || i + 1 == shape.size()))
            return {shape[i - 1] + delta * std::clamp((progress - along) / gap, 0., 1.), delta * (1 / gap), 0,
                    progress};
        along += gap;
    }
    return {};
}
std::vector<Vec2> sliceShape(const std::vector<Vec2> &shape, double from, double to) {
    std::vector<Vec2> result{alongShape(shape, from).point};
    double along = 0;
    for (std::size_t i = 1; i < shape.size(); ++i) {
        along += length(shape[i] - shape[i - 1]);
        if (along > from + .01 && along < to - .01)
            result.push_back(shape[i]);
    }
    Vec2 end = alongShape(shape, to).point;
    if (length(end - result.back()) > .01)
        result.push_back(end);
    return result;
}

double turningCost(const std::vector<Vec2> &shape) {
    std::vector<Vec2> sampled{shape.front()};
    for (std::size_t i = 1; i < shape.size(); ++i)
        if (length(shape[i] - sampled.back()) >= 3 || i + 1 == shape.size())
            sampled.push_back(shape[i]);
    double angle = 0;
    for (std::size_t i = 1; i + 1 < sampled.size(); ++i)
        angle +=
            std::abs(wrap180(heading(sampled[i + 1] - sampled[i]) - heading(sampled[i] - sampled[i - 1])));
    return angle * .6;
}
double departureHeading(const Link &link) {
    return heading(alongShape(link.shape, std::min(3., link.length)).point - link.shape.front());
}
double arrivalHeading(const Link &link) {
    return heading(link.shape.back() - alongShape(link.shape, std::max(0., link.length - 3)).point);
}
std::vector<Vec2> apronCurve(const PavementQuery &pavement, GeoPoint origin, Vec2 end, double yaw,
                             Vec2 finalDirection, double minimumRadius, const RouteOptions &options,
                             bool cockpitReference = false) {
    if (cockpitReference)
        minimumRadius =
            std::hypot(options.wheelbase + options.cockpitAheadNose,
                       std::max(minimumRadius, options.wheelbase / std::tan(options.maxSteer * rad)));
    double gap = length(end);
    for (double fraction : {0.3, 0.45, 0.65, 0.9}) {
        double handle = std::min(80.0, gap * fraction);
        Vec2 p1 = direction(yaw) * handle, p2 = end - finalDirection * handle;
        int samples = std::max(20, static_cast<int>(std::ceil((gap + 2 * handle) / 2)));
        std::vector<Vec2> points{{}};
        bool feasible = true;
        for (int i = 1; i <= samples; ++i) {
            checkPlanning(options);
            double t = static_cast<double>(i) / samples, u = 1 - t;
            Vec2 point = p1 * (3 * u * u * t) + p2 * (3 * u * t * t) + end * (t * t * t);
            Vec2 tangent = p1 * (3 * u * u) + (p2 - p1) * (6 * u * t) + (end - p2) * (3 * t * t);
            Vec2 second = (p2 - p1 * 2) * (6 * u) + (end - p2 * 2 + p1) * (6 * t);
            double speed = length(tangent);
            double curvature =
                speed > 0.01 ? std::abs(tangent.x * second.y - tangent.y * second.x) / (speed * speed * speed)
                             : 1;
            if (curvature > 1 / minimumRadius ||
                !pavement.connection(unproject(origin, points.back()), unproject(origin, point))) {
                feasible = false;
                break;
            }
            points.push_back(point);
        }
        if (feasible)
            return points;
    }
    return {};
}
} // namespace
std::vector<std::string> parseRouteVia(std::string text) {
    for (char &c : text)
        if (c == ',' || c == ';' || c == '>')
            c = ' ';
    std::istringstream input(text);
    std::vector<std::string> result;
    for (std::string token; input >> token;) {
        if (std::all_of(token.begin(), token.end(), [](unsigned char c) { return std::isdigit(c); }))
            token = "#" + token;
        result.push_back(upper(token));
        if (result.size() > 32)
            throw std::runtime_error("Custom route supports at most 32 ordered taxiways/nodes");
    }
    return result;
}
std::string rampLabel(const Ramp &r) {
    std::string label = r.name;
    for (const auto &alias : r.aliases)
        label += " / " + alias;
    return label;
}
std::string departureLabel(const Airport &a, GeoPoint p) {
    const Ramp *best = nullptr;
    double gap = 100;
    for (const auto &ramp : a.ramps) {
        double d = distance(p, ramp.position);
        if (d < gap) {
            gap = d;
            best = &ramp;
        }
    }
    if (best)
        return rampLabel(*best);
    int node = nearestNode(a, p);
    return node >= 0 ? "Current position | Node " + std::to_string(node) : "Current position";
}
bool pavedConnection(const Airport &a, GeoPoint from, GeoPoint to) {
    Vec2 delta = project(from, to);
    int samples = std::max(1, static_cast<int>(std::ceil(length(delta) / 4)));
    for (int i = 0; i <= samples; ++i)
        if (!onPavement(a, unproject(from, delta * (static_cast<double>(i) / samples))))
            return false;
    return true;
}
std::vector<Destination> destinations(const Airport &a) {
    std::vector<Destination> out;
    // Keep hold-short points beside their runway in the Runways tab.  This
    // makes every selectable hold position visible without forcing the user
    // to scroll past the complete runway list first.
    for (std::size_t i = 0; i < a.runways.size(); ++i) {
        for (int end = 0; end < 2; ++end)
            out.push_back(
                {DestinationKind::Runway, "RWY " + a.runways[i].ends[end].name, static_cast<int>(i), end});
        for (std::size_t hold = 0; hold < a.holdShortPoints.size(); ++hold)
            if (a.holdShortPoints[hold].runway == static_cast<int>(i))
                out.push_back({DestinationKind::HoldShort, a.holdShortPoints[hold].label,
                               static_cast<int>(hold), a.holdShortPoints[hold].end});
    }
    // Preserve malformed/legacy hold records that have no runway association.
    for (std::size_t i = 0; i < a.holdShortPoints.size(); ++i)
        if (a.holdShortPoints[i].runway < 0)
            out.push_back({DestinationKind::HoldShort, a.holdShortPoints[i].label, static_cast<int>(i),
                           a.holdShortPoints[i].end});
    for (std::size_t i = 0; i < a.ramps.size(); ++i)
        out.push_back({DestinationKind::Ramp, rampLabel(a.ramps[i]), static_cast<int>(i), 0});
    std::vector<int> ids;
    for (const auto &n : a.nodes)
        ids.push_back(n.first);
    std::sort(ids.begin(), ids.end());
    for (int id : ids)
        out.push_back(
            {DestinationKind::Node, "Node " + std::to_string(id) + " " + a.nodes.at(id).name, id, 0});
    return out;
}
Route planRoute(const Airport &a, GeoPoint position, double trueHeading, const Destination &d,
                const RouteOptions &o) {
    checkPlanning(o);
    if (!valid(position) || !std::isfinite(trueHeading))
        throw std::runtime_error("Invalid aircraft position or heading");
    if (o.via.size() > 32)
        throw std::runtime_error("Too many custom route waypoints");
    if (d.kind == DestinationKind::Ramp) {
        const auto &ramp = a.ramps.at(d.index);
        if (!o.ignoreStandSize && ramp.width && ramp.width < o.minimumWidth)
            throw std::runtime_error("Stand wingspan class too small for aircraft");
        if (!o.ignoreStandSize && !ramp.width && !o.allowUnknownWidth)
            throw std::runtime_error("Stand wingspan class is unknown");
    }
    if (a.nodes.empty() || a.edges.empty())
        throw std::runtime_error("Airport has no 1201/1202 taxi network");
    if (d.kind == DestinationKind::Runway && !o.runwayClearance)
        throw std::runtime_error("Runway clearance required for lineup and crossings");
    Route r;
    r.origin = position;
    r.initialHeading = trueHeading;
    r.label = d.label;
    r.runway = d.kind == DestinationKind::Runway;
    r.ramp = d.kind == DestinationKind::Ramp;
    if (r.ramp) {
        const auto &ramp = a.ramps.at(d.index);
        r.destinationRisk = ramp.width ? ramp.width < o.minimumWidth : !o.allowUnknownWidth;
        r.finalHeading = ramp.heading;
    }
    r.departure = departureLabel(a, position);
    std::string runway;
    Vec2 threshold, axis;
    double runwayLength = 0, runwayWidth = 0;
    if (r.runway) {
        const auto &rw = a.runways.at(d.index);
        if (d.end < 0 || d.end > 1)
            throw std::runtime_error("Invalid runway end");
        runway = rw.ends[d.end].name;
        threshold = project(position, rw.ends[d.end].position);
        Vec2 far = project(position, rw.ends[1 - d.end].position);
        runwayLength = length(far - threshold);
        if (runwayLength < 600)
            throw std::runtime_error("Runway too short for A350 lineup");
        axis = (far - threshold) * (1 / runwayLength);
        r.finalHeading = heading(axis);
        runwayWidth = rw.width;
    }
    std::vector<Link> links;
    std::vector<TaxiEdge> exitRunwayEdges;
    auto edgeAt = [&](int index) -> const TaxiEdge & {
        return index < static_cast<int>(a.edges.size()) ? a.edges[index]
                                                        : exitRunwayEdges.at(index - a.edges.size());
    };
    bool onRunway = false;
    for (const auto &rw : a.runways) {
        Vec2 a0 = project(position, rw.ends[0].position), b0 = project(position, rw.ends[1].position);
        auto snap = onSegment({}, a0, b0);
        if (dot(a0 * -1, b0 - a0) >= 0 && dot(b0 * -1, a0 - b0) >= 0 && snap.distance < rw.width / 2)
            onRunway = true;
    }
    std::unordered_map<int, std::vector<int>> outgoing;
    for (std::size_t i = 0; i < a.edges.size(); ++i) {
        checkPlanning(o);
        const auto &e = a.edges[i];
        if (!allowed(e, o))
            continue;
        Vec2 from = project(position, a.nodes.at(e.from).position),
             to = project(position, a.nodes.at(e.to).position);
        std::vector<Vec2> shape;
        if (e.geometry.size() >= 2)
            for (auto p : e.geometry)
                shape.push_back(project(position, p));
        else
            shape = {from, to};
        shape.front() = from;
        shape.back() = to;
        double len = 0;
        for (std::size_t k = 1; k < shape.size(); ++k)
            len += length(shape[k] - shape[k - 1]);
        bool feasible = true;
        if (e.painted)
            for (std::size_t k = 1; k + 1 < shape.size(); ++k) {
                // Junction coordinates are welded within 0.3 m; sub-metre probes
                // can therefore invent a tight bend on an otherwise straight line.
                std::size_t before = k - 1, after = k + 1;
                while (before > 0 && length(shape[k] - shape[before]) < 5)
                    --before;
                while (after + 1 < shape.size() && length(shape[after] - shape[k]) < 5)
                    ++after;
                Vec2 u = shape[k] - shape[before], v = shape[after] - shape[k];
                double area = std::abs(u.x * v.y - u.y * v.x);
                if (length(u) > .5 && length(v) > .5 && area > 1e-5 &&
                    length(u) * length(v) * length(u + v) / (2 * area) < o.minimumTurnRadius * .9)
                    feasible = false;
            }
        if (!feasible)
            continue;
        if (len < 0.05)
            continue;
        double cost = len * (e.runway ? (namedRunway(e.name, runway) ? 1.6 : 5) : 1) + turningCost(shape);
        auto add = [&](int f, int t, Vec2 x, Vec2 y, const std::vector<Vec2> &geometry) {
            outgoing[f].push_back(static_cast<int>(links.size()));
            links.push_back({f, t, static_cast<int>(i), x, y, len, cost, geometry, e.painted});
        };
        add(e.from, e.to, from, to, shape);
        if (!e.oneWay) {
            std::reverse(shape.begin(), shape.end());
            add(e.to, e.from, to, from, shape);
        }
    }
    // Exit-only connections continue the real paint tangent back to the surveyed runway axis.
    if (onRunway) {
        int virtualNode = -2;
        for (std::size_t runwayIndex = 0; runwayIndex < a.runways.size(); ++runwayIndex) {
            const auto &rw = a.runways[runwayIndex];
            const Vec2 begin = project(position, rw.ends[0].position),
                       end = project(position, rw.ends[1].position);
            auto snap = onSegment({}, begin, end);
            if (dot(begin * -1, end - begin) < 0 || dot(end * -1, begin - end) < 0 ||
                snap.distance >= rw.width / 2 || !o.runwayClearance)
                continue;
            bool hasAxis = std::any_of(a.edges.begin(), a.edges.end(), [&](const TaxiEdge &edge) {
                return edge.runway &&
                       (namedRunway(edge.name, rw.ends[0].name) || namedRunway(edge.name, rw.ends[1].name));
            });
            if (hasAxis) {
                // ATC runway nodes often omit the first/last hundred metres.
                // Extend only those terminal gaps along row 100's surveyed axis,
                // retaining the permitted directions at the existing boundary.
                const double span = length(end - begin);
                if (span < 1)
                    continue;
                const Vec2 axis = (end - begin) * (1 / span);
                for (bool farEnd : {false, true}) {
                    int boundary = -1;
                    double boundAlong = farEnd ? -1e30 : 1e30;
                    Vec2 bound;
                    for (const auto &link : links) {
                        const auto &edge = edgeAt(link.edge);
                        if (!edge.runway || (!namedRunway(edge.name, rw.ends[0].name) &&
                                             !namedRunway(edge.name, rw.ends[1].name)))
                            continue;
                        for (const auto &[id, point] : {std::pair<int, Vec2>{link.from, link.a},
                                                       std::pair<int, Vec2>{link.to, link.b}}) {
                            const double along = dot(point - begin, axis);
                            if (id < 0 || along < 0 || along > span ||
                                length(point - (begin + axis * along)) > .5)
                                continue;
                            if ((farEnd && along > boundAlong) || (!farEnd && along < boundAlong)) {
                                boundary = id;
                                boundAlong = along;
                                bound = point;
                            }
                        }
                    }
                    const Vec2 terminal = farEnd ? end : begin;
                    const double gap = length(bound - terminal);
                    if (boundary < 0 || gap <= .5)
                        continue;
                    const Vec2 inward = axis * (farEnd ? -1 : 1);
                    bool enter = false, leave = false;
                    for (const auto &link : links) {
                        const auto &edge = edgeAt(link.edge);
                        if (!edge.runway || (!namedRunway(edge.name, rw.ends[0].name) &&
                                             !namedRunway(edge.name, rw.ends[1].name)))
                            continue;
                        if (link.from == boundary &&
                            dot(direction(departureHeading(link)), inward) > std::cos(15 * rad))
                            enter = true;
                        if (link.to == boundary &&
                            dot(direction(arrivalHeading(link)), inward) < -std::cos(15 * rad))
                            leave = true;
                    }
                    if (!enter && !leave)
                        continue;
                    TaxiEdge extension;
                    extension.runway = true;
                    extension.oneWay = !enter || !leave;
                    extension.name = rw.ends[0].name + "/" + rw.ends[1].name;
                    const int edge = static_cast<int>(a.edges.size() + exitRunwayEdges.size());
                    exitRunwayEdges.push_back(std::move(extension));
                    const int terminalNode = virtualNode--;
                    if (enter) {
                        outgoing[terminalNode].push_back(static_cast<int>(links.size()));
                        links.push_back({terminalNode, boundary, edge, terminal, bound, gap, gap * 5,
                                         {terminal, bound}, false});
                    }
                    if (leave) {
                        outgoing[boundary].push_back(static_cast<int>(links.size()));
                        links.push_back({boundary, terminalNode, edge, bound, terminal, gap, gap * 5,
                                         {bound, terminal}, false});
                    }
                }
                continue;
            }
            TaxiEdge axisEdge;
            axisEdge.runway = true;
            axisEdge.name = rw.ends[0].name + "/" + rw.ends[1].name;
            const int edge = static_cast<int>(a.edges.size() + exitRunwayEdges.size());
            exitRunwayEdges.push_back(std::move(axisEdge));
            const int first = virtualNode--, last = virtualNode--;
            const double span = length(end - begin);
            outgoing[first].push_back(static_cast<int>(links.size()));
            links.push_back({first, last, edge, begin, end, span, span * 5, {begin, end}, false});
            outgoing[last].push_back(static_cast<int>(links.size()));
            links.push_back({last, first, edge, end, begin, span, span * 5, {end, begin}, false});
        }
        for (const auto &entry : a.paintedRunwayEntries) {
            int source = -1;
            for (std::size_t i = 0; i < a.edges.size(); ++i)
                if (a.edges[i].painted && !a.edges[i].runway &&
                    (a.edges[i].from == entry.node || a.edges[i].to == entry.node) &&
                    allowed(a.edges[i], o)) {
                    source = static_cast<int>(i);
                    break;
                }
            if (source < 0)
                continue;
            Vec2 join = project(position, entry.continuation.empty() ? a.nodes.at(entry.node).position
                                                                     : entry.continuation.back());
            const auto &rw = a.runways.at(entry.runway);
            int merge = entry.continuation.empty() ? entry.node : virtualNode--;
            bool connected = false;
            const std::size_t count = links.size();
            for (std::size_t i = 0; i < count; ++i) {
                auto original = links[i];
                const auto &edge = edgeAt(original.edge);
                if (!edge.runway ||
                    (!namedRunway(edge.name, rw.ends[0].name) && !namedRunway(edge.name, rw.ends[1].name)))
                    continue;
                auto snap = onSegment(join, original.a, original.b);
                if (snap.distance > .5)
                    continue;
                if (snap.t < .01 || snap.t > .99) {
                    int endpoint = snap.t < .01 ? original.from : original.to;
                    // Alias both directions to a common merge when this is an existing junction.
                    merge = endpoint;
                    connected = true;
                    break;
                }
            }
            for (std::size_t i = 0; i < count; ++i) {
                auto original = links[i];
                const auto &edge = edgeAt(original.edge);
                if (!edge.runway ||
                    (!namedRunway(edge.name, rw.ends[0].name) && !namedRunway(edge.name, rw.ends[1].name)))
                    continue;
                auto snap = onSegment(join, original.a, original.b);
                if (snap.distance > .5 || snap.t <= .01 || snap.t >= .99)
                    continue;
                const double factor = original.cost / original.length;
                links[i].to = merge;
                links[i].b = join;
                links[i].shape = {original.a, join};
                links[i].length = length(join - original.a);
                links[i].cost = links[i].length * factor;
                Link tail{merge,
                          original.to,
                          original.edge,
                          join,
                          original.b,
                          length(original.b - join),
                          length(original.b - join) * factor,
                          {join, original.b},
                          original.painted};
                outgoing[merge].push_back(static_cast<int>(links.size()));
                links.push_back(std::move(tail));
                connected = true;
            }
            if (!connected)
                continue;
            if (entry.continuation.empty()) {
                if (merge != entry.node) {
                    // An axis endpoint and painted endpoint can share a position but different node IDs.
                    for (auto &link : links)
                        if (link.from == entry.node) {
                            link.from = merge;
                            outgoing[merge].push_back(static_cast<int>(&link - links.data()));
                        }
                }
                continue;
            }
            std::vector<Vec2> curve;
            for (auto p = entry.continuation.rbegin(); p != entry.continuation.rend(); ++p)
                curve.push_back(project(position, *p));
            double extent = 0;
            for (std::size_t i = 1; i < curve.size(); ++i)
                extent += length(curve[i] - curve[i - 1]);
            outgoing[merge].push_back(static_cast<int>(links.size()));
            links.push_back(
                {merge, entry.node, source, curve.front(), curve.back(), extent, extent, curve, true});
        }
    }
    // An isolated stand spur may end just short of the apron line. Infer only the
    // missing entry turn, preserving the real lead-in and respecting directed edges.
    if (r.ramp) {
        const auto &ramp = a.ramps.at(d.index);
        Vec2 stop = project(position, ramp.position) - direction(ramp.heading) * o.mainAxleAft;
        const std::size_t originalCount = links.size();
        std::unordered_map<int, std::vector<int>> incoming;
        for (std::size_t i = 0; i < originalCount; ++i)
            if (links[i].painted && !edgeAt(links[i].edge).runway)
                incoming[links[i].to].push_back(static_cast<int>(i));
        std::set<int> roots;
        for (std::size_t i = 0; i < originalCount; ++i) {
            const auto &link = links[i];
            if (!link.painted || edgeAt(link.edge).runway)
                continue;
            auto snap = snapShape(link.shape, stop);
            if (snap.distance > 35 || std::abs(wrap180(heading(snap.tangent) - ramp.heading)) > 30)
                continue;
            int rootLink = static_cast<int>(i);
            std::set<int> visited;
            double walked = 0;
            while (visited.insert(rootLink).second && walked < 180) {
                const auto &current = links[rootLink];
                int predecessor = -1;
                for (int candidate : incoming[current.from]) {
                    if (links[candidate].from == current.to)
                        continue;
                    if (predecessor >= 0) {
                        predecessor = -2;
                        break;
                    }
                    predecessor = candidate;
                }
                if (predecessor == -1) {
                    roots.insert(rootLink);
                    break;
                }
                if (predecessor < 0)
                    break;
                walked += current.length;
                rootLink = predecessor;
            }
        }
        PavementQuery standPavement(a, position);
        int virtualNode = -1000000;
        auto addLink = [&](int from, int to, int edge, std::vector<Vec2> shape, double factor) {
            if (shape.size() < 2)
                return;
            double extent = 0;
            for (std::size_t k = 1; k < shape.size(); ++k)
                extent += length(shape[k] - shape[k - 1]);
            outgoing[from].push_back(static_cast<int>(links.size()));
            double cost = extent * factor + turningCost(shape);
            links.push_back(
                {from, to, edge, shape.front(), shape.back(), extent, cost, std::move(shape), true});
        };
        for (int root : roots) {
            const auto spur = links[root];
            auto inward = alongShape(spur.shape, 0).tangent;
            for (std::size_t i = 0; i < originalCount; ++i) {
                checkPlanning(o);
                const auto main = links[i];
                const auto &mainEdge = edgeAt(main.edge);
                if (!main.painted || mainEdge.runway || main.from == spur.from || main.to == spur.to)
                    continue;
                auto nearest = snapShape(main.shape, spur.a);
                if (nearest.distance < .3 || nearest.distance > 20 ||
                    dot(nearest.point - spur.a, inward) > -.8 * nearest.distance)
                    continue;
                double turn = std::abs(wrap180(heading(inward) - heading(nearest.tangent)));
                if (turn < 35 || turn > 145)
                    continue;
                for (double setback : {1.5, 2.5, 3.5}) {
                    double room = std::hypot(o.minimumTurnRadius, o.wheelbase + o.cockpitAheadNose) * setback;
                    double mainAt = nearest.progress - room;
                    double spurAt = std::max(8., room - nearest.distance);
                    if (mainAt < 1 || spurAt > spur.length - 1)
                        continue;
                    auto start = alongShape(main.shape, mainAt), end = alongShape(spur.shape, spurAt);
                    auto curve =
                        apronCurve(standPavement, unproject(position, start.point), end.point - start.point,
                                   heading(start.tangent), end.tangent, o.minimumTurnRadius, o, true);
                    if (curve.empty())
                        continue;
                    for (auto &point : curve)
                        point = point + start.point;
                    bool onSurface = true;
                    for (std::size_t k = 1; k < curve.size() && onSurface; ++k) {
                        Vec2 delta = curve[k] - curve[k - 1];
                        int samples = std::max(1, static_cast<int>(std::ceil(length(delta) / .5)));
                        for (int sample = 0; sample <= samples; ++sample)
                            if (!standPavement.contains(
                                    unproject(position, curve[k - 1] + delta * (double(sample) / samples)))) {
                                onSurface = false;
                                break;
                            }
                    }
                    if (!onSurface)
                        continue;
                    auto inferred = edgeAt(spur.edge);
                    inferred.oneWay = true;
                    inferred.inferredGap = true;
                    inferred.name = "Inferred stand entry";
                    inferred.width = !mainEdge.width   ? inferred.width
                                     : !inferred.width ? mainEdge.width
                                                       : std::min(mainEdge.width, inferred.width);
                    inferred.activeRunways.insert(inferred.activeRunways.end(),
                                                  mainEdge.activeRunways.begin(),
                                                  mainEdge.activeRunways.end());
                    const int edge = static_cast<int>(a.edges.size() + exitRunwayEdges.size());
                    exitRunwayEdges.push_back(std::move(inferred));
                    int first = virtualNode--, second = virtualNode--;
                    addLink(main.from, first, main.edge, sliceShape(main.shape, 0, mainAt), 1);
                    addLink(first, second, edge, std::move(curve), 1.2);
                    addLink(second, spur.to, spur.edge, sliceShape(spur.shape, spurAt, spur.length), 1);
                    break;
                }
            }
        }
    }
    std::vector<ShapeSnap> snaps;
    for (const auto &link : links) {
        snaps.push_back(snapShape(link.shape, {}));
    }
    std::vector<Start> starts;
    PavementQuery pavement(a, position);
    double nearestForward = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < links.size(); ++i) {
        checkPlanning(o);
        const auto &link = links[i];
        const auto &edge = edgeAt(link.edge);
        if (onRunway && o.paintedRunwayExitsOnly && !edge.runway &&
            (!link.painted || snaps[i].distance > 3 ||
             std::abs(wrap180(heading(snaps[i].tangent) - trueHeading)) > 15))
            continue;
        if (o.requiredDepartureNode >= 0 && link.from != o.requiredDepartureNode)
            continue;
        auto snap = snaps[i];
        double proximity = snap.distance;
        if (snap.distance > o.maxApronJoinDistance)
            continue;
        bool apron = snap.distance > o.maxJoinDistance;
        // Longer ramp exits require a continuous paved surface, including polygon holes.
        if (apron && edge.runway)
            continue;
        bool pushback =
            snap.distance > 5 && std::abs(wrap180(heading(snap.point) - trueHeading)) > o.maxTurnDegrees;
        Vec2 initial = snap.distance > 5 ? snap.point : snap.tangent;
        if (length(initial) < 1)
            initial = link.b - link.a;
        double turn = std::abs(wrap180(heading(initial) - trueHeading));
        if (turn > std::min(o.maxTurnDegrees, o.maxInitialTurnDegrees) && !pushback)
            continue;
        if (snap.distance > 5 && link.length - snap.progress > 5 &&
            std::abs(wrap180(heading(snap.tangent) - heading(snap.point))) > o.maxTurnDegrees && !pushback)
            continue;
        std::vector<Vec2> connector;
        double connectorLength = snap.distance;
        const Vec2 cockpitStart = direction(trueHeading) * (o.wheelbase + o.cockpitAheadNose - o.mainAxleAft);
        const Vec2 connectorStart = link.painted ? cockpitStart : Vec2{};
        const auto cockpitSnap = snapShape(link.shape, connectorStart);
        if (!pushback && snap.distance > 5 &&
            (turn > 20 || std::abs(wrap180(heading(snap.tangent) - heading(snap.point))) > 20)) {
            double baseProgress = std::max(snap.progress, cockpitSnap.progress);
            for (double advance : {0., 30., 60., 90., 120.}) {
                if (advance > link.length - baseProgress)
                    break;
                auto joinSnap = alongShape(link.shape, baseProgress + advance);
                Vec2 join = joinSnap.point;
                if (length(join - connectorStart) > o.maxApronJoinDistance)
                    continue;
                connector = apronCurve(pavement, unproject(position, connectorStart), join - connectorStart,
                                       trueHeading, joinSnap.tangent, o.minimumTurnRadius, o, link.painted);
                if (connector.empty() && link.painted)
                    connector = apronCurve(pavement, position, join, trueHeading, joinSnap.tangent,
                                           o.minimumTurnRadius, o, true);
                else if (!connector.empty()) {
                    for (auto &point : connector)
                        point = point + connectorStart;
                }
                if (!connector.empty()) {
                    snap.point = join;
                    snap.progress = joinSnap.progress;
                    break;
                }
            }
            if (connector.empty()) {
                bool nearStand = std::any_of(a.ramps.begin(), a.ramps.end(), [&](const Ramp &ramp) {
                    return distance(position, ramp.position) < 100;
                });
                if (!nearStand)
                    continue;
                pushback = true;
            } else {
                connectorLength = 0;
                for (std::size_t j = 1; j < connector.size(); ++j)
                    connectorLength += length(connector[j] - connector[j - 1]);
            }
        }
        // A curved apron exit may avoid an unpaved gap crossed by the direct chord.
        if (apron && connector.empty() && !pavement.connection(position, unproject(position, snap.point)))
            continue;
        if (!pushback)
            nearestForward = std::min(nearestForward, proximity);
        starts.push_back({static_cast<int>(i), snap.point,
                          connectorLength + link.length - snap.progress + turn * 0.15, proximity, pushback,
                          apron, std::move(connector), snap.progress});
    }
    if (starts.empty())
        throw std::runtime_error(onRunway ? "No runway-axis connection from current aircraft position"
                                         : "No paved gate/apron connection to the taxi network within 180 m");
    // Keep several local joins; nearest does not necessarily belong to the reachable network.
    starts.erase(std::remove_if(starts.begin(), starts.end(),
                                [&](const Start &s) { return s.proximity > nearestForward + 40; }),
                 starts.end());
    std::set<int> goals;
    std::set<int> paintedGoals;
    std::unordered_map<int, ShapeSnap> rampGoals;
    Vec2 rampTarget;
    Vec2 holdTarget;
    bool holdGoal = false;
    if (d.kind == DestinationKind::Node) {
        if (!a.nodes.count(d.index))
            throw std::runtime_error("Unknown node");
        auto alias = a.nodeAliases.find(d.index);
        goals.insert(alias == a.nodeAliases.end() ? d.index : alias->second);
    } else if (d.kind == DestinationKind::Ramp) {
        const auto &ramp = a.ramps.at(d.index);
        rampTarget = project(position, ramp.position);
        // Stand metadata is often placed at the aircraft reference point,
        // while the visible lead-in ends at the stop bar. Keep the painted
        // line authoritative, but allow a metadata-to-paint gap up to 45 m
        // when the tangent still agrees with the stand heading.
        constexpr double standLeadInSnapDistance = 45.0;
        if (std::any_of(links.begin(), links.end(), [](const Link &link) { return link.painted; })) {
            for (std::size_t i = 0; i < links.size(); ++i) {
                if (!links[i].painted || edgeAt(links[i].edge).runway)
                    continue;
                auto snap = snapShape(links[i].shape, rampTarget);
                if (snap.distance < standLeadInSnapDistance &&
                    std::abs(wrap180(heading(snap.tangent) - ramp.heading)) < 30)
                    rampGoals[static_cast<int>(i)] = snap;
            }
            if (rampGoals.empty()) {
                double nearestGap = std::numeric_limits<double>::infinity();
                double nearestHeading = 180;
                for (const auto &link : links) {
                    if (!link.painted || edgeAt(link.edge).runway)
                        continue;
                    auto snap = snapShape(link.shape, rampTarget);
                    double gap = snap.distance;
                    double angle = std::abs(wrap180(heading(snap.tangent) - ramp.heading));
                    angle = std::min(angle, 180. - angle);
                    if (gap < nearestGap) {
                        nearestGap = gap;
                        nearestHeading = angle;
                    }
                }
                if (std::isfinite(nearestGap))
                    throw std::runtime_error(
                        "Stand has no matching painted lead-in / stop position (nearest yellow " +
                        std::to_string(nearestGap) + " m, heading difference " +
                        std::to_string(nearestHeading) + " deg)");
                throw std::runtime_error("Stand has no matching painted lead-in / stop position");
            }
            double nearestLeadIn = std::numeric_limits<double>::infinity();
            for (const auto &goal : rampGoals)
                nearestLeadIn = std::min(nearestLeadIn, goal.second.distance);
            for (auto it = rampGoals.begin(); it != rampGoals.end();) {
                if (it->second.distance > nearestLeadIn + 1.5)
                    it = rampGoals.erase(it);
                else
                    ++it;
            }
        } else {
            for (const auto &n : a.nodes) {
                double gap = distance(n.second.position, ramp.position);
                if (gap <= 25 && pavement.connection(n.second.position, ramp.position))
                    goals.insert(n.first);
            }
            if (goals.empty())
                throw std::runtime_error("Stand has no paved connection to the taxi network");
        }
    } else if (d.kind == DestinationKind::HoldShort) {
        if (d.index < 0 || d.index >= static_cast<int>(a.holdShortPoints.size()))
            throw std::runtime_error("Unknown hold-short point");
        holdTarget = project(position, a.holdShortPoints[d.index].position);
        double nearest = std::numeric_limits<double>::infinity();
        int nearestNodeId = -1;
        for (const auto &node : a.nodes) {
            const double gap = length(project(position, node.second.position) - holdTarget);
            if (gap < nearest && gap <= 120) {
                nearest = gap;
                nearestNodeId = node.first;
            }
        }
        if (nearestNodeId < 0)
            throw std::runtime_error("Hold-short point has no paved taxiway connection");
        goals.insert(nearestNodeId);
        holdGoal = true;
    } else {
        double latestEntry =
            o.allowIntersectionDeparture ? runwayLength - 450 : std::min(700.0, runwayLength * 0.3);
        auto nearDepartureEnd = [&](int id, double maxCross) {
            Vec2 p = project(position, a.nodes.at(id).position) - threshold;
            double along = dot(p, axis), cross = std::abs(p.x * axis.y - p.y * axis.x);
            return along >= -15 && along <= latestEntry && cross <= maxCross;
        };
        for (const auto &e : a.edges)
            if (e.runway && namedRunway(e.name, runway))
                for (int id : {e.from, e.to})
                    if (nearDepartureEnd(id, runwayWidth / 2 + 5))
                        goals.insert(id);
        for (const auto &entry : a.paintedRunwayEntries)
            if (entry.runway == d.index &&
                nearDepartureEnd(entry.node, entry.continuation.empty() ? 2 : runwayWidth / 2) &&
                (entry.continuation.empty() ||
                 std::abs(wrap180(heading(project(entry.continuation[entry.continuation.size() - 2],
                                                  entry.continuation.back())) -
                                  r.finalHeading)) < 5)) {
                goals.insert(entry.node);
                paintedGoals.insert(entry.node);
            }
        if (goals.empty())
            throw std::runtime_error("Selected runway has no usable entry in this scenery's taxi network");
    }
    const double infinity = std::numeric_limits<double>::infinity();
    auto matchesVia = [&](const std::string &token, const Link &link, bool reachedEnd = true) {
        if (token.empty())
            return false;
        if (token[0] != '#')
            return upper(edgeAt(link.edge).name) == upper(token);
        if (!reachedEnd)
            return false;
        if (token == "#" + std::to_string(link.to))
            return true;
        for (const auto &alias : a.nodeAliases)
            if (alias.second == link.to && token == "#" + std::to_string(alias.first))
                return true;
        return false;
    };
    for (const auto &token : o.via)
        if (std::none_of(links.begin(), links.end(),
                         [&](const Link &link) { return matchesVia(token, link); }))
            throw std::runtime_error("Custom route taxiway/node unavailable: " + token);
    auto advance = [&](std::size_t stage, const Link &link, bool reachedEnd = true) {
        while (stage < o.via.size() && matchesVia(o.via[stage], link, reachedEnd))
            ++stage;
        return stage;
    };
    const int linkCount = static_cast<int>(links.size());
    if (!linkCount)
        throw std::runtime_error("No permitted taxi links");
    const std::size_t states = links.size() * (o.via.size() + 1);
    std::vector<double> costs(states, infinity);
    std::vector<int> previous(states, -1), root(states, -1);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
    // Prefer every feasible forward path over requiring an unnecessary tow.
    double pushbackPenalty = 1 + o.maxApronJoinDistance * 5;
    for (const auto &link : links)
        pushbackPenalty += link.cost + o.maxTurnDegrees * 0.25;
    for (std::size_t i = 0; i < starts.size(); ++i) {
        const auto &s = starts[i];
        double cost = s.cost + (s.pushback ? pushbackPenalty : 0);
        const int state = static_cast<int>(advance(0, links[s.link])) * linkCount + s.link;
        if (cost < costs[state]) {
            costs[state] = cost;
            root[state] = static_cast<int>(i);
            queue.push({cost, state});
        }
    }
    double best = infinity;
    int goalLink = -1;
    int goalState = -1;
    double fallbackBest = infinity;
    int fallbackGoalLink = -1;
    int fallbackState = -1;
    // Search directed links, retaining arrival heading. A rejected turn can then use an alternate path.
    while (!queue.empty()) {
        checkPlanning(o);
        auto [cost, state] = queue.top();
        queue.pop();
        if (cost > costs[state] || (rampGoals.empty() && cost > best))
            continue;
        const int id = state % linkCount;
        const std::size_t stage = state / linkCount;
        const auto &current = links[id];
        const bool partialGoal = rampGoals.count(id) && rampGoals.at(id).progress + .5 < current.length;
        const auto goalStage =
            partialGoal ? advance(previous[state] < 0 ? 0 : previous[state] / linkCount, current, false)
                        : stage;
        if (goalStage == o.via.size() && (goals.count(current.to) || rampGoals.count(id))) {
            double extra = 0;
            bool compatible = true;
            if (r.runway) {
                double limit =
                    paintedGoals.count(current.to) ? std::min(60., o.maxTurnDegrees) : o.maxTurnDegrees;
                compatible = std::abs(wrap180(r.finalHeading - arrivalHeading(current))) <= limit;
                extra = std::max(0.0, dot(current.b - threshold, axis)) * 0.25;
            } else if (d.kind == DestinationKind::Ramp) {
                if (rampGoals.count(id)) {
                    const auto &target = rampGoals.at(id);
                    compatible = !(previous[state] < 0 && root[state] >= 0 &&
                                   target.progress + .5 < starts[root[state]].progress);
                    extra = target.distance - (current.length - target.progress);
                } else {
                    Vec2 exit = rampTarget - current.b;
                    extra = length(exit);
                    if (extra > 5)
                        compatible =
                            std::abs(wrap180(heading(exit) - arrivalHeading(current))) <= o.maxTurnDegrees;
                }
            }
            if (compatible) {
                bool preferred =
                    !r.runway || paintedGoals.empty() ||
                    (paintedGoals.count(current.to) && current.painted && !edgeAt(current.edge).runway);
                if (preferred && cost + extra < best) {
                    best = cost + extra;
                    goalLink = id;
                    goalState = state;
                } else if (!preferred && cost + extra < fallbackBest) {
                    fallbackBest = cost + extra;
                    fallbackGoalLink = id;
                    fallbackState = state;
                }
            }
        }
        for (int next : outgoing[current.to]) {
            const auto &link = links[next];
            if (o.paintedRunwayExitsOnly && edgeAt(current.edge).runway && !edgeAt(link.edge).runway &&
                !link.painted)
                continue;
            if (link.to == current.from)
                continue;
            double turn = std::abs(wrap180(departureHeading(link) - arrivalHeading(current)));
            double turnLimit = current.painted && link.painted ? 35 : o.maxTurnDegrees;
            if ((current.painted || link.painted || (current.length > 4 && link.length > 4)) &&
                turn > turnLimit)
                continue;
            double candidate = cost + link.cost + turn * .6;
            const int nextState = static_cast<int>(advance(stage, link)) * linkCount + next;
            if (candidate < costs[nextState]) {
                costs[nextState] = candidate;
                previous[nextState] = state;
                root[nextState] = root[state];
                queue.push({candidate, nextState});
            }
        }
    }
    if (goalLink < 0) {
        goalLink = fallbackGoalLink;
        goalState = fallbackState;
    }
    if (goalLink < 0)
        throw std::runtime_error("No route satisfying aircraft turns, one-way edges and E/F width");
    std::vector<int> path;
    for (int state = goalState; state >= 0; state = previous[state])
        path.push_back(state % linkCount);
    std::reverse(path.begin(), path.end());
    const auto &start = starts.at(root[goalState]);
    r.requiresPushback = start.pushback;
    r.apronDeparture = start.apron;
    std::set<std::string> crossings;
    auto physicalRunway = [&](const std::string &name) {
        for (const auto &rw : a.runways)
            if (namedRunway(name, rw.ends[0].name) || namedRunway(name, rw.ends[1].name))
                return rw.ends[0].name + "/" + rw.ends[1].name;
        return name;
    };
    auto selectedRunway = physicalRunway(runway);
    double builtDistance = 0;
    bool edgeRisk = false;
    auto appendWithDistance = [&](Vec2 point) {
        if (!r.points.empty() && length(point - r.points.back()) <= 0.5)
            return;
        if (!r.points.empty())
            builtDistance += length(point - r.points.back());
        r.points.push_back(point);
        r.pavementRisk.push_back(edgeRisk);
    };
    appendWithDistance({});
    for (auto point : start.connector)
        appendWithDistance(point);
    appendWithDistance(start.snap);
    for (std::size_t pathIndex = 0; pathIndex < path.size(); ++pathIndex) {
        const int id = path[pathIndex];
        const bool finalRampLink = pathIndex + 1 == path.size() && rampGoals.count(id);
        const auto &link = links[id];
        const auto &edge = edgeAt(link.edge);
        edgeRisk = !edge.runway && edge.width && edge.width < o.minimumWidth;
        if (edge.painted && !edge.runway && !r.cockpitGuidance) {
            r.paintedStartDistance = builtDistance;
            r.cockpitGuidance = true;
        }
        std::string taxiway = edge.name.empty() ? "Painted centerline" : edge.name;
        if (r.taxiwaysAlongRoute.empty() || r.taxiwaysAlongRoute.back().label != taxiway)
            r.taxiwaysAlongRoute.push_back({builtDistance, -1, taxiway});
        double along = 0;
        for (std::size_t k = 1; k < link.shape.size(); ++k) {
            along += length(link.shape[k] - link.shape[k - 1]);
            if (finalRampLink && along > rampGoals.at(id).progress + .01)
                break;
            if (pathIndex != 0 || along > start.progress + .01)
                appendWithDistance(link.shape[k]);
        }
        if (finalRampLink)
            appendWithDistance(rampGoals.at(id).point);
        if (link.to >= 0 && (!finalRampLink || rampGoals.at(id).progress + .5 >= link.length)) {
            r.nodeIds.push_back(link.to);
            r.nodesAlongRoute.push_back({builtDistance, link.to, "Node " + std::to_string(link.to)});
        }
        if (!edge.painted && !edge.runway && !a.groundLines.empty())
            ++r.atcFallbackCount;
        if (edge.inferredGap)
            ++r.inferredGapCount;
        if (edge.runway && physicalRunway(edge.name) != selectedRunway)
            crossings.insert(physicalRunway(edge.name));
        for (const auto &active : edge.activeRunways)
            if (physicalRunway(active) != selectedRunway)
                crossings.insert(physicalRunway(active));
    }
    r.crossedRunways.assign(crossings.begin(), crossings.end());
    edgeRisk = false;
    if (d.kind == DestinationKind::Ramp && rampGoals.empty())
        appendWithDistance(rampTarget);
    if (holdGoal) {
        // A hold-short marker is itself a surveyed runway-pavement reference;
        // the final short approach is therefore authoritative even when an
        // apt.dat exporter omitted apron polygons.
        appendWithDistance(holdTarget);
    }
    if (r.ramp && rampGoals.count(goalLink))
        r.finalHeading = heading(rampGoals.at(goalLink).tangent);
    if (r.runway) {
        for (const auto &entry : a.paintedRunwayEntries)
            if (entry.runway == d.index && entry.node == links[goalLink].to &&
                entry.continuation.size() >= 2 &&
                std::abs(wrap180(heading(project(entry.continuation[entry.continuation.size() - 2],
                                                 entry.continuation.back())) -
                                 r.finalHeading)) < 5) {
                for (std::size_t i = 1; i < entry.continuation.size(); ++i)
                    appendWithDistance(project(position, entry.continuation[i]));
                break;
            }
        double along = std::max(0.0, dot(r.points.back() - threshold, axis)), join = along + 100,
               stop = join + 220;
        if (stop > runwayLength - 100)
            throw std::runtime_error("Insufficient runway length for alignment");
        appendWithDistance(threshold + axis * join);
        appendWithDistance(threshold + axis * stop);
        r.runwayRemaining = runwayLength - stop;
    }
    if (r.points.size() < 2)
        throw std::runtime_error("Aircraft already at destination");
    for (std::size_t i = 1; i < r.points.size(); ++i)
        r.length += length(r.points[i] - r.points[i - 1]);
    if (r.ramp)
        r.cockpitStop = r.points.back();
    if (r.cockpitGuidance && !r.requiresPushback) {
        Route original = r;
        try {
            applyCockpitOversteer(a, r, o);
        } catch (const PlanningCancelled &) {
            throw;
        } catch (const std::runtime_error &error) {
            if (!o.allowOversteer)
                throw;
            r = std::move(original);
            try {
                applyTurnGuidance(a, r, o);
            } catch (const PlanningCancelled &) {
                throw;
            } catch (const std::runtime_error &fallbackError) {
                throw std::runtime_error(std::string(error.what()) +
                                         "; geometric cockpit fallback: " + fallbackError.what());
            }
        }
    }
    if (r.ramp && !r.cockpitGuidance && !r.requiresPushback) {
        // The terminal marker is a cockpit target, even when the curve is tracked by the main axle.
        double trim = o.wheelbase + o.cockpitAheadNose;
        const auto forward = direction(r.finalHeading);
        while (r.points.size() > 1 && trim > .001) {
            Vec2 delta = r.points.back() - r.points[r.points.size() - 2];
            double span = length(delta);
            if (span > .01 && dot(delta * (1 / span), forward) < std::cos(5 * rad))
                throw std::runtime_error("Stand lead-in has insufficient aligned distance for cockpit stop");
            if (span > trim) {
                r.points.back() = r.points.back() - forward * trim;
                trim = 0;
            } else {
                trim -= span;
                r.points.pop_back();
            }
        }
        if (trim > .001 || r.points.size() < 2)
            throw std::runtime_error("Stand approach is too short for cockpit stop");
        r.pavementRisk.resize(r.points.size());
        r.oversteerOffsets.resize(r.points.size());
        r.length = 0;
        for (std::size_t i = 1; i < r.points.size(); ++i)
            r.length += length(r.points[i] - r.points[i - 1]);
        auto dropPassed = [&](std::vector<RouteMarker> &markers) {
            markers.erase(std::remove_if(markers.begin(), markers.end(),
                                         [&](const RouteMarker &m) { return m.distance > r.length; }),
                          markers.end());
        };
        dropPassed(r.nodesAlongRoute);
        dropPassed(r.taxiwaysAlongRoute);
        r.nodeIds.clear();
        for (const auto &marker : r.nodesAlongRoute)
            r.nodeIds.push_back(marker.node);
    }
    if (r.ramp) {
        for (const auto &marker : r.taxiwaysAlongRoute) {
            auto name = upper(marker.label);
            if (marker.distance >= r.length - 600 &&
                (name.find("APRON") != std::string::npos || name.find("RAMP") != std::string::npos ||
                 name.find("LEAD-IN") != std::string::npos || name.find("STAND") != std::string::npos)) {
                r.apronArrivalDistance = marker.distance;
                break;
            }
        }
    }
    if (!r.requiresPushback)
        assessRoutePavement(a, r, o);
    buildReferencePaths(r, o);
    return r;
}
} // namespace autotaxi
