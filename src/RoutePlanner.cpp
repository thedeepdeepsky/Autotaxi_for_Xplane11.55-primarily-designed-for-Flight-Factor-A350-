#include "RoutePlanner.h"
#include "PavementQuery.h"
#include <algorithm>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
namespace autotaxi {
namespace {
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
    return e.runway || (e.width ? e.width >= o.minimumWidth : o.allowUnknownWidth);
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
double departureHeading(const Link &link) {
    return heading(link.shape[1] - link.shape[0]);
}
double arrivalHeading(const Link &link) {
    return heading(link.shape.back() - link.shape[link.shape.size() - 2]);
}
std::vector<Vec2> apronCurve(const PavementQuery &pavement, GeoPoint origin, Vec2 end, double yaw,
                             Vec2 finalDirection, double minimumRadius) {
    double gap = length(end);
    for (double fraction : {0.3, 0.45, 0.65, 0.9}) {
        double handle = std::min(80.0, gap * fraction);
        Vec2 p1 = direction(yaw) * handle, p2 = end - finalDirection * handle;
        int samples = std::max(20, static_cast<int>(std::ceil((gap + 2 * handle) / 2)));
        std::vector<Vec2> points{{}};
        bool feasible = true;
        for (int i = 1; i <= samples; ++i) {
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
    for (std::size_t i = 0; i < a.runways.size(); ++i)
        for (int end = 0; end < 2; ++end)
            out.push_back(
                {DestinationKind::Runway, "RWY " + a.runways[i].ends[end].name, static_cast<int>(i), end});
    for (std::size_t i = 0; i < a.ramps.size(); ++i)
        if (!a.ramps[i].width || a.ramps[i].width >= 'E')
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
    if (!valid(position) || !std::isfinite(trueHeading))
        throw std::runtime_error("Invalid aircraft position or heading");
    if (a.nodes.empty() || a.edges.empty())
        throw std::runtime_error("Airport has no 1201/1202 taxi network");
    if (d.kind == DestinationKind::Runway && !o.runwayClearance)
        throw std::runtime_error("Runway clearance required for lineup and crossings");
    Route r;
    r.origin = position;
    r.label = d.label;
    r.runway = d.kind == DestinationKind::Runway;
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
    std::unordered_map<int, std::vector<int>> outgoing;
    for (std::size_t i = 0; i < a.edges.size(); ++i) {
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
                Vec2 u = shape[k] - shape[k - 1], v = shape[k + 1] - shape[k];
                double area = std::abs(u.x * v.y - u.y * v.x);
                if (length(u) > .5 && length(v) > .5 && area > 1e-5 &&
                    length(u) * length(v) * length(u + v) / (2 * area) < o.minimumTurnRadius * .9)
                    feasible = false;
            }
        if (!feasible)
            continue;
        if (len < 0.05)
            continue;
        double cost = len * (e.runway ? (namedRunway(e.name, runway) ? 1.6 : 5) : 1);
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
    std::vector<ShapeSnap> snaps;
    for (const auto &link : links) {
        snaps.push_back(snapShape(link.shape, {}));
    }
    std::vector<Start> starts;
    PavementQuery pavement(a, position);
    double nearestForward = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < links.size(); ++i) {
        const auto &link = links[i];
        const auto &edge = a.edges[link.edge];
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
        if (!pushback && snap.distance > 5 &&
            (turn > 20 || std::abs(wrap180(heading(snap.tangent) - heading(snap.point))) > 20)) {
            for (double advance : {0., 30., 60., 90.}) {
                if (advance > link.length - snap.progress)
                    break;
                auto joinSnap = alongShape(link.shape, snap.progress + advance);
                Vec2 join = joinSnap.point;
                if (length(join) > o.maxApronJoinDistance)
                    continue;
                connector =
                    apronCurve(pavement, position, join, trueHeading, joinSnap.tangent, o.minimumTurnRadius);
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
        throw std::runtime_error("No paved gate/apron connection to the taxi network within 180 m");
    // Keep several local joins; nearest does not necessarily belong to the reachable network.
    starts.erase(std::remove_if(starts.begin(), starts.end(),
                                [&](const Start &s) { return s.proximity > nearestForward + 40; }),
                 starts.end());
    std::set<int> goals;
    std::set<int> paintedGoals;
    Vec2 rampTarget;
    if (d.kind == DestinationKind::Node) {
        if (!a.nodes.count(d.index))
            throw std::runtime_error("Unknown node");
        auto alias = a.nodeAliases.find(d.index);
        goals.insert(alias == a.nodeAliases.end() ? d.index : alias->second);
    } else if (d.kind == DestinationKind::Ramp) {
        const auto &ramp = a.ramps.at(d.index);
        rampTarget = project(position, ramp.position);
        for (const auto &n : a.nodes) {
            double gap = distance(n.second.position, ramp.position);
            if (gap <= 25 ||
                (gap <= o.maxApronJoinDistance && pavement.connection(n.second.position, ramp.position)))
                goals.insert(n.first);
        }
        if (goals.empty())
            throw std::runtime_error("Stand has no paved connection to the taxi network");
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
            if (entry.runway == d.index && nearDepartureEnd(entry.node, 2)) {
                goals.insert(entry.node);
                paintedGoals.insert(entry.node);
            }
        if (goals.empty())
            throw std::runtime_error("Selected runway has no usable entry in this scenery's taxi network");
    }
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> costs(links.size(), infinity);
    std::vector<int> previous(links.size(), -1), root(links.size(), -1);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
    // Prefer every feasible forward path over requiring an unnecessary tow.
    double pushbackPenalty = 1 + o.maxApronJoinDistance * 5;
    for (const auto &link : links)
        pushbackPenalty += link.cost + o.maxTurnDegrees * 0.25;
    for (std::size_t i = 0; i < starts.size(); ++i) {
        const auto &s = starts[i];
        double cost = s.cost + (s.pushback ? pushbackPenalty : 0);
        if (cost < costs[s.link]) {
            costs[s.link] = cost;
            root[s.link] = static_cast<int>(i);
            queue.push({cost, s.link});
        }
    }
    double best = infinity;
    int goalLink = -1;
    // Search directed links, retaining arrival heading. A rejected turn can then use an alternate path.
    while (!queue.empty()) {
        auto [cost, id] = queue.top();
        queue.pop();
        if (cost > costs[id] || cost > best)
            continue;
        const auto &current = links[id];
        if (goals.count(current.to)) {
            double extra = 0;
            bool compatible = true;
            if (r.runway) {
                double limit =
                    paintedGoals.count(current.to) ? std::min(35., o.maxTurnDegrees) : o.maxTurnDegrees;
                compatible = std::abs(wrap180(r.finalHeading - arrivalHeading(current))) <= limit;
                extra = std::max(0.0, dot(current.b - threshold, axis)) * 0.25;
            } else if (d.kind == DestinationKind::Ramp) {
                Vec2 exit = rampTarget - current.b;
                extra = length(exit);
                if (extra > 5)
                    compatible =
                        std::abs(wrap180(heading(exit) - arrivalHeading(current))) <= o.maxTurnDegrees;
            }
            if (compatible && cost + extra < best) {
                best = cost + extra;
                goalLink = id;
            }
        }
        for (int next : outgoing[current.to]) {
            const auto &link = links[next];
            if (link.to == current.from)
                continue;
            double turn = std::abs(wrap180(departureHeading(link) - arrivalHeading(current)));
            double turnLimit = current.painted && link.painted ? 35 : o.maxTurnDegrees;
            if ((current.painted || link.painted || (current.length > 4 && link.length > 4)) &&
                turn > turnLimit)
                continue;
            double candidate = cost + link.cost + turn * 0.25;
            if (candidate < costs[next]) {
                costs[next] = candidate;
                previous[next] = id;
                root[next] = root[id];
                queue.push({candidate, next});
            }
        }
    }
    if (goalLink < 0)
        throw std::runtime_error("No route satisfying aircraft turns, one-way edges and E/F width");
    std::vector<int> path;
    for (int id = goalLink; id >= 0; id = previous[id])
        path.push_back(id);
    std::reverse(path.begin(), path.end());
    const auto &start = starts.at(root[goalLink]);
    r.requiresPushback = start.pushback;
    r.apronDeparture = start.apron;
    auto append = [&](Vec2 p) {
        if (r.points.empty() || length(p - r.points.back()) > 0.5)
            r.points.push_back(p);
    };
    std::set<std::string> crossings;
    auto physicalRunway = [&](const std::string &name) {
        for (const auto &rw : a.runways)
            if (namedRunway(name, rw.ends[0].name) || namedRunway(name, rw.ends[1].name))
                return rw.ends[0].name + "/" + rw.ends[1].name;
        return name;
    };
    auto selectedRunway = physicalRunway(runway);
    double builtDistance = 0;
    auto appendWithDistance = [&](Vec2 point) {
        if (!r.points.empty())
            builtDistance += length(point - r.points.back());
        append(point);
    };
    appendWithDistance({});
    for (auto point : start.connector)
        appendWithDistance(point);
    appendWithDistance(start.snap);
    for (int id : path) {
        const auto &link = links[id];
        double along = 0;
        for (std::size_t k = 1; k < link.shape.size(); ++k) {
            along += length(link.shape[k] - link.shape[k - 1]);
            if (id != path.front() || along > start.progress + .01)
                appendWithDistance(link.shape[k]);
        }
        r.nodeIds.push_back(link.to);
        const auto &edge = a.edges[link.edge];
        r.nodesAlongRoute.push_back({builtDistance, link.to, "Node " + std::to_string(link.to)});
        if (r.taxiwaysAlongRoute.empty() || r.taxiwaysAlongRoute.back().label != edge.name)
            r.taxiwaysAlongRoute.push_back(
                {builtDistance, -1, edge.name.empty() ? "Painted centerline" : edge.name});
        if (!edge.painted && !edge.runway && !a.groundLines.empty())
            ++r.atcFallbackCount;
        if (edge.runway && physicalRunway(edge.name) != selectedRunway)
            crossings.insert(physicalRunway(edge.name));
        for (const auto &active : edge.activeRunways)
            if (physicalRunway(active) != selectedRunway)
                crossings.insert(physicalRunway(active));
    }
    r.crossedRunways.assign(crossings.begin(), crossings.end());
    if (d.kind == DestinationKind::Ramp)
        appendWithDistance(rampTarget);
    if (r.runway) {
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
    return r;
}
} // namespace autotaxi
