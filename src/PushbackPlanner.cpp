#include "PushbackPlanner.h"
#include "BpGeometryCompat.h"
#include "PavementQuery.h"
#undef MAX
#undef MIN
#undef ABS
#undef DEG2RAD
#undef RAD2DEG
#undef IS_NULL_VECT
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
namespace autotaxi {
namespace {
double normalHeading(double yaw) {
    double value = std::fmod(yaw, 360.0);
    return value < 0 ? value + 360 : value;
}
bool validSegment(const PushbackSegment &s) {
    return (s.type == 0 || s.type == 1) && valid(s.start) && valid(s.end) && std::isfinite(s.startHeading) &&
           s.startHeading >= 0 && s.startHeading <= 360 && std::isfinite(s.endHeading) && s.endHeading >= 0 &&
           s.endHeading <= 360 && (s.type == 0 || (std::isfinite(s.radius) && s.radius > 0));
}
bool makeSegments(const PavementQuery &pavement, const AircraftState &aircraft, Vec2 target,
                  double finalHeading, const ControllerConfig &settings, PushbackPlan &plan,
                  bool reverseConstruction = false) {
    bpgeometry::vehicle_t vehicle{settings.wheelbase, std::min(45.0, settings.maxSteer)};
    bpgeometry::list_t list;
    Vec2 start = direction(aircraft.trueHeading) * -settings.mainAxleAft;
    double offset = reverseConstruction ? 180 : 0;
    int result = bpgeometry::compute_segs(&vehicle, start, normalHeading(aircraft.trueHeading + offset),
                                          target, normalHeading(finalHeading + offset), &list);
    struct Cleanup {
        bpgeometry::list_t &list;
        ~Cleanup() {
            for (auto *segment : list.items)
                std::free(segment);
        }
    } cleanup{list};
    if (result <= 0)
        return false;
    plan.origin = aircraft.position;
    plan.points = {start};
    for (auto *s : list.items) {
        // Construct the reverse path as forward motion of a vehicle facing the opposite way.
        if (reverseConstruction) {
            if (s->backward)
                return false;
            s->start_hdg = normalHeading(s->start_hdg + 180);
            s->end_hdg = normalHeading(s->end_hdg + 180);
            s->backward = true;
            s->turn.right = !s->turn.right;
        }
        if (!s->backward)
            return false;
        PushbackSegment segment{s->type,
                                unproject(plan.origin, s->start_pos),
                                unproject(plan.origin, s->end_pos),
                                normalHeading(s->start_hdg),
                                normalHeading(s->end_hdg),
                                true,
                                s->turn.r,
                                s->turn.right != 0,
                                false};
        plan.segments.push_back(segment);
        if (s->type == bpgeometry::SEG_TYPE_STRAIGHT) {
            double gap = length(s->end_pos - s->start_pos);
            int count = std::max(1, static_cast<int>(std::ceil(gap / 2)));
            for (int i = 1; i <= count; ++i)
                plan.points.push_back(s->start_pos +
                                      (s->end_pos - s->start_pos) * (static_cast<double>(i) / count));
            plan.length += gap;
        } else {
            Vec2 forward = direction(s->start_hdg);
            Vec2 side{s->turn.right ? forward.y : -forward.y, s->turn.right ? -forward.x : forward.x};
            Vec2 center = s->start_pos + side * s->turn.r;
            Vec2 radial = s->start_pos - center;
            double angle = wrap180(s->end_hdg - s->start_hdg);
            double arc = std::abs(angle) * rad * s->turn.r;
            int count = std::max(1, static_cast<int>(std::ceil(arc / 2)));
            for (int i = 1; i <= count; ++i)
                plan.points.push_back(center + bpgeometry::vect2_rot(radial, angle * i / count));
            if (length(plan.points.back() - s->end_pos) > 0.1)
                return false;
            plan.length += arc;
        }
    }
    if (length(plan.points.back() - target) > 0.5 || plan.length > 450)
        return false;
    // Segments are sampled at <= 2 m, denser than pavedConnection's 4 m probes.
    for (auto point : plan.points)
        if (!pavement.contains(unproject(plan.origin, point)))
            return false;
    plan.segments.back().userPlaced = true;
    return true;
}
bool reverseLeg(const PavementQuery &pavement, const AircraftState &aircraft, Vec2 target, double yaw,
                const ControllerConfig &settings, PushbackPlan &plan) {
    if (makeSegments(pavement, aircraft, target, yaw, settings, plan))
        return true;
    plan = {};
    return makeSegments(pavement, aircraft, target, yaw, settings, plan, true);
}
std::vector<PushbackPlan> apronDetours(const PavementQuery &pavement, const AircraftState &aircraft,
                                       const ControllerConfig &settings) {
    std::vector<PushbackPlan> detours;
    Vec2 forward = direction(aircraft.trueHeading), side{forward.y, -forward.x};
    Vec2 start = forward * -settings.mainAxleAft;
    for (double back : {40., 80., 120.})
        for (double lateral : {0., -60., 60., -100., 100.})
            for (double turn : {0., -60., 60., -90., 90., -120., 120., 180.}) {
                Vec2 target = start - forward * back + side * lateral;
                PushbackPlan leg;
                if (reverseLeg(pavement, aircraft, target, normalHeading(aircraft.trueHeading + turn),
                               settings, leg))
                    detours.push_back(std::move(leg));
            }
    std::sort(detours.begin(), detours.end(),
              [](const auto &a, const auto &b) { return a.length < b.length; });
    return detours;
}
bool viaDetour(const PavementQuery &pavement, const AircraftState &aircraft, Vec2 target, double yaw,
               const ControllerConfig &settings, const std::vector<PushbackPlan> &detours,
               PushbackPlan &plan) {
    double shortest = 450;
    bool found = false;
    for (const auto &first : detours) {
        if (first.length + length(target - first.points.back()) >= shortest)
            continue;
        double midYaw = first.segments.back().endHeading;
        AircraftState mid{
            unproject(aircraft.position, first.points.back() + direction(midYaw) * settings.mainAxleAft),
            midYaw, 0, true};
        PushbackPlan second;
        if (!reverseLeg(pavement, mid, project(mid.position, unproject(aircraft.position, target)), yaw,
                        settings, second) ||
            first.length + second.length >= shortest)
            continue;
        plan = first;
        plan.segments.back().userPlaced = false;
        plan.segments.insert(plan.segments.end(), second.segments.begin(), second.segments.end());
        for (std::size_t i = 1; i < second.points.size(); ++i)
            plan.points.push_back(project(plan.origin, unproject(second.origin, second.points[i])));
        plan.length += second.length;
        shortest = plan.length;
        found = true;
    }
    return found;
}
} // namespace
PushbackPlan planPushback(const Airport &airport, const AircraftState &aircraft,
                          const Destination &destination, const RouteOptions &options,
                          const ControllerConfig &controller) {
    std::vector<std::pair<double, int>> candidates;
    std::set<int> nodes;
    for (const auto &edge : airport.edges)
        if (!edge.runway && edge.activeRunways.empty() &&
            (edge.width ? edge.width >= options.minimumWidth : options.allowUnknownWidth)) {
            nodes.insert(edge.from);
            nodes.insert(edge.to);
        }
    for (int node : nodes) {
        Vec2 target = project(aircraft.position, airport.nodes.at(node).position);
        if (length(target) >= 12 && length(target) <= 250)
            candidates.push_back({length(target), node});
    }
    std::sort(candidates.begin(), candidates.end());
    double bestCost = std::numeric_limits<double>::infinity();
    PavementQuery pavement(airport, aircraft.position);
    PushbackPlan best;
    std::vector<PushbackPlan> detours;
    bool detoursGenerated = false;
    for (std::size_t i = 0; i < std::min<std::size_t>(24, candidates.size()); ++i) {
        int node = candidates[i].second;
        Vec2 target = project(aircraft.position, airport.nodes.at(node).position);
        std::vector<double> headings;
        for (const auto &edge : airport.edges) {
            if (edge.runway || !edge.activeRunways.empty() ||
                (edge.width ? edge.width < options.minimumWidth : !options.allowUnknownWidth))
                continue;
            int next = edge.from == node ? edge.to : edge.to == node && !edge.oneWay ? edge.from : -1;
            if (next >= 0) {
                GeoPoint departure = airport.nodes.at(next).position;
                if (edge.geometry.size() >= 2)
                    departure =
                        edge.from == node ? edge.geometry[1] : edge.geometry[edge.geometry.size() - 2];
                double yaw = heading(project(airport.nodes.at(node).position, departure));
                if (std::none_of(headings.begin(), headings.end(),
                                 [&](double h) { return std::abs(wrap180(h - yaw)) < 5; }))
                    headings.push_back(yaw);
            }
        }
        for (double yaw : headings) {
            PushbackPlan plan;
            if (!reverseLeg(pavement, aircraft, target, yaw, controller, plan)) {
                if (!detoursGenerated) {
                    detours = apronDetours(pavement, aircraft, controller);
                    detoursGenerated = true;
                }
                if (!viaDetour(pavement, aircraft, target, yaw, controller, detours, plan))
                    continue;
            }
            plan.handoff = unproject(aircraft.position, target + direction(yaw) * controller.mainAxleAft);
            try {
                auto handoffOptions = options;
                handoffOptions.requiredDepartureNode = node;
                handoffOptions.maxInitialTurnDegrees = 15;
                plan.taxi = planRoute(airport, plan.handoff, yaw, destination, handoffOptions);
            } catch (const std::exception &) {
                continue;
            }
            if (plan.taxi.requiresPushback)
                continue;
            auto firstLeg = std::find_if(plan.taxi.points.begin() + 1, plan.taxi.points.end(),
                                         [&](Vec2 point) { return length(point) > 8; });
            if (firstLeg == plan.taxi.points.end() || std::abs(wrap180(heading(*firstLeg) - yaw)) > 10)
                continue;
            double cost = plan.taxi.length + plan.length * 4;
            if (cost >= bestCost)
                continue;
            plan.heading = normalHeading(yaw);
            plan.nodeId = node;
            plan.preview = plan.taxi;
            plan.preview.origin = aircraft.position;
            plan.preview.departure = departureLabel(airport, aircraft.position);
            plan.preview.requiresPushback = true;
            plan.preview.apronDeparture = true;
            plan.preview.points = plan.points;
            plan.preview.pushbackPointCount = plan.points.size();
            for (auto p : plan.taxi.points)
                plan.preview.points.push_back(project(aircraft.position, unproject(plan.taxi.origin, p)));
            plan.preview.length = plan.length + plan.taxi.length;
            plan.preview.taxiStartNode = node;
            best = std::move(plan);
            bestCost = cost;
        }
    }
    if (best.nodeId < 0)
        throw std::runtime_error("No paved reverse route to a taxi node with a forward departure");
    return best;
}
std::vector<std::vector<PushbackSegment>> readPushbackCache(std::istream &input) {
    std::ostringstream text;
    for (std::string line; std::getline(input, line);) {
        auto comment = line.find('#');
        text << line.substr(0, comment) << '\n';
    }
    std::istringstream tokens(text.str());
    tokens.imbue(std::locale::classic());
    std::vector<std::vector<PushbackSegment>> routes;
    for (std::string word; tokens >> word;) {
        if (word == "route") {
            if (!routes.empty() && routes.back().empty())
                throw std::runtime_error("Empty route in BetterPushback cache");
            routes.emplace_back();
        } else if (word == "seg" && !routes.empty()) {
            PushbackSegment s;
            int backward = 0, right = 0, placed = 0;
            if (!(tokens >> s.type >> s.start.lat >> s.start.lon >> s.startHeading >> s.end.lat >>
                  s.end.lon >> s.endHeading >> backward))
                throw std::runtime_error("Malformed BetterPushback segment");
            if (s.type == 1 && !(tokens >> s.radius >> right))
                throw std::runtime_error("Malformed BetterPushback arc");
            if (!(tokens >> placed) || backward < 0 || backward > 1 || right < 0 || right > 1 || placed < 0 ||
                placed > 1 || !validSegment(s))
                throw std::runtime_error("Invalid BetterPushback segment values");
            s.backward = backward != 0;
            s.right = right != 0;
            s.userPlaced = placed != 0;
            routes.back().push_back(s);
        } else
            throw std::runtime_error("Unknown BetterPushback cache format; cache preserved");
    }
    if (!routes.empty() && routes.back().empty())
        throw std::runtime_error("Empty route in BetterPushback cache");
    return routes;
}
void writePushbackCache(std::ostream &output, const std::vector<std::vector<PushbackSegment>> &routes) {
    output.imbue(std::locale::classic());
    output << "### BetterPushback segment table; includes an A350AutoTaxi planned departure ###\n";
    output << std::setprecision(17);
    for (const auto &route : routes) {
        if (route.empty())
            throw std::runtime_error("Cannot write an empty pushback route");
        output << "\nroute\n";
        for (const auto &s : route) {
            if (!validSegment(s))
                throw std::runtime_error("Cannot write an invalid pushback segment");
            output << "  seg " << s.type << ' ' << s.start.lat << ' ' << s.start.lon << ' ' << s.startHeading
                   << ' ' << s.end.lat << ' ' << s.end.lon << ' ' << s.endHeading << ' ' << s.backward << ' ';
            if (s.type == 1)
                output << s.radius << ' ' << s.right << ' ';
            output << s.userPlaced << '\n';
        }
    }
    if (!output)
        throw std::runtime_error("Failed to write BetterPushback cache");
}
void storePushbackRoute(const std::filesystem::path &cache, const PushbackPlan &plan) {
    if (plan.segments.empty())
        throw std::runtime_error("No pushback segments to submit");
    std::vector<std::vector<PushbackSegment>> routes;
    bool existed = std::filesystem::exists(cache);
    if (existed) {
        std::ifstream input(cache);
        if (!input)
            throw std::runtime_error("Cannot read BetterPushback cache");
        routes = readPushbackCache(input);
    }
    const auto &start = plan.segments.front();
    routes.erase(
        std::remove_if(routes.begin(), routes.end(),
                       [&](const auto &route) {
                           return distance(plan.origin, route.front().start) <= 35 &&
                                  std::abs(wrap180(start.startHeading - route.front().startHeading)) <= 10;
                       }),
        routes.end());
    routes.push_back(plan.segments);
    std::filesystem::create_directories(cache.parent_path());
    auto temporary = cache;
    temporary += ".autotaxi.tmp";
    auto backup = cache;
    backup += ".autotaxi.bak";
    if (existed && !std::filesystem::exists(backup))
        std::filesystem::copy_file(cache, backup);
    {
        std::ofstream output(temporary, std::ios::trunc);
        if (!output)
            throw std::runtime_error("Cannot create BetterPushback cache update");
        writePushbackCache(output, routes);
        output.close();
        if (!output)
            throw std::runtime_error("Cannot flush BetterPushback cache update");
    }
#if defined(_WIN32)
    if (!MoveFileExW(temporary.c_str(), cache.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        throw std::runtime_error("Cannot replace BetterPushback cache; original preserved");
#else
    std::filesystem::rename(temporary, cache);
#endif
}
} // namespace autotaxi
