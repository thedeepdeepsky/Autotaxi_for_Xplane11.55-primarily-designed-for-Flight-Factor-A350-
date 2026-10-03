#include "TurnGuidance.h"
#include "PavementQuery.h"
#include <algorithm>
#include <stdexcept>
namespace autotaxi {
namespace {
struct Curve {
    Vec2 tangent;
    double radius = 1e30, sign = 0;
};
Curve curveAt(const std::vector<Vec2> &points, const std::vector<double> &at, std::size_t index) {
    std::size_t before = index, after = index;
    while (before > 0 && at[index] - at[before] < 12)
        --before;
    while (after + 1 < points.size() && at[after] - at[index] < 12)
        ++after;
    Vec2 tangent = points[after] - points[before];
    double span = length(tangent);
    Curve result{span > .01 ? tangent * (1 / span) : Vec2{0, 1}};
    Vec2 u = points[index] - points[before], v = points[after] - points[index];
    double cross = u.x * v.y - u.y * v.x;
    if (length(u) > .5 && length(v) > .5 && std::abs(cross) > 1e-5) {
        result.radius = length(u) * length(v) * length(u + v) / (2 * std::abs(cross));
        result.sign = cross < 0 ? 1 : -1;
    }
    return result;
}
std::vector<double> distances(const std::vector<Vec2> &points) {
    std::vector<double> result(points.size());
    for (std::size_t i = 1; i < points.size(); ++i)
        result[i] = result[i - 1] + length(points[i] - points[i - 1]);
    return result;
}
class Surface {
    const Airport &airport_;
    GeoPoint origin_;
    PavementQuery pavement_;

  public:
    Surface(const Airport &airport, GeoPoint origin)
        : airport_(airport), origin_(origin), pavement_(airport, origin) {}
    bool contains(Vec2 point) const {
        GeoPoint geo = unproject(origin_, point);
        if (pavement_.contains(geo))
            return true;
        for (const auto &runway : airport_.runways) {
            Vec2 a = project(origin_, runway.ends[0].position), b = project(origin_, runway.ends[1].position);
            auto p = onSegment(point, a, b);
            if (dot(point - a, b - a) >= 0 && dot(point - b, a - b) >= 0 && p.distance <= runway.width / 2)
                return true;
        }
        return false;
    }
    double width(Vec2 point, Vec2 inward) const {
        if (!contains(point))
            return -1;
        for (double gap = .5; gap <= 60; gap += .5)
            if (!contains(point + inward * gap))
                return gap - .5;
        return 60;
    }
};
std::vector<double> sweptHeadings(const Route &route, double front, double mainAxleAft) {
    std::vector<double> result(route.points.size(), route.initialHeading);
    Vec2 position = direction(route.initialHeading) * (front - mainAxleAft);
    double closest = 1e30, walked = 0;
    std::size_t first = 1;
    Vec2 start = route.points.front();
    for (std::size_t i = 1; i < route.points.size() && walked < front + 50; ++i) {
        const auto candidate = onSegment(position, route.points[i - 1], route.points[i]);
        if (candidate.distance < closest) {
            closest = candidate.distance;
            first = i;
            start = candidate.point;
        }
        walked += length(route.points[i] - route.points[i - 1]);
    }
    double yaw = route.initialHeading * rad;
    for (std::size_t i = first; i < route.points.size(); ++i) {
        Vec2 delta = route.points[i] - start;
        double distance = length(delta), tangent = heading(delta) * rad;
        int count = std::max(1, static_cast<int>(std::ceil(distance / 1)));
        double step = distance / count;
        for (int k = 0; k < count; ++k) {
            double middle = yaw + .5 * step * std::sin(tangent - yaw) / front;
            yaw += step * std::sin(tangent - middle) / front;
        }
        result[i] = yaw / rad;
        start = route.points[i];
    }
    return result;
}
} // namespace
TurnClearance turnClearance(double radius, double insideWidth, const RouteOptions &options) {
    TurnClearance result;
    double front = options.wheelbase + options.cockpitAheadNose;
    double rear = std::sqrt(std::max(0.0, radius * radius - front * front));
    result.offtracking = radius - rear;
    double neededRear =
        std::max(options.minimumTurnRadius, options.wheelbase / std::tan(options.maxSteer * rad));
    if (insideWidth >= 0 && !options.ignorePavementLimits)
        neededRear = std::max(neededRear, radius - insideWidth + options.mainGearHalfSpan +
                                              options.wheelEdgeMargin + options.trackingAllowance);
    else
        neededRear = std::max(neededRear, rear);
    result.outwardOffset = std::max(0.0, std::hypot(neededRear, front) - radius);
    result.feasible = result.outwardOffset <= (options.allowOversteer ? options.maxOversteer : 0);
    return result;
}
void applyTurnGuidance(const Airport &airport, Route &route, const RouteOptions &options) {
    checkPlanning(options);
    if (!route.cockpitGuidance || route.requiresPushback || route.points.size() < 3)
        return;
    const double front = options.wheelbase + options.cockpitAheadNose;
    Vec2 cockpit = direction(route.initialHeading) * (front - options.mainAxleAft);
    double initialGap = 1e30, walked = 0;
    for (std::size_t i = 1; i < route.points.size() && walked <= front + 10; ++i) {
        initialGap = std::min(initialGap, onSegment(cockpit, route.points[i - 1], route.points[i]).distance);
        walked += length(route.points[i] - route.points[i - 1]);
    }
    if (initialGap > options.trackingAllowance)
        throw std::runtime_error("Cockpit cannot join the initial apron path within tracking allowance");
    std::vector<Vec2> sampled{route.points.front()};
    std::vector<bool> risks{!route.pavementRisk.empty() && route.pavementRisk.front()};
    for (std::size_t i = 1; i < route.points.size(); ++i) {
        const auto delta = route.points[i] - route.points[i - 1];
        int count = std::max(1, static_cast<int>(std::ceil(length(delta) / 2)));
        for (int k = 1; k <= count; ++k) {
            sampled.push_back(route.points[i - 1] + delta * (static_cast<double>(k) / count));
            risks.push_back(i < route.pavementRisk.size() && route.pavementRisk[i]);
        }
    }
    route.points = std::move(sampled);
    route.pavementRisk = std::move(risks);
    const auto original = route.points;
    const auto at = distances(original);
    Surface surface(airport, route.origin);
    std::vector<Curve> curves;
    std::vector<double> left(original.size()), right(original.size());
    for (std::size_t i = 0; i < original.size(); ++i) {
        checkPlanning(options);
        auto curve = curveAt(original, at, i);
        curves.push_back(curve);
        if (curve.radius > 1000 || at[i] < route.paintedStartDistance + 12)
            continue;
        Vec2 inward = Vec2{curve.tangent.y, -curve.tangent.x} * curve.sign;
        double width = surface.width(original[i], inward);
        if (width < 0)
            route.turnClearanceKnown = false;
        auto clearance = turnClearance(curve.radius, width, options);
        if (!clearance.feasible)
            throw std::runtime_error("Turn requires more oversteer than configured pavement allowance (R=" +
                                     std::to_string(curve.radius) +
                                     ", offset=" + std::to_string(clearance.outwardOffset) + ")");
        if (width < 0 && clearance.outwardOffset > .1)
            throw std::runtime_error("Tight painted turn needs oversteer but pavement is unavailable");
        (curve.sign > 0 ? left[i] : right[i]) = clearance.outwardOffset;
    }
    // Keep one offset through a continuous bend; varying it within the arc can tighten its radius.
    for (std::size_t i = 0; i < curves.size();) {
        if (curves[i].radius > 1000 || curves[i].sign == 0 || at[i] < route.paintedStartDistance + 12) {
            ++i;
            continue;
        }
        std::size_t end = i + 1;
        double leftMax = left[i], rightMax = right[i];
        while (end < curves.size() && curves[end].radius <= 1000 && curves[end].sign == curves[i].sign) {
            leftMax = std::max(leftMax, left[end]);
            rightMax = std::max(rightMax, right[end]);
            ++end;
        }
        for (; i < end; ++i) {
            left[i] = leftMax;
            right[i] = rightMax;
        }
    }
    // Limit lateral slope so compensation enters and exits progressively rather than adding a kink.
    for (auto *side : {&left, &right}) {
        for (std::size_t i = 1; i < at.size(); ++i)
            (*side)[i] = std::max((*side)[i], (*side)[i - 1] - .12 * (at[i] - at[i - 1]));
        for (std::size_t i = at.size() - 1; i > 0; --i)
            (*side)[i - 1] = std::max((*side)[i - 1], (*side)[i] - .12 * (at[i] - at[i - 1]));
    }
    route.oversteerOffsets.resize(original.size());
    for (std::size_t i = 0; i < original.size(); ++i) {
        double offset = right[i] - left[i];
        if (std::abs(offset) > .12 * (at.back() - at[i]) + .1)
            throw std::runtime_error("Insufficient straight distance to enter and recover oversteer at " +
                                     std::to_string(at[i]) + " m (offset=" + std::to_string(offset) + ")");
        route.oversteerOffsets[i] = std::abs(offset);
        route.maximumOversteer = std::max(route.maximumOversteer, std::abs(offset));
        route.points[i] = original[i] + Vec2{curves[i].tangent.y, -curves[i].tangent.x} * offset;
    }
    if (route.maximumOversteer > .1) {
        auto changedAt = distances(route.points);
        auto sweptYaw = sweptHeadings(route, front, options.mainAxleAft);
        for (std::size_t i = 0; i < route.points.size(); ++i) {
            checkPlanning(options);
            // The initial CG-to-join prefix lies behind the cockpit and is not flown by this reference.
            if (route.oversteerOffsets[i] < .1 || at[i] < std::max(front, route.paintedStartDistance))
                continue;
            const auto curve = curveAt(route.points, changedAt, i);
            // The 12 m curvature stencil is approximate; steering remains hard-limited in the controller.
            if (curve.radius < .95 * std::hypot(front, options.wheelbase / std::tan(options.maxSteer * rad)))
                throw std::runtime_error("Compensated turn exceeds nosewheel steering capability at " +
                                         std::to_string(at[i]) + " m (R=" + std::to_string(curve.radius) +
                                         ", original R=" + std::to_string(curves[i].radius) + ")");
            double yaw = sweptYaw[i];
            Vec2 rear = route.points[i] - direction(yaw) * front;
            Vec2 lateral = Vec2{direction(yaw).y, -direction(yaw).x} *
                           (options.mainGearHalfSpan + options.wheelEdgeMargin);
            Vec2 nose = route.points[i] - direction(yaw) * options.cockpitAheadNose;
            if (!surface.contains(rear + lateral) || !surface.contains(rear - lateral) ||
                !surface.contains(nose)) {
                if (options.ignorePavementLimits) {
                    route.pavementRisk[i] = true;
                    route.turnClearanceKnown = false;
                } else
                    throw std::runtime_error(
                        "Oversteer wheel envelope leaves known taxiway/runway pavement at " +
                        std::to_string(changedAt[i]) +
                        " m (offset=" + std::to_string(route.oversteerOffsets[i]) +
                        ", R=" + std::to_string(curve.radius) + ")");
            }
        }
        auto remap = [&](std::vector<RouteMarker> &markers) {
            for (auto &marker : markers) {
                auto next = std::lower_bound(at.begin() + 1, at.end(), marker.distance);
                std::size_t i = std::min<std::size_t>(next - at.begin(), at.size() - 1);
                double fraction =
                    std::clamp((marker.distance - at[i - 1]) / std::max(.01, at[i] - at[i - 1]), 0., 1.);
                marker.distance = changedAt[i - 1] + fraction * (changedAt[i] - changedAt[i - 1]);
            }
        };
        remap(route.nodesAlongRoute);
        remap(route.taxiwaysAlongRoute);
        route.length = changedAt.back();
    }
}
void applyAxleOversteer(const Airport &airport, Route &route, const RouteOptions &options) {
    checkPlanning(options);
    route.cockpitGuidance = false;
    route.mainAxleOversteer = true;
    route.maximumOversteer = 0;
    route.oversteerOffsets.assign(route.points.size(), 0);
    route.pavementRisk.resize(route.points.size(), false);
    const auto at = distances(route.points);
    const double front = options.wheelbase + options.cockpitAheadNose;
    Surface surface(airport, route.origin);
    for (std::size_t i = 0; i < route.points.size(); ++i) {
        checkPlanning(options);
        if (at[i] < route.paintedStartDistance + 12)
            continue;
        const auto curve = curveAt(route.points, at, i);
        if (curve.radius > 1000)
            continue;
        const double offset = std::hypot(curve.radius, front) - curve.radius;
        if (offset > (options.allowOversteer ? options.maxOversteer : 0))
            throw std::runtime_error("Main-axle oversteer exceeds configured outward allowance");
        route.oversteerOffsets[i] = offset;
        route.maximumOversteer = std::max(route.maximumOversteer, offset);
        if (!surface.contains(route.points[i])) {
            if (airport.pavementCoverageComplete && !options.ignorePavementLimits)
                throw std::runtime_error("Main-axle oversteer reference leaves known pavement");
            route.turnClearanceKnown = false;
            route.pavementRisk[i] = true;
            continue;
        }
        Vec2 lateral =
            Vec2{curve.tangent.y, -curve.tangent.x} * (options.mainGearHalfSpan + options.wheelEdgeMargin);
        Vec2 nose = route.points[i] + curve.tangent * options.wheelbase;
        if (!surface.contains(route.points[i] + lateral) || !surface.contains(route.points[i] - lateral) ||
            !surface.contains(nose)) {
            if (airport.pavementCoverageComplete && !options.ignorePavementLimits)
                throw std::runtime_error("Main-axle oversteer wheel envelope leaves known pavement at " +
                                         std::to_string(at[i]) +
                                         " m (x=" + std::to_string(route.points[i].x) +
                                         ", y=" + std::to_string(route.points[i].y) + ")");
            route.turnClearanceKnown = false;
            route.pavementRisk[i] = true;
        }
    }
}
void assessRoutePavement(const Airport &airport, Route &route, const RouteOptions &options) {
    checkPlanning(options);
    route.pavementRisk.resize(route.points.size(), false);
    Surface surface(airport, route.origin);
    const auto yaw =
        route.cockpitGuidance
            ? sweptHeadings(route, options.wheelbase + options.cockpitAheadNose, options.mainAxleAft)
            : std::vector<double>{};
    double traveled = 0;
    for (std::size_t i = 1; i < route.points.size(); ++i) {
        checkPlanning(options);
        Vec2 delta = route.points[i] - route.points[i - 1];
        double span = length(delta);
        if (span < .01)
            continue;
        Vec2 forward = delta * (1 / span);
        int samples = std::max(1, static_cast<int>(std::ceil(span / 4)));
        for (int k = 0; k <= samples; ++k) {
            const double prefix = route.cockpitGuidance
                                      ? options.wheelbase + options.cockpitAheadNose - options.mainAxleAft
                                      : 0;
            if (traveled + span * k / samples < route.paintedStartDistance + prefix)
                continue;
            Vec2 reference = route.points[i - 1] + delta * (static_cast<double>(k) / samples);
            if (route.cockpitGuidance)
                forward = direction(yaw[i - 1] + wrap180(yaw[i] - yaw[i - 1]) * k / samples);
            Vec2 rear = reference -
                        forward * (route.cockpitGuidance ? options.wheelbase + options.cockpitAheadNose : 0);
            Vec2 side{forward.y * (options.mainGearHalfSpan + options.wheelEdgeMargin),
                      -forward.x * (options.mainGearHalfSpan + options.wheelEdgeMargin)};
            Vec2 nose = rear + forward * options.wheelbase;
            if (!surface.contains(rear + side) || !surface.contains(rear - side) || !surface.contains(nose)) {
                route.pavementRisk[i] = true;
                if (airport.pavementCoverageComplete && !options.ignorePavementLimits)
                    throw std::runtime_error("Route wheel envelope leaves known pavement");
                route.turnClearanceKnown = false;
            }
        }
        traveled += span;
    }
}
void applyCockpitOversteer(const Airport &airport, Route &route, const RouteOptions &options) {
    checkPlanning(options);
    if (!route.cockpitGuidance || route.requiresPushback || route.points.size() < 2)
        return;
    // Short bends need a transient wheel sweep rather than the steady-circle approximation.
    const double front = options.wheelbase + options.cockpitAheadNose;
    const auto source = route.points;
    const auto risks = route.pavementRisk;
    const auto at = distances(source);
    auto pointAt = [&](double distance) {
        if (distance < 0)
            return source.front() + direction(route.initialHeading) * distance;
        if (distance > at.back())
            return source.back() + direction(route.finalHeading) * (distance - at.back());
        auto next = std::lower_bound(at.begin() + 1, at.end(), distance);
        auto i = std::min<std::size_t>(next - at.begin(), at.size() - 1);
        return source[i - 1] +
               (source[i] - source[i - 1]) * ((distance - at[i - 1]) / std::max(.001, at[i] - at[i - 1]));
    };
    const Vec2 cockpitStart = direction(route.initialHeading) * (front - options.mainAxleAft);
    double prefix = 0, nearest = 1e30;
    for (std::size_t i = 1; i < source.size() && at[i - 1] < front + 50; ++i) {
        auto snap = onSegment(cockpitStart, source[i - 1], source[i]);
        if (snap.distance < nearest) {
            nearest = snap.distance;
            prefix = at[i - 1] + snap.t * (at[i] - at[i - 1]);
        }
    }
    prefix = std::min(at.back(), std::max(prefix, front - options.mainAxleAft));
    if (at.back() - prefix < .01)
        throw std::runtime_error("Cockpit already at destination");
    std::vector<Vec2> painted{cockpitStart}, outset{cockpitStart};
    std::vector<double> sourceAt{prefix};
    std::vector<bool> sourceRisks{false};
    std::size_t segment = 1;
    int count = std::max(1, static_cast<int>(std::ceil((at.back() - prefix) / 2)));
    for (int i = 1; i <= count; ++i) {
        checkPlanning(options);
        double along = prefix + (at.back() - prefix) * i / count;
        while (segment + 1 < at.size() && at[segment] < along)
            ++segment;
        double wheelAlong = along - front;
        Vec2 tangent = pointAt(wheelAlong + 12) - pointAt(wheelAlong - 12);
        if (length(tangent) < .01)
            continue;
        tangent = tangent * (1 / length(tangent));
        if (wheelAlong < 12) {
            double t = std::clamp(wheelAlong / 12, 0., 1.);
            tangent = direction(route.initialHeading +
                                wrap180(heading(tangent) - route.initialHeading) * t * t * (3 - 2 * t));
        }
        painted.push_back(pointAt(along));
        outset.push_back(i == count ? source.back() : pointAt(wheelAlong) + tangent * front);
        sourceAt.push_back(along);
        sourceRisks.push_back(segment < risks.size() && risks[segment]);
    }
    if (painted.size() < 2)
        throw std::runtime_error("No usable cockpit trajectory");
    Surface surface(airport, route.origin);
    std::vector<double> weights(painted.size());
    route.cockpitGuidance = true;
    route.mainAxleOversteer = false;
    route.pavementRisk = sourceRisks;
    auto violations = [&] {
        route.pavementRisk = sourceRisks;
        route.points.resize(painted.size());
        route.oversteerOffsets.assign(painted.size(), 0);
        route.maximumOversteer = 0;
        for (std::size_t i = 0; i < painted.size(); ++i) {
            route.points[i] = painted[i] + (outset[i] - painted[i]) * weights[i];
            auto near = std::lower_bound(at.begin() + 1, at.end(), sourceAt[i] - front - 20);
            double offset = 1e30;
            for (std::size_t k = near - at.begin();
                 k < source.size() && at[k - 1] <= sourceAt[i] + front + 20; ++k)
                offset = std::min(offset, onSegment(route.points[i], source[k - 1], source[k]).distance);
            // The actual initial cockpit position is an initial join, not planned oversteer.
            route.oversteerOffsets[i] = i ? offset : 0;
            route.maximumOversteer = std::max(route.maximumOversteer, route.oversteerOffsets[i]);
        }
        std::vector<std::size_t> failed;
        auto yaw = sweptHeadings(route, front, options.mainAxleAft);
        const double maxSlip = std::atan(front / options.wheelbase * std::tan(options.maxSteer * rad));
        for (std::size_t i = 1; i < route.points.size(); ++i) {
            checkPlanning(options);
            Vec2 delta = route.points[i] - route.points[i - 1];
            if (length(delta) < .01)
                continue;
            double midYaw = yaw[i - 1] + .5 * wrap180(yaw[i] - yaw[i - 1]);
            bool unsafe = std::abs(wrap180(heading(delta) - midYaw)) * rad > maxSlip * .98;
            Vec2 forward = direction(yaw[i]);
            Vec2 rear = route.points[i] - forward * front;
            Vec2 side{forward.y, -forward.x};
            side = side * (options.mainGearHalfSpan + options.wheelEdgeMargin + options.trackingAllowance);
            const bool offPavement = !surface.contains(rear + side) || !surface.contains(rear - side) ||
                                     !surface.contains(rear + forward * options.wheelbase);
            if (sourceAt[i] >= route.paintedStartDistance + front - options.mainAxleAft && offPavement) {
                if (airport.pavementCoverageComplete && !options.ignorePavementLimits)
                    unsafe = true;
                else {
                    route.turnClearanceKnown = false;
                    route.pavementRisk[i] = true;
                }
            }
            if (unsafe)
                failed.push_back(i);
        }
        // The cockpit stop must also leave the fuselage aligned with the stand/runway.
        if ((route.ramp || route.runway) && std::abs(wrap180(yaw.back() - route.finalHeading)) > 1.5) {
            for (std::size_t i = 1; i < painted.size(); ++i)
                if (sourceAt.back() - sourceAt[i] < 5 * front && length(outset[i] - painted[i]) > .25)
                    failed.push_back(i);
            if (failed.empty())
                throw std::runtime_error("Insufficient final straight distance for cockpit alignment");
        }
        return failed;
    };
    // Only expand near failed wheel/steering samples; otherwise keep the cockpit on paint.
    for (int attempt = 0;; ++attempt) {
        auto failed = violations();
        if (route.maximumOversteer > (options.allowOversteer ? options.maxOversteer : 0) + .01)
            throw std::runtime_error("Cockpit oversteer exceeds configured outward allowance");
        if (failed.empty())
            break;
        if (!options.allowOversteer)
            throw std::runtime_error("Cockpit turn needs oversteer but it is disabled");
        if (attempt == 20)
            throw std::runtime_error("No local cockpit oversteer satisfies steering and pavement limits at " +
                                     std::to_string(sourceAt[failed.front()]) + " m");
        auto previous = weights;
        for (auto index : failed) {
            const double extent = 2 * front;
            auto begin = std::lower_bound(sourceAt.begin(), sourceAt.end(), sourceAt[index] - extent);
            for (std::size_t k = begin - sourceAt.begin();
                 k < weights.size() && sourceAt[k] < sourceAt[index] + extent; ++k) {
                double t = std::clamp(1 - std::abs(sourceAt[k] - sourceAt[index]) / extent, 0., 1.);
                double smooth = t * t * (3 - 2 * t);
                weights[k] = std::max(weights[k], std::min(1., previous[index] + .1) * smooth);
            }
        }
    }
    const auto changedAt = distances(route.points);
    auto remap = [&](std::vector<RouteMarker> &markers) {
        for (auto &marker : markers) {
            auto next =
                std::lower_bound(sourceAt.begin() + 1, sourceAt.end(), std::min(marker.distance, at.back()));
            auto i = std::min<std::size_t>(next - sourceAt.begin(), sourceAt.size() - 1);
            double fraction =
                (marker.distance - sourceAt[i - 1]) / std::max(.001, sourceAt[i] - sourceAt[i - 1]);
            marker.distance =
                changedAt[i - 1] + std::clamp(fraction, 0., 1.) * (changedAt[i] - changedAt[i - 1]);
        }
    };
    remap(route.nodesAlongRoute);
    remap(route.taxiwaysAlongRoute);
    route.paintedStartDistance = std::max(0., route.paintedStartDistance - prefix);
    route.length = changedAt.back();
}
void buildReferencePaths(Route &route, const RouteOptions &options) {
    route.cockpitPath.clear();
    route.mainAxlePath.clear();
    if (route.requiresPushback || route.points.size() < 2)
        return;
    const auto at = distances(route.points);
    const double front = options.wheelbase + options.cockpitAheadNose;
    const auto yaw =
        route.cockpitGuidance ? sweptHeadings(route, front, options.mainAxleAft) : std::vector<double>{};
    double prefix = 0, nearest = 1e30;
    if (route.cockpitGuidance)
        for (std::size_t i = 1; i < route.points.size() && at[i - 1] < front + 50; ++i) {
            auto snap = onSegment(direction(route.initialHeading) * (front - options.mainAxleAft),
                                  route.points[i - 1], route.points[i]);
            if (snap.distance < nearest) {
                nearest = snap.distance;
                prefix = at[i - 1] + snap.t * (at[i] - at[i - 1]);
            }
        }
    route.cockpitPath.reserve(route.points.size() + 1);
    route.mainAxlePath.reserve(route.points.size() + 1);
    route.cockpitPath.push_back({direction(route.initialHeading) * (front - options.mainAxleAft), 0, false});
    route.mainAxlePath.push_back({direction(route.initialHeading) * -options.mainAxleAft, 0, false});
    for (std::size_t i = 0; i < route.points.size(); ++i) {
        checkPlanning(options);
        if (route.cockpitGuidance && at[i] < prefix)
            continue;
        const auto point = route.points[i];
        const auto forward = route.cockpitGuidance ? direction(yaw[i]) : curveAt(route.points, at, i).tangent;
        const bool risk = i < route.pavementRisk.size() && route.pavementRisk[i];
        route.cockpitPath.push_back({route.cockpitGuidance ? point : point + forward * front, at[i], risk});
        route.mainAxlePath.push_back({route.cockpitGuidance ? point - forward * front : point, at[i], risk});
    }
}
} // namespace autotaxi
