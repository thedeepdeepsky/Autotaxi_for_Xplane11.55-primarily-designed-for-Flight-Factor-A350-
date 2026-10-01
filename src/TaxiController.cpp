#include "TaxiController.h"
#include <algorithm>
#include <stdexcept>
namespace autotaxi {
const char *phaseName(TaxiPhase p) {
    switch (p) {
    case TaxiPhase::Idle:
        return "Idle";
    case TaxiPhase::Taxi:
        return "Taxi";
    case TaxiPhase::Align:
        return "Align";
    case TaxiPhase::Braking:
        return "Braking";
    case TaxiPhase::Complete:
        return "Complete";
    case TaxiPhase::Fault:
        return "Fault";
    }
    return "?";
}
bool TaxiController::active() const {
    return phase_ == TaxiPhase::Taxi || phase_ == TaxiPhase::Align || phase_ == TaxiPhase::Braking;
}
void TaxiController::stop() {
    phase_ = TaxiPhase::Idle;
    steer_ = integral_ = settled_ = 0;
}
void TaxiController::start(Route route, ControllerConfig config) {
    if (route.requiresPushback)
        throw std::runtime_error("Pushback must finish before forward taxi");
    if (route.points.size() < 2 || config.wheelbase < 5)
        throw std::runtime_error("Invalid route or wheelbase");
    route_ = std::move(route);
    config_ = config;
    segment_ = 0;
    steer_ = integral_ = settled_ = 0;
    phase_ = TaxiPhase::Taxi;
}
ControlOutput TaxiController::update(const AircraftState &s, double dt) {
    ControlOutput out;
    out.phase = phase_;
    if (!active())
        return out;
    dt = std::clamp(dt, 0.001, 0.1);
    auto fault = [&](const std::string &reason) {
        phase_ = TaxiPhase::Fault;
        out.phase = phase_;
        out.reason = reason;
        out.brake = 0.8;
        out.throttle = 0;
        return out;
    };
    if (!s.onGround)
        return fault("Aircraft is not on ground");
    if (!valid(s.position) || !std::isfinite(s.speed) || !std::isfinite(s.trueHeading))
        return fault("Invalid position telemetry");
    if (s.speed > 12)
        return fault("Groundspeed exceeds 23 kt");
    Vec2 forward = direction(s.trueHeading);
    Vec2 p = project(route_.origin, s.position) - forward * config_.mainAxleAft;
    auto projection = onSegment(p, route_.points[segment_], route_.points[segment_ + 1]);
    while (segment_ + 2 < route_.points.size()) {
        auto next = onSegment(p, route_.points[segment_ + 1], route_.points[segment_ + 2]);
        if (projection.t < 0.98 && !(projection.t > 0.65 && next.distance + 0.5 < projection.distance))
            break;
        ++segment_;
        projection = next;
    }
    out.crossTrack = projection.distance;
    if (out.crossTrack > config_.maxCrossTrack)
        return fault("Cross-track limit exceeded");
    out.remaining = length(route_.points[segment_ + 1] - projection.point);
    for (std::size_t i = segment_ + 2; i < route_.points.size(); ++i)
        out.remaining += length(route_.points[i] - route_.points[i - 1]);
    out.progress = std::max(0.0, route_.length - out.remaining);
    bool last = segment_ + 2 == route_.points.size();
    Vec2 end = route_.points.back();
    double endDistance = length(end - p);
    double headingError = std::abs(wrap180(route_.finalHeading - s.trueHeading));
    if (last && out.remaining < 0.2 && endDistance > 6)
        return fault("Passed endpoint without stopping");
    if (last && out.remaining < 2.5 && endDistance < 4) {
        if (route_.runway && (headingError > 2.0 || out.crossTrack > 2.0))
            return fault("Runway alignment outside tolerance");
        phase_ = TaxiPhase::Braking;
        out.brake = 0.65;
        out.throttle = 0;
        settled_ = s.speed < 0.12 ? settled_ + dt : 0;
        if (settled_ > 1.0)
            phase_ = TaxiPhase::Complete;
        out.phase = phase_;
        out.steerDegrees = 0;
        return out;
    }
    double lookahead = std::clamp(10.0 + s.speed * 2.5, 12.0, 25.0);
    Vec2 target = projection.point;
    double left = lookahead;
    std::size_t cursor = segment_ + 1;
    while (cursor < route_.points.size()) {
        Vec2 diff = route_.points[cursor] - target;
        double len = length(diff);
        if (len >= left && len > 0) {
            target = target + diff * (left / len);
            break;
        }
        left -= len;
        target = route_.points[cursor++];
    }
    Vec2 delta = target - p;
    double d2 = dot(delta, delta);
    double right = delta.x * forward.y - delta.y * forward.x;
    double requested = std::atan2(2 * config_.wheelbase * right, std::max(1.0, d2)) / rad;
    double limit = s.speed > 4 ? 35 : config_.maxSteer;
    requested = std::clamp(requested, -limit, limit);
    steer_ += std::clamp(requested - steer_, -config_.steerRate * dt, config_.steerRate * dt);
    out.steerDegrees = steer_;
    // Anticipate upcoming bends before commanding thrust into a corner.
    double bend = 0, ahead = length(route_.points[segment_ + 1] - projection.point);
    for (std::size_t i = segment_ + 1; i + 1 < route_.points.size() && ahead < 70; ++i) {
        Vec2 incoming = route_.points[i] - route_.points[i - 1],
             outgoing = route_.points[i + 1] - route_.points[i];
        bend = std::max(bend, std::abs(wrap180(heading(outgoing) - heading(incoming))));
        ahead += length(outgoing);
    }
    double turn = std::max(bend, std::abs(requested));
    double targetSpeed = config_.taxiSpeed;
    if (turn > 10)
        targetSpeed = std::min(targetSpeed, std::max(config_.turnSpeed, config_.taxiSpeed * (1 - turn / 75)));
    if (route_.runway && segment_ + 3 >= route_.points.size()) {
        phase_ = TaxiPhase::Align;
        targetSpeed = std::min(targetSpeed, 2.0);
    }
    targetSpeed = std::min(targetSpeed, std::sqrt(2 * 0.45 * std::max(0.0, out.remaining - 2)));
    out.targetSpeed = targetSpeed;
    double error = targetSpeed - s.speed;
    integral_ = std::clamp(integral_ + error * dt, -2.0, 2.0);
    if (error < -0.15) {
        out.brake = std::clamp(-error * 0.3, 0.0, 0.75);
        integral_ = 0;
    } else if (error > 0.1)
        out.throttle = std::clamp(0.025 + 0.04 * error + 0.012 * integral_, 0.0, config_.maxThrottle);
    out.phase = phase_;
    return out;
}
} // namespace autotaxi
