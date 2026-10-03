#include "TaxiController.h"
#include <algorithm>
#include <stdexcept>
namespace autotaxi {
Vec2 mainAxlePosition(GeoPoint origin, const ControllerConfig &config, const AircraftState &state) {
    return project(origin, state.position) - direction(state.trueHeading) * config.mainAxleAft;
}
Vec2 cockpitPosition(GeoPoint origin, const ControllerConfig &config, const AircraftState &state) {
    return mainAxlePosition(origin, config, state) +
           direction(state.trueHeading) * (config.wheelbase + config.cockpitAheadNose);
}
Vec2 trackingPosition(const Route &route, const ControllerConfig &config, const AircraftState &state) {
    return route.cockpitGuidance ? cockpitPosition(route.origin, config, state)
                                 : mainAxlePosition(route.origin, config, state);
}
double routeSpeedForBend(const Route &route, const ControllerConfig &config, std::size_t segment,
                         double remaining, double requestedSteer, double bend, double oversteerAhead) {
    double turn = std::max(bend, std::abs(requestedSteer));
    double speed = config.taxiSpeed;
    if (turn > 10)
        speed = std::min(speed, std::max(config.turnSpeed, config.taxiSpeed * (1 - turn / 75)));
    if (route.runway && segment + 3 >= route.points.size())
        speed = std::min(speed, 2.0);
    if (route.oversteerOffsets.size() == route.points.size() &&
        std::max(route.oversteerOffsets[segment], route.oversteerOffsets[segment + 1]) > .25)
        speed = std::min(speed, config.turnSpeed);
    speed = std::min(speed, std::sqrt(config.turnSpeed * config.turnSpeed +
                                      2 * config.deceleration * std::max(0., oversteerAhead - 15)));
    const double stopMargin = route.ramp ? .05 : 2.0;
    return std::min(speed, std::sqrt(2 * config.deceleration * std::max(0.0, remaining - stopMargin)));
}
double trackingSteerDegrees(const Route &route, const ControllerConfig &config, Vec2 delta, Vec2 forward) {
    double right = delta.x * forward.y - delta.y * forward.x;
    double front = config.wheelbase + config.cockpitAheadNose;
    return route.cockpitGuidance
               ? std::atan2(config.wheelbase * right, front * std::max(1.0, dot(delta, forward))) / rad
               : std::atan2(2 * config.wheelbase * right, std::max(1.0, dot(delta, delta))) / rad;
}
double routeSpeedLimit(const Route &route, const ControllerConfig &config, std::size_t segment, Vec2 position,
                       double remaining, double requestedSteer) {
    double bend = 0, ahead = length(route.points[segment + 1] - position), oversteerAhead = 1e30;
    double horizon = std::max(70., config.taxiSpeed * config.taxiSpeed / (2 * config.deceleration) + 25);
    for (std::size_t i = segment + 1; i + 1 < route.points.size() && ahead < horizon; ++i) {
        Vec2 incoming = route.points[i] - route.points[i - 1],
             outgoing = route.points[i + 1] - route.points[i];
        bend = std::max(bend, std::abs(wrap180(heading(outgoing) - heading(incoming))));
        if (route.oversteerOffsets.size() == route.points.size() && route.oversteerOffsets[i] > .25)
            oversteerAhead = std::min(oversteerAhead, ahead);
        ahead += length(outgoing);
    }
    return routeSpeedForBend(route, config, segment, remaining, requestedSteer, bend, oversteerAhead);
}
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
    case TaxiPhase::Hold:
        return "Emergency brake";
    }
    return "?";
}
bool TaxiController::active() const {
    return phase_ == TaxiPhase::Taxi || phase_ == TaxiPhase::Align || phase_ == TaxiPhase::Braking ||
           phase_ == TaxiPhase::Hold;
}
void TaxiController::stop() {
    phase_ = TaxiPhase::Idle;
    steer_ = integral_ = settled_ = 0;
    emergency_ = false;
    speedInitialized_ = false;
    apronSpeedCap_ = -1;
}
void TaxiController::setTaxiSpeed(double speed) {
    if (std::isfinite(speed))
        config_.taxiSpeed = std::clamp(speed, .5, config_.maxTaxiSpeed);
}
void TaxiController::seekProgress(double progress) {
    if (!std::isfinite(progress))
        return;
    progress = std::clamp(progress, 0., route_.length);
    segment_ = 0;
    double distance = 0;
    while (segment_ + 1 < segmentLengths_.size() && distance + segmentLengths_[segment_] <= progress) {
        distance += segmentLengths_[segment_];
        ++segment_;
    }
    trackingInitialized_ = true;
}
void TaxiController::setEmergencyBrake(bool hold) {
    if (!active())
        return;
    emergency_ = hold;
    if (!hold && phase_ == TaxiPhase::Hold)
        phase_ = TaxiPhase::Taxi;
}
void TaxiController::start(Route route, ControllerConfig config) {
    if (route.requiresPushback)
        throw std::runtime_error("Pushback must finish before forward taxi");
    if (route.points.size() < 2 || config.wheelbase < 1)
        throw std::runtime_error("Invalid route or wheelbase");
    route_ = std::move(route);
    const auto count = route_.points.size();
    segmentLengths_.resize(count - 1);
    remainingAtPoint_.assign(count, 0);
    bends_.assign(count, 0);
    double previousHeading = 0;
    for (std::size_t i = 0; i + 1 < count; ++i) {
        const auto delta = route_.points[i + 1] - route_.points[i];
        segmentLengths_[i] = length(delta);
        const double currentHeading = heading(delta);
        if (i > 0)
            bends_[i] = std::abs(wrap180(currentHeading - previousHeading));
        previousHeading = currentHeading;
    }
    for (std::size_t i = count - 1; i > 0; --i)
        remainingAtPoint_[i - 1] = remainingAtPoint_[i] + segmentLengths_[i - 1];
    config_ = config;
    segment_ = 0;
    trackingInitialized_ = false;
    emergency_ = false;
    steer_ = integral_ = settled_ = 0;
    speedInitialized_ = false;
    apronSpeedCap_ = -1;
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
    if (s.speed > config_.maxTaxiSpeed + 3 * .514444 && !emergency_)
        return fault("Groundspeed exceeds configured limit plus 3 kt");
    Vec2 forward = direction(s.trueHeading);
    const double referenceAhead = route_.cockpitGuidance ? config_.wheelbase + config_.cockpitAheadNose : 0;
    Vec2 p = trackingPosition(route_, config_, s);
    auto projection = onSegment(p, route_.points[segment_], route_.points[segment_ + 1]);
    if (!trackingInitialized_) {
        // The cockpit can already be beyond the CG-to-network prefix after a tow.
        double walked = 0;
        for (std::size_t i = 0; i + 1 < route_.points.size() && walked <= referenceAhead + 10; ++i) {
            auto candidate = onSegment(p, route_.points[i], route_.points[i + 1]);
            if (candidate.distance + .01 < projection.distance) {
                segment_ = i;
                projection = candidate;
            }
            walked += segmentLengths_[i];
        }
        trackingInitialized_ = true;
    }
    while (segment_ + 2 < route_.points.size()) {
        auto next = onSegment(p, route_.points[segment_ + 1], route_.points[segment_ + 2]);
        if (projection.t < 0.98 && !(projection.t > 0.65 && next.distance + 0.5 < projection.distance))
            break;
        ++segment_;
        projection = next;
    }
    out.crossTrack = projection.distance;
    if (route_.oversteerOffsets.size() == route_.points.size())
        out.oversteerOffset = route_.oversteerOffsets[segment_] * (1 - projection.t) +
                              route_.oversteerOffsets[segment_ + 1] * projection.t;
    if (out.crossTrack > config_.maxCrossTrack && !emergency_)
        return fault("Cross-track limit exceeded");
    const double segmentRemaining = length(route_.points[segment_ + 1] - projection.point);
    out.remaining = segmentRemaining + remainingAtPoint_[segment_ + 1];
    out.progress = std::max(0.0, route_.length - out.remaining);
    if (emergency_) {
        phase_ = out.phase = TaxiPhase::Hold;
        out.brake = config_.emergencyBrake;
        out.steerDegrees = steer_;
        return out;
    }
    bool last = segment_ + 2 == route_.points.size();
    Vec2 end = route_.points.back();
    double endDistance = route_.ramp ? length(route_.cockpitStop - cockpitPosition(route_.origin, config_, s))
                                     : length(end - p);
    double headingError = std::abs(wrap180(route_.finalHeading - s.trueHeading));
    if (last && out.remaining < 0.2 && endDistance > 6)
        return fault("Passed endpoint without stopping");
    if (last && out.remaining < (route_.ramp ? .3 : 2.5) && endDistance < (route_.ramp ? 1.0 : 4)) {
        if ((route_.runway || route_.ramp) && (headingError > 2.0 || out.crossTrack > 2.0))
            return fault("Destination alignment outside tolerance");
        phase_ = TaxiPhase::Braking;
        out.brake = config_.maxBrake;
        out.throttle = 0;
        settled_ = s.speed < 0.12 ? settled_ + dt : 0;
        if (settled_ > 1.0)
            phase_ = TaxiPhase::Complete;
        out.phase = phase_;
        out.steerDegrees = 0;
        return out;
    }
    double lookahead = std::clamp(10.0 + s.speed * 2.5, 12.0, 25.0);
    if (route_.cockpitGuidance)
        lookahead = std::clamp(5.0 + s.speed, 6.0, 12.0);
    if (route_.mainAxleOversteer) {
        bool angularJoin = false;
        double ahead = segmentRemaining;
        for (std::size_t i = segment_ + 1; i + 1 < route_.points.size() && ahead < 40; ++i) {
            angularJoin = angularJoin || bends_[i] > 10;
            ahead += segmentLengths_[i];
        }
        if (!angularJoin)
            lookahead = std::clamp(5.0 + s.speed, 6.0, 12.0);
    }
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
    double requested = trackingSteerDegrees(route_, config_, delta, forward);
    double limit = config_.maxSteer - (config_.maxSteer - std::min(35., config_.maxSteer)) *
                                          std::clamp((s.speed - 3.) / 2., 0., 1.);
    requested = std::clamp(requested, -limit, limit);
    steer_ += std::clamp(requested - steer_, -config_.steerRate * dt, config_.steerRate * dt);
    out.steerDegrees = steer_;
    if (route_.runway && segment_ + 3 >= route_.points.size()) {
        phase_ = TaxiPhase::Align;
    }
    double bend = 0, ahead = segmentRemaining, oversteerAhead = 1e30;
    double horizon = std::max(70., config_.taxiSpeed * config_.taxiSpeed / (2 * config_.deceleration) + 25);
    for (std::size_t i = segment_ + 1; i + 1 < route_.points.size() && ahead < horizon; ++i) {
        bend = std::max(bend, bends_[i]);
        if (route_.oversteerOffsets.size() == route_.points.size() && route_.oversteerOffsets[i] > .25)
            oversteerAhead = std::min(oversteerAhead, ahead);
        ahead += segmentLengths_[i];
    }
    double targetSpeed =
        routeSpeedForBend(route_, config_, segment_, out.remaining, requested, bend, oversteerAhead);
    if (route_.ramp && (out.remaining <= config_.apronApproachDistance ||
                        (route_.apronArrivalDistance >= 0 && out.progress >= route_.apronArrivalDistance))) {
        if (apronSpeedCap_ < 0)
            apronSpeedCap_ = speedInitialized_ ? targetSpeed_ : config_.apronSpeed;
        targetSpeed = std::min({targetSpeed, config_.apronSpeed, apronSpeedCap_});
    }
    if (!speedInitialized_) {
        targetSpeed_ = targetSpeed;
        speedInitialized_ = true;
    }
    // Changes in steering demand must not step the speed setpoint. The stopping envelope stays hard.
    targetSpeed_ += std::clamp(targetSpeed - targetSpeed_, -config_.deceleration * dt,
                               std::max(.1, config_.acceleration) * dt);
    const double stopMargin = route_.ramp ? .05 : 2.;
    targetSpeed = std::min(targetSpeed_,
                           std::sqrt(2 * config_.deceleration * std::max(0., out.remaining - stopMargin)));
    targetSpeed = std::min(targetSpeed, config_.taxiSpeed);
    if (apronSpeedCap_ >= 0) {
        targetSpeed = std::min(targetSpeed, apronSpeedCap_);
        apronSpeedCap_ = targetSpeed;
    }
    out.targetSpeed = targetSpeed;
    const double overspeed = s.speed - targetSpeed;
    double demand = overspeed * .6 + integral_;
    if (overspeed < 0 || demand < config_.maxBrake)
        integral_ = std::clamp(integral_ + overspeed * .15 * dt, 0.0, config_.maxBrake);
    // Unwind leftover braking promptly after a slowdown instead of holding it through a stop.
    if (overspeed < -.3)
        integral_ *= std::exp(-dt / .5);
    out.brake = std::clamp(overspeed * .6 + integral_, 0.0, config_.maxBrake);
    out.phase = phase_;
    return out;
}
} // namespace autotaxi
