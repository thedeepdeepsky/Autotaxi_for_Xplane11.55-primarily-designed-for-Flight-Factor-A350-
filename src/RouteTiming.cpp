#include "RouteTiming.h"
#include <algorithm>
#include <chrono>
namespace autotaxi {
std::vector<RoutePathPoint> smoothCockpitPrediction(const std::vector<RoutePathPoint> &previous,
                                                    std::vector<RoutePathPoint> next, double elapsed,
                                                    bool braking) {
    if (next.size() < 2)
        return {};
    for (std::size_t i = 0; i < next.size(); ++i) {
        if (!std::isfinite(next[i].position.x) || !std::isfinite(next[i].position.y) ||
            !std::isfinite(next[i].distance))
            return {};
        if (i)
            next[i].distance = std::max(next[i].distance, next[i - 1].distance);
    }
    const auto raw = next;
    // Bound spatial filtering so real bends and docking targets remain recognizable.
    for (std::size_t i = 1; i + 1 < next.size(); ++i) {
        double left = length(raw[i].position - raw[i - 1].position);
        double right = length(raw[i + 1].position - raw[i].position);
        if (left + right < .001)
            continue;
        Vec2 chord =
            raw[i - 1].position + (raw[i + 1].position - raw[i - 1].position) * (left / (left + right));
        Vec2 correction = (chord - raw[i].position) * (braking ? .35 : .15);
        double limit = braking ? .6 : .2;
        if (length(correction) > limit)
            correction = correction * (limit / length(correction));
        next[i].position = raw[i].position + correction;
        if (length(correction) > .001)
            next[i].pavementRisk = raw[i - 1].pavementRisk || raw[i].pavementRisk || raw[i + 1].pavementRisk;
    }
    if (previous.size() < 2 || !std::isfinite(elapsed) || elapsed < 0)
        return next;
    for (std::size_t i = 0; i < previous.size(); ++i)
        if (!std::isfinite(previous[i].position.x) || !std::isfinite(previous[i].position.y) ||
            !std::isfinite(previous[i].distance) || (i && previous[i].distance < previous[i - 1].distance))
            return next;
    const double weight = 1 - std::exp(-elapsed / (braking ? 2.5 : 1.));
    const double movementLimit = (braking ? 1.5 : 3.) * elapsed;
    std::size_t upper = 1;
    // Match by route progress, not sample index: braking changes spacing and sample count.
    for (std::size_t i = 1; i + 1 < next.size(); ++i) {
        const double distance = next[i].distance;
        if (distance < previous.front().distance || distance > previous.back().distance)
            continue;
        while (upper + 1 < previous.size() && previous[upper].distance < distance)
            ++upper;
        const auto &a = previous[upper - 1], &b = previous[upper];
        double fraction =
            std::clamp((distance - a.distance) / std::max(.001, b.distance - a.distance), 0., 1.);
        Vec2 old = a.position + (b.position - a.position) * fraction;
        Vec2 correction = (next[i].position - old) * weight;
        if (length(correction) > movementLimit)
            correction = correction * (movementLimit / length(correction));
        double fade = std::clamp((distance - next.front().distance) / 10, 0., 1.) *
                      std::clamp((next.back().distance - distance) / 10, 0., 1.);
        next[i].position = next[i].position + (old + correction - next[i].position) * fade;
    }
    return next;
}
RouteTiming stabilizeTiming(const RouteTiming &previous, RouteTiming next, double elapsed) {
    if (previous.totalSeconds < 0 || next.totalSeconds < 0 || previous.samples.empty() ||
        next.samples.empty() || elapsed <= 0)
        return next;
    const double originTime = previous.secondsTo(next.samples.front().distance);
    const double oldRemaining = std::max(0., previous.totalSeconds - originTime);
    double weight = 1 - std::exp(-elapsed / 5.);
    double correction = std::abs(next.totalSeconds - oldRemaining);
    if (correction > 0)
        weight = std::min(weight, 2 * elapsed / correction);
    auto oldSample =
        std::lower_bound(previous.samples.begin(), previous.samples.end(), next.samples.front().distance,
                         [](const TimingSample &s, double d) { return s.distance < d; });
    // Both timelines are ordered. Merge linearly to keep publication cheap on the simulator thread.
    for (auto &sample : next.samples) {
        while (oldSample != previous.samples.end() && oldSample->distance < sample.distance)
            ++oldSample;
        double at = 0;
        if (oldSample == previous.samples.end())
            at = previous.samples.back().seconds;
        else if (oldSample != previous.samples.begin()) {
            const auto &before = *(oldSample - 1);
            double fraction = (sample.distance - before.distance) / (oldSample->distance - before.distance);
            at = before.seconds + fraction * (oldSample->seconds - before.seconds);
        }
        double oldSeconds = std::max(0., at - originTime);
        sample.seconds =
            oldSeconds + std::clamp(weight * (sample.seconds - oldSeconds), -2 * elapsed, 2 * elapsed);
    }
    next.totalSeconds = oldRemaining + weight * (next.totalSeconds - oldRemaining);
    return next;
}
double RouteTiming::secondsTo(double routeDistance) const {
    if (samples.empty() || !std::isfinite(routeDistance))
        return -1;
    if (routeDistance <= samples.front().distance)
        return 0;
    auto next = std::lower_bound(samples.begin(), samples.end(), routeDistance,
                                 [](const TimingSample &s, double d) { return s.distance < d; });
    if (next == samples.end())
        return samples.back().seconds;
    const auto &before = *(next - 1);
    double fraction = (routeDistance - before.distance) / (next->distance - before.distance);
    return before.seconds + fraction * (next->seconds - before.seconds);
}
void TaxiMotionEstimator::reset(const ControllerConfig &config) {
    model_ = {config.acceleration, config.brakeAuthority};
    initialized_ = false;
    hasWindow_ = false;
    elapsed_ = brakeTime_ = 0;
}
void TaxiMotionEstimator::observe(double speed, double appliedBrake, double dt) {
    if (!std::isfinite(speed) || speed < 0 || !std::isfinite(appliedBrake) || !std::isfinite(dt) || dt <= 0 ||
        dt > .2) {
        initialized_ = false;
        hasWindow_ = false;
        elapsed_ = brakeTime_ = 0;
        return;
    }
    if (!initialized_) {
        initialized_ = true;
        speed_ = speed;
        return;
    }
    elapsed_ += dt;
    brakeTime_ += std::clamp(appliedBrake, 0., 1.) * dt;
    if (elapsed_ < .5)
        return;
    double acceleration = (speed - speed_) / elapsed_, brake = brakeTime_ / elapsed_;
    double weight = 1 - std::exp(-elapsed_ / 3.0);
    if (std::abs(acceleration) <= 5) {
        if (hasWindow_ && brake - previousBrake_ > .08 && speed_ > .3 && speed > .3) {
            // Brake changes identify its response without assuming the current engine thrust.
            double authority = (previousAcceleration_ - acceleration) / (brake - previousBrake_);
            if (authority > .1 && authority <= 8)
                model_.brakeAuthority +=
                    (1 - std::exp(-elapsed_ / 1.0)) * (authority - model_.brakeAuthority);
        }
        if (brake < .025 || (speed_ > .3 && speed > .3)) {
            double free = std::clamp(acceleration + model_.brakeAuthority * brake, -.5, 3.);
            model_.freeAcceleration += weight * (free - model_.freeAcceleration);
        }
        previousAcceleration_ = acceleration;
        previousBrake_ = brake;
        hasWindow_ = true;
    } else {
        hasWindow_ = false;
    }
    elapsed_ = brakeTime_ = 0;
    speed_ = speed;
}
RouteTiming forecastTaxiTiming(TaxiController controller, AircraftState state, TaxiMotionModel model,
                               const std::atomic<bool> *cancelled) {
    RouteTiming result;
    if (!controller.active() || controller.emergencyBrake() || !valid(state.position) ||
        !std::isfinite(state.speed) || state.speed < 0 || !std::isfinite(state.trueHeading) ||
        !std::isfinite(model.freeAcceleration) || !std::isfinite(model.brakeAuthority) ||
        model.brakeAuthority <= 0)
        return result;
    const auto config = controller.config();
    const auto origin = controller.route().origin;
    Vec2 axle = project(origin, state.position) - direction(state.trueHeading) * config.mainAxleAft;
    constexpr double dt = .1;
    auto output = controller.update(state, .001);
    result.samples.reserve(static_cast<std::size_t>(std::min(40000., controller.route().length * 4 + 64)));
    result.samples.push_back({output.progress, state.speed, 0});
    auto recordCockpit = [&](bool final) {
        auto point = cockpitPosition(origin, config, state);
        bool risk = controller.segment() + 1 < controller.route().pavementRisk.size() &&
                    controller.route().pavementRisk[controller.segment() + 1];
        if (final || result.cockpitPath.empty() || length(point - result.cockpitPath.back().position) >= 2 ||
            risk != result.cockpitPath.back().pavementRisk)
            result.cockpitPath.push_back({point, output.progress, risk});
    };
    recordCockpit(false);
    double stationary = 0;
    // Simulate the real feedback controller and vehicle response, not an ideal speed envelope.
    for (int step = 0; step < 180000; ++step) {
        if ((step % 128 == 0 && cancelled && cancelled->load(std::memory_order_relaxed)) ||
            output.phase == TaxiPhase::Fault || output.phase == TaxiPhase::Hold)
            return {};
        if (output.phase == TaxiPhase::Complete) {
            result.totalSeconds = step * dt;
            recordCockpit(true);
            return result;
        }
        double speed =
            std::max(0., state.speed + (model.freeAcceleration - model.brakeAuthority * output.brake) * dt);
        double averageSpeed = (state.speed + speed) * .5;
        double yawChange = averageSpeed / config.wheelbase * std::tan(output.steerDegrees * rad) * dt / rad;
        axle = axle + direction(state.trueHeading + yawChange * .5) * (averageSpeed * dt);
        state.trueHeading = wrap180(state.trueHeading + yawChange);
        state.speed = speed;
        state.position = unproject(origin, axle + direction(state.trueHeading) * config.mainAxleAft);
        output = controller.update(state, dt);
        recordCockpit(false);
        if (output.progress > result.samples.back().distance + 1e-6)
            result.samples.push_back({output.progress, state.speed, (step + 1) * dt});
        stationary = state.speed < .02 && output.phase != TaxiPhase::Braking ? stationary + dt : 0;
        if (stationary > 5)
            return {};
    }
    return {};
}
RouteTiming estimateRouteTiming(const Route &route, double progress, double actualSpeed,
                                const ControllerConfig &config) {
    if (route.requiresPushback || route.points.size() < 2 || !std::isfinite(progress) ||
        !std::isfinite(actualSpeed) || !std::isfinite(config.taxiSpeed) || config.taxiSpeed <= 0)
        return {};
    Route copy = route;
    copy.length = 0;
    for (std::size_t i = 1; i < copy.points.size(); ++i)
        copy.length += length(copy.points[i] - copy.points[i - 1]);
    progress = std::clamp(progress, 0., copy.length);
    std::size_t segment = 0;
    double remaining = progress;
    while (segment + 2 < copy.points.size() &&
           remaining >= length(copy.points[segment + 1] - copy.points[segment])) {
        remaining -= length(copy.points[segment + 1] - copy.points[segment]);
        ++segment;
    }
    Vec2 tangent = copy.points[segment + 1] - copy.points[segment];
    if (length(tangent) < .01)
        return {};
    tangent = tangent * (1 / length(tangent));
    Vec2 reference = copy.points[segment] + tangent * remaining;
    double ahead = copy.cockpitGuidance ? config.wheelbase + config.cockpitAheadNose : 0;
    AircraftState state{unproject(copy.origin, reference + tangent * (config.mainAxleAft - ahead)),
                        heading(tangent), std::max(0., actualSpeed), true};
    TaxiController controller;
    controller.start(std::move(copy), config);
    controller.seekProgress(progress);
    return forecastTaxiTiming(std::move(controller), state, {config.acceleration, config.brakeAuthority});
}
TimingWorker::~TimingWorker() {
    cancel();
}
void TimingWorker::cancel() {
    if (cancelled_)
        cancelled_->store(true, std::memory_order_relaxed);
}
void TimingWorker::submit(TaxiController controller, AircraftState state, TaxiMotionModel model) {
    if (worker_.valid())
        return;
    cancelled_ = std::make_shared<std::atomic<bool>>(false);
    worker_ = std::async(std::launch::async,
                         [controller = std::move(controller), state, model, stop = cancelled_]() mutable {
                             return forecastTaxiTiming(std::move(controller), state, model, stop.get());
                         });
}
std::optional<RouteTiming> TimingWorker::poll() {
    if (!worker_.valid() || worker_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        return {};
    auto result = worker_.get();
    if (cancelled_->load(std::memory_order_relaxed))
        return {};
    return result;
}
} // namespace autotaxi
