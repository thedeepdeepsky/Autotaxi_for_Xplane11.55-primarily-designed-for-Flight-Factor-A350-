#include "RouteTiming.h"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace autotaxi;
namespace {
void check(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
Route fixture(bool curved) {
    Route r;
    r.origin = {31, 121};
    if (curved) {
        for (int i = 0; i <= 60; ++i)
            r.points.push_back({0, i * 2.});
        for (int i = 1; i <= 80; ++i) {
            double theta = i * pi / 160;
            r.points.push_back({80 - 80 * std::cos(theta), 120 + 80 * std::sin(theta)});
        }
        r.points.push_back({400, 200});
        r.finalHeading = 90;
        r.runway = true;
    } else {
        r.points = {{0, 0}, {0, 750}};
    }
    for (std::size_t i = 1; i < r.points.size(); ++i)
        r.length += length(r.points[i] - r.points[i - 1]);
    return r;
}
void forecastAgainstDriving(bool curved, bool stand = false) {
    auto route = fixture(curved);
    ControllerConfig c;
    c.taxiSpeed = 20 * .514444;
    if (stand) {
        route.runway = false;
        route.ramp = true;
        route.cockpitStop =
            route.points.back() + direction(route.finalHeading) * (c.wheelbase + c.cockpitAheadNose);
    }
    TaxiController driver;
    driver.start(route, c);
    AircraftState state{route.origin, 0, 0, true};
    const auto prediction = forecastTaxiTiming(driver, state, {.39, 1.5});
    check(prediction.totalSeconds > 0, "Full-route prediction must reach a stop");
    check(prediction.cockpitPath.size() > 2 && length(prediction.cockpitPath.front().position -
                                                      cockpitPosition(route.origin, c, state)) < .001,
          "Trajectory prediction starts at the actual cockpit, not CG or axle");
    constexpr double dt = .05;
    Vec2 axle{0, -c.mainAxleAft};
    double elapsed = 0, nodeTime = -1, laterPrediction = -1, pathError = 0, arrivalTarget = -1;
    std::size_t predictedSegment = 0;
    for (int step = 0; step < 100000 && driver.active(); ++step) {
        const auto output = driver.update(state, dt);
        check(output.phase != TaxiPhase::Fault, "Independent control drive faulted");
        if (stand && output.remaining <= c.apronApproachDistance) {
            if (arrivalTarget >= 0) {
                check(output.targetSpeed <= arrivalTarget + 1e-6,
                      "Stand arrival target cannot accelerate after the bend");
                if (output.phase == TaxiPhase::Taxi && output.remaining > 10)
                    check(arrivalTarget - output.targetSpeed <= c.deceleration * dt + 1e-5,
                          "Entry to the apron cap must preserve smooth setpoint deceleration");
            }
            arrivalTarget = output.targetSpeed;
        }
        if (nodeTime < 0 && output.progress >= 100)
            nodeTime = elapsed;
        if (laterPrediction < 0 && elapsed >= 30) {
            const auto live = forecastTaxiTiming(driver, state, {.39, 1.5});
            check(live.totalSeconds > 0, "A live controller snapshot must remain forecastable");
            laterPrediction = elapsed + live.totalSeconds;
        }
        if (output.phase == TaxiPhase::Complete)
            break;
        Vec2 cockpit = cockpitPosition(route.origin, c, state);
        auto closest = onSegment(cockpit, prediction.cockpitPath[predictedSegment].position,
                                 prediction.cockpitPath[predictedSegment + 1].position);
        while (predictedSegment + 2 < prediction.cockpitPath.size()) {
            auto next = onSegment(cockpit, prediction.cockpitPath[predictedSegment + 1].position,
                                  prediction.cockpitPath[predictedSegment + 2].position);
            if (next.distance >= closest.distance)
                break;
            closest = next;
            ++predictedSegment;
        }
        pathError = std::max(pathError, closest.distance);
        // Independent integration at twice the predictor's temporal resolution.
        state.speed = std::max(0., state.speed + (.39 - 1.5 * output.brake) * dt);
        state.trueHeading = wrap180(state.trueHeading + state.speed / c.wheelbase *
                                                            std::tan(output.steerDegrees * rad) * dt / rad);
        axle = axle + direction(state.trueHeading) * (state.speed * dt);
        state.position = unproject(route.origin, axle + direction(state.trueHeading) * c.mainAxleAft);
        elapsed += dt;
    }
    check(driver.phase() == TaxiPhase::Complete, "Independent control drive must stop");
    check(pathError < 1 &&
              length(prediction.cockpitPath.back().position - cockpitPosition(route.origin, c, state)) < 1,
          "Predicted cockpit movement must match independently integrated motion and stop");
    check(prediction.cockpitPath.size() < prediction.samples.size(),
          "Map path is sampled spatially instead of retaining every control step");
    if (stand)
        check(length(cockpitPosition(route.origin, c, state) - route.cockpitStop) < 1,
              "Axle-guided prediction and drive must stop the cockpit at the stand target");
    const double error = std::abs(prediction.totalSeconds - elapsed);
    check(error < std::max(2., elapsed * .02),
          "Predicted stop time differs from actual drive by more than 2 percent");
    check(std::abs(laterPrediction - elapsed) < std::max(2., elapsed * .02),
          "A live ETA must retain the existing brake integral and tracking state");
    check(std::abs(prediction.secondsTo(100) - nodeTime) < 1,
          "Node ETA must match the actual first passage, not the speed limit");
    std::cout << (stand    ? "Stand arrival"
                  : curved ? "Curved lineup"
                           : "750 m straight")
              << ": predicted " << prediction.totalSeconds << " s, driven " << elapsed << " s, error "
              << error << " s\n";
    driver.start(route, c);
    driver.setTaxiSpeed(5 * .514444);
    const auto slower = forecastTaxiTiming(driver, {route.origin, 0, 0, true}, {.39, 1.5});
    check(slower.totalSeconds > prediction.totalSeconds,
          "Live speed settings must change the full-route ETA");
    driver.setEmergencyBrake(true);
    check(forecastTaxiTiming(driver, {route.origin, 0, 0, true}, {.39, 1.5}).totalSeconds < 0,
          "An indefinite emergency hold cannot have a finite ETA");
}
void calibration() {
    TaxiMotionEstimator estimator;
    ControllerConfig c;
    estimator.reset(c);
    constexpr double dt = .05;
    double speed = 8;
    estimator.observe(speed, 0, dt);
    for (int i = 0; i < 600; ++i) {
        speed += .39 * dt;
        estimator.observe(speed, 0, dt);
    }
    check(std::abs(estimator.model().freeAcceleration - .39) < .005,
          "Measured free acceleration must replace the nominal 0.25 value");
    for (int i = 0; i < 600; ++i) {
        double brake = (i / 40) % 2 ? .25 : .5;
        speed += (.39 - 1.8 * brake) * dt;
        estimator.observe(speed, brake, dt);
    }
    check(std::abs(estimator.model().brakeAuthority - 1.8) < .02,
          "Brake response must adapt to the aircraft's observed deceleration");
    for (int i = 0; i < 300; ++i) {
        speed += .1 * dt;
        estimator.observe(speed, 0, dt);
    }
    check(std::abs(estimator.model().freeAcceleration - .1) < .005,
          "A manual thrust change must update the future acceleration model");
    for (int i = 0; i < 600; ++i) {
        const double thrust = i < 300 ? .1 : .65;
        speed += (thrust - 1.8 * .25) * dt;
        estimator.observe(speed, .25, dt);
    }
    check(std::abs(estimator.model().freeAcceleration - .65) < .015,
          "A thrust change during continuous brake speed control must also update ETA");
    TaxiController driver;
    auto route = fixture(false);
    driver.start(route, c);
    check(forecastTaxiTiming(driver, {route.origin, 0, 0, true}, {0, 1.5}).totalSeconds < 0,
          "A stopped aircraft without thrust must not show an invented finite ETA");
    std::atomic<bool> cancelled{true};
    check(forecastTaxiTiming(driver, {route.origin, 0, 0, true}, {.39, 1.5}, &cancelled).totalSeconds < 0,
          "Cancelled forecasts must return promptly");
    TimingWorker worker;
    worker.submit(driver, {route.origin, 0, 0, true}, {.39, 1.5});
    worker.cancel();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (worker.hasWork()) {
        check(!worker.poll(), "A cancelled background forecast must not publish stale ETA");
        check(std::chrono::steady_clock::now() < deadline, "Timing worker cancellation timed out");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
void stableRefresh() {
    RouteTiming previous{{{0, 5, 0}, {100, 5, 20}, {500, 5, 100}}, 101};
    RouteTiming incoming{{{5, 5, 0}, {100, 5, 26}, {500, 5, 106}}, 107};
    auto stable = stabilizeTiming(previous, incoming, 1);
    check(std::abs(stable.totalSeconds - 100) < 2,
          "Seven seconds of forecast noise must not jump directly into the displayed countdown");
    check(stable.samples.front().seconds == 0 && stable.secondsTo(100) < stable.secondsTo(500),
          "Blend future passage times at shared progress, preserving a monotone timeline");
    auto localChange = incoming;
    localChange.samples[1].seconds = 90;
    check(stabilizeTiming(previous, localChange, 1).secondsTo(100) <= 21,
          "A node ETA must also bound local forecast noise even when total ETA barely changes");
    check(stabilizeTiming(previous, {}, 1).totalSeconds < 0,
          "An invalid forecast must clear ETA instead of retaining false confidence");
    for (int i = 0; i < 40; ++i)
        previous = stabilizeTiming(previous, incoming, 1);
    check(std::abs(previous.totalSeconds - incoming.totalSeconds) < .02,
          "Sustained changes in vehicle response must converge to the new forecast");
}
void smoothPrediction() {
    std::vector<RoutePathPoint> old{{{0, 0}, 0, false}, {{0, 100}, 100, false}, {{0, 200}, 200, false}};
    std::vector<RoutePathPoint> incoming;
    for (int i = 0; i <= 10; ++i)
        incoming.push_back({{4, i * 20.}, i * 20., i == 5});
    auto normal = smoothCockpitPrediction(old, incoming, 1, false);
    auto braking = smoothCockpitPrediction(old, incoming, 1, true);
    check(braking[5].position.x > 0 && braking[5].position.x < normal[5].position.x &&
              braking[5].position.x <= 1.5 && normal[5].position.x <= 3,
          "Brake forecasts must limit visual movement more than normal refreshes");
    for (std::size_t i = 1; i + 1 < braking.size(); ++i)
        check(std::abs(braking[i].position.y - incoming[i].distance) < .001,
              "Different prediction sampling must blend at shared progress instead of shared index");
    check(length(braking.front().position - incoming.front().position) < .001 &&
              length(braking.back().position - incoming.back().position) < .001 && braking[5].pavementRisk,
          "Filtering must preserve the actual cockpit, predicted stop and current orange warnings");
    auto converged = braking;
    for (int i = 0; i < 30; ++i)
        converged = smoothCockpitPrediction(converged, incoming, 1, true);
    check(length(converged[5].position - incoming[5].position) < .001,
          "A sustained new trajectory must converge despite braking smoothing");
    std::vector<RoutePathPoint> teeth;
    for (int i = 0; i <= 40; ++i)
        teeth.push_back({{i == 0 || i == 40 ? 0. : i % 2 ? .8 : -.8, i * 2.}, i * 2., false});
    auto filtered = smoothCockpitPrediction({}, teeth, 1, true);
    for (std::size_t i = 5; i < 35; ++i)
        check(std::abs(filtered[i].position.x) < .4 &&
                  length(filtered[i].position - teeth[i].position) <= .600001,
              "Brake filtering must remove alternating teeth with bounded spatial displacement");
    auto corner = std::vector<RoutePathPoint>{{{0, 0}, 0, false}, {{0, 10}, 10, true}, {{10, 10}, 20, false}};
    auto smoothedCorner = smoothCockpitPrediction({}, corner, 1, true);
    check(length(smoothedCorner[1].position - corner[1].position) <= .600001 &&
              smoothedCorner[1].pavementRisk,
          "Spatial smoothing must not flatten a real turn or suppress its pavement warning");
    incoming[4].position.x = std::numeric_limits<double>::quiet_NaN();
    check(smoothCockpitPrediction(old, incoming, 1, true).empty(),
          "Invalid prediction coordinates must never reach the renderer");
    incoming = teeth;
    incoming[5].distance = incoming[4].distance - .1;
    auto ordered = smoothCockpitPrediction(filtered, incoming, 1, true);
    for (std::size_t i = 1; i < ordered.size(); ++i)
        check(ordered[i].distance >= ordered[i - 1].distance && std::isfinite(ordered[i].position.x),
              "Small progress reversals while braking must retain a finite ordered display path");
}
void continuousExit() {
    auto route = fixture(true);
    route.mainAxleOversteer = true;
    route.atcFallbackCount = 2;
    route.oversteerOffsets.assign(route.points.size(), 0);
    for (std::size_t i = 61; i <= 140; ++i)
        route.oversteerOffsets[i] = 1;
    ControllerConfig config;
    config.taxiSpeed = 20 * .514444;
    TaxiController controller;
    controller.start(route, config);
    AircraftState state{route.origin, 0, 0, true};
    Vec2 axle{0, -config.mainAxleAft};
    double worstCte = 0;
    int interiorStops = 0;
    for (int step = 0; step < 30000 && controller.active(); ++step) {
        auto output = controller.update(state, .05);
        check(output.phase != TaxiPhase::Fault, "Exit simulation faulted");
        if (output.progress > 20 && output.remaining > 20 && state.speed < .15)
            ++interiorStops;
        worstCte = std::max(worstCte, output.crossTrack);
        state.speed = std::max(0., state.speed + (.39 - 4 * output.brake) * .05);
        state.trueHeading = wrap180(state.trueHeading + state.speed / config.wheelbase *
                                                            std::tan(output.steerDegrees * rad) * .05 / rad);
        axle = axle + direction(state.trueHeading) * (state.speed * .05);
        state.position = unproject(route.origin, axle + direction(state.trueHeading) * config.mainAxleAft);
    }
    check(controller.phase() == TaxiPhase::Complete && interiorStops == 0,
          "Strong brakes must not produce stop-and-go motion through a painted exit");
    check(worstCte < 3, "Distant ATC fallback must not increase painted-curve lookahead and CTE");
    std::cout << "Strong-brake exit: interior stops " << interiorStops << ", max CTE " << worstCte << " m\n";
}
} // namespace
int main() {
    try {
        forecastAgainstDriving(false);
        forecastAgainstDriving(true);
        forecastAgainstDriving(true, true);
        calibration();
        stableRefresh();
        smoothPrediction();
        continuousExit();
        std::cout << "ETA drive comparison, adaptation, speed change and cancellation passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
}
