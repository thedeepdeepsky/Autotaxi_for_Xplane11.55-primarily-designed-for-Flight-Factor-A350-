#pragma once
#include "RoutePlanner.h"
namespace autotaxi {
struct ControllerConfig {
    double wheelbase = 28.35, mainAxleAft = 2.2;
    double maxSteer = 65, steerRate = 16;
    double taxiSpeed = 5.14, turnSpeed = 1.54, maxThrottle = 0.20;
    double maxCrossTrack = 14;
    double cockpitAheadNose = 1.8;
    double mainGearHalfTrack = 5.1816;
    double maxTaxiSpeed = 20 * .514444, acceleration = .25, deceleration = .45;
    double maxBrake = 1, emergencyBrake = 1;
    double brakeAuthority = 1.5;
    double apronSpeed = 3 * .514444, apronApproachDistance = 180;
};
struct AircraftState {
    GeoPoint position;
    double trueHeading = 0, speed = 0;
    bool onGround = false;
};
enum class TaxiPhase { Idle, Taxi, Align, Braking, Complete, Fault, Hold };
struct ControlOutput {
    double steerDegrees = 0, throttle = 0, brake = 0;
    double targetSpeed = 0, crossTrack = 0, remaining = 0, progress = 0;
    TaxiPhase phase = TaxiPhase::Idle;
    std::string reason;
    double oversteerOffset = 0;
    bool pavementChecked = false, outsideKnownPavement = false;
};
double routeSpeedLimit(const Route &route, const ControllerConfig &config, std::size_t segment, Vec2 position,
                       double remaining, double requestedSteer);
double routeSpeedForBend(const Route &route, const ControllerConfig &config, std::size_t segment,
                         double remaining, double requestedSteer, double bend, double oversteerAhead = 1e30);
double trackingSteerDegrees(const Route &route, const ControllerConfig &config, Vec2 delta, Vec2 forward);
Vec2 trackingPosition(const Route &route, const ControllerConfig &config, const AircraftState &state);
Vec2 cockpitPosition(GeoPoint origin, const ControllerConfig &config, const AircraftState &state);
Vec2 mainAxlePosition(GeoPoint origin, const ControllerConfig &config, const AircraftState &state);
class TaxiController {
  public:
    void start(Route route, ControllerConfig config);
    ControlOutput update(const AircraftState &state, double dt);
    void stop();
    void setTaxiSpeed(double speed);
    void setEmergencyBrake(bool hold);
    bool emergencyBrake() const {
        return emergency_;
    }
    bool active() const;
    TaxiPhase phase() const {
        return phase_;
    }
    const Route &route() const {
        return route_;
    }
    const ControllerConfig &config() const {
        return config_;
    }
    std::size_t segment() const {
        return segment_;
    }
    void seekProgress(double progress);

  private:
    Route route_;
    std::vector<double> segmentLengths_, remainingAtPoint_, bends_;
    ControllerConfig config_;
    TaxiPhase phase_ = TaxiPhase::Idle;
    std::size_t segment_ = 0;
    bool trackingInitialized_ = false;
    bool emergency_ = false;
    double steer_ = 0, integral_ = 0, settled_ = 0;
    double targetSpeed_ = 0;
    bool speedInitialized_ = false;
    double apronSpeedCap_ = -1;
};
const char *phaseName(TaxiPhase phase);
} // namespace autotaxi
