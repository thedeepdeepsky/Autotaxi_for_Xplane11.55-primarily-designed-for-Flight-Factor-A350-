#pragma once
#include "RoutePlanner.h"
namespace autotaxi {
struct ControllerConfig {
    double wheelbase = 28.35, mainAxleAft = 2.2;
    double maxSteer = 65, steerRate = 16;
    double taxiSpeed = 5.14, turnSpeed = 1.54, maxThrottle = 0.20;
    double maxCrossTrack = 14;
};
struct AircraftState {
    GeoPoint position;
    double trueHeading = 0, speed = 0;
    bool onGround = false;
};
enum class TaxiPhase { Idle, Taxi, Align, Braking, Complete, Fault };
struct ControlOutput {
    double steerDegrees = 0, throttle = 0, brake = 0;
    double targetSpeed = 0, crossTrack = 0, remaining = 0, progress = 0;
    TaxiPhase phase = TaxiPhase::Idle;
    std::string reason;
};
class TaxiController {
  public:
    void start(Route route, ControllerConfig config);
    ControlOutput update(const AircraftState &state, double dt);
    void stop();
    bool active() const;
    TaxiPhase phase() const {
        return phase_;
    }

  private:
    Route route_;
    ControllerConfig config_;
    TaxiPhase phase_ = TaxiPhase::Idle;
    std::size_t segment_ = 0;
    double steer_ = 0, integral_ = 0, settled_ = 0;
};
const char *phaseName(TaxiPhase phase);
} // namespace autotaxi
