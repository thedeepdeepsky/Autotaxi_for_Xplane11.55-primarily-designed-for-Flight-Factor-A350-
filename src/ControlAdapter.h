#pragma once
#include "Config.h"
#include "XPLMDataAccess.h"
namespace autotaxi {
class NumericRef {
  public:
    void bind(const std::string &name, int index, bool writable);
    double read() const;
    void write(double value) const;
    bool bound() const {
        return ref_ != nullptr;
    }
    XPLMDataRef ref() const {
        return ref_;
    }

  private:
    XPLMDataRef ref_ = nullptr;
    int index_ = -1, type_ = 0;
};
class ControlAdapter {
  public:
    void initialize(const Config &config = Config{});
    AircraftState state() const;
    bool isA350() const;
    void acquire(const Config &config);
    void apply(const ControlOutput &output);
    std::string monitor(double dt);
    void beginStop();
    bool stopping() const {
        return stopping_;
    }
    bool ownsControls() const {
        return owned_;
    }
    bool updateStop(double dt);
    void release(bool holdParkingBrake);
    double actualSteer() const;
    std::string description() const;

  private:
    NumericRef lat_, lon_, heading_, speed_, ground_, park_, ffPark_, left_, right_, steer_, feedback_;
    XPLMDataRef wheelOverride_ = nullptr, throttleOverride_ = nullptr, brakeOverride_ = nullptr;
    XPLMDataRef steerOn_ = nullptr, throttle_ = nullptr, icao_ = nullptr;
    Config config_;
    std::string throttleSource_;
    bool owned_ = false, stopping_ = false, useThrottleOverride_ = false, useBrakeOverride_ = false;
    bool ownWheelOverride_ = false, ownBrakeOverride_ = false;
    int savedSteerOn_ = 0, engines_ = 2, throttleType_ = 0;
    double previousLeft_ = 0, previousRight_ = 0, previousSteer_ = 0;
    double command_ = 0, mismatch_ = 0, stoppedTime_ = 0, stalledTime_ = 0, lastTarget_ = 0;
    void writeThrottle(double value);
};
} // namespace autotaxi
