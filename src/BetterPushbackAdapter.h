#pragma once
#include "PushbackPlanner.h"
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"
namespace autotaxi {
enum class PushbackPhase {
    Idle,
    Connecting,
    ClosingPlanner,
    Accepting,
    Pushing,
    Disconnecting,
    Complete,
    Failed
};
class BetterPushbackAdapter {
  public:
    void start(const std::filesystem::path &simulatorRoot, const PushbackPlan &plan,
               const AircraftState &state, bool taxiControlsOwned);
    void update(const AircraftState &state, double actualSteer, double dt);
    void cancel();
    bool active() const;
    PushbackPhase phase() const {
        return phase_;
    }
    const std::string &status() const {
        return status_;
    }
    void releaseParkingBrake();
    void holdParkingBrake();
    static bool tugBusy();

  private:
    XPLMPluginID plugin_ = XPLM_NO_PLUGIN_ID;
    XPLMDataRef started_ = nullptr, connected_ = nullptr, completed_ = nullptr, accepted_ = nullptr;
    XPLMDataRef park_ = nullptr, legacyPark_ = nullptr, customPark_ = nullptr, ffPark_ = nullptr;
    XPLMCommandRef connect_ = nullptr, start_ = nullptr, close_ = nullptr, stop_ = nullptr,
                   disconnect_ = nullptr;
    PushbackPlan plan_;
    PushbackPhase phase_ = PushbackPhase::Idle;
    std::string status_;
    double elapsed_ = 0, settled_ = 0, retry_ = 0;
    bool initiated_ = false, sawStarted_ = false;
    void bind();
    void transition(PushbackPhase phase, const std::string &status);
    void fail(const std::string &reason);
    void parkingBrake(bool set);
    bool atTarget(const AircraftState &state) const;
};
} // namespace autotaxi
