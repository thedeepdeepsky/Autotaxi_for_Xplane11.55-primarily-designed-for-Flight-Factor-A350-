#include "BetterPushbackAdapter.h"
#include "FlightFactorCompatibility.h"
#include <algorithm>
#include <array>
#include <fstream>
#include <stdexcept>
namespace autotaxi {
namespace {
XPLMDataRef requiredRef(const char *name, bool writable = false) {
    auto ref = XPLMFindDataRef(name);
    if (!ref || (writable && !XPLMCanWriteDataRef(ref)))
        throw std::runtime_error(std::string("BetterPushback integration missing dataref: ") + name);
    return ref;
}
XPLMCommandRef requiredCommand(const char *name) {
    auto command = XPLMFindCommand(name);
    if (!command)
        throw std::runtime_error(std::string("BetterPushback command unavailable: ") + name);
    return command;
}
void writeScalar(XPLMDataRef ref, double value) {
    int type = XPLMGetDataRefTypes(ref);
    if (type & xplmType_Double)
        XPLMSetDatad(ref, value);
    else if (type & xplmType_Float)
        XPLMSetDataf(ref, static_cast<float>(value));
    else if (type & xplmType_Int)
        XPLMSetDatai(ref, static_cast<int>(value));
    else
        throw std::runtime_error("Unsupported parking brake dataref type");
}
bool supportedVersion(XPLMPluginID id) {
    std::array<char, 2048> file{};
    XPLMGetPluginInfo(id, nullptr, file.data(), nullptr, nullptr);
    auto directory = std::filesystem::u8path(file.data()).parent_path();
    for (int level = 0; level < 3 && !directory.empty(); ++level, directory = directory.parent_path()) {
        std::ifstream metadata(directory / "skunkcrafts_updater.cfg");
        for (std::string line; std::getline(metadata, line);) {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            if (line.rfind("version|", 0) == 0)
                return line == "version|1.10";
        }
    }
    return false;
}
bool idleThrottles() {
    auto ref = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio");
    std::array<float, 8> values{};
    if (!ref)
        return false;
    auto engines = XPLMFindDataRef("sim/aircraft/engine/acf_num_engines");
    int requested = engines ? XPLMGetDatai(engines) : 2;
    if (requested < 1 || requested > 8)
        return false;
    int count = XPLMGetDatavf(ref, values.data(), 0, requested);
    return count == requested && std::all_of(values.begin(), values.begin() + count, [](float value) {
               return std::isfinite(value) && std::abs(value) <= .02f;
           });
}
} // namespace
bool BetterPushbackAdapter::tugBusy() {
    auto started = XPLMFindDataRef("bp/started"), connected = XPLMFindDataRef("bp/connected");
    return (started && XPLMGetDatai(started)) || (connected && XPLMGetDatai(connected));
}
bool BetterPushbackAdapter::active() const {
    return phase_ != PushbackPhase::Idle && phase_ != PushbackPhase::Complete &&
           phase_ != PushbackPhase::Failed;
}
void BetterPushbackAdapter::bind() {
    started_ = requiredRef("bp/started");
    connected_ = requiredRef("bp/connected");
    completed_ = requiredRef("bp/op_complete");
    accepted_ = requiredRef("bp/plan_complete");
    if (XPLMGetDatai(requiredRef("bp/slave_mode")))
        throw std::runtime_error("Automatic pushback is unavailable in BetterPushback slave mode");
    if (XPLMGetDatai(requiredRef("bp/parking_brake_override")))
        throw std::runtime_error("BetterPushback parking brake input is externally overridden");
    connect_ = requiredCommand("BetterPushback/connect_first");
    start_ = requiredCommand("BetterPushback/start");
    close_ = requiredCommand("BetterPushback/stop_planner");
    stop_ = requiredCommand("BetterPushback/stop");
    disconnect_ = requiredCommand("BetterPushback/disconnect");
    park_ = requiredRef("sim/cockpit2/controls/parking_brake_ratio", true);
    legacyPark_ = requiredRef("sim/flightmodel/controls/parkbrake", true);
    ffPark_ = flightFactorA350Loaded() ? XPLMFindDataRef("1-sim/parckBrake") : nullptr;
    if (ffPark_ && !XPLMCanWriteDataRef(ffPark_))
        throw std::runtime_error("FF A350 parking brake is read-only; automatic departure unavailable");
    customPark_ = XPLMFindDataRef("model/controls/park_break");
    if (!customPark_)
        customPark_ = XPLMFindDataRef("sim/custom/controll/parking_brake");
    if (customPark_ && !XPLMCanWriteDataRef(customPark_))
        throw std::runtime_error(
            "BetterPushback uses a read-only aircraft parking brake; automatic departure unavailable");
}
void BetterPushbackAdapter::transition(PushbackPhase phase, const std::string &status) {
    phase_ = phase;
    status_ = status;
    elapsed_ = settled_ = retry_ = 0;
}
void BetterPushbackAdapter::parkingBrake(bool set) {
    // Match BP's aircraft-specific brake input as well as the standard cockpit indicator.
    if (customPark_)
        writeScalar(customPark_, set ? 1 : 0);
    if (ffPark_)
        writeScalar(ffPark_, set ? 0 : 1);
    writeScalar(legacyPark_, set ? 1 : 0);
    writeScalar(park_, set ? 1 : 0);
}
void BetterPushbackAdapter::releaseParkingBrake() {
    parkingBrake(false);
}
void BetterPushbackAdapter::holdParkingBrake() {
    parkingBrake(true);
}
void BetterPushbackAdapter::start(const std::filesystem::path &root, const PushbackPlan &plan,
                                  const AircraftState &state, bool taxiControlsOwned) {
    if (active() || taxiControlsOwned || tugBusy())
        throw std::runtime_error("Finish the current taxi/tug operation before automatic pushback");
    if (!state.onGround || !std::isfinite(state.speed) || std::abs(state.speed) > .15)
        throw std::runtime_error("Hold parking brake, throttle idle; stop below 0.29 kt before pushback");
    if (distance(state.position, plan.origin) > 2 || plan.segments.empty() ||
        std::abs(wrap180(state.trueHeading - plan.segments.front().startHeading)) > 2)
        throw std::runtime_error("Aircraft moved since pushback planning; preview again");
    if (distance(state.position, plan.segments.front().start) > 20)
        throw std::runtime_error("Main axle offset exceeds BetterPushback cache lookup tolerance");
    if (!idleThrottles())
        throw std::runtime_error("Set throttle levers to idle before pushback");
    plugin_ = XPLMFindPluginBySignature("skiselkov.BetterPushback");
    if (plugin_ == XPLM_NO_PLUGIN_ID || !XPLMIsPluginEnabled(plugin_))
        throw std::runtime_error("Install and enable BetterPushback 1.10 for automatic pushback");
    if (!supportedVersion(plugin_))
        throw std::runtime_error(
            "This cache bridge supports BetterPushback 1.10 only (check updater metadata)");
    bind();
    // BP manages steering itself; an aircraft-owned override is not pilot tiller input.
    plan_ = plan;
    // Reinitialize only an idle tug so an earlier in-memory plan cannot overwrite our submitted route.
    XPLMDisablePlugin(plugin_);
    if (!XPLMEnablePlugin(plugin_))
        throw std::runtime_error("BetterPushback failed to re-enable; enable it in Plugin Admin");
    bind();
    if (tugBusy())
        throw std::runtime_error("BetterPushback became active during initialization");
    storePushbackRoute(root / "Output" / "caches" / "BetterPushback_routes.dat", plan_);
    parkingBrake(true);
    initiated_ = true;
    sawStarted_ = false;
    transition(PushbackPhase::Connecting, "Connecting tug / Node " + std::to_string(plan_.nodeId));
    XPLMCommandOnce(connect_);
}
bool BetterPushbackAdapter::atTarget(const AircraftState &state) const {
    return distance(state.position, plan_.handoff) <= 6 &&
           std::abs(wrap180(state.trueHeading - plan_.heading)) <= 3;
}
void BetterPushbackAdapter::cancel() {
    if (initiated_ && plugin_ != XPLM_NO_PLUGIN_ID && XPLMIsPluginEnabled(plugin_)) {
        XPLMCommandOnce(close_);
        XPLMCommandOnce(stop_);
    }
    initiated_ = false;
    transition(PushbackPhase::Idle, "Automatic departure cancelled; finish tug disconnect in BetterPushback");
}
void BetterPushbackAdapter::fail(const std::string &reason) {
    cancel();
    transition(PushbackPhase::Failed, "Pushback stopped: " + reason);
}
void BetterPushbackAdapter::update(const AircraftState &state, double actualSteer, double dt) {
    if (!active())
        return;
    if (!XPLMIsPluginEnabled(plugin_)) {
        initiated_ = false;
        transition(PushbackPhase::Failed, "BetterPushback disabled; departure cancelled");
        return;
    }
    if (!valid(state.position) || !std::isfinite(state.speed) || !std::isfinite(state.trueHeading) ||
        !std::isfinite(actualSteer) || !state.onGround || distance(plan_.origin, state.position) > 500) {
        fail("Aircraft repositioned or telemetry unavailable");
        return;
    }
    elapsed_ += std::clamp(dt, 0.0, 1.0);
    retry_ += std::clamp(dt, 0.0, 1.0);
    bool started = XPLMGetDatai(started_) != 0, connected = XPLMGetDatai(connected_) != 0;
    sawStarted_ = sawStarted_ || started;
    if (!idleThrottles()) {
        fail("Throttle levers moved away from idle");
        return;
    }
    if (phase_ == PushbackPhase::Connecting) {
        if (elapsed_ > 300 || (sawStarted_ && !started)) {
            fail("Tug connection failed or timed out");
            return;
        }
        if (connected && started) {
            XPLMCommandOnce(start_);
            transition(PushbackPhase::ClosingPlanner, "Loading automatic pushback route");
        }
        return;
    }
    if (phase_ == PushbackPhase::ClosingPlanner) {
        XPLMCommandOnce(close_);
        transition(PushbackPhase::Accepting, "Waiting for BetterPushback route acceptance");
        return;
    }
    if (phase_ == PushbackPhase::Accepting) {
        if (!started || !connected || elapsed_ > 30) {
            fail("BetterPushback did not accept the cached route");
            return;
        }
        if (XPLMGetDatai(accepted_)) {
            parkingBrake(false);
            transition(PushbackPhase::Pushing,
                       "Pushback / Node " + std::to_string(plan_.nodeId) + " / heading " +
                           std::to_string(static_cast<int>(std::round(plan_.heading))));
        }
        return;
    }
    if (phase_ == PushbackPhase::Pushing) {
        if (!started || !connected || elapsed_ > 900) {
            fail("Pushback interrupted or timed out");
            return;
        }
        double crossTrack = 1e9;
        auto position = project(plan_.origin, state.position);
        for (std::size_t i = 1; i < plan_.points.size(); ++i) {
            Vec2 a = plan_.points[i - 1], v = plan_.points[i] - a;
            if (dot(v, v) > .001)
                crossTrack =
                    std::min(crossTrack, length(position - a -
                                                v * std::clamp(dot(position - a, v) / dot(v, v), 0.0, 1.0)));
        }
        if (crossTrack > 25) {
            fail("Tug deviated from the planned apron path");
            return;
        }
        // op_complete precedes BP's heading/wheel correction; wait for actual settling.
        bool settled = XPLMGetDatai(completed_) && atTarget(state) && std::abs(state.speed) < .1 &&
                       std::abs(actualSteer) < 5;
        settled_ = settled ? settled_ + dt : 0;
        if (settled_ >= 3) {
            parkingBrake(true);
            transition(PushbackPhase::Disconnecting,
                       "Disconnecting tug / Node " + std::to_string(plan_.nodeId));
        }
        return;
    }
    if (phase_ == PushbackPhase::Disconnecting) {
        if (!atTarget(state) || std::abs(state.speed) >= .3) {
            fail("Aircraft moved during tug disconnection");
            return;
        }
        if (elapsed_ > 300) {
            fail("Tug disconnection timed out; finish manually");
            return;
        }
        if (started || connected) {
            if (retry_ >= 1) {
                XPLMCommandOnce(disconnect_);
                retry_ = 0;
            }
            settled_ = 0;
        } else {
            settled_ += dt;
            if (settled_ >= 2) {
                initiated_ = false;
                transition(PushbackPhase::Complete, "Tug clear / Node " + std::to_string(plan_.nodeId));
            }
        }
    }
}
} // namespace autotaxi
