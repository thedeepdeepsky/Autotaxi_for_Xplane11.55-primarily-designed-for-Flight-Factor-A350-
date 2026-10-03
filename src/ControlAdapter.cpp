#include "ControlAdapter.h"
#include "FlightFactorCompatibility.h"
#include "XPLMUtilities.h"
#include <algorithm>
#include <array>
#include <stdexcept>
namespace autotaxi {
void NumericRef::bind(const std::string &name, int index, bool writable) {
    ref_ = nullptr;
    auto ref = XPLMFindDataRef(name.c_str());
    if (!ref)
        throw std::runtime_error("Missing dataref: " + name);
    if (writable && !XPLMCanWriteDataRef(ref))
        throw std::runtime_error("Read-only dataref: " + name);
    int types = XPLMGetDataRefTypes(ref);
    index_ = index;
    if (index >= 0) {
        if (types & xplmType_FloatArray) {
            type_ = xplmType_FloatArray;
            if (XPLMGetDatavf(ref, nullptr, 0, 0) <= index)
                throw std::runtime_error("Array index out of range: " + name);
        } else if (types & xplmType_IntArray) {
            type_ = xplmType_IntArray;
            if (XPLMGetDatavi(ref, nullptr, 0, 0) <= index)
                throw std::runtime_error("Array index out of range: " + name);
        } else
            throw std::runtime_error("Expected array dataref: " + name);
    } else if (types & xplmType_Double)
        type_ = xplmType_Double;
    else if (types & xplmType_Float)
        type_ = xplmType_Float;
    else if (types & xplmType_Int)
        type_ = xplmType_Int;
    else
        throw std::runtime_error("Expected numeric scalar dataref: " + name);
    ref_ = ref;
}
double NumericRef::read() const {
    if (!ref_)
        return 0;
    if (type_ == xplmType_FloatArray) {
        float v = 0;
        XPLMGetDatavf(ref_, &v, index_, 1);
        return v;
    }
    if (type_ == xplmType_IntArray) {
        int v = 0;
        XPLMGetDatavi(ref_, &v, index_, 1);
        return v;
    }
    if (type_ == xplmType_Double)
        return XPLMGetDatad(ref_);
    if (type_ == xplmType_Float)
        return XPLMGetDataf(ref_);
    return XPLMGetDatai(ref_);
}
void NumericRef::write(double v) const {
    if (!ref_)
        return;
    if (type_ == xplmType_FloatArray) {
        float f = static_cast<float>(v);
        XPLMSetDatavf(ref_, &f, index_, 1);
    } else if (type_ == xplmType_IntArray) {
        int i = static_cast<int>(v);
        XPLMSetDatavi(ref_, &i, index_, 1);
    } else if (type_ == xplmType_Double)
        XPLMSetDatad(ref_, v);
    else if (type_ == xplmType_Float)
        XPLMSetDataf(ref_, static_cast<float>(v));
    else
        XPLMSetDatai(ref_, static_cast<int>(v));
}
void ControlAdapter::initialize(const Config &config) {
    config_ = config;
    lat_.bind(config.latitudeDataref, -1, false);
    lon_.bind(config.longitudeDataref, -1, false);
    heading_.bind(config.headingDataref, -1, false);
    speed_.bind(config.speedDataref, -1, false);
    ground_.bind(config.onGroundDataref, -1, false);
    park_.bind(config.parkingBrakeDataref, config.parkingBrakeIndex, true);
    icao_ = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    wheelOverride_ = XPLMFindDataRef("sim/operation/override/override_wheel_steer");
    brakeOverride_ = XPLMFindDataRef("sim/operation/override/override_toe_brakes");
    steerOn_ = XPLMFindDataRef("sim/cockpit2/controls/nosewheel_steer_on");
    feedback_.bind(config.feedbackDataref, config.feedbackIndex, false);
}
AircraftState ControlAdapter::state() const {
    return {{lat_.read(), lon_.read()}, heading_.read(), speed_.read(), ground_.read() != 0};
}
bool ControlAdapter::isA350() const {
    if (!icao_)
        return false;
    std::array<char, 41> text{};
    XPLMGetDatab(icao_, text.data(), 0, 40);
    return std::string(text.data()).find("A35") != std::string::npos;
}
void ControlAdapter::acquire(const Config &c) {
    if (owned_)
        throw std::runtime_error("Controls already engaged");
    for (const char *name : {"bp/started", "bp/connected"}) {
        auto ref = XPLMFindDataRef(name);
        if (ref && XPLMGetDatai(ref))
            throw std::runtime_error("Wait for BetterPushback to finish and disconnect before taxi");
    }
    config_ = c;
    auto s = state();
    if (!s.onGround || s.speed > 1)
        throw std::runtime_error("Start on ground below 2 kt");
    if (c.requireA350 && !isA350())
        throw std::runtime_error("Loaded aircraft ICAO is not A350");
    bool cooperate = flightFactorA350Loaded();
    bool ffParkingBrake = cooperate && XPLMFindDataRef("1-sim/parckBrake");
    // FF A350's cockpit/checklist defines 0 = set, 1 = released.
    ffPark_ = {};
    if (ffParkingBrake)
        ffPark_.bind("1-sim/parckBrake", -1, true);
    bool parkingHeld = parkingRatio() > 0.1 || (ffPark_.bound() && ffPark_.read() < .9);
    steer_.bind(c.steeringDataref, c.steeringIndex, true);
    feedback_.bind(c.feedbackDataref, c.feedbackIndex, false);
    if (steer_.ref() == feedback_.ref())
        throw std::runtime_error("Feedback must be actual wheel angle, separate from command");
    left_.bind(c.leftBrakeDataref, c.leftBrakeIndex, true);
    right_.bind(c.rightBrakeDataref, c.rightBrakeIndex, true);
    useBrakeOverride_ = c.leftBrakeDataref == "sim/cockpit2/controls/left_brake_ratio" &&
                        c.rightBrakeDataref == "sim/cockpit2/controls/right_brake_ratio";
    auto check = [cooperate](XPLMDataRef ref, const char *name, bool allowAircraftOverride = false) {
        if (!ref || !XPLMCanWriteDataRef(ref))
            throw std::runtime_error(std::string("Missing override: ") + name);
        if (XPLMGetDatai(ref) != 0 && !(cooperate && allowAircraftOverride))
            throw std::runtime_error(std::string("Override already owned: ") + name);
    };
    if (c.steeringMode == "direct")
        check(wheelOverride_, "wheel steering", true);
    if (useBrakeOverride_)
        check(brakeOverride_, "toe brakes", true);
    previousLeft_ = left_.read();
    previousRight_ = right_.read();
    previousSteer_ = steer_.read();
    if (previousLeft_ > 0.1 || previousRight_ > 0.1)
        throw std::runtime_error("Release toe brakes before starting");
    if (std::abs(feedback_.read()) > 8)
        throw std::runtime_error("Center nosewheel before starting");
    savedSteerOn_ = steerOn_ ? XPLMGetDatai(steerOn_) : 0;
    ownWheelOverride_ = c.steeringMode == "direct" && XPLMGetDatai(wheelOverride_) == 0;
    ownBrakeOverride_ = useBrakeOverride_ && XPLMGetDatai(brakeOverride_) == 0;
    if (cooperate &&
        ((c.steeringMode == "direct" && !ownWheelOverride_) || (useBrakeOverride_ && !ownBrakeOverride_)))
        XPLMDebugString("[A350AutoTaxi] FF A350: preserving existing wheel/toe overrides; "
                        "monitoring actual nosewheel response.\n");
    if (ownWheelOverride_)
        XPLMSetDatai(wheelOverride_, 1);
    if (ownBrakeOverride_)
        XPLMSetDatai(brakeOverride_, 1);
    if (steerOn_ && XPLMCanWriteDataRef(steerOn_))
        XPLMSetDatai(steerOn_, 1);
    owned_ = true;
    stopping_ = false;
    emergencyHeld_ = false;
    command_ = mismatch_ = stoppedTime_ = stalledTime_ = lastTarget_ = 0;
    if (parkingHeld) {
        // Transfer the hold to service brakes before releasing the parking brake.
        left_.write(.75 * c.brakeScale);
        right_.write(.75 * c.brakeScale);
        if (ffPark_.bound())
            ffPark_.write(1);
        park_.write(c.parkingBrakeReleased);
    }
    XPLMDebugString(("[A350AutoTaxi] Acquired " + description() + "\n").c_str());
}
double ControlAdapter::parkingRatio() const {
    return (park_.read() - config_.parkingBrakeReleased) /
           (config_.parkingBrakeSet - config_.parkingBrakeReleased);
}
void ControlAdapter::apply(const ControlOutput &out) {
    if (!owned_)
        return;
    emergencyHeld_ = out.phase == TaxiPhase::Hold && !stopping_;
    if (!stopping_)
        command_ = out.steerDegrees;
    lastTarget_ = stopping_ ? 0 : out.targetSpeed;
    steer_.write(command_ * config_.steeringScale * config_.steeringSign);
    double brake =
        (stopping_ ? config_.controller.maxBrake : std::clamp(out.brake, 0.0, 1.0)) * config_.brakeScale;
    left_.write(brake);
    right_.write(brake);
}
std::string ControlAdapter::monitor(double dt) {
    if (!owned_ || stopping_)
        return {};
    if (ownWheelOverride_ && XPLMGetDatai(wheelOverride_) != 1)
        return "Wheel override was taken away";
    if (ownBrakeOverride_ && XPLMGetDatai(brakeOverride_) != 1)
        return "Brake override was taken away";
    double actual = feedback_.read();
    if (!std::isfinite(actual))
        return "Invalid nosewheel feedback";
    // Read enacted wheel angle after physics, not the value just written into the command ref.
    mismatch_ = emergencyHeld_                     ? 0
                : std::abs(command_ - actual) > 12 ? mismatch_ + dt
                                                   : std::max(0.0, mismatch_ - dt);
    if (mismatch_ > config_.feedbackTimeout)
        return "Nosewheel does not follow command; check FF steering profile";
    auto s = state();
    if (!s.onGround)
        return "Aircraft became airborne";
    if (!emergencyHeld_ && (parkingRatio() > 0.2 || (ffPark_.bound() && ffPark_.read() < .8)))
        return "Parking brake applied";
    stalledTime_ = lastTarget_ > 0.8 && s.speed < 0.15 ? stalledTime_ + dt : 0;
    if (stalledTime_ > 30)
        return "No movement: set manual taxi thrust and check brakes/chocks";
    return {};
}
void ControlAdapter::beginStop() {
    if (owned_) {
        stopping_ = true;
        stoppedTime_ = 0;
    }
}
bool ControlAdapter::updateStop(double dt) {
    if (!owned_ || !stopping_)
        return false;
    auto s = state();
    if (!s.onGround) {
        release(false);
        return true;
    }
    command_ -= std::clamp(command_, -config_.controller.steerRate * dt, config_.controller.steerRate * dt);
    apply({});
    stoppedTime_ = s.speed < 0.12 ? stoppedTime_ + dt : 0;
    if (stoppedTime_ > 1) {
        release(true);
        return true;
    }
    return false;
}
void ControlAdapter::release(bool hold) {
    if (!owned_)
        return;
    steer_.write(previousSteer_);
    if (hold) {
        park_.write(config_.parkingBrakeSet);
        if (ffPark_.bound())
            ffPark_.write(0);
    }
    left_.write(previousLeft_);
    right_.write(previousRight_);
    if (ownWheelOverride_)
        XPLMSetDatai(wheelOverride_, 0);
    if (ownBrakeOverride_)
        XPLMSetDatai(brakeOverride_, 0);
    if (steerOn_ && XPLMCanWriteDataRef(steerOn_))
        XPLMSetDatai(steerOn_, savedSteerOn_);
    owned_ = stopping_ = false;
    ownWheelOverride_ = ownBrakeOverride_ = false;
}
double ControlAdapter::actualSteer() const {
    return feedback_.bound() ? feedback_.read() : 0;
}
std::string ControlAdapter::description() const {
    return config_.steeringMode + " [" + std::to_string(config_.steeringIndex) + "] " +
           config_.steeringDataref + " | manual thrust / brake speed control";
}
} // namespace autotaxi
