#include "ControlAdapter.h"
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <map>
#include <stdexcept>
using namespace autotaxi;
namespace {
struct Ref {
    int type;
    bool writable;
    std::vector<double> values;
    std::string bytes;
};
std::map<std::string, Ref> refs;
bool ffEnabled = false;
bool checkBrakeTransfer = false;
Ref &data(XPLMDataRef ref) {
    return *static_cast<Ref *>(ref);
}
Ref &named(const std::string &name) {
    return refs.at(name);
}
void scalar(const std::string &name, int type, double value) {
    refs[name] = {type, true, {value}, {}};
}
void array(const std::string &name, int type, int count) {
    refs[name] = {type, true, std::vector<double>(count), {}};
}
void check(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
template <class F> void rejects(F call, const char *reason) {
    bool failed = false;
    try {
        call();
    } catch (const std::exception &) {
        failed = true;
    }
    check(failed, reason);
}
void setup() {
    scalar("sim/flightmodel/position/latitude", xplmType_Double, 31);
    scalar("sim/flightmodel/position/longitude", xplmType_Double, 121);
    scalar("sim/flightmodel/position/psi", xplmType_Float, 0);
    scalar("sim/flightmodel/position/groundspeed", xplmType_Float, 0);
    scalar("sim/flightmodel/failures/onground_any", xplmType_Int, 1);
    scalar("sim/cockpit2/controls/parking_brake_ratio", xplmType_Float, 0);
    scalar("sim/cockpit2/controls/left_brake_ratio", xplmType_Float, 0);
    scalar("sim/cockpit2/controls/right_brake_ratio", xplmType_Float, 0);
    scalar("sim/operation/override/override_wheel_steer", xplmType_Int, 0);
    scalar("sim/operation/override/override_throttles", xplmType_Int, 0);
    scalar("sim/operation/override/override_toe_brakes", xplmType_Int, 0);
    scalar("sim/cockpit2/controls/nosewheel_steer_on", xplmType_Int, 0);
    scalar("sim/aircraft/engine/acf_num_engines", xplmType_Int, 2);
    array("sim/flightmodel2/gear/tire_steer_command_deg", xplmType_FloatArray, 10);
    array("sim/flightmodel2/gear/tire_steer_actual_deg", xplmType_FloatArray, 10);
    array("sim/flightmodel/engine/ENGN_thro_use", xplmType_FloatArray, 8);
    array("sim/cockpit2/engine/actuators/throttle_ratio", xplmType_FloatArray, 8);
    refs["sim/aircraft/view/acf_ICAO"] = {xplmType_Data, false, {}, "A359"};
    scalar("test/ff/tiller", xplmType_Float, 0);
}
} // namespace
extern "C" {
XPLMPluginID XPLMFindPluginBySignature(const char *signature) {
    return std::string(signature) == "1-sim A350" ? 88 : XPLM_NO_PLUGIN_ID;
}
int XPLMIsPluginEnabled(XPLMPluginID id) {
    return id == 88 && ffEnabled;
}
XPLMDataRef XPLMFindDataRef(const char *name) {
    auto it = refs.find(name);
    return it == refs.end() ? nullptr : &it->second;
}
XPLMDataTypeID XPLMGetDataRefTypes(XPLMDataRef ref) {
    return data(ref).type;
}
int XPLMCanWriteDataRef(XPLMDataRef ref) {
    return data(ref).writable;
}
int XPLMGetDatai(XPLMDataRef ref) {
    return static_cast<int>(data(ref).values[0]);
}
float XPLMGetDataf(XPLMDataRef ref) {
    return static_cast<float>(data(ref).values[0]);
}
double XPLMGetDatad(XPLMDataRef ref) {
    return data(ref).values[0];
}
void XPLMSetDatai(XPLMDataRef ref, int value) {
    data(ref).values[0] = value;
}
void XPLMSetDataf(XPLMDataRef ref, float value) {
    if (checkBrakeTransfer &&
        ((ref == XPLMFindDataRef("sim/cockpit2/controls/parking_brake_ratio") && value == 0) ||
         (ref == XPLMFindDataRef("1-sim/parckBrake") && value == 1)))
        check(named("sim/cockpit2/controls/left_brake_ratio").values[0] >= .75 &&
                  named("sim/cockpit2/controls/right_brake_ratio").values[0] >= .75,
              "Service brakes must hold before parking brake releases");
    data(ref).values[0] = value;
}
void XPLMSetDatad(XPLMDataRef ref, double value) {
    data(ref).values[0] = value;
}
int XPLMGetDatavf(XPLMDataRef ref, float *out, int offset, int count) {
    auto &v = data(ref).values;
    if (!out)
        return static_cast<int>(v.size());
    int n = std::min(count, static_cast<int>(v.size()) - offset);
    for (int i = 0; i < n; ++i)
        out[i] = static_cast<float>(v[offset + i]);
    return n;
}
int XPLMGetDatavi(XPLMDataRef ref, int *out, int offset, int count) {
    auto &v = data(ref).values;
    if (!out)
        return static_cast<int>(v.size());
    int n = std::min(count, static_cast<int>(v.size()) - offset);
    for (int i = 0; i < n; ++i)
        out[i] = static_cast<int>(v[offset + i]);
    return n;
}
void XPLMSetDatavf(XPLMDataRef ref, float *values, int offset, int count) {
    for (int i = 0; i < count; ++i)
        data(ref).values[offset + i] = values[i];
}
void XPLMSetDatavi(XPLMDataRef ref, int *values, int offset, int count) {
    for (int i = 0; i < count; ++i)
        data(ref).values[offset + i] = values[i];
}
int XPLMGetDatab(XPLMDataRef ref, void *out, int offset, int count) {
    const auto &bytes = data(ref).bytes;
    int n = std::min(count, static_cast<int>(bytes.size()) - offset);
    if (out && n > 0)
        std::memcpy(out, bytes.data() + offset, n);
    return n;
}
void XPLMDebugString(const char *) {}
}
int main() {
    try {
        setup();
        Config c;
        ControlAdapter adapter;
        adapter.initialize();
        scalar("bp/started", xplmType_Int, 1);
        scalar("bp/connected", xplmType_Int, 0);
        rejects([&] { adapter.acquire(c); },
                "Tug started interlock applies even after mechanical disconnection");
        named("bp/started").values[0] = 0;
        named("bp/connected").values[0] = 1;
        rejects([&] { adapter.acquire(c); }, "Connected tug prevents taxi control ownership");
        named("bp/connected").values[0] = 0;
        named(c.steeringDataref).values[1] = 17;
        named(c.throttleDataref).values[2] = 0.77;
        named("sim/operation/override/override_wheel_steer").values[0] = 1;
        named("sim/cockpit2/controls/parking_brake_ratio").values[0] = 1;
        rejects([&] { adapter.acquire(c); }, "Do not steal another plugin's override");
        check(!adapter.ownsControls(), "Failed acquire owns no controls");
        check(named("sim/cockpit2/controls/parking_brake_ratio").values[0] == 1,
              "Failed start must leave parking brake held");
        named("sim/operation/override/override_wheel_steer").values[0] = 0;
        named("sim/cockpit2/controls/parking_brake_ratio").values[0] = 1;
        checkBrakeTransfer = true;
        adapter.acquire(c);
        check(named("sim/cockpit2/controls/parking_brake_ratio").values[0] == 0,
              "Start can transfer a parked aircraft to taxi without a rolling gap");
        ControlOutput out;
        out.steerDegrees = 30;
        out.throttle = 0.1;
        out.targetSpeed = 2;
        adapter.apply(out);
        check(named(c.steeringDataref).values[0] == 30 && named(c.steeringDataref).values[1] == 17,
              "Write only the selected nosewheel");
        check(std::abs(named(c.throttleDataref).values[1] - 0.1) < 1e-6 &&
                  named(c.throttleDataref).values[2] == 0.77,
              "Write only installed engines");
        std::string fault;
        for (int i = 0; i < 6; ++i)
            fault = adapter.monitor(1);
        check(!fault.empty(), "Actual wheel feedback must detect FF overwrite");
        adapter.beginStop();
        check(!adapter.updateStop(0.5), "Hold brakes before stop is confirmed");
        check(std::abs(named(c.steeringDataref).values[0] - 22) < 1e-6,
              "Center nosewheel gradually while stopping");
        check(adapter.updateStop(1.1), "Stopped aircraft must release controls");
        check(named("sim/cockpit2/controls/parking_brake_ratio").values[0] == 1,
              "Hold parking brake on completion");
        check(named("sim/operation/override/override_wheel_steer").values[0] == 0 &&
                  named("sim/operation/override/override_throttles").values[0] == 0 &&
                  named("sim/operation/override/override_toe_brakes").values[0] == 0,
              "Release all owned overrides");
        check(named("sim/cockpit2/controls/nosewheel_steer_on").values[0] == 0,
              "Restore steering enable switch");
        named("sim/cockpit2/controls/parking_brake_ratio").values[0] = 0;
        c.steeringMode = "custom";
        c.steeringDataref = "test/ff/tiller";
        c.steeringIndex = -1;
        c.steeringScale = 1.0 / 65;
        c.steeringSign = -1;
        adapter.acquire(c);
        out.steerDegrees = 13;
        adapter.apply(out);
        named(c.feedbackDataref).values[0] = 13;
        check(std::abs(named(c.steeringDataref).values[0] + 0.2) < 1e-6, "Custom FF scalar scaling and sign");
        check(adapter.monitor(1).empty(), "Physical wheel feedback matches custom command");
        adapter.release(false);
        check(!adapter.ownsControls(), "Immediate manual disconnect");
        check(named(c.throttleDataref).values[0] == 0, "Disconnect sets thrust idle");
        c = Config{};
        named(c.feedbackDataref).values[0] = 0;
        auto &overrideRef = named("sim/operation/override/override_throttles");
        overrideRef.values[0] = 1;
        named(c.throttleDataref).values[0] = 0.33;
        const std::string lever = "sim/cockpit2/engine/actuators/throttle_ratio";
        named(lever).writable = false;
        rejects([&] { adapter.acquire(c); }, "Unavailable fallback must fail before acquiring controls");
        check(!adapter.ownsControls() && overrideRef.values[0] == 1 &&
                  named("sim/operation/override/override_wheel_steer").values[0] == 0,
              "Failed fallback preserves existing overrides");
        named(lever).writable = true;
        named(lever).values[2] = 0.66;
        adapter.acquire(c);
        adapter.apply(out);
        check(std::abs(named(lever).values[0] - out.throttle) < 1e-6 &&
                  std::abs(named(lever).values[1] - out.throttle) < 1e-6 && named(lever).values[2] == 0.66,
              "Existing engine override routes commands to installed-engine lever inputs");
        check(named(c.throttleDataref).values[0] == 0.33 && overrideRef.values[0] == 1,
              "Do not write aircraft-owned engine output or override");
        adapter.release(false);
        check(named(lever).values[0] == 0 && named(c.throttleDataref).values[0] == 0.33 &&
                  overrideRef.values[0] == 1,
              "Disconnect idles lever and preserves aircraft throttle override");
        ffEnabled = true;
        scalar("1-sim/parckBrake", xplmType_Float, 0);
        named("sim/operation/override/override_wheel_steer").values[0] = 1;
        named("sim/operation/override/override_toe_brakes").values[0] = 1;
        adapter.acquire(c);
        check(named("1-sim/parckBrake").values[0] == 1,
              "FF parked start releases inverse brake only after service brake hold");
        adapter.apply(out);
        named(c.feedbackDataref).values[0] = out.steerDegrees;
        check(adapter.monitor(1).empty(), "Known FF aircraft permits monitored cooperative inputs");
        adapter.release(true);
        check(named("sim/operation/override/override_wheel_steer").values[0] == 1 &&
                  named("sim/operation/override/override_toe_brakes").values[0] == 1,
              "Release preserves FF-owned steering and brake flags");
        check(named("1-sim/parckBrake").values[0] == 0, "Completion sets the actual FF parking brake");
        named("1-sim/parckBrake").values[0] = 1;
        named("sim/cockpit2/controls/parking_brake_ratio").values[0] = 0;
        named(c.feedbackDataref).values[0] = 0;
        adapter.acquire(c);
        out.steerDegrees = 30;
        adapter.apply(out);
        for (int i = 0; i < 6; ++i)
            fault = adapter.monitor(1);
        check(fault.find("Nosewheel") != std::string::npos,
              "Cooperative mode still detects aircraft overwriting the steering command");
        adapter.release(false);
        refs["sim/aircraft/view/acf_ICAO"].bytes = "B738";
        c.requireA350 = false;
        rejects([&] { adapter.acquire(c); }, "Loaded FF plugin cannot authorize a different aircraft");
        std::cout << "Control adapter checks passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAILED: " << e.what() << "\n";
        return 1;
    }
}
