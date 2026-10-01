#include "BetterPushbackAdapter.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
using namespace autotaxi;
namespace {
struct Ref {
    int type;
    bool writable = true;
    double value = 0;
};
std::map<std::string, Ref> refs;
std::map<std::string, std::string> commands;
std::vector<std::string> calls;
std::filesystem::path root;
bool enabled = true, plannerOpen = false, ffEnabled = false;
int resets = 0;
Ref &ref(XPLMDataRef value) {
    return *static_cast<Ref *>(value);
}
double &value(const std::string &name) {
    return refs.at(name).value;
}
void check(bool condition, const char *reason) {
    if (!condition)
        throw std::runtime_error(reason);
}
template <class F> void rejects(F call, const char *reason) {
    bool rejected = false;
    try {
        call();
    } catch (const std::exception &) {
        rejected = true;
    }
    check(rejected, reason);
}
void setup(const std::string &version = "1.10") {
    refs.clear();
    commands.clear();
    calls.clear();
    enabled = true;
    ffEnabled = false;
    plannerOpen = false;
    resets = 0;
    for (auto name : {"bp/started", "bp/connected", "bp/op_complete", "bp/plan_complete", "bp/slave_mode",
                      "bp/parking_brake_override", "sim/operation/override/override_wheel_steer",
                      "sim/operation/override/override_toe_brakes"})
        refs[name] = {xplmType_Int};
    for (auto name : {"sim/cockpit2/controls/parking_brake_ratio", "sim/flightmodel/controls/parkbrake"})
        refs[name] = {xplmType_Float};
    refs["sim/cockpit2/engine/actuators/throttle_ratio"] = {xplmType_FloatArray};
    for (auto name : {"connect_first", "start", "stop_planner", "stop", "disconnect"}) {
        std::string full = std::string("BetterPushback/") + name;
        commands[full] = full;
    }
    auto directory = root / "Resources/plugins/BetterPushback";
    std::filesystem::create_directories(directory / "64");
    std::ofstream metadata(directory / "skunkcrafts_updater.cfg");
    metadata << "version|" << version << '\n';
}
PushbackPlan makePlan() {
    PushbackPlan plan;
    plan.origin = {31, 121};
    plan.heading = 0;
    plan.length = 80;
    plan.nodeId = 55;
    auto start = unproject(plan.origin, {0, -2.2});
    auto end = unproject(plan.origin, {0, -80});
    plan.handoff = unproject(plan.origin, {0, -77.8});
    plan.points = {{0, -2.2}, {0, -80}};
    plan.segments = {{0, start, end, 0, 0, true, 0, false, true}};
    return plan;
}
void import(BetterPushbackAdapter &adapter, const AircraftState &state) {
    value("bp/connected") = 1;
    adapter.update(state, 0, .25);
    check(plannerOpen && adapter.phase() == PushbackPhase::ClosingPlanner, "Connected tug loads planner");
    adapter.update(state, 0, .25);
    check(!plannerOpen && adapter.phase() == PushbackPhase::Accepting, "Close planner on a later tick");
    adapter.update(state, 0, .25);
    check(value("sim/cockpit2/controls/parking_brake_ratio") == 1,
          "Do not release brakes before route acceptance");
    if (ffEnabled)
        check(value("1-sim/parckBrake") == 0, "FF brake held until route accepted");
    value("bp/plan_complete") = 1;
    adapter.update(state, 0, .25);
    check(adapter.phase() == PushbackPhase::Pushing, "Accepted route begins tow");
    check(value("sim/cockpit2/controls/parking_brake_ratio") == 0, "Release parking brake for tow");
    if (ffEnabled)
        check(value("1-sim/parckBrake") == 1, "FF inverse parking brake releases for tow");
}
} // namespace
extern "C" {
XPLMDataRef XPLMFindDataRef(const char *name) {
    auto item = refs.find(name);
    return item == refs.end() ? nullptr : &item->second;
}
XPLMDataTypeID XPLMGetDataRefTypes(XPLMDataRef r) {
    return ref(r).type;
}
int XPLMCanWriteDataRef(XPLMDataRef r) {
    return ref(r).writable;
}
int XPLMGetDatai(XPLMDataRef r) {
    return static_cast<int>(ref(r).value);
}
void XPLMSetDatai(XPLMDataRef r, int v) {
    ref(r).value = v;
}
void XPLMSetDataf(XPLMDataRef r, float v) {
    ref(r).value = v;
}
void XPLMSetDatad(XPLMDataRef r, double v) {
    ref(r).value = v;
}
int XPLMGetDatavf(XPLMDataRef r, float *out, int, int count) {
    int n = std::min(count, 2);
    for (int i = 0; i < n; ++i)
        out[i] = static_cast<float>(ref(r).value);
    return n;
}
int XPLMGetDatab(XPLMDataRef, void *out, int, int count) {
    if (out && count >= 4)
        std::memcpy(out, "A359", 4);
    return 4;
}
XPLMPluginID XPLMFindPluginBySignature(const char *signature) {
    if (std::string(signature) == "1-sim A350")
        return ffEnabled ? 88 : XPLM_NO_PLUGIN_ID;
    return std::string(signature) == "skiselkov.BetterPushback" ? 99 : XPLM_NO_PLUGIN_ID;
}
int XPLMIsPluginEnabled(XPLMPluginID id) {
    return id == 88 ? ffEnabled : enabled;
}
void XPLMDisablePlugin(XPLMPluginID) {
    enabled = false;
    ++resets;
    check(!BetterPushbackAdapter::tugBusy(), "Never reset an active tug");
}
int XPLMEnablePlugin(XPLMPluginID) {
    enabled = true;
    value("bp/started") = value("bp/connected") = value("bp/op_complete") = value("bp/plan_complete") = 0;
    return 1;
}
void XPLMGetPluginInfo(XPLMPluginID, char *, char *path, char *, char *) {
    std::strcpy(path, (root / "Resources/plugins/BetterPushback/64/win.xpl").string().c_str());
}
XPLMCommandRef XPLMFindCommand(const char *name) {
    auto item = commands.find(name);
    return item == commands.end() ? nullptr : &item->second;
}
void XPLMCommandOnce(XPLMCommandRef command) {
    const auto &name = *static_cast<std::string *>(command);
    calls.push_back(name);
    if (name == "BetterPushback/connect_first")
        value("bp/started") = 1;
    if (name == "BetterPushback/start") {
        plannerOpen = true;
        std::ifstream input(root / "Output/caches/BetterPushback_routes.dat");
        auto routes = readPushbackCache(input);
        check(!routes.empty() && distance(routes.back().back().end, makePlan().segments.back().end) < .01,
              "Planner imports the submitted endpoint from the actual BP cache filename");
    }
    if (name == "BetterPushback/stop_planner")
        plannerOpen = false;
}
} // extern C
int main() {
    try {
        root = std::filesystem::temp_directory_path() / "autotaxi-bp-lifecycle-tests";
        auto plan = makePlan();
        AircraftState initial{plan.origin, 0, 0, true};
        setup();
        BetterPushbackAdapter adapter;
        value("bp/started") = 1;
        rejects([&] { adapter.start(root, plan, initial, false); }, "Reject an existing tug session");
        check(resets == 0 && calls.empty(), "Existing tug session is untouched");
        value("bp/started") = 0;
        rejects([&] { adapter.start(root, plan, initial, true); }, "Reject AutoTaxi-owned controls");
        value("bp/slave_mode") = 1;
        rejects([&] { adapter.start(root, plan, initial, false); }, "Reject slave mode");
        value("bp/slave_mode") = 0;
        value("sim/cockpit2/engine/actuators/throttle_ratio") = .2;
        rejects([&] { adapter.start(root, plan, initial, false); }, "Idle throttle interlock");
        value("sim/cockpit2/engine/actuators/throttle_ratio") = 0;
        value("sim/operation/override/override_wheel_steer") = 1;
        value("sim/operation/override/override_toe_brakes") = 1;
        ffEnabled = true;
        refs["sim/aircraft/view/acf_ICAO"] = {xplmType_Data, false};
        refs["1-sim/parckBrake"] = {xplmType_Float, true, 1};
        value("sim/cockpit2/controls/parking_brake_ratio") = 1;
        value("1-sim/parckBrake") = 0;
        adapter.start(root, plan, initial, false);
        check(value("sim/operation/override/override_wheel_steer") == 1 &&
                  value("sim/operation/override/override_toe_brakes") == 1,
              "Existing aircraft overrides do not block BP and are not modified by this adapter");
        check(resets == 1, "Reinitialize an idle BP instance once");
        import(adapter, initial);
        for (int i = 0; i < 5; ++i)
            adapter.update(initial, 0, 1);
        check(adapter.phase() == PushbackPhase::Pushing, "plan_complete does not imply arrival");
        AircraftState target{plan.handoff, 20, 0, true};
        value("bp/op_complete") = 1;
        for (int i = 0; i < 4; ++i)
            adapter.update(target, 0, 1);
        check(adapter.phase() == PushbackPhase::Pushing,
              "op_complete before heading correction cannot hand off");
        target.trueHeading = plan.heading;
        for (int i = 0; i < 4; ++i)
            adapter.update(target, 15, 1);
        check(adapter.phase() == PushbackPhase::Pushing, "Physical nosewheel must settle first");
        for (int i = 0; i < 3; ++i)
            adapter.update(target, 0, 1);
        check(adapter.phase() == PushbackPhase::Disconnecting,
              "Settled target holds parking brake and disconnects");
        auto disconnects = std::count(calls.begin(), calls.end(), "BetterPushback/disconnect");
        for (int i = 0; i < 3; ++i)
            adapter.update(target, 0, 1);
        check(std::count(calls.begin(), calls.end(), "BetterPushback/disconnect") >= disconnects + 3,
              "Retry disconnect because early BP commands are ignored");
        value("bp/connected") = 0;
        for (int i = 0; i < 3; ++i)
            adapter.update(target, 0, 1);
        check(adapter.phase() == PushbackPhase::Disconnecting, "Tug started flag prevents premature handoff");
        value("bp/started") = 0;
        adapter.update(target, 0, 1);
        adapter.update(target, 0, 1);
        check(adapter.phase() == PushbackPhase::Complete && !adapter.active(),
              "Handoff only when BP fully clears");
        check(value("sim/cockpit2/controls/parking_brake_ratio") == 1,
              "Brake remains held until forward route validated");
        check(value("1-sim/parckBrake") == 0, "Hold FF parking brake during tug disconnection");
        adapter.releaseParkingBrake();
        check(value("sim/flightmodel/controls/parkbrake") == 0,
              "Release both standard brake inputs for taxi");
        check(value("1-sim/parckBrake") == 1, "Release FF parking brake before AutoTaxi handoff");
        setup("1.11");
        rejects([&] { adapter.start(root, plan, initial, false); }, "Reject unverified BP cache version");
        check(resets == 0, "Unsupported version is untouched");
        setup();
        refs["model/controls/park_break"] = {xplmType_Float, false};
        rejects([&] { adapter.start(root, plan, initial, false); },
                "Read-only aircraft brake cannot be automated");
        check(resets == 0, "Brake capability is checked before resetting BP");
        setup();
        adapter.start(root, plan, initial, false);
        enabled = false;
        adapter.update(initial, 0, 1);
        check(adapter.phase() == PushbackPhase::Failed, "Disabling BP cancels rather than advancing handoff");
        setup();
        adapter.start(root, plan, initial, false);
        value("bp/connected") = 1;
        adapter.update(initial, 0, 1);
        adapter.update(initial, 0, 1);
        for (int i = 0; i < 31; ++i)
            adapter.update(initial, 0, 1);
        check(adapter.phase() == PushbackPhase::Failed,
              "No accepted cached plan must time out without releasing brakes");
        check(value("sim/cockpit2/controls/parking_brake_ratio") == 1, "Missing plan keeps aircraft parked");
        setup();
        adapter.start(root, plan, initial, false);
        import(adapter, initial);
        adapter.cancel();
        check(calls.back() == "BetterPushback/stop" && !adapter.active(),
              "Manual/Stop cancels only our initiated operation");
        auto count = calls.size();
        adapter.update(target, 0, 1);
        check(calls.size() == count, "Cancellation prevents later automatic handoff");
        setup();
        adapter.start(root, plan, initial, false);
        for (int i = 0; i < 301; ++i)
            adapter.update(initial, 0, 1);
        check(adapter.phase() == PushbackPhase::Failed, "Connection timeout cancels departure");
        setup();
        adapter.start(root, plan, initial, false);
        import(adapter, initial);
        target.position = unproject(plan.origin, {100, -40});
        adapter.update(target, 0, 1);
        check(adapter.phase() == PushbackPhase::Failed, "Tow path deviation stops automatic departure");
        setup();
        adapter.start(root, plan, initial, false);
        import(adapter, initial);
        target = {plan.handoff, 0, 0, true};
        value("bp/op_complete") = 1;
        for (int i = 0; i < 3; ++i)
            adapter.update(target, 0, 1);
        value("bp/connected") = value("bp/started") = 0;
        target.position = initial.position;
        adapter.update(target, 0, 1);
        check(adapter.phase() == PushbackPhase::Failed,
              "Wrong endpoint cannot start taxi even with tug clear");
        std::cout << "BetterPushback lifecycle checks passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
}
