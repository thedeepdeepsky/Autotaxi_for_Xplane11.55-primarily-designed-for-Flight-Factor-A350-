#include "../src/Plugin.cpp"
#include <iostream>
#include <thread>
using namespace autotaxi;
namespace {
const auto mainThread = std::this_thread::get_id();
AircraftState aircraft{{31, 121}, 0, 0, true};
bool simulatorPaused = false, towBusy = false, towComplete = false;
int acquired = 0, parkingHolds = 0;
bool previewAvailable = false;
double acquiredSpeed = 0;
void check(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}
void onMainThread() {
    check(std::this_thread::get_id() == mainThread, "SDK/control accessed by worker");
}
template <class F> void until(F done) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!done()) {
        check(std::chrono::steady_clock::now() < end, "Plugin planning timed out");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
struct Gate {
    std::promise<void> release;
    std::shared_future<void> wait = release.get_future().share();
    std::atomic<bool> entered{false};
    ~Gate() {
        if (!opened)
            open();
    }
    bool opened = false;
    void open() {
        if (!opened) {
            opened = true;
            release.set_value();
        }
    }
};
struct Unlock {
    std::shared_ptr<Gate> gate;
    ~Unlock() {
        gate->open();
    }
};
void prepare(Plugin &p, const std::shared_ptr<Gate> &gate, bool reverse = false) {
    aircraft = {{31, 121}, 0, 0, true};
    acquired = parkingHolds = 0;
    previewAvailable = false;
    acquiredSpeed = 0;
    towBusy = towComplete = simulatorPaused = false;
    p.ready = p.enabled = true;
    p.autoScan = false;
    p.paused = reinterpret_cast<XPLMDataRef>(1);
    p.ui = std::make_unique<TaxiUI>(UIActions{});
    p.planningAirport = std::make_shared<const Airport>();
    p.planningWorker = PlanningWorker([gate, reverse](const Airport &, const PlanningRequest &request) {
        check(std::this_thread::get_id() != mainThread, "Planner ran on main thread");
        gate->entered = true;
        gate->wait.wait();
        Route route;
        route.origin = request.state.position;
        route.initialHeading = route.finalHeading = request.state.trueHeading;
        route.label = request.destination.label;
        route.points = {{0, 0}, {0, 100}};
        route.length = 100;
        if (reverse && request.kind != PlanningKind::Handoff) {
            route.requiresPushback = true;
            PushbackPlan pushback;
            pushback.handoff = request.state.position;
            pushback.heading = request.state.trueHeading;
            pushback.nodeId = 42;
            return DeparturePlan{route, pushback};
        }
        if (request.kind == PlanningKind::Handoff)
            check(request.config.route.requiredDepartureNode == 42 &&
                      request.config.route.maxInitialTurnDegrees == 15,
                  "Tow handoff lost required node/heading constraint");
        return DeparturePlan{route, {}};
    });
    p.planningWorker.setAirport(p.planningAirport);
}
const Destination destination{DestinationKind::Node, "Test end", 2};
void startAndLiveSpeed() {
    auto gate = std::make_shared<Gate>();
    Plugin p;
    Unlock unlock{gate};
    prepare(p, gate);
    p.plan(destination, true, 10, true);
    until([&] { return gate->entered.load(); });
    check(p.busy() && acquired == 0, "Start acquired controls before a route was calculated");
    for (int i = 0; i < 20; ++i)
        p.tick(.01f);
    check(acquired == 0, "Flight loop blocked or acquired an unfinished route");
    p.speedChanged(7);
    simulatorPaused = true;
    gate->open();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    p.tick(.01f);
    check(acquired == 0, "Paused simulator engaged automatic controls");
    simulatorPaused = false;
    until([&] {
        p.tick(.01f);
        return p.controller.active();
    });
    check(acquired == 1 && std::abs(acquiredSpeed - 7 * .514444) < 1e-9,
          "Latest speed was lost during background start");
}
void invalidatedStart(bool moved) {
    auto gate = std::make_shared<Gate>();
    Plugin p;
    Unlock unlock{gate};
    prepare(p, gate);
    p.plan(destination, true, 10, true);
    until([&] { return gate->entered.load(); });
    if (moved)
        aircraft.position = unproject(aircraft.position, {5, 0});
    else
        p.disconnect();
    gate->open();
    until([&] {
        p.tick(.01f);
        return !p.planningWorker.hasWork();
    });
    check(acquired == 0 && !p.busy() && !p.controller.active(),
          "Moved/cancelled start applied stale controls");
}
void pausedPreview() {
    auto gate = std::make_shared<Gate>();
    Plugin p;
    Unlock unlock{gate};
    prepare(p, gate);
    simulatorPaused = true;
    p.plan(destination, true, 10, false);
    gate->open();
    until([&] {
        p.tick(.01f);
        return previewAvailable;
    });
    check(acquired == 0 && !p.completedPlanning && !p.planningWorker.hasWork() &&
              p.status.find("Preview:") == 0,
          "Paused preview did not finish/update destination independently of engagement");
}
void pushbackHandoff(bool cancelled) {
    auto gate = std::make_shared<Gate>();
    Plugin p;
    Unlock unlock{gate};
    prepare(p, gate, true);
    p.plan(destination, true, 10, true);
    gate->open();
    until([&] {
        p.tick(.01f);
        return p.pending.has_value();
    });
    check(acquired == 0 && p.tug.active(), "Taxi engaged while pushback was pending");
    towComplete = true;
    aircraft.speed = .3;
    p.tick(.01f);
    check(!p.pending && acquired == 0, "Moving tow handoff was accepted");
    // Restart at a stationary, valid handoff and cancel a still-running calculation.
    prepare(p, gate, true);
    p.plan(destination, true, 10, true);
    until([&] {
        p.tick(.01f);
        return p.pending.has_value();
    });
    towComplete = true;
    p.advanceDeparture(aircraft, .01);
    check(p.handoffPlanning && acquired == 0, "Tow handoff was planned synchronously");
    if (cancelled)
        p.disconnect();
    else
        p.speedChanged(6);
    until([&] {
        p.tick(.01f);
        return !p.planningWorker.hasWork();
    });
    check(cancelled ? acquired == 0 && !p.controller.active()
                    : acquired == 1 && p.controller.active() && std::abs(acquiredSpeed - 6 * .514444) < 1e-9,
          "Handoff ignored cancellation or latest speed");
}
} // namespace
namespace autotaxi {
TaxiUI::TaxiUI(UIActions actions) : actions_(std::move(actions)) {}
TaxiUI::~TaxiUI() = default;
MapRenderer::~MapRenderer() = default;
void TaxiUI::show() {
    onMainThread();
}
bool TaxiUI::visible() const {
    onMainThread();
    return false;
}
void TaxiUI::setAirport(const Airport &) {
    onMainThread();
}
void TaxiUI::setStatus(const std::string &) {
    onMainThread();
}
void TaxiUI::setBusy(bool, bool) {
    onMainThread();
}
void TaxiUI::setRoute(const Route &) {
    onMainThread();
}
void TaxiUI::clearRoute() {
    onMainThread();
}
void TaxiUI::setTiming(RouteTiming, double) {
    onMainThread();
}
void TaxiUI::setAvailability(const std::unordered_map<std::string, Availability> &) {
    onMainThread();
}
void TaxiUI::setDestinationAvailability(const std::string &, const Availability &availability) {
    onMainThread();
    previewAvailable = availability.reachable;
}
void TaxiUI::setTelemetry(const AircraftState &, const ControlOutput &, double, bool) {
    onMainThread();
}
void TaxiUI::setSpeed(double) {
    onMainThread();
}
void TaxiUI::setControllerConfig(const ControllerConfig &) {
    onMainThread();
}
void TaxiUI::setRouteOptions(const RouteOptions &) {
    onMainThread();
}
void ControlAdapter::initialize(const Config &) {
    onMainThread();
}
AircraftState ControlAdapter::state() const {
    onMainThread();
    return aircraft;
}
bool ControlAdapter::isA350() const {
    onMainThread();
    return true;
}
void ControlAdapter::acquire(const Config &config) {
    onMainThread();
    owned_ = true;
    ++acquired;
    acquiredSpeed = config.controller.taxiSpeed;
}
void ControlAdapter::apply(const ControlOutput &) {
    onMainThread();
}
std::string ControlAdapter::monitor(double) {
    onMainThread();
    return {};
}
void ControlAdapter::beginStop() {
    onMainThread();
    stopping_ = owned_;
}
bool ControlAdapter::updateStop(double) {
    onMainThread();
    owned_ = stopping_ = false;
    return true;
}
void ControlAdapter::release(bool) {
    onMainThread();
    owned_ = stopping_ = false;
}
double ControlAdapter::actualSteer() const {
    onMainThread();
    return 0;
}
void BetterPushbackAdapter::start(const std::filesystem::path &, const PushbackPlan &, const AircraftState &,
                                  bool) {
    onMainThread();
    phase_ = PushbackPhase::Connecting;
}
void BetterPushbackAdapter::update(const AircraftState &, double, double) {
    onMainThread();
    if (towComplete)
        phase_ = PushbackPhase::Complete;
}
void BetterPushbackAdapter::cancel() {
    onMainThread();
    phase_ = PushbackPhase::Idle;
}
bool BetterPushbackAdapter::active() const {
    onMainThread();
    return phase_ == PushbackPhase::Connecting;
}
void BetterPushbackAdapter::holdParkingBrake() {
    onMainThread();
    ++parkingHolds;
}
bool BetterPushbackAdapter::tugBusy() {
    onMainThread();
    return towBusy;
}
} // namespace autotaxi
extern "C" {
void XPLMDebugString(const char *) {
    onMainThread();
}
int XPLMGetDatai(XPLMDataRef) {
    onMainThread();
    return simulatorPaused;
}
XPLMDataRef XPLMFindDataRef(const char *) {
    onMainThread();
    return reinterpret_cast<XPLMDataRef>(1);
}
void XPLMEnableFeature(const char *, int) {}
void XPLMGetSystemPath(char *path) {
    std::strcpy(path, ".");
}
XPLMPluginID XPLMGetMyID() {
    return 1;
}
void XPLMGetPluginInfo(XPLMPluginID, char *, char *path, char *, char *) {
    std::strcpy(path, "./64/win.xpl");
}
XPLMMenuID XPLMFindPluginsMenu() {
    return nullptr;
}
int XPLMAppendMenuItem(XPLMMenuID, const char *, void *, int) {
    return 1;
}
XPLMMenuID XPLMCreateMenu(const char *, XPLMMenuID, int, XPLMMenuHandler_f, void *) {
    return nullptr;
}
void XPLMDestroyMenu(XPLMMenuID) {}
void XPLMRemoveMenuItem(XPLMMenuID, int) {}
XPLMCommandRef XPLMCreateCommand(const char *, const char *) {
    return nullptr;
}
void XPLMRegisterCommandHandler(XPLMCommandRef, XPLMCommandCallback_f, int, void *) {}
void XPLMUnregisterCommandHandler(XPLMCommandRef, XPLMCommandCallback_f, int, void *) {}
XPLMFlightLoopID XPLMCreateFlightLoop(XPLMCreateFlightLoop_t *) {
    return nullptr;
}
void XPLMDestroyFlightLoop(XPLMFlightLoopID) {}
void XPLMScheduleFlightLoop(XPLMFlightLoopID, float, int) {}
}
int main() {
    try {
        startAndLiveSpeed();
        invalidatedStart(false);
        invalidatedStart(true);
        pausedPreview();
        pushbackHandoff(false);
        pushbackHandoff(true);
        std::cout
            << "Plugin asynchronous start/handoff, pause, movement, cancellation and live speed passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
