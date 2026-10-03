#include "AptDatabase.h"
#include "BetterPushbackAdapter.h"
#include "CenterlineNetwork.h"
#include "ControlAdapter.h"
#include "DsfLoader.h"
#include "PavementQuery.h"
#include "PlanningWorker.h"
#include "TaxiUI.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#if IBM
#include <windows.h>
#endif
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
namespace autotaxi {
namespace {
struct ScanResult {
    std::shared_ptr<AptDatabase> database;
    Airport airport;
    std::shared_ptr<const Airport> planningAirport;
    std::unique_ptr<PavementQuery> pavement;
};
struct Plugin {
    Config config;
    std::filesystem::path root, configPath;
    std::shared_ptr<AptDatabase> database;
    Airport airport;
    std::future<ScanResult> worker;
    std::unique_ptr<PavementQuery> pavement;
    struct RouteScan {
        std::string airportId;
        AircraftState origin;
        bool cancelled = false;
        std::unordered_map<std::string, Availability> items;
        unsigned revision = 0;
    };
    std::future<RouteScan> routeWorker;
    PlanningWorker planningWorker;
    std::optional<PlanningResult> completedPlanning;
    std::shared_ptr<const Airport> planningAirport;
    bool starting = false, handoffPlanning = false;
    std::shared_ptr<PlanningControl> routeScanCancellation;
    AircraftState lastRouteScan;
    bool routeScanNeeded = true;
    unsigned routeRevision = 0;
    struct PendingDeparture {
        Destination destination;
        bool clearance;
        GeoPoint origin;
        double settled = 0;
        std::optional<PushbackPlan> pushback;
        double handoffWait = 0;
        double retryWait = 0;
    };
    std::optional<PendingDeparture> pending;
    ControlOutput lastOutput;
    std::unique_ptr<TaxiUI> ui;
    ControlAdapter adapter;
    BetterPushbackAdapter tug;
    TaxiController controller;
    std::optional<TaxiController> previewController;
    TimingWorker timingWorker;
    TaxiMotionEstimator motionEstimator;
    double timingTime = 1;
    double timingPublishTime = 0;
    XPLMFlightLoopID before = nullptr, after = nullptr;
    XPLMMenuID menu = nullptr;
    XPLMCommandRef stopCommand = nullptr, disconnectCommand = nullptr, emergencyCommand = nullptr;
    XPLMDataRef paused = nullptr;
    int menuItem = -1;
    bool enabled = false, ready = false, autoScan = true;
    double uiTime = 0;
    std::string status = "Ready";
    GeoPoint lastPosition, lastScanPosition;
    bool hasPosition = false, hasScanPosition = false;
    void report(const std::string &text) {
        status = text;
        if (ui)
            ui->setStatus(text);
        XPLMDebugString(("[A350AutoTaxi] " + text + "\n").c_str());
    }
    bool loading() const {
        return worker.valid();
    }
    bool busy() const {
        return starting || pending.has_value() || adapter.ownsControls();
    }
    void cancelPlanning() {
        timingWorker.cancel();
        previewController.reset();
        if (ui)
            ui->setTiming({});
        planningWorker.cancel();
        completedPlanning.reset();
        if (routeScanCancellation)
            routeScanCancellation->cancelled.store(true, std::memory_order_relaxed);
        starting = handoffPlanning = false;
    }
    void updatePlanning(bool isPaused = false) {
        if (auto result = planningWorker.poll())
            completedPlanning = std::move(result);
        if (!completedPlanning || loading())
            return;
        if (isPaused && completedPlanning->plan && completedPlanning->request.kind != PlanningKind::Preview) {
            const auto text = "Route ready: resume simulator to continue";
            if (status != text)
                report(text);
            return;
        }
        auto completed = std::move(completedPlanning);
        completedPlanning.reset();
        const auto &request = completed->request;
        const bool preview = request.kind == PlanningKind::Preview;
        const bool handoff = request.kind == PlanningKind::Handoff;
        if ((preview && busy()) || (handoff && !pending) || (!preview && !handoff && !starting))
            return;
        if (handoff)
            handoffPlanning = false;
        bool routeReady = false;
        try {
            const auto current = adapter.state();
            if (!current.onGround || !valid(current.position) || !std::isfinite(current.trueHeading) ||
                !std::isfinite(current.speed) ||
                distance(current.position, request.state.position) >= (preview ? 30 : 1) ||
                std::abs(wrap180(current.trueHeading - request.state.trueHeading)) >= (preview ? 20 : 3))
                throw std::runtime_error("Aircraft moved during planning; request a new route");
            if (!completed->error.empty()) {
                ui->setDestinationAvailability(request.destination.label,
                                               {false, false, 0, completed->error});
                throw std::runtime_error(completed->error);
            }
            if (!preview && BetterPushbackAdapter::tugBusy())
                throw std::runtime_error("Finish the existing BetterPushback operation first");
            if (handoff && std::abs(current.speed) > .15)
                throw std::runtime_error("Aircraft must remain stopped for taxi handoff");
            auto departure = std::move(*completed->plan);
            auto &route = departure.route;
            if (handoff && route.requiresPushback)
                throw std::runtime_error("Forward taxi connection still requires pushback");
            ui->setRoute(route);
            ui->setDestinationAvailability(request.destination.label,
                                           {true, route.requiresPushback, route.length, {}});
            routeReady = true;
            if (preview) {
                if (!route.requiresPushback) {
                    previewController.emplace();
                    auto predictionConfig = config.controller;
                    previewController->start(route, predictionConfig);
                    motionEstimator.reset(predictionConfig);
                    timingTime = 1;
                }
                report("Preview: " + request.destination.label + " / " +
                       std::to_string(static_cast<int>(route.length)) + " m / ATC fallback " +
                       std::to_string(route.atcFallbackCount) + " / paint gaps " +
                       std::to_string(route.inferredGapCount) + " / guidance " +
                       (route.mainAxleOversteer ? "main-axle-oversteer" :
                        route.cockpitGuidance ? "cockpit" : "main-axle") +
                       " / planned oversteer " +
                       std::to_string(route.maximumOversteer) + " m");
                std::ostringstream path;
                path << "[A350AutoTaxi] Route:";
                for (int id : route.nodeIds)
                    path << ' ' << id;
                path << '\n';
                XPLMDebugString(path.str().c_str());
                return;
            }
            if (route.requiresPushback) {
                if (departure.pushback)
                    tug.start(root, *departure.pushback, current, adapter.ownsControls());
                pending = PendingDeparture{request.destination, request.config.route.runwayClearance,
                                           current.position, 0, std::move(departure.pushback)};
                starting = false;
                report(pending->pushback ? tug.status()
                                         : "Awaiting pushback: " + route.departure + " -> " + route.label);
                return;
            }
            auto settings = request.config;
            // Speed can change while a request is being calculated or while the tug is connected.
            settings.controller.taxiSpeed = config.controller.taxiSpeed;
            const auto message = "Taxi from " + route.departure + " to " + route.label;
            adapter.acquire(settings);
            try {
                controller.start(std::move(route), settings.controller);
                motionEstimator.reset(settings.controller);
                timingTime = 1;
            } catch (...) {
                adapter.release(false);
                throw;
            }
            starting = false;
            pending.reset();
            report(message);
            ui->setBusy(true, false);
        } catch (const std::exception &e) {
            if (handoff && pending) {
                if (pending->pushback)
                    tug.holdParkingBrake();
                pending->retryWait = 1;
                pending->settled = 0;
                const auto text = "Departure waiting: " + std::string(e.what());
                if (status != text)
                    report(text);
            } else {
                starting = false;
                if (!routeReady)
                    ui->clearRoute();
                ui->setBusy(busy(), loading());
                report(e.what());
            }
        }
    }
    void refreshRoutes(const AircraftState &state) {
        if (routeWorker.valid() || planningWorker.hasWork() || busy() || loading() || airport.id.empty() ||
            !ui->visible())
            return;
        if (!routeScanNeeded && distance(lastRouteScan.position, state.position) < 30 &&
            std::abs(wrap180(lastRouteScan.trueHeading - state.trueHeading)) < 20)
            return;
        routeScanNeeded = false;
        lastRouteScan = state;
        auto data = planningAirport;
        auto options = config.route;
        auto settings = config;
        options.runwayClearance = true;
        routeScanCancellation = std::make_shared<PlanningControl>();
        auto cancellation = routeScanCancellation;
        options.computation = cancellation;
        auto revision = routeRevision;
        routeWorker = std::async(
            std::launch::async, [data = std::move(data), state, options, settings, cancellation, revision] {
#if IBM
                SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
                RouteScan result;
                result.airportId = data->id;
                result.origin = state;
                result.revision = revision;
                for (const auto &d : destinations(*data))
                    if (d.kind == DestinationKind::Runway) {
                        if (cancellation->cancelled.load(std::memory_order_relaxed)) {
                            result.cancelled = true;
                            break;
                        }
                        Availability info;
                        try {
                            auto route = planDeparture(*data, state, d, settings, options).route;
                            info.reachable = true;
                            info.pushback = route.requiresPushback;
                            info.length = route.length;
                        } catch (const PlanningCancelled &) {
                            result.cancelled = true;
                            break;
                        } catch (const std::exception &e) {
                            info.reason = e.what();
                        }
                        result.items[d.label] = info;
                    }
                return result;
            });
    }
    void scan() {
        if (busy() || loading() || !ready)
            return;
        cancelPlanning();
        ++routeRevision;
        auto p = adapter.state().position;
        auto db = database;
        auto simulatorRoot = root;
        auto dsfToolPath = config.dsfToolPath;
        auto dsfPavements = config.dsfPavementResources;
        lastScanPosition = p;
        hasScanPosition = true;
        worker = std::async(std::launch::async, [p, db, simulatorRoot, dsfToolPath, dsfPavements]() mutable {
#if IBM
            SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
            if (!db) {
                db = std::make_shared<AptDatabase>();
                db->scan(simulatorRoot);
            }
            auto detected = db->nearest(p);
            DsfLoadResult dsf;
            std::ifstream source(std::filesystem::u8path(detected.source));
            if (source) {
                auto raw = parseAirport(source, detected.source, false);
                dsf = loadDsfPaintedLines(raw, simulatorRoot, dsfToolPath, dsfPavements);
                if (dsf.lines > 0 || dsf.contours > 0) {
                    raw.nodeAliases = detected.nodeAliases;
                    for (auto &ramp : raw.ramps)
                        for (const auto &other : detected.ramps)
                            if (distance(ramp.position, other.position) < 25)
                                ramp.aliases = other.aliases;
                    buildCenterlineNetwork(raw);
                    detected = std::move(raw);
                }
            }
            auto surface = std::make_unique<PavementQuery>(detected, p, true);
            auto snapshot = std::make_shared<const Airport>(detected);
            return ScanResult{db, std::move(detected), std::move(snapshot), std::move(surface)};
        });
        report(database ? "Detecting current airport..." : "Indexing active apt.dat files...");
        ui->setBusy(false, true);
    }
    void reload() {
        if (busy() || loading())
            return;
        try {
            auto loaded = loadConfig(configPath);
            adapter.initialize(loaded);
            config = loaded;
            ui->setControllerConfig(config.controller);
            ui->setRouteOptions(config.route);
            ui->setSpeed(config.controller.taxiSpeed / 0.514444);
            ready = true;
            database.reset();
            scan();
        } catch (const std::exception &e) {
            ready = false;
            report(e.what());
        }
    }
    void stop(const std::string &why = "Stopping taxi") {
        cancelPlanning();
        controller.stop();
        bool towing = tug.active();
        tug.cancel();
        pending.reset();
        if (adapter.ownsControls())
            adapter.beginStop();
        report(towing ? why + "; departure cancelled, finish tug disconnect in BetterPushback" : why);
        if (ui)
            ui->setBusy(adapter.ownsControls(), loading());
    }
    void disconnect() {
        cancelPlanning();
        controller.stop();
        bool towing = tug.active();
        tug.cancel();
        pending.reset();
        adapter.release(false);
        report(towing ? "Departure cancelled; finish tug disconnect in BetterPushback"
                      : "Manual control: steering/brakes released; thrust remains manual");
        if (ui)
            ui->setBusy(false, loading());
    }
    void goNow() {
        if (!pending) {
            report("GO NOW unavailable: no BetterPushback departure is pending");
            return;
        }
        if (pending->pushback && (tug.active() || BetterPushbackAdapter::tugBusy())) {
            report("GO NOW waiting: BetterPushback must disconnect first");
            return;
        }
        // The normal flight-loop handoff remains authoritative.  Reset its
        // retry/settling gates so the forward taxi request is submitted on
        // the next tick after the tug has cleared.
        pending->retryWait = 0;
        pending->handoffWait = 0;
        pending->settled = 2;
        report("GO NOW: starting forward taxi handoff");
    }
    void plan(const Destination &d, bool clearance, double speed, bool start) {
        if (!ready || busy() || loading())
            return;
        try {
            auto s = adapter.state();
            if (!s.onGround)
                throw std::runtime_error("Aircraft must be on ground");
            if (config.requireA350 && !adapter.isA350())
                throw std::runtime_error("Loaded aircraft ICAO is not A350");
            if (start && BetterPushbackAdapter::tugBusy())
                throw std::runtime_error("Finish the existing BetterPushback operation first");
            auto settings = config;
            settings.route.runwayClearance = start ? clearance : true;
            config.controller.taxiSpeed =
                std::clamp(speed * .514444, .514444, config.controller.maxTaxiSpeed);
            settings.controller.taxiSpeed = config.controller.taxiSpeed;
            std::ostringstream originLog;
            originLog << "[A350AutoTaxi] Planning origin: " << std::fixed << std::setprecision(8)
                      << s.position.lat << ',' << s.position.lon << " heading=" << std::setprecision(2)
                      << s.trueHeading << " relaxed pavement=" << settings.route.ignorePavementLimits
                      << " relaxed stand=" << settings.route.ignoreStandSize << '\n';
            XPLMDebugString(originLog.str().c_str());
            cancelPlanning();
            planningWorker.submit({start ? PlanningKind::Start : PlanningKind::Preview, d, s, settings});
            starting = start;
            if (routeScanCancellation)
                routeScanCancellation->cancelled.store(true, std::memory_order_relaxed);
            ui->clearRoute();
            ui->setBusy(busy(), false);
            report("Planning " + d.label + (start ? " for taxi..." : "..."));
        } catch (const std::exception &e) {
            starting = false;
            ui->setBusy(busy(), loading());
            ui->clearRoute();
            report(e.what());
        }
    }
    void routeSettings(const RouteOptions &options) {
        if (busy())
            return;
        cancelPlanning();
        config.route = options;
        ++routeRevision;
        routeScanNeeded = true;
        if (routeScanCancellation)
            routeScanCancellation->cancelled.store(true, std::memory_order_relaxed);
        ui->setAvailability({});
    }
    void speedChanged(double knots) {
        timingWorker.cancel();
        timingTime = 1;
        ui->setTiming({}, 0);
        config.controller.taxiSpeed = std::clamp(knots * .514444, .514444, config.controller.maxTaxiSpeed);
        controller.setTaxiSpeed(config.controller.taxiSpeed);
        ui->setControllerConfig(config.controller);
    }
    void emergencyBrake() {
        if (!controller.active() || pending || adapter.stopping())
            return;
        bool hold = !controller.emergencyBrake();
        controller.setEmergencyBrake(hold);
        timingWorker.cancel();
        timingTime = 1;
        ui->setTiming({});
        report(hold ? "Emergency brake held; route retained" : "Taxi resumed; thrust remains manual");
    }
    void advanceDeparture(const AircraftState &state, double dt) {
        if (!pending)
            return;
        if (!state.onGround || distance(pending->origin, state.position) > 500) {
            stop("Departure cancelled: aircraft repositioned");
            return;
        }
        if (pending->pushback) {
            tug.update(state, adapter.actualSteer(), dt);
            if (tug.phase() == PushbackPhase::Failed) {
                auto failure = tug.status();
                pending.reset();
                ui->setBusy(false, loading());
                report(failure);
                return;
            }
            if (tug.phase() != PushbackPhase::Complete) {
                if (status != tug.status())
                    report(tug.status());
                return;
            }
            if (BetterPushbackAdapter::tugBusy() || std::abs(state.speed) > .15 ||
                distance(state.position, pending->pushback->handoff) > 6 ||
                std::abs(wrap180(state.trueHeading - pending->pushback->heading)) > 3) {
                stop("Departure cancelled: tow handoff no longer valid");
                return;
            }
            pending->handoffWait += dt;
            if (pending->handoffWait > 60) {
                tug.holdParkingBrake();
                stop("Taxi handoff timed out; parking brake held, check engines/brakes/nosewheel");
                return;
            }
        } else {
            bool moving = std::abs(state.speed) >= .15 || BetterPushbackAdapter::tugBusy();
            pending->settled = moving ? 0 : pending->settled + dt;
            if (pending->settled < 2 || distance(pending->origin, state.position) < 10)
                return;
        }
        pending->retryWait = std::max(0.0, pending->retryWait - dt);
        if (handoffPlanning || pending->retryWait > 0)
            return;
        auto request = *pending;
        pending->settled = 1.5;
        try {
            auto settings = config;
            settings.route.runwayClearance = request.clearance;
            if (request.pushback) {
                settings.route.requiredDepartureNode = request.pushback->nodeId;
                settings.route.maxInitialTurnDegrees = 15;
            }
            planningWorker.submit({PlanningKind::Handoff, request.destination, state, settings});
            handoffPlanning = true;
            report("Planning forward taxi handoff...");
        } catch (const std::exception &e) {
            if (pending->pushback) {
                tug.holdParkingBrake();
            }
            pending->retryWait = 1;
            std::string text = "Departure waiting: " + std::string(e.what());
            if (text != status)
                report(text);
        }
    }
    float tick(float dt) {
        if (!enabled)
            return -1;
        if (autoScan && ready && !loading() && !busy() && adapter.state().onGround) {
            autoScan = false;
            scan();
        }
        if (worker.valid() && worker.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            try {
                auto result = worker.get();
                database = std::move(result.database);
                if (distance(adapter.state().position, lastScanPosition) > 500)
                    autoScan = true;
                else {
                    airport = std::move(result.airport);
                    pavement = std::move(result.pavement);
                    planningAirport = std::move(result.planningAirport);
                    planningWorker.setAirport(planningAirport);
                    // Airport fitting happens inside setAirport(). Push the current
                    // simulator position first so the initial view includes the aircraft.
                    ui->setTelemetry(adapter.state(), lastOutput, adapter.actualSteer(), pending.has_value());
                    ui->setAirport(airport);
                    routeScanNeeded = true;
                    XPLMDebugString(("[A350AutoTaxi] Airport source: " + airport.source + "\n").c_str());
                    XPLMDebugString(
                        ("[A350AutoTaxi] Ground markings: " + std::to_string(airport.groundLines.size()) +
                         "; hold-short points: " + std::to_string(airport.holdShortPoints.size()) +
                         "; airport signs: " + std::to_string(airport.signs.size()) +
                         "; ATC fallback links: " + std::to_string(airport.atcFallbackEdges) +
                         "; repaired paint gaps: " + std::to_string(airport.centerlineGapLinks) + "\n")
                            .c_str());
                    if (!airport.dsfStatus.empty())
                        XPLMDebugString(("[A350AutoTaxi] " + airport.dsfStatus + "\n").c_str());
                    if (airport.edges.empty())
                        report("Airport " + airport.id + " has no taxi network (1201/1202)");
                    else
                        report("Airport detected: " + airport.id);
                }
            } catch (const std::exception &e) {
                report(e.what());
            }
            ui->setBusy(busy(), false);
        }
        if (!ready)
            return -1;
        if (routeWorker.valid() &&
            routeWorker.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            try {
                auto result = routeWorker.get();
                auto current = adapter.state();
                bool currentOrigin = distance(result.origin.position, current.position) < 30 &&
                                     std::abs(wrap180(result.origin.trueHeading - current.trueHeading)) < 20;
                if (!result.cancelled && currentOrigin && result.airportId == airport.id &&
                    result.revision == routeRevision)
                    ui->setAvailability(result.items);
                else
                    routeScanNeeded = true;
            } catch (const std::exception &e) {
                XPLMDebugString(
                    ("[A350AutoTaxi] Route availability: " + std::string(e.what()) + "\n").c_str());
            }
        }
        bool isPaused = paused && XPLMGetDatai(paused);
        auto state = adapter.state();
        if (!isPaused) {
            advanceDeparture(state, std::clamp(static_cast<double>(dt), 0.0, 1.0));
        }
        updatePlanning(isPaused);
        if (hasPosition && adapter.ownsControls() && distance(lastPosition, state.position) > 50)
            stop("Aircraft repositioned; stopping");
        lastPosition = state.position;
        hasPosition = true;
        if (isPaused) {
            // Preserve commands while paused; do not advance steering or watchdog timers.
        } else if (adapter.stopping()) {
            if (adapter.updateStop(std::clamp(static_cast<double>(dt), 0.001, 0.1)))
                ui->setBusy(false, loading());
        } else if (controller.active()) {
            motionEstimator.observe(state.speed, lastOutput.brake, dt);
            auto out = controller.update(state, dt);
            lastOutput = out;
            if (out.phase == TaxiPhase::Fault)
                stop("Fault: " + out.reason);
            else if (out.phase == TaxiPhase::Complete)
                stop("Complete: " + airport.id + " / parking brake held");
            adapter.apply(out);
        }
        if (!isPaused)
            timingPublishTime += std::clamp(static_cast<double>(dt), 0., 1.);
        if (auto timing = timingWorker.poll()) {
            ui->setTiming(std::move(*timing), std::max(.001, timingPublishTime));
            timingPublishTime = 0;
        }
        if (!isPaused)
            timingTime += std::clamp(static_cast<double>(dt), 0., 1.);
        if (timingTime >= 1 && !timingWorker.hasWork() && !pending && !adapter.stopping() && ui->visible()) {
            if (controller.active() && !controller.emergencyBrake()) {
                timingWorker.submit(controller, state, motionEstimator.model());
                timingTime = 0;
            } else if (previewController && !busy()) {
                previewController->setTaxiSpeed(config.controller.taxiSpeed);
                timingWorker.submit(*previewController, state,
                                    {config.controller.acceleration, config.controller.brakeAuthority});
                timingTime = 0;
            }
        }
        uiTime += dt;
        if (uiTime > 0.25) {
            uiTime = 0;
            if (const auto progress = planningWorker.progress()) {
                auto text = "Planning " + progress->destination + ": ";
                if (progress->queued)
                    text += "cancelling previous request...";
                else {
                    text += progress->stage == PlanningStage::Forward ? "forward taxi" : "pushback";
                    if (progress->candidates)
                        text += " " + std::to_string(progress->candidate) + "/" +
                                std::to_string(progress->candidates);
                    text += " / " + std::to_string(static_cast<int>(progress->seconds)) + " s";
                }
                if (text != status) {
                    status = text;
                    ui->setStatus(text);
                }
            }
            lastOutput.pavementChecked = pavement && !airport.pavements.empty() && state.onGround;
            lastOutput.outsideKnownPavement =
                lastOutput.pavementChecked &&
                !pavement->wheelEnvelope(state.position, state.trueHeading, config.controller.wheelbase,
                                         config.controller.mainAxleAft, config.route.mainGearHalfSpan);
            ui->setTelemetry(state, lastOutput, adapter.actualSteer(), pending.has_value());
            if (routeScanCancellation &&
                (busy() || loading() || !ui->visible() ||
                 distance(lastRouteScan.position, state.position) >= 30 ||
                 std::abs(wrap180(lastRouteScan.trueHeading - state.trueHeading)) >= 20))
                routeScanCancellation->cancelled.store(true, std::memory_order_relaxed);
            refreshRoutes(state);
            // Detect airport changes automatically while the plugin is idle and on ground.
            if (state.onGround && !busy() && !loading() && hasScanPosition &&
                distance(lastScanPosition, state.position) > 500)
                scan();
        }
        return -1;
    }
};
std::unique_ptr<Plugin> plugin;
float beforeCallback(float dt, float, int, void *) {
    try {
        return plugin ? plugin->tick(dt) : -1;
    } catch (const std::exception &e) {
        if (plugin)
            plugin->stop(std::string("Internal error: ") + e.what());
        return -1;
    }
}
float afterCallback(float dt, float, int, void *) {
    if (plugin && plugin->enabled && plugin->adapter.ownsControls() &&
        !(plugin->paused && XPLMGetDatai(plugin->paused))) {
        auto problem = plugin->adapter.monitor(std::clamp(static_cast<double>(dt), 0.001, 0.1));
        if (!problem.empty())
            plugin->stop("Fault: " + problem);
    }
    return -1;
}
void menuCallback(void *, void *) {
    if (plugin && plugin->ui)
        plugin->ui->show();
}
int stopCallback(XPLMCommandRef, XPLMCommandPhase phase, void *) {
    if (phase == xplm_CommandBegin && plugin)
        plugin->stop();
    return 0;
}
int disconnectCallback(XPLMCommandRef, XPLMCommandPhase phase, void *) {
    if (phase == xplm_CommandBegin && plugin)
        plugin->disconnect();
    return 0;
}
int emergencyCallback(XPLMCommandRef, XPLMCommandPhase phase, void *) {
    if (phase == xplm_CommandBegin && plugin)
        plugin->emergencyBrake();
    return 0;
}
} // namespace
} // namespace autotaxi
using namespace autotaxi;
PLUGIN_API int XPluginStart(char *name, char *signature, char *description) {
    std::strcpy(name, "FF A350 AutoTaxi");
    std::strcpy(signature, "org.autotaxi.ff.a350");
    std::strcpy(description, "apt.dat taxi routing and nosewheel control");
    plugin = std::make_unique<Plugin>();
    plugin->paused = XPLMFindDataRef("sim/time/paused");
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
    XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);
    char path[2048]{};
    XPLMGetSystemPath(path);
    plugin->root = std::filesystem::u8path(path);
    XPLMGetPluginInfo(XPLMGetMyID(), nullptr, path, nullptr, nullptr);
    plugin->configPath = std::filesystem::u8path(path).parent_path().parent_path() / "A350AutoTaxi.ini";
    try {
        plugin->config = loadConfig(plugin->configPath);
        plugin->adapter.initialize(plugin->config);
        plugin->ready = true;
    } catch (const std::exception &e) {
        plugin->status = e.what();
    }
    plugin->ui = std::make_unique<TaxiUI>(
        UIActions{[] { plugin->scan(); }, [] { plugin->reload(); }, [] { plugin->stop(); },
                  [] { plugin->disconnect(); }, [] { plugin->goNow(); },
                  [](const Destination &d, bool clearance, double speed, bool start) {
                      plugin->plan(d, clearance, speed, start);
                  },
                  [](const RouteOptions &options) { plugin->routeSettings(options); },
                  [](double speed) { plugin->speedChanged(speed); }, [] { plugin->emergencyBrake(); }});
    plugin->ui->setStatus(plugin->status);
    plugin->ui->setControllerConfig(plugin->config.controller);
    plugin->ui->setRouteOptions(plugin->config.route);
    plugin->ui->setSpeed(plugin->config.controller.taxiSpeed / 0.514444);
    plugin->menuItem = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "FF A350 AutoTaxi", nullptr, 0);
    plugin->menu =
        XPLMCreateMenu("FF A350 AutoTaxi", XPLMFindPluginsMenu(), plugin->menuItem, menuCallback, nullptr);
    XPLMAppendMenuItem(plugin->menu, "Open AutoTaxi", nullptr, 0);
    plugin->stopCommand = XPLMCreateCommand("autotaxi/a350/stop", "Stop A350 AutoTaxi and brake to a halt");
    XPLMRegisterCommandHandler(plugin->stopCommand, stopCallback, 1, nullptr);
    plugin->disconnectCommand =
        XPLMCreateCommand("autotaxi/a350/disconnect", "Immediately return A350 AutoTaxi controls to pilot");
    XPLMRegisterCommandHandler(plugin->disconnectCommand, disconnectCallback, 1, nullptr);
    plugin->emergencyCommand =
        XPLMCreateCommand("autotaxi/a350/emergency_brake", "Hold/resume AutoTaxi without discarding route");
    XPLMRegisterCommandHandler(plugin->emergencyCommand, emergencyCallback, 1, nullptr);
    XPLMCreateFlightLoop_t params{};
    params.structSize = sizeof(params);
    params.phase = xplm_FlightLoop_Phase_BeforeFlightModel;
    params.callbackFunc = beforeCallback;
    plugin->before = XPLMCreateFlightLoop(&params);
    params.phase = xplm_FlightLoop_Phase_AfterFlightModel;
    params.callbackFunc = afterCallback;
    plugin->after = XPLMCreateFlightLoop(&params);
    XPLMScheduleFlightLoop(plugin->before, -1, 1);
    XPLMScheduleFlightLoop(plugin->after, -1, 1);
    return 1;
}
PLUGIN_API int XPluginEnable() {
    if (plugin)
        plugin->enabled = true;
    return 1;
}
PLUGIN_API void XPluginDisable() {
    if (plugin) {
        plugin->enabled = false;
        plugin->cancelPlanning();
        plugin->controller.stop();
        plugin->tug.cancel();
        plugin->pending.reset();
        plugin->adapter.release(false);
        plugin->ui->setBusy(false, plugin->loading());
    }
}
PLUGIN_API void XPluginStop() {
    if (!plugin)
        return;
    XPluginDisable();
    XPLMDestroyFlightLoop(plugin->before);
    XPLMDestroyFlightLoop(plugin->after);
    XPLMUnregisterCommandHandler(plugin->stopCommand, stopCallback, 1, nullptr);
    XPLMDestroyMenu(plugin->menu);
    XPLMUnregisterCommandHandler(plugin->disconnectCommand, disconnectCallback, 1, nullptr);
    XPLMUnregisterCommandHandler(plugin->emergencyCommand, emergencyCallback, 1, nullptr);
    XPLMRemoveMenuItem(XPLMFindPluginsMenu(), plugin->menuItem);
    plugin.reset();
}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int message, void *parameter) {
    if (!plugin)
        return;
    if ((message == XPLM_MSG_PLANE_LOADED && reinterpret_cast<intptr_t>(parameter) == 0) ||
        message == XPLM_MSG_AIRPORT_LOADED) {
        plugin->cancelPlanning();
        plugin->controller.stop();
        plugin->tug.cancel();
        plugin->pending.reset();
        plugin->adapter.release(false);
        plugin->hasPosition = false;
        plugin->ui->setBusy(false, plugin->loading());
        try {
            plugin->config = loadConfig(plugin->configPath);
            plugin->adapter.initialize(plugin->config);
            plugin->ui->setControllerConfig(plugin->config.controller);
            plugin->ui->setRouteOptions(plugin->config.route);
            plugin->ui->setSpeed(plugin->config.controller.taxiSpeed / .514444);
            plugin->ready = true;
        } catch (const std::exception &e) {
            plugin->ready = false;
            plugin->report(e.what());
        }
        plugin->autoScan = true;
    }
}
