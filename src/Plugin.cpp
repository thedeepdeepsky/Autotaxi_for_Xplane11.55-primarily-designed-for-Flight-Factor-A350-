#include "AptDatabase.h"
#include "BetterPushbackAdapter.h"
#include "CenterlineNetwork.h"
#include "ControlAdapter.h"
#include "DsfLoader.h"
#include "TaxiUI.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include <algorithm>
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
};
struct DeparturePlan {
    Route route;
    std::optional<PushbackPlan> pushback;
};
DeparturePlan planDeparture(const Airport &airport, const AircraftState &state, const Destination &d,
                            const Config &config, const RouteOptions &options) {
    std::optional<Route> route;
    std::string forwardError;
    try {
        route = planRoute(airport, state.position, state.trueHeading, d, options);
    } catch (const std::exception &e) {
        if (!config.automaticPushback)
            throw;
        forwardError = e.what();
    }
    if (config.automaticPushback && (!route || route->requiresPushback)) {
        try {
            auto tow = planPushback(airport, state, d, options, config.controller);
            return {tow.preview, std::move(tow)};
        } catch (const std::exception &e) {
            if (forwardError.empty())
                throw;
            throw std::runtime_error(forwardError + " / Pushback: " + e.what());
        }
    }
    return {std::move(*route), std::nullopt};
}
struct Plugin {
    Config config;
    std::filesystem::path root, configPath;
    std::shared_ptr<AptDatabase> database;
    Airport airport;
    std::future<ScanResult> worker;
    struct RouteScan {
        std::string airportId;
        std::unordered_map<std::string, Availability> items;
    };
    std::future<RouteScan> routeWorker;
    AircraftState lastRouteScan;
    bool routeScanNeeded = true;
    struct PendingDeparture {
        Destination destination;
        bool clearance;
        double speed;
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
    XPLMFlightLoopID before = nullptr, after = nullptr;
    XPLMMenuID menu = nullptr;
    XPLMCommandRef stopCommand = nullptr, disconnectCommand = nullptr;
    XPLMDataRef paused = nullptr;
    int menuItem = -1;
    bool enabled = false, ready = false, autoScan = true;
    double uiTime = 0;
    std::string status = "Ready", lastControls;
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
        return pending.has_value() || adapter.ownsControls();
    }
    void refreshRoutes(const AircraftState &state) {
        if (routeWorker.valid() || busy() || loading() || airport.id.empty())
            return;
        if (!routeScanNeeded && distance(lastRouteScan.position, state.position) < 30 &&
            std::abs(wrap180(lastRouteScan.trueHeading - state.trueHeading)) < 20)
            return;
        routeScanNeeded = false;
        lastRouteScan = state;
        auto data = airport;
        auto options = config.route;
        auto settings = config;
        options.runwayClearance = true;
        routeWorker = std::async(std::launch::async, [data, state, options, settings] {
            RouteScan result;
            result.airportId = data.id;
            for (const auto &d : destinations(data))
                if (d.kind == DestinationKind::Runway) {
                    Availability info;
                    try {
                        auto route = planDeparture(data, state, d, settings, options).route;
                        info.reachable = true;
                        info.pushback = route.requiresPushback;
                        info.length = route.length;
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
        auto p = adapter.state().position;
        auto db = database;
        auto simulatorRoot = root;
        auto dsfToolPath = config.dsfToolPath;
        lastScanPosition = p;
        hasScanPosition = true;
        worker = std::async(std::launch::async, [p, db, simulatorRoot, dsfToolPath]() mutable {
            if (!db) {
                db = std::make_shared<AptDatabase>();
                db->scan(simulatorRoot);
            }
            auto detected = db->nearest(p);
            DsfLoadResult dsf;
            std::ifstream source(std::filesystem::u8path(detected.source));
            if (source) {
                auto raw = parseAirport(source, detected.source, false);
                dsf = loadDsfPaintedLines(raw, simulatorRoot, dsfToolPath);
                if (dsf.lines > 0) {
                    raw.nodeAliases = detected.nodeAliases;
                    for (auto &ramp : raw.ramps)
                        for (const auto &other : detected.ramps)
                            if (distance(ramp.position, other.position) < 25)
                                ramp.aliases = other.aliases;
                    buildCenterlineNetwork(raw);
                    detected = std::move(raw);
                }
            }
            return ScanResult{db, std::move(detected)};
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
        controller.stop();
        bool towing = tug.active();
        tug.cancel();
        pending.reset();
        adapter.release(false);
        report(towing ? "Departure cancelled; finish tug disconnect in BetterPushback"
                      : "Manual control: overrides released, throttle idle");
        if (ui)
            ui->setBusy(false, loading());
    }
    void plan(const Destination &d, bool clearance, double speed, bool start) {
        if (!ready || busy() || loading())
            return;
        bool routeReady = false;
        try {
            auto s = adapter.state();
            if (!s.onGround)
                throw std::runtime_error("Aircraft must be on ground");
            if (config.requireA350 && !adapter.isA350())
                throw std::runtime_error("Loaded aircraft ICAO is not A350");
            if (start && BetterPushbackAdapter::tugBusy())
                throw std::runtime_error("Finish the existing BetterPushback operation first");
            auto options = config.route;
            options.runwayClearance = start ? clearance : true;
            auto departure = planDeparture(airport, s, d, config, options);
            auto route = departure.route;
            ui->setRoute(route);
            routeReady = true;
            auto settings = config;
            settings.controller.taxiSpeed = speed * 0.514444;
            if (start) {
                if (route.requiresPushback) {
                    if (departure.pushback)
                        tug.start(root, *departure.pushback, s, adapter.ownsControls());
                    pending =
                        PendingDeparture{d, clearance, speed, s.position, 0, std::move(departure.pushback)};
                    ui->setBusy(true, false);
                    report(pending->pushback ? tug.status()
                                             : "Awaiting pushback: " + route.departure + " -> " + d.label);
                    return;
                }
                adapter.acquire(settings);
                try {
                    controller.start(std::move(route), settings.controller);
                } catch (...) {
                    adapter.release(false);
                    throw;
                }
                lastControls = adapter.description();
                report("Taxi to " + d.label);
                ui->setBusy(true, false);
            } else {
                std::ostringstream info;
                info << "Preview: " << d.label << " / " << std::fixed << std::setprecision(0) << route.length
                     << " m / " << route.nodeIds.size() << " nodes";
                if (route.runway)
                    info << " / heading " << std::setprecision(1) << route.finalHeading;
                if (departure.pushback)
                    info << " / tow " << std::setprecision(0) << departure.pushback->length << " m -> Node "
                         << departure.pushback->nodeId << " / tow heading " << std::setprecision(1)
                         << departure.pushback->heading;
                if (route.atcFallbackCount)
                    info << " / ATC fallback: " << route.atcFallbackCount << " links";
                report(info.str());
                std::ostringstream path;
                path << "[A350AutoTaxi] Route:";
                for (int id : route.nodeIds)
                    path << ' ' << id;
                path << "\n";
                XPLMDebugString(path.str().c_str());
            }
        } catch (const std::exception &e) {
            if (!routeReady)
                ui->clearRoute();
            report(e.what());
        }
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
            pending->retryWait = std::max(0.0, pending->retryWait - dt);
            if (pending->retryWait > 0)
                return;
        } else {
            bool moving = std::abs(state.speed) >= .15 || BetterPushbackAdapter::tugBusy();
            pending->settled = moving ? 0 : pending->settled + dt;
            if (pending->settled < 2 || distance(pending->origin, state.position) < 10)
                return;
        }
        auto request = *pending;
        pending->settled = 1.5;
        try {
            auto options = config.route;
            options.runwayClearance = request.clearance;
            if (request.pushback) {
                options.requiredDepartureNode = request.pushback->nodeId;
                options.maxInitialTurnDegrees = 15;
            }
            auto route = planRoute(airport, state.position, state.trueHeading, request.destination, options);
            if (route.requiresPushback)
                return;
            auto settings = config;
            settings.controller.taxiSpeed = request.speed * 0.514444;
            adapter.acquire(settings);
            try {
                controller.start(route, settings.controller);
            } catch (...) {
                adapter.release(false);
                throw;
            }
            pending.reset();
            ui->setRoute(route);
            report("Taxi from " + route.departure + " to " + route.label);
        } catch (const std::exception &e) {
            if (pending->pushback) {
                tug.holdParkingBrake();
                pending->retryWait = 1;
            }
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
                    // Airport fitting happens inside setAirport(). Push the current
                    // simulator position first so the initial view includes the aircraft.
                    ui->setTelemetry(adapter.state(), lastOutput, adapter.actualSteer(), pending.has_value());
                    ui->setAirport(airport);
                    routeScanNeeded = true;
                    XPLMDebugString(("[A350AutoTaxi] Airport source: " + airport.source + "\n").c_str());
                    XPLMDebugString(
                        ("[A350AutoTaxi] Ground markings: " + std::to_string(airport.groundLines.size()) +
                         "; ATC fallback links: " + std::to_string(airport.atcFallbackEdges) + "\n")
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
                if (result.airportId == airport.id)
                    ui->setAvailability(result.items);
            } catch (const std::exception &e) {
                XPLMDebugString(
                    ("[A350AutoTaxi] Route availability: " + std::string(e.what()) + "\n").c_str());
            }
        }
        bool isPaused = paused && XPLMGetDatai(paused);
        auto state = adapter.state();
        if (!isPaused)
            advanceDeparture(state, std::clamp(static_cast<double>(dt), 0.0, 1.0));
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
            auto out = controller.update(state, dt);
            lastOutput = out;
            if (out.phase == TaxiPhase::Fault)
                stop("Fault: " + out.reason);
            else if (out.phase == TaxiPhase::Complete)
                stop("Complete: " + airport.id + " / parking brake held");
            adapter.apply(out);
            lastControls = std::string(phaseName(out.phase)) + " | cmd " +
                           std::to_string(out.steerDegrees).substr(0, 5) + " deg | CTE " +
                           std::to_string(out.crossTrack).substr(0, 4) + " m | remaining " +
                           std::to_string(static_cast<int>(out.remaining)) + " m";
        }
        uiTime += dt;
        if (uiTime > 0.25) {
            uiTime = 0;
            ui->setTelemetry(state, lastOutput, adapter.actualSteer(), pending.has_value());
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
        plugin->lastControls =
            plugin->config.steeringMode + " steering / gear " + std::to_string(plugin->config.steeringIndex);
    } catch (const std::exception &e) {
        plugin->status = e.what();
    }
    plugin->ui =
        std::make_unique<TaxiUI>(UIActions{[] { plugin->scan(); }, [] { plugin->reload(); },
                                           [] { plugin->stop(); }, [] { plugin->disconnect(); },
                                           [](const Destination &d, bool clearance, double speed,
                                              bool start) { plugin->plan(d, clearance, speed, start); }});
    plugin->ui->setStatus(plugin->status);
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
    XPLMRemoveMenuItem(XPLMFindPluginsMenu(), plugin->menuItem);
    plugin.reset();
}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int message, void *parameter) {
    if (!plugin)
        return;
    if ((message == XPLM_MSG_PLANE_LOADED && reinterpret_cast<intptr_t>(parameter) == 0) ||
        message == XPLM_MSG_AIRPORT_LOADED) {
        plugin->controller.stop();
        plugin->tug.cancel();
        plugin->pending.reset();
        plugin->adapter.release(false);
        plugin->hasPosition = false;
        plugin->ui->setBusy(false, plugin->loading());
        try {
            plugin->config = loadConfig(plugin->configPath);
            plugin->adapter.initialize(plugin->config);
            plugin->ready = true;
        } catch (const std::exception &e) {
            plugin->ready = false;
            plugin->report(e.what());
        }
        plugin->autoScan = true;
    }
}
