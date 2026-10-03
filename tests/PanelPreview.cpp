#include "ConsolasFont.h"
#include "Panel.h"
#include "PushbackPlanner.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace autotaxi;
namespace {
ConsolasFont previewFont;
double measureText(const std::string &text) {
    return previewFont.supports(text) ? previewFont.measure(text) : text.size() * 7.5;
}
std::string escape(const std::string &text) {
    std::string out;
    for (char c : text) {
        if (c == '&')
            out += "&amp;";
        else if (c == '<')
            out += "&lt;";
        else if (c == '\"')
            out += "&quot;";
        else
            out += c;
    }
    return out;
}
int channel(float value) {
    return static_cast<int>(value * 255);
}
void exportFrame(const PanelFrame &frame, int width, int height, const std::filesystem::path &path) {
    std::ofstream out(path);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\"" << height
        << "\">\n";
    for (const auto &d : frame.commands) {
        std::string color = "rgb(" + std::to_string(channel(d.color.r)) + "," +
                            std::to_string(channel(d.color.g)) + "," + std::to_string(channel(d.color.b)) +
                            ")";
        if (d.kind == DrawKind::Rectangle)
            out << "<rect x=\"" << d.box.x << "\" y=\"" << d.box.y << "\" width=\"" << d.box.w
                << "\" height=\"" << d.box.h << "\" fill=\"" << color << "\"/>\n";
        else if (d.kind == DrawKind::Line)
            out << "<line x1=\"" << d.box.x << "\" y1=\"" << d.box.y << "\" x2=\"" << d.x2 << "\" y2=\""
                << d.y2 << "\" stroke=\"" << color << "\" stroke-width=\"" << d.width << "\"/>\n";
        else if (d.kind == DrawKind::Triangle)
            out << "<polygon points=\"" << d.box.x << "," << d.box.y << " " << d.x2 << "," << d.y2 << " "
                << d.x3 << "," << d.y3 << "\" fill=\"" << color << "\"/>\n";
        else
            out << "<text x=\"" << d.box.x << "\" y=\"" << d.box.y + 12
                << "\" font-family=\"Consolas,monospace\" font-size=\"14\" fill=\"" << color << "\">"
                << escape(d.text) << "</text>\n";
    }
    out << "</svg>\n";
}
void checkFrame(const PanelState &state, int width, int height) {
    auto measure = measureText;
    auto frame = buildPanel(state, width, height, measure);
    for (const auto &d : frame.commands) {
        if (d.box.x < -.01 || d.box.y < -.01)
            throw std::runtime_error("Drawing outside top/left bounds");
        if (d.kind == DrawKind::Text &&
            (!d.text.empty() && (d.box.x + measure(d.text) > width + .01 || d.box.y + 14 > height + .01)))
            throw std::runtime_error("Text outside window");
        if (d.kind == DrawKind::Rectangle &&
            (d.box.x + d.box.w > width + .01 || d.box.y + d.box.h > height + .01))
            throw std::runtime_error("Rectangle outside window");
    }
    for (const auto &hit : frame.hits)
        if (hit.box.x < 0 || hit.box.y < 0 || hit.box.x + hit.box.w > width || hit.box.y + hit.box.h > height)
            throw std::runtime_error("Control outside window");
}
void checkLocalRoute(const PanelState &state, int width, int height) {
    auto frame = buildPanel(state, width, height, measureText);
    const auto &route = *state.route;
    std::size_t count = std::min(route.pushbackPointCount, route.points.size());
    for (std::size_t i = 0; i < count; ++i) {
        auto point = project(state.mapOrigin, unproject(route.origin, route.points[i])) - state.mapCenter;
        double x = frame.mapContent.x + frame.mapContent.w / 2 + point.x * frame.mapScale;
        double y = frame.mapContent.y + frame.mapContent.h / 2 - point.y * frame.mapScale;
        if (!frame.mapContent.contains(x - 10, y - 10) || !frame.mapContent.contains(x + 10, y + 10))
            throw std::runtime_error("Default local map must contain the entire pushback path with margins");
    }
    int purpleSegments = 0;
    bool local = false, airport = false;
    for (const auto &command : frame.commands)
        if (command.kind == DrawKind::Line && command.width == 4 && command.color.b > .95 &&
            command.color.r > .7 && command.color.g < .6)
            ++purpleSegments;
    for (const auto &hit : frame.hits) {
        if (hit.action == Action::LocalMap)
            local = hit.enabled;
        if (hit.action == Action::AirportMap)
            airport = hit.enabled;
    }
    if (purpleSegments != static_cast<int>(count) - 1 || !local || !airport)
        throw std::runtime_error("Purple reverse path and both map modes must be available");
}
void checkMapModes() {
    PanelState state;
    state.aircraft = {{31, 121}, 72, 0, true};
    state.airport.nodes[1] = {1, state.aircraft.position, "Stand"};
    state.airport.nodes[2] = {2, unproject(state.aircraft.position, {4000, 2000}), "Far end"};
    fitAirport(state);
    Route reverse;
    reverse.origin = state.aircraft.position;
    reverse.label = "RWY 17L";
    reverse.requiresPushback = true;
    reverse.points = {{0, 0}, {-50, -10}, {-85, -30}, {-100, -70}, {4000, 2000}};
    reverse.pushbackPointCount = 4;
    reverse.taxiStartNode = 887;
    setPanelRoute(state, reverse);
    if (state.mapView != MapView::Local || state.mapFitExtent.x > 200 || state.mapFitExtent.y > 200)
        throw std::runtime_error("Reverse departure defaults to local extent excluding the long taxi route");
    for (auto size : {std::pair<int, int>{900, 600}, {1080, 720}, {1440, 900}}) {
        checkFrame(state, size.first, size.second);
        checkLocalRoute(state, size.first, size.second);
        state.busy = true;
        checkLocalRoute(state, size.first, size.second);
        state.busy = false;
    }
    selectMapView(state, MapView::Airport);
    setPanelRoute(state, reverse);
    if (state.mapView != MapView::Airport || state.mapFitExtent.x < 1500)
        throw std::runtime_error("Explicit overview selection survives preview/start recomputation");
    selectMapView(state, MapView::Local);
    state.mapScale = 2;
    state.mapCenter = {-50, -30};
    setPanelRoute(state, reverse);
    if (state.mapScale != 2 || length(state.mapCenter - Vec2{-50, -30}) > .01)
        throw std::runtime_error("Unchanged route must preserve manual map navigation");
    fitMap(state);
    checkLocalRoute(state, 900, 600);
    state.preferredMapView.reset();
    Route forward = reverse;
    forward.requiresPushback = false;
    forward.pushbackPointCount = 0;
    forward.taxiStartNode = -1;
    setPanelRoute(state, forward);
    if (state.mapView != MapView::Airport)
        throw std::runtime_error("Automatic default returns to airport map after pushback handoff");
}
void checkStandList() {
    PanelState state;
    state.aircraft = {{31, 121}, 0, 0, true};
    state.airport.ramps = {{state.aircraft.position, 0, 'C', "Small Stand", {}},
                           {state.aircraft.position, 0, 'E', "Large Stand", {}}};
    state.choices = destinations(state.airport);
    state.category = 1;
    fitAirport(state);
    for (auto size : {std::pair<int, int>{900, 600}, {1080, 720}, {1440, 900}}) {
        checkFrame(state, size.first, size.second);
        auto frame = buildPanel(state, size.first, size.second, measureText);
        bool small = false, large = false, reason = false;
        for (const auto &hit : frame.hits)
            if (hit.action == Action::Destination) {
                if (hit.index == 0)
                    small = !hit.enabled && !hit.tooltip.empty();
                if (hit.index == 1)
                    large = hit.enabled;
            }
        for (const auto &command : frame.commands)
            if (command.kind == DrawKind::Text && command.text.find("Class C") != std::string::npos)
                reason = true;
        if (!small || !large || !reason)
            throw std::runtime_error("Small stands must remain visible with a reason and disabled selection");
        state.query = "Small";
        if (filteredChoices(state).size() != 1)
            throw std::runtime_error("Previously hidden stands must be searchable");
        state.query.clear();
    }
}
void checkRouteTimingDisplay() {
    PanelState state;
    state.aircraft = {{31, 121}, 0, 0, true};
    Route route;
    route.origin = state.aircraft.position;
    route.points = {{0, 0}, {0, 100}};
    route.length = 100;
    state.route = route;
    // A completed control output can remain after planning a new destination.
    state.timing = estimateRouteTiming(route, 0, 0, state.controllerConfig);
    state.output.phase = TaxiPhase::Complete;
    fitAirport(state);
    auto frame = buildPanel(state, 1080, 720, measureText);
    bool timing = false;
    for (const auto &command : frame.commands)
        if (command.kind == DrawKind::Text && command.text.find("TOTAL") != std::string::npos) {
            timing = true;
            if (command.text.find("TOTAL 00:00") != std::string::npos ||
                command.text.find("TOTAL --:--") != std::string::npos)
                throw std::runtime_error("A new route must recompute ETA after a completed taxi");
        }
    if (!timing)
        throw std::runtime_error("Forward route timing must be shown in the panel");
    state.route->requiresPushback = true;
    frame = buildPanel(state, 1080, 720, measureText);
    for (const auto &command : frame.commands)
        if (command.kind == DrawKind::Text && command.text.find("TOTAL") != std::string::npos &&
            command.text.find("TOTAL --:--") == std::string::npos)
            throw std::runtime_error("Pushback previews must not show taxi-speed ETA");
}
void checkTurnGuidanceDisplay() {
    PanelState state;
    state.aircraft = {{31, 121}, 0, 0, true};
    Route route;
    route.origin = state.aircraft.position;
    route.points = {{0, 0}, {0, 100}};
    route.length = 100;
    route.mainAxleOversteer = true;
    route.turnClearanceKnown = false;
    route.maximumOversteer = 7;
    state.route = route;
    fitAirport(state);
    for (bool busy : {false, true}) {
        state.busy = busy;
        state.output.oversteerOffset = 4;
        for (auto size : {std::pair<int, int>{900, 600}, {1080, 720}, {1440, 900}}) {
            checkFrame(state, size.first, size.second);
            std::string labels;
            for (const auto &command : buildPanel(state, size.first, size.second, measureText).commands)
                if (command.kind == DrawKind::Text)
                    labels += command.text + " ";
            if (labels.find("Main-axle oversteer") == std::string::npos ||
                labels.find("Turn pavement clearance unverified") == std::string::npos ||
                labels.find(busy ? "Oversteer 4.0 m" : "Planned oversteer 7.0 m") == std::string::npos)
                throw std::runtime_error("Turn guidance mode, offset and clearance must be visible");
        }
    }
}
void checkTrackingMarkers() {
    PanelState state;
    state.aircraft = {{31, 121}, 0, 0, true};
    Route route;
    route.origin = state.aircraft.position;
    route.points = {{0, 0}, {0, 120}};
    route.length = 120;
    route.cockpitGuidance = true;
    state.route = route;
    fitAirport(state);
    state.mapScale = 2;
    auto frame = buildPanel(state, 1080, 720, measureText);
    bool axle = false, cockpit = false, basis = false, joined = false, gearAxle = false;
    for (const auto &draw : frame.commands)
        if (draw.kind == DrawKind::Text) {
            axle = axle || draw.text == "AXLE";
            cockpit = cockpit || draw.text == "COCKPIT";
            basis = basis || draw.text == "COCKPIT CTE";
        }
    for (const auto &draw : frame.commands)
        if (draw.kind == DrawKind::Line &&
            std::abs(length(Vec2{draw.x2 - draw.box.x, draw.y2 - draw.box.y}) - 60.3) < .01)
            joined = true;
        else if (draw.kind == DrawKind::Line &&
                 std::abs(length(Vec2{draw.x2 - draw.box.x, draw.y2 - draw.box.y}) - 20.7264) < .01)
            gearAxle = true;
    if (!axle || !cockpit || !basis || !joined || !gearAxle)
        throw std::runtime_error("Actual cockpit, axle midpoint and left/right main gear must be joined");
    for (const auto &draw : frame.commands) {
        if (draw.kind == DrawKind::Triangle && frame.mapContent.contains(draw.box.x, draw.box.y))
            throw std::runtime_error("Aircraft map symbol must not have a filled triangle");
        if (draw.kind == DrawKind::Rectangle && draw.box.w == 6 && draw.box.h == 6)
            throw std::runtime_error("Aircraft points must be hollow rather than filled squares");
    }
    auto point = trackingPosition(route, state.controllerConfig, state.aircraft);
    if (std::abs(point.y - 27.95) > .001 || std::abs(point.x) > .001)
        throw std::runtime_error("Map/controller reference must share the configured A350 geometry");
    state.controllerConfig.mainGearHalfTrack = 4.5;
    state.controllerConfig.wheelbase = 20;
    state.controllerConfig.cockpitAheadNose = 3;
    state.controllerConfig.mainAxleAft = 4;
    frame = buildPanel(state, 1080, 720, measureText);
    bool customGear = false, customCockpit = false;
    for (const auto &draw : frame.commands)
        if (draw.kind == DrawKind::Line && draw.width == 1.5) {
            double span = length(Vec2{draw.x2 - draw.box.x, draw.y2 - draw.box.y});
            customGear = customGear || std::abs(span - 18) < .01;
            customCockpit = customCockpit || std::abs(span - 46) < .01;
        }
    if (!customGear || !customCockpit)
        throw std::runtime_error(
            "Aircraft symbol must use the supplied INI geometry instead of fixed A350 dimensions");
}
void checkDualCte() {
    PanelState state;
    state.aircraft = {{31, 121}, 0, 0, true};
    Route route;
    route.origin = state.aircraft.position;
    route.length = 200;
    route.cockpitPath = {{{10, 0}, 0, false}, {{10, 220}, 200, false}};
    route.mainAxlePath = {{{0, 0}, 0, false}, {{0, 200}, 200, false}};
    state.aircraft.position = unproject(route.origin, {3, 50});
    state.output.progress = 50;
    state.timing = RouteTiming{};
    state.timing->totalSeconds = 100;
    state.timing->cockpitPath = {{{3, 77.95}, 50, false}, {{10, 220}, 200, false}};
    fitAirport(state);
    for (bool cockpitMode : {false, true}) {
        route.cockpitGuidance = cockpitMode;
        route.points =
            cockpitMode ? std::vector<Vec2>{{10, 0}, {10, 220}} : std::vector<Vec2>{{0, 0}, {0, 200}};
        state.route = route;
        state.output.crossTrack = cockpitMode ? 7 : 3;
        for (bool busy : {false, true}) {
            state.busy = busy;
            for (auto size : {std::pair<int, int>{900, 600}, {1080, 720}, {1440, 900}}) {
                checkFrame(state, size.first, size.second);
                auto frame = buildPanel(state, size.first, size.second, measureText);
                for (const auto &label : {std::string("COCKPIT CTE"), std::string("MAIN AXLE CTE")}) {
                    auto caption = std::find_if(frame.commands.begin(), frame.commands.end(),
                                                [&](const DrawCommand &d) { return d.text == label; });
                    if (caption == frame.commands.end())
                        throw std::runtime_error("Both CTE captions must be visible in either guidance mode");
                    const auto expected = label == "COCKPIT CTE" ? "7.0 m" : "3.0 m";
                    auto value =
                        std::find_if(frame.commands.begin(), frame.commands.end(), [&](const DrawCommand &d) {
                            return d.kind == DrawKind::Text && d.box.x == caption->box.x && d.box.y == 86;
                        });
                    if (value == frame.commands.end() || value->text != expected)
                        throw std::runtime_error(
                            "CTE must compare each actual reference to its fixed planned path");
                    bool selected = (label == "COCKPIT CTE") == cockpitMode;
                    if (selected != (caption->color.g > .8))
                        throw std::runtime_error("The controlling CTE caption must be highlighted");
                }
                if (cockpitMode && busy && size.first == 1080) {
                    std::filesystem::create_directories("cockpit-preview");
                    exportFrame(frame, size.first, size.second, "cockpit-preview/dual-cte.svg");
                }
            }
        }
    }
    state.waitingPushback = true;
    for (const auto &draw : buildPanel(state, 900, 600, measureText).commands)
        if (draw.kind == DrawKind::Text && draw.box.y == 86 && draw.box.x > 450 && draw.text != "--")
            throw std::runtime_error("Towing must hide forward-route CTE values");
}
void checkCockpitPath() {
    PanelState state;
    state.aircraft = {{31, 121}, 0, 0, true};
    Route route;
    route.origin = state.aircraft.position;
    route.points = {{0, 0}, {0, 100}, {100, 100}};
    route.length = 200;
    route.mainAxleOversteer = true;
    route.cockpitPath = {{{0, 28}, 0, false}, {{15, 115}, 100, true}, {{130, 100}, 200, false}};
    route.ramp = true;
    route.cockpitStop = {130, 100};
    setPanelRoute(state, route);
    state.mapCenter = {60, 70};
    state.mapScale = 1;
    auto checkPath = [&](std::vector<RoutePathPoint> expected) {
        if (state.busy && !state.predictedCockpitPath.empty())
            expected.front().position = cockpitPosition(route.origin, state.controllerConfig, state.aircraft);
        auto frame = buildPanel(state, 1080, 720, measureText);
        auto pixel = [&](Vec2 point) {
            return Vec2{frame.mapContent.x + frame.mapContent.w / 2 + point.x - state.mapCenter.x,
                        frame.mapContent.y + frame.mapContent.h / 2 - point.y + state.mapCenter.y};
        };
        for (std::size_t i = 1; i < expected.size(); ++i) {
            auto a = pixel(expected[i - 1].position), b = pixel(expected[i].position);
            bool found = false;
            for (const auto &draw : frame.commands)
                if (draw.kind == DrawKind::Line && draw.width == 3 &&
                    length(Vec2{draw.box.x, draw.box.y} - a) < .01 &&
                    length(Vec2{draw.x2, draw.y2} - b) < .01 &&
                    (expected[i].pavementRisk ? draw.color.r > .9 : draw.color.g > .7))
                    found = true;
            if (!found)
                throw std::runtime_error(
                    "Map must draw the cockpit path and its risk, not the axle/yellow path");
        }
        auto stop = pixel(route.cockpitStop);
        bool found = false;
        for (const auto &draw : frame.commands)
            if (draw.kind == DrawKind::Rectangle && draw.box.w == 8 && draw.box.h == 8 &&
                length(Vec2{draw.box.x + 4, draw.box.y + 4} - stop) < .01)
                found = true;
        if (!found)
            throw std::runtime_error("Stand stop marker must represent the cockpit target");
    };
    checkPath(route.cockpitPath);
    RouteTiming incoming;
    incoming.totalSeconds = 60;
    incoming.cockpitPath = {{{0, 28}, 0, false}, {{12, 111}, 100, true}, {{129.8, 100}, 200, false}};
    state.busy = true;
    state.output.phase = TaxiPhase::Taxi;
    setPanelTiming(state, incoming);
    if (state.predictedCockpitPath.empty() ||
        length(state.predictedCockpitPath[1].position - route.cockpitPath[1].position) < 1)
        throw std::runtime_error("A valid realtime forecast must replace the planned display geometry");
    checkPath(state.predictedCockpitPath);
    for (double brake : {0., .4, 1.}) {
        state.output.brake = brake;
        incoming.cockpitPath[1].position.x += 2;
        auto before = state.predictedCockpitPath[1].position;
        setPanelTiming(state, incoming);
        if (brake > .05 && length(state.predictedCockpitPath[1].position - before) > 1.500001)
            throw std::runtime_error("Brake refresh must bound the visible forecast's motion");
        checkPath(state.predictedCockpitPath);
    }
    state.output.phase = TaxiPhase::Braking;
    const auto retained = state.predictedCockpitPath;
    for (int i = 0; i < 4; ++i) {
        setPanelTiming(state, {});
        if (!state.timing || state.timing->totalSeconds >= 0 ||
            state.predictedCockpitPath.size() != retained.size())
            throw std::runtime_error(
                "Failed forecast must clear ETA but retain its short-lived display path");
        for (std::size_t k = 0; k < retained.size(); ++k)
            if (length(state.predictedCockpitPath[k].position - retained[k].position) > .001)
                throw std::runtime_error("Brief brake prediction failure must not switch line geometry");
        checkPath(state.predictedCockpitPath);
    }
    setPanelTiming(state, {});
    if (state.predictedCockpitPath.empty() ||
        length(state.predictedCockpitPath[1].position - retained[1].position) > 1.500001)
        throw std::runtime_error("An expired forecast must ease toward the geometric fallback");
    checkPath(state.predictedCockpitPath);
    std::filesystem::create_directories("cockpit-preview");
    exportFrame(buildPanel(state, 1080, 720, measureText), 1080, 720, "cockpit-preview/panel.svg");
    state.output.progress = 110;
    auto clipped = buildPanel(state, 1080, 720, measureText);
    int routeSegments = 0;
    for (const auto &draw : clipped.commands)
        if (draw.kind == DrawKind::Line && draw.width == 3)
            ++routeSegments;
    if (routeSegments != 1)
        throw std::runtime_error("Moving aircraft must trim already-passed prediction points");
    setPanelRoute(state, route);
    if (state.timing || !state.predictedCockpitPath.empty() || state.predictionInvalidSeconds != 0)
        throw std::runtime_error("Replanning must discard the previous route's forecast filter");
    setPanelTiming(state, incoming);
    state.output.phase = TaxiPhase::Hold;
    setPanelTiming(state, {});
    if (!state.predictedCockpitPath.empty())
        throw std::runtime_error("An emergency hold must not retain a moving prediction");
    state.route.reset();
    resetPanelTiming(state);
    auto frame = buildPanel(state, 900, 600, measureText);
    bool cockpit = false, axle = false;
    for (const auto &draw : frame.commands)
        if (draw.kind == DrawKind::Text) {
            cockpit = cockpit || draw.text == "COCKPIT";
            axle = axle || draw.text == "AXLE";
        }
    if (!cockpit || !axle)
        throw std::runtime_error("Both actual aircraft points must remain visible without a route");
}
void checkFrameCache() {
    PanelState state;
    state.aircraft = {{31, 121}, 0, 0, true};
    state.airport.groundLines.push_back(
        {1, "Test paint", {state.aircraft.position, unproject(state.aircraft.position, {30, 30})}});
    state.airport.ramps.push_back({unproject(state.aircraft.position, {30, 0}), 0, 'E', "Lead-in stand"});
    state.airport.groundLines.push_back(
        {20, "White lead-in", {unproject(state.aircraft.position, {-20, 0}),
                                unproject(state.aircraft.position, {30, 0})}, true, 0});
    state.airport.groundLines.push_back(
        {51, "Yellow lead-in", {unproject(state.aircraft.position, {-20, -12}),
                                 unproject(state.aircraft.position, {30, -12})}, true, 0});
    state.airport.groundLines.push_back(
        {3, "Taxiway boundary", {unproject(state.aircraft.position, {-20, 12}),
                                  unproject(state.aircraft.position, {30, 12})}});
    state.airport.groundLines.push_back(
        {20, "Ground service marking", {unproject(state.aircraft.position, {-20, -12}),
                                          unproject(state.aircraft.position, {30, -12})}});
    fitAirport(state);
    PanelFrameCache cache;
    auto measure = measureText;
    const auto &initial = cache.get(state, 1080, 720, measure);
    if (initial.staticMapBegin >= initial.staticMapEnd || initial.staticMapEnd > initial.commands.size())
        throw std::runtime_error("Static airport drawing range must be present and valid");
    PanelState zoomed = state;
    zoomed.mapScale = .6;
    auto zoomedFrame = buildPanel(zoomed, 1080, 720, measure);
    bool standNameVisible = false;
    for (const auto &draw : zoomedFrame.commands)
        if (draw.kind == DrawKind::Text && draw.text == "Lead-in stand" &&
            zoomedFrame.map.contains(draw.box.x, draw.box.y))
            standNameVisible = true;
    if (!standNameVisible)
        throw std::runtime_error("Zoomed map must show stand names");
    zoomed.mapScale = .08;
    auto overviewFrame = buildPanel(zoomed, 1080, 720, measure);
    for (const auto &draw : overviewFrame.commands)
        if (draw.kind == DrawKind::Text && draw.text == "Lead-in stand" &&
            overviewFrame.map.contains(draw.box.x, draw.box.y))
            throw std::runtime_error("Airport overview must keep dense stand names hidden");
    // Keep a small, deterministic visual fixture for reviewing the lead-in
    // split and its two transverse yellow markers without loading scenery.
    std::filesystem::create_directories("cockpit-preview");
    exportFrame(initial, 1080, 720, "cockpit-preview/lead-in-markers.svg");
    int leadInMarkers = 0;
    bool yellowCenterline = false, dimBoundary = false, whiteLeadIn = false, whiteMarking = false,
         yellowLeadInMarkers = false;
    for (std::size_t i = initial.staticMapBegin; i < initial.staticMapEnd; ++i)
        if (initial.commands[i].kind == DrawKind::Line) {
            const auto &draw = initial.commands[i];
            if (draw.width > 2.5)
                ++leadInMarkers;
            if (draw.width > 2.8 && draw.color.r > .8 && draw.color.g > .5 && draw.color.b < .3)
                yellowLeadInMarkers = true;
            if (draw.width > 1.2 && draw.width < 1.5 && draw.color.r > .8 && draw.color.g > .5 && draw.color.b < .4)
                yellowCenterline = true;
            if (draw.width > 1.0 && draw.width < 1.2 && draw.color.r < .5 && draw.color.g < .5 && draw.color.b < .5)
                dimBoundary = true;
            if (draw.width > 1.8 && draw.width < 2.2 && draw.color.r > .8 && draw.color.g > .8 && draw.color.b > .8)
                whiteLeadIn = true;
            if (draw.width > 1.1 && draw.width < 1.3 && draw.color.r > .6 && draw.color.g > .6 &&
                draw.color.b > .6)
                whiteMarking = true;
        }
    if (leadInMarkers < 2)
        throw std::runtime_error("Lead-in start and stop markers must be rendered as transverse lines");
    if (!yellowCenterline || !dimBoundary || !whiteLeadIn || !whiteMarking || !yellowLeadInMarkers)
        throw std::runtime_error("Centerlines, boundaries and lead-ins must use distinct map colors and widths");
    // A scenery package may already provide the stand stop line.  The
    // synthetic marker must then be suppressed rather than drawn on top of it.
    PanelState withStopLine = state;
    // Keep this regression fixture focused on the white lead-in under test.
    // The panel also contains a separate yellow lead-in so that its styling is
    // covered above; retaining it here would add its own two transverse
    // markers and make this stop-line count ambiguous.
    withStopLine.airport.groundLines.erase(
        std::remove_if(withStopLine.airport.groundLines.begin(), withStopLine.airport.groundLines.end(),
                       [](const GroundLine &line) { return line.name != "White lead-in"; }),
        withStopLine.airport.groundLines.end());
    withStopLine.airport.groundLines.push_back(
        {4, "Existing stand stop", {unproject(state.aircraft.position, {30, -5}),
                                      unproject(state.aircraft.position, {30, 5})}});
    fitAirport(withStopLine);
    PanelFrameCache stopCache;
    const auto &stopFrame = stopCache.get(withStopLine, 1080, 720, measure);
    int markersWithExistingStop = 0;
    for (std::size_t i = stopFrame.staticMapBegin; i < stopFrame.staticMapEnd; ++i)
        if (stopFrame.commands[i].kind == DrawKind::Line && stopFrame.commands[i].width > 2.5)
            ++markersWithExistingStop;
    if (markersWithExistingStop != 1)
        throw std::runtime_error("Existing stand stop lines must suppress the duplicate lead-in marker");
    for (std::size_t i = initial.staticMapBegin; i < initial.staticMapEnd; ++i)
        if (initial.commands[i].kind == DrawKind::Triangle)
            throw std::runtime_error("Static map cache must not contain route aircraft geometry");
    const auto *commands = cache.get(state, 1080, 720, measure).commands.data();
    for (int i = 0; i < 60; ++i)
        if (cache.get(state, 1080, 720, measure).commands.data() != commands)
            throw std::runtime_error("Unchanged frames must reuse drawing storage");
    if (cache.rebuildCount() != 1)
        throw std::runtime_error("Unchanged frames must not recompute map and ETA");
    state.aircraft.speed = 1;
    cache.invalidate();
    bool updated = false;
    for (const auto &command : cache.get(state, 1080, 720, measure).commands)
        if (command.kind == DrawKind::Text && command.text == "1.9 kt")
            updated = true;
    if (!updated || cache.rebuildCount() != 2 || cache.mapRebuildCount() != 1)
        throw std::runtime_error("Telemetry invalidation must refresh cached labels");
    cache.get(state, 900, 600, measure);
    if (cache.rebuildCount() != 3 || cache.mapRebuildCount() != 2)
        throw std::runtime_error("Resizing must refresh layout and hit boxes");
    state.mapCenter = {20, 20};
    cache.invalidate();
    auto uncached = buildPanel(state, 900, 600, measure);
    const auto &cached = cache.get(state, 900, 600, measure);
    if (cached.commands.size() != uncached.commands.size() || cached.hits.size() != uncached.hits.size() ||
        cached.staticMapBegin != uncached.staticMapBegin || cached.staticMapEnd != uncached.staticMapEnd)
        throw std::runtime_error("Cached and direct layouts must match after interaction");
    if (cache.mapRebuildCount() != 3)
        throw std::runtime_error("Panning must invalidate static airport geometry");
    for (std::size_t i = 0; i < cached.commands.size(); ++i) {
        const auto &a = cached.commands[i];
        const auto &b = uncached.commands[i];
        if (a.kind != b.kind || a.box.x != b.box.x || a.box.y != b.box.y || a.box.w != b.box.w ||
            a.box.h != b.box.h || a.x2 != b.x2 || a.y2 != b.y2 || a.width != b.width || a.text != b.text ||
            a.color.r != b.color.r || a.color.g != b.color.g || a.color.b != b.color.b)
            throw std::runtime_error("Caching must preserve drawing order, geometry and colors");
    }
    state.airport.groundLines.push_back(
        {51, "New airport paint", {state.aircraft.position, unproject(state.aircraft.position, {10, 20})}});
    cache.invalidate(true);
    const auto &newAirport = cache.get(state, 900, 600, measure);
    if (cache.mapRebuildCount() != 4 ||
        newAirport.commands.size() != buildPanel(state, 900, 600, measure).commands.size())
        throw std::runtime_error("Airport replacement must invalidate cached scenery geometry");
    std::vector<DrawCommand> batch(5);
    for (auto &command : batch) {
        command.kind = DrawKind::Line;
        command.width = 1;
    }
    batch[1].color = {.8f, .2f, .1f};
    batch[2].width = 2;
    batch[3].kind = DrawKind::Text;
    if (lineBatchEnd(batch, 0) != 2 || lineBatchEnd(batch, 2) != 3 || lineBatchEnd(batch, 4) != 5)
        throw std::runtime_error("Line batching must preserve text order and split at width changes");
}
void checkRouteControls() {
    PanelState state;
    state.aircraft = {{31, 121}, 0, 0, true};
    state.airport.nodes[12] = {12, state.aircraft.position, "Pick"};
    state.airport.nodes[13] = {13, unproject(state.aircraft.position, {100, 200}), "End"};
    state.viaText = "A B #12 C";
    state.routeOptions.via = {"A", "B", "#12", "C"};
    Route route;
    route.origin = state.aircraft.position;
    route.label = "Small stand";
    route.points = {{0, 0}, {100, 200}};
    route.length = length(route.points.back());
    route.pavementRisk = {false, true};
    route.destinationRisk = true;
    setPanelRoute(state, route);
    for (auto size : {std::pair<int, int>{900, 600}, {1080, 720}, {1440, 900}}) {
        state.optionsOpen = true;
        checkFrame(state, size.first, size.second);
        auto frame = buildPanel(state, size.first, size.second, measureText);
        for (Action action :
             {Action::AllowOversteer, Action::RelaxPavement, Action::RelaxStand, Action::ViaInput,
              Action::PickVia, Action::UndoVia, Action::ClearVia, Action::ApplyVia}) {
            if (std::none_of(frame.hits.begin(), frame.hits.end(),
                             [&](const Hit &hit) { return hit.action == action && hit.enabled; }))
                throw std::runtime_error("All route settings must be usable in the smallest panel");
        }
        state.optionsOpen = false;
        state.pickVia = true;
        checkFrame(state, size.first, size.second);
        state.pickVia = false;
        state.busy = true;
        state.emergencyHeld = true;
        state.output.phase = TaxiPhase::Hold;
        frame = buildPanel(state, size.first, size.second, measureText);
        bool resume = false, orange = false;
        for (const auto &hit : frame.hits) {
            if ((hit.action == Action::SpeedUp || hit.action == Action::SpeedDown) && !hit.enabled)
                throw std::runtime_error("Taxi speed controls remain enabled during emergency hold");
            if (hit.action == Action::EmergencyBrake)
                resume = hit.enabled;
        }
        for (const auto &draw : frame.commands)
            if (draw.kind == DrawKind::Line && draw.width == 3 && draw.color.r > .9 && draw.color.g > .5 &&
                draw.color.b < .5)
                orange = true;
        if (!resume || !orange)
            throw std::runtime_error("Held taxi retains Resume and orange pavement risk");
        checkFrame(state, size.first, size.second);
        std::filesystem::create_directories("route-ui-preview");
        exportFrame(frame, size.first, size.second,
                    std::filesystem::path("route-ui-preview") /
                        ("hold-" + std::to_string(size.first) + ".svg"));
        state.busy = false;
        state.emergencyHeld = false;
        state.output.phase = TaxiPhase::Idle;
        state.optionsOpen = true;
        exportFrame(buildPanel(state, size.first, size.second, measureText), size.first, size.second,
                    std::filesystem::path("route-ui-preview") /
                        ("options-" + std::to_string(size.first) + ".svg"));
    }
}
} // namespace
int main(int argc, char **argv) {
    try {
#if defined(_WIN32)
        if (!previewFont.load())
            throw std::runtime_error("Installed Consolas must rasterize successfully");
        if (previewFont.measure("iii") != previewFont.measure("WWW") || previewFont.advance < 1)
            throw std::runtime_error("Consolas must use fixed character advances");
        for (int code = 33; code <= 126; ++code) {
            int column = (code - 32) % 16, row = (code - 32) / 16;
            bool ink = false;
            for (int y = 0; y < ConsolasFont::cellHeight; ++y)
                for (int x = 0; x < ConsolasFont::cellWidth; ++x)
                    ink =
                        ink || previewFont.rgba[((row * ConsolasFont::cellHeight + y) * ConsolasFont::width +
                                                 column * ConsolasFont::cellWidth + x) *
                                                    4 +
                                                3] != 0;
            if (!ink)
                throw std::runtime_error("Consolas atlas contains a blank printable glyph");
        }
        std::cout << "Consolas rasterization: 14 px, advance " << previewFont.advance << " px\n";
#endif
        checkMapModes();
        checkStandList();
        checkRouteTimingDisplay();
        checkTurnGuidanceDisplay();
        checkTrackingMarkers();
        checkDualCte();
        checkCockpitPath();
        checkFrameCache();
        checkRouteControls();
        PanelState state;
        state.aircraft = {{31.135482, 121.802825}, 0, 0, true};
        if (argc >= 4)
            state.aircraft.trueHeading = std::stod(argv[3]);
        if (argc >= 3) {
            std::ifstream in(std::filesystem::u8path(argv[1]));
            state.airport = parseAirport(in, argv[1]);
            const bool arrival = argc >= 7 && std::string(argv[6]) == "arrival";
            const std::string selectedRunway = argc >= 6 ? "RWY " + std::string(argv[5]) : "RWY 17L";
            const std::string stand = argc >= 5 ? argv[4] : "539";
            if (arrival) {
                const std::string runwayName = argc >= 6 ? argv[5] : "36L";
                const auto runway = std::find_if(
                    state.airport.runways.begin(), state.airport.runways.end(), [&](const Runway &candidate) {
                        return candidate.ends[0].name == runwayName || candidate.ends[1].name == runwayName;
                    });
                if (runway == state.airport.runways.end())
                    throw std::runtime_error("Requested arrival runway not found");
                const int end = runway->ends[0].name == runwayName ? 0 : 1;
                const int opposite = end == 0 ? 1 : 0;
                state.aircraft.position = runway->ends[end].position;
                state.aircraft.trueHeading =
                    heading(project(state.aircraft.position, runway->ends[opposite].position));
                state.category = 1;
            } else if (argc >= 5) {
                auto ramp = std::find_if(state.airport.ramps.begin(), state.airport.ramps.end(),
                                         [&](const Ramp &r) { return r.name == argv[4]; });
                if (ramp == state.airport.ramps.end())
                    throw std::runtime_error("Requested preview stand not found");
                state.aircraft.position = ramp->position;
                state.aircraft.trueHeading = ramp->heading;
            }
            for (auto &ramp : state.airport.ramps)
                if (!arrival && argc < 5 && distance(ramp.position, state.aircraft.position) < 20)
                    ramp.aliases.push_back("539");
            state.choices = destinations(state.airport);
            RouteOptions options;
            options.runwayClearance = true;
            for (std::size_t i = 0; i < state.choices.size(); ++i) {
                const auto &d = state.choices[i];
                const bool target = arrival ? (d.kind == DestinationKind::Ramp && d.label == stand)
                                            : d.kind == DestinationKind::Runway;
                if (!target)
                    continue;
                Availability info;
                try {
                    std::optional<Route> route;
                    try {
                        route = planRoute(state.airport, state.aircraft.position, state.aircraft.trueHeading,
                                          d, options);
                    } catch (const std::exception &) {
                    }
                    if (!arrival && (!route || route->requiresPushback))
                        route = planPushback(state.airport, state.aircraft, d, options, {}).preview;
                    if (arrival && route && route->requiresPushback)
                        route.reset();
                    info = {true, route->requiresPushback, route->length, {}};
                    if (arrival ? d.label == stand : d.label == selectedRunway) {
                        setPanelRoute(state, *route);
                        state.selected = static_cast<int>(i);
                        state.scroll = std::max(0, static_cast<int>(i) - 2);
                    }
                } catch (const std::exception &e) {
                    info.reason = e.what();
                }
                state.availability[d.label] = info;
            }
            state.clearance = true;
            state.status = arrival
                               ? "Preview: " + selectedRunway + " -> " + stand
                               : state.route && state.route->pushbackPointCount
                               ? "Preview: " + stand + " pushback -> Node " +
                                     std::to_string(state.route->taxiStartNode) + " -> " + selectedRunway
                               : "Preview: " + stand + " departure -> " + selectedRunway;
            fitMap(state);
            // The normal Local view is intentionally fitted to the complete
            // pushback path.  For exported review sheets, an optional
            // `final` argument requests a separate local view centred on the
            // last 500 m of the route so the stand lead-in geometry and its
            // stop line can be inspected independently of pushback.
            const bool finalLocal = (argc >= 7 && std::string(argv[6]) == "final") ||
                                    (argc >= 8 && std::string(argv[7]) == "final");
            if (finalLocal && state.route && state.route->points.size() >= 2) {
                selectMapView(state, MapView::Local);
                state.mapOrigin = state.route->origin;
                const Vec2 end = state.route->points.back();
                Vec2 tangent = end - state.route->points[state.route->points.size() - 2];
                if (length(tangent) > .01)
                    tangent = tangent * (1 / length(tangent));
                state.mapCenter = end - tangent * 100;
                state.mapFitExtent = {250, 250};
            }
            if (!state.route && argc >= 5) {
                selectMapView(state, MapView::Local);
                state.mapFitExtent = {250, 250};
                state.status = std::string("Stand ") + argv[4] + " departure unavailable";
            }
            auto output = std::filesystem::u8path(argv[2]);
            std::filesystem::create_directories(output);
            for (auto size : {std::pair<int, int>{1080, 720}, {900, 600}, {1440, 900}}) {
                checkFrame(state, size.first, size.second);
                if (state.route && state.route->requiresPushback && !finalLocal)
                    checkLocalRoute(state, size.first, size.second);
                exportFrame(buildPanel(state, size.first, size.second, measureText), size.first, size.second,
                            output / ("panel-" + std::to_string(size.first) + ".svg"));
                auto overview = state;
                selectMapView(overview, MapView::Airport);
                checkFrame(overview, size.first, size.second);
                exportFrame(buildPanel(overview, size.first, size.second, measureText), size.first,
                            size.second, output / ("panel-airport-" + std::to_string(size.first) + ".svg"));
            }
        }
        for (auto size : {std::pair<int, int>{1080, 720}, {900, 600}, {1440, 900}}) {
            state.status = "No route satisfying aircraft turns, one-way edges and E/F width. Selected runway "
                           "has no usable entry in this scenery's taxi network.";
            checkFrame(state, size.first, size.second);
            state.loading = true;
            checkFrame(state, size.first, size.second);
            state.loading = false;
            state.busy = true;
            state.waitingPushback = true;
            checkFrame(state, size.first, size.second);
            auto frame = buildPanel(state, size.first, size.second, measureText);
            bool stop = false;
            for (const auto &hit : frame.hits)
                if (hit.action == Action::Stop)
                    stop = hit.enabled;
            if (!stop)
                throw std::runtime_error("Stop must remain available during departure");
            state.busy = false;
            state.waitingPushback = false;
        }
        std::cout << "Panel layout checks passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
