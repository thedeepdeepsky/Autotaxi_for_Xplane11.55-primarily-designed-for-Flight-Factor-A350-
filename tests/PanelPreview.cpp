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
        PanelState state;
        state.aircraft = {{31.135482, 121.802825}, 0, 0, true};
        if (argc >= 4)
            state.aircraft.trueHeading = std::stod(argv[3]);
        if (argc >= 3) {
            std::ifstream in(std::filesystem::u8path(argv[1]));
            state.airport = parseAirport(in, argv[1]);
            if (argc >= 5) {
                auto ramp = std::find_if(state.airport.ramps.begin(), state.airport.ramps.end(),
                                         [&](const Ramp &r) { return r.name == argv[4]; });
                if (ramp == state.airport.ramps.end())
                    throw std::runtime_error("Requested preview stand not found");
                state.aircraft.position = ramp->position;
                state.aircraft.trueHeading = ramp->heading;
            }
            for (auto &ramp : state.airport.ramps)
                if (argc < 5 && distance(ramp.position, state.aircraft.position) < 20)
                    ramp.aliases.push_back("539");
            state.choices = destinations(state.airport);
            std::string selectedRunway = argc >= 6 ? "RWY " + std::string(argv[5]) : "RWY 17L";
            RouteOptions options;
            options.runwayClearance = true;
            for (std::size_t i = 0; i < state.choices.size(); ++i) {
                const auto &d = state.choices[i];
                if (d.kind != DestinationKind::Runway)
                    continue;
                Availability info;
                try {
                    std::optional<Route> route;
                    try {
                        route = planRoute(state.airport, state.aircraft.position, state.aircraft.trueHeading,
                                          d, options);
                    } catch (const std::exception &) {
                    }
                    if (!route || route->requiresPushback)
                        route = planPushback(state.airport, state.aircraft, d, options, {}).preview;
                    info = {true, route->requiresPushback, route->length, {}};
                    if (d.label == selectedRunway) {
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
            std::string stand = argc >= 5 ? argv[4] : "539";
            state.status = state.route && state.route->pushbackPointCount
                               ? "Preview: " + stand + " pushback -> Node " +
                                     std::to_string(state.route->taxiStartNode) + " -> " + selectedRunway
                               : "Preview: " + stand + " departure -> " + selectedRunway;
            fitMap(state);
            if (!state.route && argc >= 5) {
                selectMapView(state, MapView::Local);
                state.mapFitExtent = {250, 250};
                state.status = std::string("Stand ") + argv[4] + " departure unavailable";
            }
            auto output = std::filesystem::u8path(argv[2]);
            std::filesystem::create_directories(output);
            for (auto size : {std::pair<int, int>{1080, 720}, {900, 600}, {1440, 900}}) {
                checkFrame(state, size.first, size.second);
                if (state.route && state.route->requiresPushback)
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
