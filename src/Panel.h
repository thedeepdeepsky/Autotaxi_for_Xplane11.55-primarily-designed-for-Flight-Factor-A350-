#pragma once
#include "TaxiController.h"
#include <functional>
#include <optional>
#include <unordered_map>
namespace autotaxi {
struct Color {
    float r, g, b;
};
struct Box {
    double x, y, w, h;
    bool contains(double px, double py) const {
        return px >= x && px <= x + w && py >= y && py <= y + h;
    }
};
enum class DrawKind { Rectangle, Line, Triangle, Text };
struct DrawCommand {
    DrawKind kind;
    Box box;
    Color color;
    std::string text;
    double x2 = 0, y2 = 0, x3 = 0, y3 = 0, width = 1;
};
enum class Action {
    None,
    Runways,
    Ramps,
    Nodes,
    Search,
    Destination,
    SpeedDown,
    SpeedUp,
    Clearance,
    Preview,
    Start,
    Stop,
    Manual,
    Refresh,
    Reload,
    Fit,
    LocalMap,
    AirportMap,
    ZoomIn,
    ZoomOut,
    Map
};
struct Hit {
    Action action;
    Box box;
    int index = -1;
    bool enabled = true;
    std::string tooltip;
};
struct Availability {
    bool reachable = false, pushback = false;
    double length = 0;
    std::string reason;
};
enum class MapView { Local, Airport };
struct PanelState {
    Airport airport;
    std::vector<Destination> choices;
    std::unordered_map<std::string, Availability> availability;
    std::optional<Route> route;
    AircraftState aircraft;
    ControlOutput output;
    double actualSteer = 0, speedKnots = 10;
    int category = 0, selected = -1, scroll = 0;
    std::string query, status = "Detecting airport...";
    bool searchFocus = false, busy = false, loading = false, clearance = false, waitingPushback = false;
    GeoPoint mapOrigin;
    Vec2 mapCenter;
    Vec2 mapFitExtent{500, 500};
    double mapScale = 0;
    MapView mapView = MapView::Airport;
    std::optional<MapView> preferredMapView;
};
using MeasureText = std::function<double(const std::string &)>;
struct PanelFrame {
    std::vector<DrawCommand> commands;
    std::vector<Hit> hits;
    Box map, mapContent, list;
    double mapScale = 1;
    int visibleRows = 0;
};
std::vector<int> filteredChoices(const PanelState &state);
PanelFrame buildPanel(const PanelState &state, double width, double height, const MeasureText &measure);
void fitAirport(PanelState &state);
void fitMap(PanelState &state);
void selectMapView(PanelState &state, MapView view);
void setPanelRoute(PanelState &state, const Route &route);
} // namespace autotaxi
