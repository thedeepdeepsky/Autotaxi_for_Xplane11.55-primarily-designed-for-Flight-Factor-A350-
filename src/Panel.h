#pragma once
#include "RouteTiming.h"
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
enum class TextStyle { Ui, AirportSign };
struct DrawCommand {
    DrawKind kind;
    Box box;
    Color color;
    std::string text;
    double x2 = 0, y2 = 0, x3 = 0, y3 = 0, width = 1;
    TextStyle textStyle = TextStyle::Ui;
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
    Map,
    RouteOptions,
    AllowOversteer,
    RelaxPavement,
    RelaxStand,
    ToggleStandLabels,
    ToggleAirportSigns,
    GoNow,
    ViaInput,
    ApplyVia,
    ClearVia,
    PickVia,
    UndoVia,
    EmergencyBrake
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
    ControllerConfig controllerConfig;
    std::optional<RouteTiming> timing;
    std::vector<RoutePathPoint> predictedCockpitPath;
    double predictionInvalidSeconds = 0;
    RouteOptions routeOptions;
    std::string viaText;
    bool optionsOpen = false, viaFocus = false, pickVia = false, emergencyHeld = false;
    bool showStandLabels = true, showAirportSigns = true;
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
    std::size_t staticMapBegin = 0, staticMapEnd = 0;
};
struct MapDrawCache {
    std::vector<DrawCommand> commands;
    const Airport *airport = nullptr;
    GeoPoint origin;
    Vec2 center;
    Box content{};
    double scale = 0;
    MapView view = MapView::Airport;
    bool valid = false;
    std::size_t rebuildCount = 0;
};
std::vector<int> filteredChoices(const PanelState &state);
PanelFrame buildPanel(const PanelState &state, double width, double height, const MeasureText &measure,
                      MapDrawCache *mapCache = nullptr, MeasureText signMeasure = {});
class PanelFrameCache {
  public:
    const PanelFrame &get(const PanelState &state, double width, double height, const MeasureText &measure,
                          MeasureText signMeasure = {});
    void invalidate(bool airportChanged = false) {
        dirty_ = true;
        if (airportChanged)
            mapCache_.valid = false;
    }
    std::size_t rebuildCount() const {
        return rebuildCount_;
    }
    std::size_t mapRebuildCount() const {
        return mapCache_.rebuildCount;
    }

  private:
    PanelFrame frame_;
    MapDrawCache mapCache_;
    double width_ = 0, height_ = 0;
    bool dirty_ = true;
    std::size_t rebuildCount_ = 0;
};
std::size_t lineBatchEnd(const std::vector<DrawCommand> &commands, std::size_t begin);
void fitAirport(PanelState &state);
void fitMap(PanelState &state);
void selectMapView(PanelState &state, MapView view);
void setPanelRoute(PanelState &state, const Route &route);
void resetPanelTiming(PanelState &state);
void setPanelTiming(PanelState &state, RouteTiming timing, double elapsed = 1);
} // namespace autotaxi
