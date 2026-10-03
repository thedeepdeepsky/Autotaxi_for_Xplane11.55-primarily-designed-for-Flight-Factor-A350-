#pragma once
#include "AirportSignFont.h"
#include "ConsolasFont.h"
#include "MapRenderer.h"
#include "Panel.h"
#include "XPLMDisplay.h"
#include <functional>
namespace autotaxi {
struct UIActions {
    std::function<void()> scan, reload, stop, disconnect;
    std::function<void()> goNow;
    std::function<void(const Destination &, bool, double, bool)> planOrStart;
    std::function<void(const RouteOptions &)> routeSettings;
    std::function<void(double)> speedChanged;
    std::function<void()> emergencyBrake;
};
class TaxiUI {
  public:
    explicit TaxiUI(UIActions actions);
    ~TaxiUI();
    void show();
    bool visible() const;
    void setAirport(const Airport &airport);
    void setStatus(const std::string &status);
    void setBusy(bool busy, bool loading);
    void setRoute(const Route &route);
    void clearRoute();
    void setTiming(RouteTiming timing, double elapsed = 1);
    void setAvailability(const std::unordered_map<std::string, Availability> &availability);
    void setDestinationAvailability(const std::string &label, const Availability &availability);
    void setTelemetry(const AircraftState &aircraft, const ControlOutput &output, double actualSteer,
                      bool waiting);
    void setSpeed(double knots);
    void setControllerConfig(const ControllerConfig &config);
    void setRouteOptions(const RouteOptions &options);

  private:
    static void draw(XPLMWindowID, void *);
    static int mouse(XPLMWindowID, int, int, XPLMMouseStatus, void *);
    static int wheel(XPLMWindowID, int, int, int, int, void *);
    static void key(XPLMWindowID, char, XPLMKeyFlags, char, void *, int);
    static XPLMCursorStatus cursor(XPLMWindowID, int, int, void *);
    void act(Hit hit);
    void preview();
    void applyRouteSettings();
    const PanelFrame &frame() const;
    double measure(const std::string &text) const;
    double measureSign(const std::string &text) const;
    void drawText(double x, double baseline, const std::string &text, Color color,
                  TextStyle style = TextStyle::Ui);
    int beginText();
    void drawGlyphs(double x, double baseline, const std::string &text, Color color);
    int beginSignText();
    void drawSignGlyphs(double x, double baseline, const std::string &text, Color color);
    void endText(int previousEnvironment);
    UIActions actions_;
    PanelState state_;
    mutable PanelFrameCache frameCache_;
    MapRenderer mapRenderer_;
    XPLMWindowID window_ = nullptr;
    ConsolasFont font_;
    AirportSignFont signFont_;
    int fontTexture_ = 0;
    int signFontTexture_ = 0;
    bool dragging_ = false;
    Vec2 dragStart_, dragCenter_;
};
} // namespace autotaxi
