#pragma once
#include "ConsolasFont.h"
#include "Panel.h"
#include "XPLMDisplay.h"
#include <functional>
namespace autotaxi {
struct UIActions {
    std::function<void()> scan, reload, stop, disconnect;
    std::function<void(const Destination &, bool, double, bool)> planOrStart;
};
class TaxiUI {
  public:
    explicit TaxiUI(UIActions actions);
    ~TaxiUI();
    void show();
    void setAirport(const Airport &airport);
    void setStatus(const std::string &status);
    void setBusy(bool busy, bool loading);
    void setRoute(const Route &route);
    void clearRoute();
    void setAvailability(const std::unordered_map<std::string, Availability> &availability);
    void setTelemetry(const AircraftState &aircraft, const ControlOutput &output, double actualSteer,
                      bool waiting);
    void setSpeed(double knots);

  private:
    static void draw(XPLMWindowID, void *);
    static int mouse(XPLMWindowID, int, int, XPLMMouseStatus, void *);
    static int wheel(XPLMWindowID, int, int, int, int, void *);
    static void key(XPLMWindowID, char, XPLMKeyFlags, char, void *, int);
    static XPLMCursorStatus cursor(XPLMWindowID, int, int, void *);
    void act(const Hit &hit);
    void preview();
    PanelFrame frame() const;
    double measure(const std::string &text) const;
    void drawText(double x, double baseline, const std::string &text, Color color);
    UIActions actions_;
    PanelState state_;
    XPLMWindowID window_ = nullptr;
    ConsolasFont font_;
    int fontTexture_ = 0;
    bool dragging_ = false;
    Vec2 dragStart_, dragCenter_;
};
} // namespace autotaxi
