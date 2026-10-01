#pragma once
#include "TaxiController.h"
#include <filesystem>
namespace autotaxi {
struct PushbackSegment {
    int type = 0;
    GeoPoint start, end;
    double startHeading = 0, endHeading = 0;
    bool backward = true;
    double radius = 0;
    bool right = false, userPlaced = false;
};
struct PushbackPlan {
    GeoPoint origin, handoff;
    double heading = 0, length = 0;
    int nodeId = -1;
    std::vector<PushbackSegment> segments;
    std::vector<Vec2> points;
    Route taxi;
    Route preview;
};
PushbackPlan planPushback(const Airport &airport, const AircraftState &aircraft,
                          const Destination &destination, const RouteOptions &options,
                          const ControllerConfig &controller);
std::vector<std::vector<PushbackSegment>> readPushbackCache(std::istream &input);
void writePushbackCache(std::ostream &output, const std::vector<std::vector<PushbackSegment>> &routes);
void storePushbackRoute(const std::filesystem::path &cache, const PushbackPlan &plan);
} // namespace autotaxi
