#pragma once
#include "AptDatabase.h"
#include "PlanningControl.h"
#include <memory>
namespace autotaxi {
enum class DestinationKind { Runway, Ramp, Node, HoldShort };
struct Destination {
    DestinationKind kind;
    std::string label;
    int index = 0, end = 0;
};
struct RouteMarker {
    double distance = 0;
    int node = -1;
    std::string label;
};
struct RoutePathPoint {
    Vec2 position;
    double distance = 0;
    bool pavementRisk = false;
};
struct RouteOptions {
    bool runwayClearance = false;
    bool allowUnknownWidth = true;
    char minimumWidth = 'E';
    double maxJoinDistance = 35;
    double maxApronJoinDistance = 180;
    double maxTurnDegrees = 110;
    double minimumTurnRadius = 10.3632;
    bool allowIntersectionDeparture = false;
    int requiredDepartureNode = -1;
    double maxInitialTurnDegrees = 110;
    double wheelbase = 28.35, cockpitAheadNose = 1.8, mainAxleAft = 2.2, maxSteer = 65;
    double mainGearHalfSpan = 6.5, wheelEdgeMargin = 4, trackingAllowance = 3, maxOversteer = 15;
    bool allowOversteer = true, ignorePavementLimits = false, ignoreStandSize = false;
    bool paintedRunwayExitsOnly = true;
    std::vector<std::string> via;
    std::shared_ptr<PlanningControl> computation;
};
inline void checkPlanning(const RouteOptions &options) {
    if (options.computation)
        options.computation->check();
}
struct Route {
    GeoPoint origin;
    std::vector<Vec2> points;
    std::vector<int> nodeIds;
    std::string label;
    bool runway = false;
    double finalHeading = 0, length = 0;
    bool requiresPushback = false, apronDeparture = false;
    std::string departure;
    std::vector<std::string> crossedRunways;
    double runwayRemaining = 0;
    std::size_t pushbackPointCount = 0;
    int taxiStartNode = -1;
    int atcFallbackCount = 0;
    int inferredGapCount = 0;
    std::vector<RouteMarker> nodesAlongRoute;
    std::vector<RouteMarker> taxiwaysAlongRoute;
    bool cockpitGuidance = false, turnClearanceKnown = true;
    bool mainAxleOversteer = false;
    double maximumOversteer = 0;
    double paintedStartDistance = 0;
    double initialHeading = 0;
    std::vector<double> oversteerOffsets;
    std::vector<bool> pavementRisk;
    bool destinationRisk = false;
    bool ramp = false;
    std::vector<RoutePathPoint> cockpitPath;
    std::vector<RoutePathPoint> mainAxlePath;
    Vec2 cockpitStop;
    double apronArrivalDistance = -1;
};
std::string rampLabel(const Ramp &ramp);
std::string departureLabel(const Airport &airport, GeoPoint position);
bool pavedConnection(const Airport &airport, GeoPoint from, GeoPoint to);
std::vector<Destination> destinations(const Airport &airport);
Route planRoute(const Airport &airport, GeoPoint position, double trueHeading, const Destination &destination,
                const RouteOptions &options);
std::vector<std::string> parseRouteVia(std::string text);
} // namespace autotaxi
