#pragma once
#include "AptDatabase.h"
namespace autotaxi {
enum class DestinationKind { Runway, Ramp, Node };
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
struct RouteOptions {
    bool runwayClearance = false;
    bool allowUnknownWidth = true;
    char minimumWidth = 'E';
    double maxJoinDistance = 35;
    double maxApronJoinDistance = 180;
    double maxTurnDegrees = 110;
    double minimumTurnRadius = 20;
    bool allowIntersectionDeparture = false;
    int requiredDepartureNode = -1;
    double maxInitialTurnDegrees = 110;
};
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
    std::vector<RouteMarker> nodesAlongRoute;
    std::vector<RouteMarker> taxiwaysAlongRoute;
};
std::string rampLabel(const Ramp &ramp);
std::string departureLabel(const Airport &airport, GeoPoint position);
bool pavedConnection(const Airport &airport, GeoPoint from, GeoPoint to);
std::vector<Destination> destinations(const Airport &airport);
Route planRoute(const Airport &airport, GeoPoint position, double trueHeading, const Destination &destination,
                const RouteOptions &options);
} // namespace autotaxi
