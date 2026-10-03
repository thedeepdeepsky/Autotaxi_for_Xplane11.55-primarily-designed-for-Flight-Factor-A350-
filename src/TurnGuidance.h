#pragma once
#include "RoutePlanner.h"
namespace autotaxi {
void buildReferencePaths(Route &route, const RouteOptions &options);
struct TurnClearance {
    double offtracking = 0, outwardOffset = 0;
    bool feasible = true;
};
TurnClearance turnClearance(double radius, double insideWidth, const RouteOptions &options);
void applyTurnGuidance(const Airport &airport, Route &route, const RouteOptions &options);
void applyAxleOversteer(const Airport &airport, Route &route, const RouteOptions &options);
void applyCockpitOversteer(const Airport &airport, Route &route, const RouteOptions &options);
void assessRoutePavement(const Airport &airport, Route &route, const RouteOptions &options);
} // namespace autotaxi
