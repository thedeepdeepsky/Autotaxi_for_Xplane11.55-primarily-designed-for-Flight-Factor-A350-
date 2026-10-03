#pragma once
#include "Geo.h"
#include <filesystem>
#include <istream>
#include <string>
#include <unordered_map>
#include <vector>
namespace autotaxi {
struct TaxiNode {
    int id = -1;
    GeoPoint position;
    std::string name;
};
struct TaxiEdge {
    int from = -1, to = -1;
    bool oneWay = false, runway = false;
    char width = 0;
    std::string name;
    std::vector<std::string> activeRunways;
    std::vector<GeoPoint> geometry;
    bool painted = false;
    bool inferredGap = false;
    int standLeadInRamp = -1;
};
struct RunwayEnd {
    std::string name;
    GeoPoint position;
    double displaced = 0;
};
struct Runway {
    double width = 0;
    RunwayEnd ends[2];
};
struct Ramp {
    GeoPoint position;
    double heading = 0;
    char width = 0;
    std::string name;
    std::vector<std::string> aliases;
};
struct Pavement {
    std::vector<std::vector<GeoPoint>> rings;
    std::string source;
};
struct GroundLine {
    int style = 0;
    std::string name;
    std::vector<GeoPoint> points;
    bool standLeadIn = false;
    int standLeadInRamp = -1;
    bool paintedCenterline() const {
        return style == 1 || style == 7 || style == 51 || style == 57 || style == 101 || style == 105;
    }
    bool centerline() const {
        // apt.dat line types: 1/7 are yellow taxiway centerlines, 51/57 are
        // their black-border variants, and 101/105 are centerline light
        // strings. White/custom roadway markings remain display-only; painted
        // centerlines may receive a stand lead-in association.
        return standLeadIn || paintedCenterline();
    }
    bool whiteLeadIn() const {
        return standLeadIn && !paintedCenterline();
    }
    bool whiteMarking() const {
        return style == 19 || style == 20 || style == 22 || style == 30 || style == 31;
    }
};
struct HoldShortPoint {
    GeoPoint position;
    int runway = -1;
    int end = -1;
    double heading = 0;
    std::string label;
};
struct AirportSign {
    GeoPoint position;
    double heading = 0;
    std::string text;
    bool runway = false;
    bool noEntry = false;
};
struct PaintedRunwayEntry {
    int node = -1;
    int runway = -1;
    std::vector<GeoPoint> continuation;
};
struct Airport {
    std::string id, name, source;
    std::unordered_map<int, TaxiNode> nodes;
    std::vector<TaxiEdge> edges;
    std::vector<Runway> runways;
    std::vector<Ramp> ramps;
    std::vector<Pavement> pavements;
    std::vector<GroundLine> groundLines;
    std::vector<HoldShortPoint> holdShortPoints;
    std::vector<AirportSign> signs;
    std::vector<PaintedRunwayEntry> paintedRunwayEntries;
    std::unordered_map<int, int> nodeAliases;
    std::size_t atcFallbackEdges = 0;
    std::size_t centerlineGapLinks = 0;
    std::string dsfStatus;
    // apt.dat does not prove that custom DSF meshes contain no additional paved surface.
    bool pavementCoverageComplete = false;
    std::vector<Pavement> sceneryContours;
};
struct AirportIndex {
    std::string id, name;
    std::filesystem::path file;
    std::streamoff offset = 0;
    GeoPoint anchor;
    double radius = 0;
    bool hasPosition = false;
};
std::vector<std::filesystem::path> discoverAptFiles(const std::filesystem::path &root);
Airport parseAirport(std::istream &input, const std::string &source = {}, bool buildNetwork = true);
class AptDatabase {
  public:
    void scan(const std::filesystem::path &root);
    Airport nearest(GeoPoint position, double maxDistance = 5000) const;
    const std::vector<AirportIndex> &index() const {
        return index_;
    }

  private:
    std::vector<AirportIndex> index_;
    std::vector<AirportIndex> alternatives_;
};
int nearestNode(const Airport &airport, GeoPoint position, double *meters = nullptr);
} // namespace autotaxi
