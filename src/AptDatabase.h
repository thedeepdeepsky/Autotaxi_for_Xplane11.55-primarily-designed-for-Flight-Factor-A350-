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
};
struct GroundLine {
    int style = 0;
    std::string name;
    std::vector<GeoPoint> points;
    bool centerline() const {
        return style == 1 || style == 7 || style == 51 || style == 57;
    }
};
struct PaintedRunwayEntry {
    int node = -1;
    int runway = -1;
};
struct Airport {
    std::string id, name, source;
    std::unordered_map<int, TaxiNode> nodes;
    std::vector<TaxiEdge> edges;
    std::vector<Runway> runways;
    std::vector<Ramp> ramps;
    std::vector<Pavement> pavements;
    std::vector<GroundLine> groundLines;
    std::vector<PaintedRunwayEntry> paintedRunwayEntries;
    std::unordered_map<int, int> nodeAliases;
    std::size_t atcFallbackEdges = 0;
    std::string dsfStatus;
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
