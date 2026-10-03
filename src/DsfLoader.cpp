#include "DsfLoader.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
namespace autotaxi {
namespace {
std::string quote(const std::filesystem::path &path) {
    std::string value = path.u8string();
    std::string out = "\"";
    for (char c : value)
        out += c == '"' ? "\\\"" : std::string(1, c);
    return out + "\"";
}
std::string tilePart(double value) {
    int tile = static_cast<int>(std::floor(value));
    return std::string(tile >= 0 ? "+" : "-") + std::to_string(std::abs(tile));
}
std::filesystem::path airportTile(const Airport &airport) {
    if (airport.runways.empty())
        return {};
    auto point = airport.runways.front().ends[0].position;
    // Overlay DSF tiles are stored in the preceding 10x10-degree directory
    // (for example +31+121.dsf lives under +30+120).
    return std::filesystem::path("Earth nav data") /
           (tilePart(std::floor(point.lat / 10) * 10) + tilePart(std::floor(point.lon / 10) * 10)) /
           (tilePart(point.lat) + tilePart(point.lon) + ".dsf");
}
int yellowStyle(const std::string &definition) {
    if (definition.find("Single_Yellow_solid") != std::string::npos)
        return 51;
    if (definition.find("Double_Yellow") != std::string::npos)
        return 2; // display-only boundary/holding marking; never a taxi route
    return 0;
}
bool pavedResource(const std::filesystem::path &pack, const std::string &resource,
                   const std::set<std::string> &selected) {
    if (selected.count(resource))
        return true;
    std::ifstream file(pack / std::filesystem::u8path(resource));
    std::string surface, layer;
    for (std::string row; std::getline(file, row);) {
        std::istringstream input(row);
        std::string token, value;
        input >> token >> value;
        if (token == "SURFACE")
            surface = value;
        if (token == "LAYER_GROUP")
            layer = value;
    }
    return (surface == "asphalt" || surface == "concrete") && (layer == "taxiways" || layer == "runways");
}
struct DsfVertex {
    GeoPoint point, incoming, outgoing;
};
std::vector<GeoPoint> sampleWinding(const std::vector<DsfVertex> &vertices, bool closed) {
    if (vertices.empty())
        return {};
    std::vector<DsfVertex> knots;
    for (const auto &vertex : vertices) {
        // DSF repeats an anchor to encode independent incoming/outgoing handles.
        if (!knots.empty() && distance(knots.back().point, vertex.point) < .001)
            knots.back().outgoing = vertex.outgoing;
        else
            knots.push_back(vertex);
    }
    if (closed && knots.size() > 1 && distance(knots.front().point, knots.back().point) < .001) {
        knots.front().incoming = knots.back().incoming;
        knots.pop_back();
    }
    std::vector<GeoPoint> result{knots.front().point};
    const std::size_t segments = closed ? knots.size() : knots.size() - 1;
    for (std::size_t i = 0; i < segments; ++i) {
        const auto &a = knots[i], &b = knots[(i + 1) % knots.size()];
        Vec2 end = project(a.point, b.point), c1 = project(a.point, a.outgoing),
             c2 = project(a.point, b.incoming);
        double extent = length(c1) + length(c2 - c1) + length(end - c2);
        int count = std::clamp(static_cast<int>(std::ceil(extent / 3)), 1, 10000);
        if (length(c1) < .001 && length(c2 - end) < .001)
            count = 1;
        for (int k = 1; k <= count; ++k) {
            double t = static_cast<double>(k) / count, u = 1 - t;
            result.push_back(k == count ? b.point
                                        : unproject(a.point, c1 * (3 * u * u * t) + c2 * (3 * u * t * t) +
                                                                 end * (t * t * t)));
        }
    }
    return result;
}
} // namespace
DsfLoadResult parseDsfGeometry(std::istream &input, Airport &airport,
                               const std::filesystem::path &sceneryPack,
                               const std::string &pavementResources) {
    DsfLoadResult result;
    std::set<std::string> selected;
    std::istringstream resources(pavementResources);
    for (std::string name; std::getline(resources, name, ';');) {
        auto first = name.find_first_not_of(" \t"), last = name.find_last_not_of(" \t");
        if (first != std::string::npos)
            selected.insert(name.substr(first, last - first + 1));
    }
    std::vector<std::string> definitions;
    std::vector<bool> paved;
    std::string definition;
    std::vector<DsfVertex> points;
    std::vector<std::vector<GeoPoint>> rings;
    int active = -1;
    int dimensions = 0, parameter = 0;
    bool curved = false, closed = false;
    bool malformed = false;
    auto endWinding = [&] {
        if (!points.empty()) {
            rings.push_back(sampleWinding(points, closed));
            points.clear();
        }
    };
    auto finish = [&] {
        endWinding();
        int style = active >= 0 && static_cast<std::size_t>(active) < definitions.size()
                        ? yellowStyle(definitions[active])
                        : 0;
        if (!malformed && style) {
            for (auto &ring : rings)
                if (ring.size() >= 2) {
                    airport.groundLines.push_back({style, "DSF " + definitions[active], std::move(ring)});
                    ++result.lines;
                }
        } else if (!malformed && active >= 0 && static_cast<std::size_t>(active) < definitions.size() &&
                   std::filesystem::path(definitions[active]).extension() == ".pol" && !rings.empty() &&
                   std::all_of(rings.begin(), rings.end(),
                               [](const auto &ring) { return ring.size() >= 3; })) {
            Pavement polygon{rings, "DSF " + definitions[active]};
            airport.sceneryContours.push_back(polygon);
            ++result.contours;
            if (paved[active]) {
                airport.pavements.push_back(std::move(polygon));
                ++result.pavements;
            }
        }
        rings.clear();
        points.clear();
        malformed = false;
    };
    for (std::string row; std::getline(input, row);) {
        std::istringstream record(row);
        std::string command;
        record >> command;
        if (command == "POLYGON_DEF") {
            std::getline(record >> std::ws, definition);
            definitions.push_back(definition);
            paved.push_back(pavedResource(sceneryPack, definition, selected));
        } else if (command == "BEGIN_POLYGON") {
            if (active >= 0)
                malformed = true;
            finish();
            int index = -1;
            parameter = dimensions = 0;
            record >> index >> parameter >> dimensions;
            active = index;
            malformed = !record || dimensions < 2 || dimensions > 64 || index < 0 ||
                        static_cast<std::size_t>(index) >= definitions.size();
            curved = closed = false;
            if (!malformed) {
                auto extension = std::filesystem::path(definitions[index]).extension();
                if (extension == ".lin") {
                    malformed = (dimensions != 2 && dimensions != 4) || (parameter != 0 && parameter != 1);
                    curved = dimensions == 4;
                    closed = parameter == 1;
                } else if (extension == ".pol") {
                    curved = parameter == 65535 ? dimensions == 8 : dimensions == 4;
                    closed = true;
                }
            }
        } else if (command == "BEGIN_WINDING" || command == "END_WINDING") {
            endWinding();
        } else if (command == "POLYGON_POINT" && active >= 0) {
            if (malformed)
                continue;
            std::vector<double> coordinates(dimensions);
            for (double &coordinate : coordinates)
                if (!(record >> coordinate) || !std::isfinite(coordinate))
                    malformed = true;
            GeoPoint point{coordinates[1], coordinates[0]};
            GeoPoint control = curved ? GeoPoint{coordinates[3], coordinates[2]} : point;
            if (malformed || !valid(point) || !valid(control)) {
                malformed = true;
                continue;
            }
            points.push_back({point, unproject(point, project(point, control) * -1), control});
        } else if (command == "END_POLYGON") {
            finish();
            active = -1;
        }
    }
    // Unterminated polygons must not create navigable shortcuts.
    return result;
}
DsfLoadResult loadDsfPaintedLines(Airport &airport, const std::filesystem::path &simulatorRoot,
                                  const std::string &toolPath, const std::string &pavementResources) {
    DsfLoadResult result;
    auto tile = airportTile(airport);
    if (tile.empty() || airport.source.empty()) {
        result.message = "DSF tile unavailable";
        return result;
    }
    // The airport's apt.dat identifies the active package. This avoids pulling
    // unrelated global DSF terrain into the airport route graph.
    auto sceneryPack = std::filesystem::path(airport.source).parent_path().parent_path();
    auto dsf = sceneryPack / tile;
    if (!std::filesystem::is_regular_file(dsf)) {
        result.message = "No DSF tile for active airport package";
        return result;
    }
    std::filesystem::path tool =
        toolPath.empty() ? std::filesystem::path("DSFTool.exe") : std::filesystem::u8path(toolPath);
    if (toolPath.empty()) {
        auto bundled = simulatorRoot / "Resources/plugins/A350AutoTaxi/DSFTool.exe";
        if (std::filesystem::is_regular_file(bundled))
            tool = bundled;
    }
    auto text = std::filesystem::temp_directory_path() / ("A350AutoTaxi-" + airport.id + "-dsf.txt");
    std::string command = quote(tool) + " --dsf2text " + quote(dsf) + " " + quote(text);
    // cmd.exe needs an outer quote when the executable path itself is quoted.
    std::string shellCommand = "cmd.exe /d /s /c \"" + command + "\"";
    if (std::system(shellCommand.c_str()) != 0) {
        result.message = "DSFTool conversion failed; apt.dat only (check dsf_tool_path)";
        return result;
    }
    result.files = 1;
    std::ifstream converted(text);
    result = parseDsfGeometry(converted, airport, sceneryPack, pavementResources);
    result.files = 1;
    converted.close();
    std::error_code error;
    std::filesystem::remove(text, error);
    result.message = "DSF: " + std::to_string(result.lines) + " marking chains, " +
                     std::to_string(result.pavements) + " identified paved polygons, " +
                     std::to_string(result.contours) + " polygon contours; surface coverage incomplete";
    airport.dsfStatus = result.message;
    return result;
}
} // namespace autotaxi
