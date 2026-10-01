#include "DsfLoader.h"
#include <cmath>
#include <cstdlib>
#include <fstream>
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
int parseText(const std::filesystem::path &text, Airport &airport) {
    std::ifstream input(text);
    if (!input)
        return 0;
    std::vector<std::string> definitions;
    std::string definition;
    std::vector<GeoPoint> points;
    int active = -1, lines = 0;
    auto finish = [&] {
        int style = active >= 0 && static_cast<std::size_t>(active) < definitions.size()
                        ? yellowStyle(definitions[active])
                        : 0;
        if (style && points.size() >= 2) {
            airport.groundLines.push_back({style, "DSF " + definitions[active], std::move(points)});
            ++lines;
        }
        points.clear();
    };
    for (std::string row; std::getline(input, row);) {
        std::istringstream record(row);
        std::string command;
        record >> command;
        if (command == "POLYGON_DEF") {
            std::getline(record >> std::ws, definition);
            definitions.push_back(definition);
        } else if (command == "BEGIN_POLYGON") {
            finish();
            int index = -1, parameter = 0, dimensions = 0;
            record >> index >> parameter >> dimensions;
            active = index;
        } else if (command == "POLYGON_POINT" && active >= 0) {
            GeoPoint point;
            if (record >> point.lon >> point.lat && valid(point))
                points.push_back(point);
        } else if (command == "END_POLYGON") {
            finish();
            active = -1;
        }
    }
    finish();
    return lines;
}
} // namespace
DsfLoadResult loadDsfPaintedLines(Airport &airport, const std::filesystem::path &simulatorRoot,
                                  const std::string &toolPath) {
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
    result.lines = parseText(text, airport);
    std::error_code error;
    std::filesystem::remove(text, error);
    result.message = result.lines ? "DSF yellow marking lines loaded" : "DSF has no supported yellow lines";
    return result;
}
} // namespace autotaxi
