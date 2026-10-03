#pragma once
#include "AptDatabase.h"
#include <filesystem>
#include <string>
namespace autotaxi {
struct DsfLoadResult {
    int files = 0;
    int lines = 0;
    int pavements = 0, contours = 0;
    std::string message;
};
DsfLoadResult loadDsfPaintedLines(Airport &airport, const std::filesystem::path &simulatorRoot,
                                  const std::string &toolPath, const std::string &pavementResources = {});
DsfLoadResult parseDsfGeometry(std::istream &input, Airport &airport,
                               const std::filesystem::path &sceneryPack,
                               const std::string &pavementResources = {});
} // namespace autotaxi
