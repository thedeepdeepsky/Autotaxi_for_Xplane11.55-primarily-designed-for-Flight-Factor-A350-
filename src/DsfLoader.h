#pragma once
#include "AptDatabase.h"
#include <filesystem>
#include <string>
namespace autotaxi {
struct DsfLoadResult {
    int files = 0;
    int lines = 0;
    std::string message;
};
DsfLoadResult loadDsfPaintedLines(Airport &airport, const std::filesystem::path &simulatorRoot,
                                  const std::string &toolPath);
} // namespace autotaxi
