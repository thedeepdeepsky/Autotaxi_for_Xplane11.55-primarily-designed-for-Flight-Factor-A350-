#pragma once
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include <array>
#include <string>
namespace autotaxi {
inline bool flightFactorA350Loaded() {
    auto plugin = XPLMFindPluginBySignature("1-sim A350");
    if (plugin == XPLM_NO_PLUGIN_ID || !XPLMIsPluginEnabled(plugin))
        return false;
    auto icao = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    if (!icao)
        return false;
    std::array<char, 41> code{};
    XPLMGetDatab(icao, code.data(), 0, 40);
    return std::string(code.data()).find("A35") != std::string::npos;
}
} // namespace autotaxi
