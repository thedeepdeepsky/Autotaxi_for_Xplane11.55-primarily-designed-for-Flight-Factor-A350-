#pragma once
#include "TaxiController.h"
#include <filesystem>
namespace autotaxi {
struct Config {
    ControllerConfig controller;
    RouteOptions route;
    std::string steeringMode = "direct";
    std::string steeringDataref = "sim/flightmodel2/gear/tire_steer_command_deg";
    int steeringIndex = 0;
    double steeringScale = 1.0, steeringSign = 1.0;
    std::string feedbackDataref = "sim/flightmodel2/gear/tire_steer_actual_deg";
    int feedbackIndex = 0;
    std::string throttleDataref = "sim/flightmodel/engine/ENGN_thro_use";
    std::string leftBrakeDataref = "sim/cockpit2/controls/left_brake_ratio";
    std::string rightBrakeDataref = "sim/cockpit2/controls/right_brake_ratio";
    double feedbackTimeout = 4.0;
    bool requireA350 = true;
    bool automaticPushback = true;
    std::string dsfToolPath;
};
Config loadConfig(const std::filesystem::path &path);
} // namespace autotaxi
