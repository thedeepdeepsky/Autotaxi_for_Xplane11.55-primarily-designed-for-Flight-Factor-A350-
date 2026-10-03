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
    bool requireA350 = false;
    bool automaticPushback = true;
    std::string dsfToolPath;
    std::string dsfPavementResources;
    std::string latitudeDataref = "sim/flightmodel/position/latitude";
    std::string longitudeDataref = "sim/flightmodel/position/longitude";
    std::string headingDataref = "sim/flightmodel/position/psi";
    std::string speedDataref = "sim/flightmodel/position/groundspeed";
    std::string onGroundDataref = "sim/flightmodel/failures/onground_any";
    std::string parkingBrakeDataref = "sim/cockpit2/controls/parking_brake_ratio";
    int leftBrakeIndex = -1, rightBrakeIndex = -1, parkingBrakeIndex = -1;
    double brakeScale = 1, parkingBrakeSet = 1, parkingBrakeReleased = 0;
};
Config loadConfig(const std::filesystem::path &path);
} // namespace autotaxi
