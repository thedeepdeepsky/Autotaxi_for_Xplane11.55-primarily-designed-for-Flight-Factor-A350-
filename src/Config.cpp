#include "Config.h"
#include <algorithm>
#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
namespace autotaxi {
Config loadConfig(const std::filesystem::path &path) {
    Config c;
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("Missing A350AutoTaxi.ini");
    std::unordered_map<std::string, std::string> values;
    auto trim = [](std::string s) {
        auto first = s.find_first_not_of(" \t\r\n"), last = s.find_last_not_of(" \t\r\n");
        return first == std::string::npos ? std::string{} : s.substr(first, last - first + 1);
    };
    for (std::string line; std::getline(in, line);) {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';' || line[0] == '[')
            continue;
        auto pos = line.find('=');
        if (pos != std::string::npos)
            values[trim(line.substr(0, pos))] = trim(line.substr(pos + 1));
    }
    auto text = [&](const std::string &key, std::string &out) {
        auto it = values.find(key);
        if (it != values.end())
            out = it->second;
    };
    auto numeric = [&](const std::string &key, double &out, double lo, double hi) {
        auto it = values.find(key);
        if (it == values.end())
            return;
        std::istringstream s(it->second);
        s.imbue(std::locale::classic());
        double n;
        if (!(s >> n) || !std::isfinite(n) || n < lo || n > hi)
            throw std::runtime_error("Invalid config: " + key);
        s >> std::ws;
        if (!s.eof())
            throw std::runtime_error("Invalid config: " + key);
        out = n;
    };
    text("steering_mode", c.steeringMode);
    text("steering_dataref", c.steeringDataref);
    text("feedback_dataref", c.feedbackDataref);
    text("throttle_dataref", c.throttleDataref);
    text("left_brake_dataref", c.leftBrakeDataref);
    text("right_brake_dataref", c.rightBrakeDataref);
    text("dsf_tool_path", c.dsfToolPath);
    text("dsf_pavement_resources", c.dsfPavementResources);
    text("latitude_dataref", c.latitudeDataref);
    text("longitude_dataref", c.longitudeDataref);
    text("heading_dataref", c.headingDataref);
    text("groundspeed_dataref", c.speedDataref);
    text("on_ground_dataref", c.onGroundDataref);
    text("parking_brake_dataref", c.parkingBrakeDataref);
    auto index = [&](const char *key, int &target) {
        double value = target;
        numeric(key, value, -1, 31);
        if (value != std::floor(value))
            throw std::runtime_error(std::string(key) + " must be integer");
        target = static_cast<int>(value);
    };
    index("left_brake_index", c.leftBrakeIndex);
    index("right_brake_index", c.rightBrakeIndex);
    index("parking_brake_index", c.parkingBrakeIndex);
    numeric("brake_scale", c.brakeScale, .001, 100);
    numeric("parking_brake_set", c.parkingBrakeSet, -100, 100);
    numeric("parking_brake_released", c.parkingBrakeReleased, -100, 100);
    if (c.parkingBrakeSet == c.parkingBrakeReleased)
        throw std::runtime_error("Parking brake set/released values must differ");
    double idx = c.steeringIndex;
    numeric("steering_index", idx, -1, 9);
    if (idx != std::floor(idx))
        throw std::runtime_error("steering_index must be integer");
    c.steeringIndex = static_cast<int>(idx);
    idx = c.feedbackIndex;
    numeric("feedback_index", idx, -1, 9);
    if (idx != std::floor(idx))
        throw std::runtime_error("feedback_index must be integer");
    c.feedbackIndex = static_cast<int>(idx);
    numeric("steering_scale", c.steeringScale, 0.00001, 100);
    numeric("steering_sign", c.steeringSign, -1, 1);
    if (c.steeringSign != 1 && c.steeringSign != -1)
        throw std::runtime_error("steering_sign must be 1 or -1");
    numeric("wheelbase_m", c.controller.wheelbase, 1, 65);
    numeric("main_axle_aft_m", c.controller.mainAxleAft, -10, 20);
    numeric("cockpit_ahead_nose_m", c.controller.cockpitAheadNose, 0, 8);
    numeric("main_gear_half_track_m", c.controller.mainGearHalfTrack, .3, 20);
    numeric("main_gear_half_span_m", c.route.mainGearHalfSpan, .3, 20);
    numeric("wheel_edge_margin_m", c.route.wheelEdgeMargin, 0, 10);
    numeric("tracking_allowance_m", c.route.trackingAllowance, .1, 10);
    numeric("max_oversteer_m", c.route.maxOversteer, 0, 30);
    numeric("max_steer_deg", c.controller.maxSteer, 5, 70);
    numeric("steer_rate_deg_s", c.controller.steerRate, 1, 40);
    double speed = c.controller.taxiSpeed / 0.514444;
    double maximum = c.controller.maxTaxiSpeed / .514444;
    numeric("max_taxi_speed_kt", maximum, 2, 100);
    c.controller.maxTaxiSpeed = maximum * .514444;
    numeric("taxi_speed_kt", speed, 1, maximum);
    if (speed > maximum)
        throw std::runtime_error("taxi_speed_kt exceeds max_taxi_speed_kt");
    c.controller.taxiSpeed = speed * 0.514444;
    speed = c.controller.turnSpeed / 0.514444;
    numeric("turn_speed_kt", speed, .5, maximum);
    c.controller.turnSpeed = speed * 0.514444;
    speed = c.controller.apronSpeed / .514444;
    numeric("apron_speed_kt", speed, .5, maximum);
    c.controller.apronSpeed = speed * .514444;
    numeric("apron_approach_distance_m", c.controller.apronApproachDistance, 30, 1000);
    numeric("nominal_acceleration_m_s2", c.controller.acceleration, .01, 3);
    numeric("planned_deceleration_m_s2", c.controller.deceleration, .05, 5);
    numeric("nominal_brake_authority_m_s2", c.controller.brakeAuthority, .1, 8);
    numeric("max_brake_ratio", c.controller.maxBrake, .1, 1);
    numeric("emergency_brake_ratio", c.controller.emergencyBrake, .1, 1);
    numeric("max_cross_track_m", c.controller.maxCrossTrack, 3, 20);
    numeric("feedback_timeout_s", c.feedbackTimeout, 1, 15);
    double unknown = 1;
    numeric("allow_unknown_width", unknown, 0, 1);
    c.route.allowUnknownWidth = unknown == 1;
    double intersections = 0;
    numeric("allow_intersection_departure", intersections, 0, 1);
    c.route.allowIntersectionDeparture = intersections == 1;
    // The default geometric floor follows the main-gear track.  The actual
    // nose-wheel steering limit is checked separately by turnClearance().
    c.route.minimumTurnRadius = std::max(2.0, 2.0 * c.controller.mainGearHalfTrack);
    numeric("minimum_turn_radius_m", c.route.minimumTurnRadius, 2, 150);
    c.route.wheelbase = c.controller.wheelbase;
    c.route.cockpitAheadNose = c.controller.cockpitAheadNose;
    c.route.mainAxleAft = c.controller.mainAxleAft;
    c.route.maxSteer = c.controller.maxSteer;
    std::string width(1, c.route.minimumWidth);
    text("minimum_width_class", width);
    if (width.size() != 1 || width[0] < 'A' || width[0] > 'F')
        throw std::runtime_error("minimum_width_class must be A..F");
    c.route.minimumWidth = width[0];
    auto boolean = [&](const char *key, bool &target) {
        double value = target ? 1 : 0;
        numeric(key, value, 0, 1);
        if (value != 0 && value != 1)
            throw std::runtime_error(std::string(key) + " must be 0 or 1");
        target = value == 1;
    };
    boolean("allow_oversteer", c.route.allowOversteer);
    boolean("ignore_pavement_limits", c.route.ignorePavementLimits);
    boolean("ignore_stand_size", c.route.ignoreStandSize);
    boolean("painted_runway_exits_only", c.route.paintedRunwayExitsOnly);
    std::string via;
    text("route_via", via);
    c.route.via = parseRouteVia(via);
    double require = 0;
    numeric("require_a350", require, 0, 1);
    c.requireA350 = require == 1;
    double automatic = 1;
    numeric("automatic_pushback", automatic, 0, 1);
    if (automatic != 0 && automatic != 1)
        throw std::runtime_error("automatic_pushback must be 0 or 1");
    c.automaticPushback = automatic == 1;
    if (c.steeringMode != "direct" && c.steeringMode != "custom")
        throw std::runtime_error("steering_mode must be direct or custom");
    if (c.steeringMode == "direct" && (c.steeringIndex < 0 || c.steeringScale != 1 || c.steeringSign != 1))
        throw std::runtime_error("Direct steering requires array index, degree scale 1 and sign 1");
    return c;
}
} // namespace autotaxi
