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
    numeric("wheelbase_m", c.controller.wheelbase, 5, 45);
    numeric("main_axle_aft_m", c.controller.mainAxleAft, -10, 20);
    numeric("max_steer_deg", c.controller.maxSteer, 5, 70);
    numeric("steer_rate_deg_s", c.controller.steerRate, 1, 40);
    double speed = c.controller.taxiSpeed / 0.514444;
    numeric("taxi_speed_kt", speed, 2, 20);
    c.controller.taxiSpeed = speed * 0.514444;
    speed = c.controller.turnSpeed / 0.514444;
    numeric("turn_speed_kt", speed, 1, 5);
    c.controller.turnSpeed = speed * 0.514444;
    numeric("max_throttle", c.controller.maxThrottle, 0.02, 0.3);
    numeric("max_cross_track_m", c.controller.maxCrossTrack, 3, 20);
    numeric("feedback_timeout_s", c.feedbackTimeout, 1, 15);
    double unknown = 1;
    numeric("allow_unknown_width", unknown, 0, 1);
    c.route.allowUnknownWidth = unknown == 1;
    double intersections = 0;
    numeric("allow_intersection_departure", intersections, 0, 1);
    c.route.allowIntersectionDeparture = intersections == 1;
    c.route.minimumTurnRadius =
        std::max(20.0, 1.4 * c.controller.wheelbase / std::tan(c.controller.maxSteer * rad));
    double require = 1;
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
