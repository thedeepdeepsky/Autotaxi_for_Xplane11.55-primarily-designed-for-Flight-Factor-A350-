#include "TaxiController.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace autotaxi;
namespace {
double originalGeometry(const Route &route, const ControllerConfig &config, Vec2 position) {
    const auto projection = onSegment(position, route.points[0], route.points[1]);
    double remaining = length(route.points[1] - projection.point);
    for (std::size_t i = 2; i < route.points.size(); ++i)
        remaining += length(route.points[i] - route.points[i - 1]);
    return remaining + routeSpeedLimit(route, config, 0, projection.point, remaining, 0);
}
template <class F> double measure(int iterations, F run, double &sum) {
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; ++i)
        sum += run();
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}
} // namespace
int main() {
    try {
        constexpr int iterations = 20000;
        ControllerConfig config;
        config.mainAxleAft = 0;
        config.taxiSpeed = 20 * .514444;
        std::cout << "CPU only; original geometry alone vs complete cached control update.\n"
                  << std::fixed << std::setprecision(3);
        for (int count : {251, 2501, 10001}) {
            Route route;
            route.origin = {31, 121};
            route.length = 5000;
            for (int i = 0; i < count; ++i)
                route.points.push_back({0, 5000.0 * i / (count - 1)});
            const auto position = route.points[1] * .25;
            const AircraftState state{unproject(route.origin, position), 0, 3, true};
            const auto actualPosition = project(route.origin, state.position);
            TaxiController controller;
            controller.start(route, config);
            double oldSum = 0, newSum = 0;
            const auto oldMs =
                measure(iterations, [&] { return originalGeometry(route, config, actualPosition); }, oldSum);
            const auto newMs = measure(
                iterations,
                [&] {
                    const auto out = controller.update(state, .02);
                    if (out.phase != TaxiPhase::Taxi)
                        throw std::runtime_error("Benchmark unexpectedly stopped");
                    return out.remaining + out.targetSpeed;
                },
                newSum);
            if (std::abs(oldSum - newSum) > std::abs(oldSum) * 1e-9)
                throw std::runtime_error("Cached and original geometry differ");
            std::cout << count << " points, " << iterations << " cycles: original geometry " << oldMs
                      << " ms; cached full controller " << newMs << " ms (" << newMs * 1000 / iterations
                      << " us/update).\n";
        }
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
