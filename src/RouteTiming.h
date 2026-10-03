#pragma once
#include "TaxiController.h"
#include <atomic>
#include <future>
#include <memory>
#include <optional>
namespace autotaxi {
struct TimingSample {
    double distance = 0, speed = 0, seconds = 0;
};
struct RouteTiming {
    std::vector<TimingSample> samples;
    double totalSeconds = -1;
    std::vector<RoutePathPoint> cockpitPath;
    double secondsTo(double routeDistance) const;
};
RouteTiming stabilizeTiming(const RouteTiming &previous, RouteTiming next, double elapsed);
std::vector<RoutePathPoint> smoothCockpitPrediction(const std::vector<RoutePathPoint> &previous,
                                                    std::vector<RoutePathPoint> next, double elapsed,
                                                    bool braking);
struct TaxiMotionModel {
    double freeAcceleration = .25, brakeAuthority = 1.5;
};
class TaxiMotionEstimator {
  public:
    void reset(const ControllerConfig &config);
    void observe(double speed, double appliedBrake, double dt);
    TaxiMotionModel model() const {
        return model_;
    }

  private:
    TaxiMotionModel model_;
    bool initialized_ = false;
    double speed_ = 0, elapsed_ = 0, brakeTime_ = 0;
    bool hasWindow_ = false;
    double previousAcceleration_ = 0, previousBrake_ = 0;
};
RouteTiming forecastTaxiTiming(TaxiController controller, AircraftState state, TaxiMotionModel model,
                               const std::atomic<bool> *cancelled = nullptr);
// Standalone preview; the live plugin passes a snapshot including its brake integrator.
RouteTiming estimateRouteTiming(const Route &route, double progress, double actualSpeed,
                                const ControllerConfig &config);
class TimingWorker {
  public:
    ~TimingWorker();
    void submit(TaxiController controller, AircraftState state, TaxiMotionModel model);
    void cancel();
    std::optional<RouteTiming> poll();
    bool hasWork() const {
        return worker_.valid();
    }

  private:
    std::future<RouteTiming> worker_;
    std::shared_ptr<std::atomic<bool>> cancelled_;
};
} // namespace autotaxi
