#pragma once
#include "Config.h"
#include "PushbackPlanner.h"
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <optional>
namespace autotaxi {
struct DeparturePlan {
    Route route;
    std::optional<PushbackPlan> pushback;
};
DeparturePlan planDeparture(const Airport &airport, const AircraftState &state,
                            const Destination &destination, const Config &config,
                            const RouteOptions &options);
enum class PlanningKind { Preview, Start, Handoff };
struct PlanningRequest {
    PlanningKind kind = PlanningKind::Preview;
    Destination destination;
    AircraftState state;
    Config config;
};
struct PlanningResult {
    PlanningRequest request;
    std::optional<DeparturePlan> plan;
    std::string error;
};
struct PlanningProgress {
    std::string destination;
    PlanningStage stage = PlanningStage::Forward;
    std::size_t candidate = 0, candidates = 0;
    double seconds = 0;
    bool queued = false;
};
// One running calculation and one latest request. Cancellation never joins the worker.
class PlanningWorker {
  public:
    using Compute = std::function<DeparturePlan(const Airport &, const PlanningRequest &)>;
    explicit PlanningWorker(Compute compute = {});
    void setAirport(std::shared_ptr<const Airport> airport);
    void submit(PlanningRequest request);
    void cancel();
    std::optional<PlanningResult> poll();
    bool hasWork() const;
    std::optional<PlanningProgress> progress() const;

  private:
    struct Job {
        PlanningRequest request;
        std::size_t revision;
    };
    void launch();
    Compute compute_;
    std::shared_ptr<const Airport> airport_;
    std::optional<Job> queued_;
    std::future<PlanningResult> worker_;
    std::shared_ptr<PlanningControl> runningControl_;
    std::string runningDestination_;
    std::chrono::steady_clock::time_point started_;
    std::size_t revision_ = 0, runningRevision_ = 0;
};
} // namespace autotaxi
