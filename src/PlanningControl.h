#pragma once
#include <atomic>
#include <cstddef>
#include <stdexcept>
namespace autotaxi {
class PlanningCancelled : public std::runtime_error {
  public:
    PlanningCancelled() : std::runtime_error("Route planning cancelled") {}
};
enum class PlanningStage { Forward, Pushback };
struct PlanningControl {
    std::atomic<bool> cancelled{false};
    std::atomic<PlanningStage> stage{PlanningStage::Forward};
    std::atomic<std::size_t> candidate{0}, candidates{0};
    void check() const {
        if (cancelled.load(std::memory_order_relaxed))
            throw PlanningCancelled();
    }
};
} // namespace autotaxi
