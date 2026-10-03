#include "PlanningWorker.h"
#include <chrono>
#include <stdexcept>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
namespace autotaxi {
DeparturePlan planDeparture(const Airport &airport, const AircraftState &state, const Destination &d,
                            const Config &config, const RouteOptions &options) {
    checkPlanning(options);
    if (options.computation)
        options.computation->stage.store(PlanningStage::Forward, std::memory_order_relaxed);
    std::optional<Route> route;
    std::string forwardError;
    try {
        route = planRoute(airport, state.position, state.trueHeading, d, options);
    } catch (const PlanningCancelled &) {
        throw;
    } catch (const std::exception &e) {
        if (!config.automaticPushback)
            throw;
        forwardError = e.what();
    }
    if (config.automaticPushback && (!route || route->requiresPushback)) {
        try {
            auto tow = planPushback(airport, state, d, options, config.controller);
            return {tow.preview, std::move(tow)};
        } catch (const PlanningCancelled &) {
            throw;
        } catch (const std::exception &e) {
            if (forwardError.empty())
                throw;
            throw std::runtime_error(forwardError + " / Pushback: " + e.what());
        }
    }
    return {std::move(*route), std::nullopt};
}
PlanningWorker::PlanningWorker(Compute compute) : compute_(std::move(compute)) {
    if (!compute_)
        compute_ = [](const Airport &airport, const PlanningRequest &request) {
            if (request.kind == PlanningKind::Handoff)
                return DeparturePlan{planRoute(airport, request.state.position, request.state.trueHeading,
                                               request.destination, request.config.route),
                                     std::nullopt};
            return planDeparture(airport, request.state, request.destination, request.config,
                                 request.config.route);
        };
}
void PlanningWorker::setAirport(std::shared_ptr<const Airport> airport) {
    cancel();
    airport_ = std::move(airport);
}
void PlanningWorker::submit(PlanningRequest request) {
    if (!airport_)
        throw std::runtime_error("Detect an airport before planning");
    if (runningControl_)
        runningControl_->cancelled.store(true, std::memory_order_relaxed);
    request.config.route.computation = std::make_shared<PlanningControl>();
    queued_ = Job{std::move(request), ++revision_};
    launch();
}
void PlanningWorker::cancel() {
    if (runningControl_)
        runningControl_->cancelled.store(true, std::memory_order_relaxed);
    ++revision_;
    queued_.reset();
}
bool PlanningWorker::hasWork() const {
    return worker_.valid() || queued_.has_value();
}
std::optional<PlanningProgress> PlanningWorker::progress() const {
    if (queued_)
        return PlanningProgress{queued_->request.destination.label, PlanningStage::Forward, 0, 0, 0, true};
    if (!worker_.valid() || !runningControl_ || runningRevision_ != revision_)
        return std::nullopt;
    return PlanningProgress{
        runningDestination_,
        runningControl_->stage.load(std::memory_order_relaxed),
        runningControl_->candidate.load(std::memory_order_relaxed),
        runningControl_->candidates.load(std::memory_order_relaxed),
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count(),
        false};
}
void PlanningWorker::launch() {
    if (worker_.valid() || !queued_)
        return;
    auto job = std::move(*queued_);
    queued_.reset();
    runningRevision_ = job.revision;
    runningControl_ = job.request.config.route.computation;
    runningDestination_ = job.request.destination.label;
    started_ = std::chrono::steady_clock::now();
    worker_ = std::async(std::launch::async, [airport = airport_, request = std::move(job.request),
                                              compute = compute_]() mutable {
#if defined(_WIN32)
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
        PlanningResult result{std::move(request), std::nullopt, {}};
        try {
            checkPlanning(result.request.config.route);
            result.plan = compute(*airport, result.request);
        } catch (const std::exception &e) {
            result.error = e.what();
        } catch (...) {
            result.error = "Unknown route planning failure";
        }
        return result;
    });
}
std::optional<PlanningResult> PlanningWorker::poll() {
    std::optional<PlanningResult> result;
    if (worker_.valid() && worker_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto completed = worker_.get();
        runningControl_.reset();
        if (runningRevision_ == revision_)
            result = std::move(completed);
    }
    launch();
    return result;
}
} // namespace autotaxi
