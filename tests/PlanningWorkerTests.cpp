#include "PlanningWorker.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace autotaxi;
namespace {
void check(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}
template <class Predicate> void until(Predicate done) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!done()) {
        check(std::chrono::steady_clock::now() < end, "Planning test timed out");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
PlanningResult completed(PlanningWorker &worker) {
    std::optional<PlanningResult> result;
    until([&] {
        result = worker.poll();
        return result.has_value();
    });
    return std::move(*result);
}
struct Gate {
    std::promise<void> ready;
    std::shared_future<void> wait = ready.get_future().share();
    std::atomic<bool> entered{false};
    bool released = false;
    void release() {
        if (!released) {
            ready.set_value();
            released = true;
        }
    }
};
struct Release {
    Gate &gate;
    ~Release() {
        gate.release();
    }
};
PlanningRequest request(PlanningKind kind, const char *label) {
    PlanningRequest r;
    r.kind = kind;
    r.destination.label = label;
    r.config.route.runwayClearance = kind == PlanningKind::Preview;
    return r;
}
void latestRequest() {
    Gate gate;
    const auto main = std::this_thread::get_id();
    std::atomic<int> calls{0};
    PlanningWorker worker([&](const Airport &airport, const PlanningRequest &r) {
        check(std::this_thread::get_id() != main, "Planning ran on the simulator thread");
        if (++calls == 1) {
            gate.entered = true;
            gate.wait.wait();
        }
        Route route;
        route.label = airport.id + "/" + r.destination.label;
        return DeparturePlan{route, {}};
    });
    Release release{gate};
    auto airport = std::make_shared<Airport>();
    airport->id = "FIRST";
    worker.setAirport(airport);
    worker.submit(request(PlanningKind::Preview, "old"));
    until([&] { return gate.entered.load(); });
    worker.submit(request(PlanningKind::Preview, "discarded"));
    worker.submit(request(PlanningKind::Start, "engage"));
    check(!worker.poll() && calls == 1, "Submission/poll must return while computation is blocked");
    gate.release();
    const auto result = completed(worker);
    check(calls == 2 && result.request.kind == PlanningKind::Start && result.error.empty() &&
              result.plan->route.label == "FIRST/engage" && !result.request.config.route.runwayClearance,
          "Only latest request and its clearance policy may be returned");
    check(!worker.hasWork(), "Completed worker still reports work");
}
void cancelledAirport() {
    Gate gate;
    std::atomic<int> calls{0};
    PlanningWorker worker([&](const Airport &airport, const PlanningRequest &) {
        if (++calls == 1) {
            gate.entered = true;
            gate.wait.wait();
        }
        Route route;
        route.label = airport.id;
        return DeparturePlan{route, {}};
    });
    Release release{gate};
    auto first = std::make_shared<Airport>();
    first->id = "OLD";
    worker.setAirport(first);
    worker.submit(request(PlanningKind::Start, "old"));
    until([&] { return gate.entered.load(); });
    worker.submit(request(PlanningKind::Handoff, "queued"));
    worker.cancel();
    auto second = std::make_shared<Airport>();
    second->id = "NEW";
    worker.setAirport(second);
    gate.release();
    until([&] {
        check(!worker.poll(), "Cancelled result was delivered");
        return !worker.hasWork();
    });
    check(calls == 1, "Cancelled queued request was computed");
    worker.submit(request(PlanningKind::Handoff, "new"));
    const auto result = completed(worker);
    check(result.plan->route.label == "NEW" && result.request.kind == PlanningKind::Handoff,
          "Airport replacement did not change the worker snapshot");
}
void errorsAndRealPlanner() {
    PlanningWorker fails([](const Airport &, const PlanningRequest &) -> DeparturePlan {
        throw std::runtime_error("No painted exit");
    });
    fails.setAirport(std::make_shared<Airport>());
    fails.submit(request(PlanningKind::Start, "invalid"));
    const auto error = completed(fails);
    check(!error.plan && error.error == "No painted exit", "Background errors must reach the main thread");
    auto airport = std::make_shared<Airport>();
    const GeoPoint origin{31, 121};
    airport->nodes[1] = {1, origin, {}};
    airport->nodes[2] = {2, unproject(origin, {0, 200}), {}};
    TaxiEdge edge;
    edge.from = 1;
    edge.to = 2;
    edge.width = 'E';
    airport->edges.push_back(edge);
    PlanningWorker worker;
    worker.setAirport(airport);
    for (auto kind : {PlanningKind::Preview, PlanningKind::Start, PlanningKind::Handoff}) {
        auto r = request(kind, "Node 2");
        r.destination = {DestinationKind::Node, "Node 2", 2};
        r.state = {origin, 0, 0, true};
        r.config.automaticPushback = false;
        const auto expected = planDeparture(*airport, r.state, r.destination, r.config, r.config.route);
        worker.submit(r);
        const auto actual = completed(worker);
        check(actual.error.empty() && actual.plan &&
                  std::equal(actual.plan->route.points.begin(), actual.plan->route.points.end(),
                             expected.route.points.begin(), expected.route.points.end(),
                             [](Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; }) &&
                  actual.plan->route.nodeIds == expected.route.nodeIds && !actual.plan->pushback,
              "Background planning changed forward route geometry");
    }
}
void cooperativeCancellation() {
    std::atomic<int> calls{0};
    std::atomic<bool> entered{false};
    PlanningWorker worker([&](const Airport &, const PlanningRequest &r) {
        if (++calls == 1) {
            r.config.route.computation->stage = PlanningStage::Pushback;
            r.config.route.computation->candidate = 3;
            r.config.route.computation->candidates = 24;
            entered = true;
            for (;;) {
                checkPlanning(r.config.route);
                std::this_thread::yield();
            }
        }
        return DeparturePlan{};
    });
    struct CancelOnExit {
        PlanningWorker &worker;
        ~CancelOnExit() {
            worker.cancel();
        }
    } cancel{worker};
    worker.setAirport(std::make_shared<Airport>());
    worker.submit(request(PlanningKind::Preview, "old"));
    until([&] { return entered.load(); });
    const auto progress = worker.progress();
    check(progress && !progress->queued && progress->stage == PlanningStage::Pushback &&
              progress->candidate == 3 && progress->candidates == 24,
          "Worker did not expose the actual planning stage/candidate");
    worker.submit(request(PlanningKind::Preview, "new"));
    check(worker.progress()->destination == "new", "Progress still describes an obsolete request");
    const auto result = completed(worker);
    check(calls == 2 && result.request.destination.label == "new" && result.error.empty(),
          "New request did not interrupt the old calculation");
    RouteOptions options;
    options.computation = std::make_shared<PlanningControl>();
    options.computation->cancelled = true;
    bool cancelled = false;
    try {
        planDeparture({}, {}, {}, {}, options);
    } catch (const PlanningCancelled &) {
        cancelled = true;
    }
    check(cancelled, "Cancellation was replaced by a route/pushback failure");
}
} // namespace
int main() {
    try {
        latestRequest();
        cancelledAirport();
        errorsAndRealPlanner();
        cooperativeCancellation();
        std::cout << "Async planning, cancellation, airport replacement and real routes passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
